#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header

import pyrealsense2 as rs
import numpy as np
import math

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Camera Node Started')
        self.msg_count = 0

        self.camera_width = 640
        self.camera_height = 480

        self.depth_horizontal_fov = math.radians(87)

        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, self.camera_width, self.camera_height, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, self.camera_width, self.camera_height, rs.format.z16, 30)
            config.enable_stream(rs.stream.gyro)
            self.pipeline.start(config)

            self.timer = self.create_timer(0.1, self.camera_callback)
            self.get_logger().info( "INTEL REALSENSE CONNECTED!" )
        except Exception as e:
            self.get_logger().error(f"{e}")

        self.image_publisher = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'camera/depth/image_rect_raw', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

    
    def camera_callback(self):
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        gyro_frame = frames.first_or_default(rs.stream.gyro)

        if not color_frame or not depth_frame:
            self.get_logger().warning("Frames not received")
            return

        depth_data = np.asanyarray(depth_frame.get_data())
        gyro_data = gyro_frame.as_motion_frame().get_motion_data()
        angle = gyro_data.y * math.pi * 2

        message = LaserScan()

        # Create Header for message
        message.header = Header()
        message.header.frame_id = "laser_scan"
        message.header.stamp = self.get_clock().now().to_msg()

        # Set LaserScan message parameters
        message.angle_min = -self.depth_horizontal_fov / 2 + angle 
        message.angle_max = self.depth_horizontal_fov / 2 + angle 
        message.angle_increment = self.depth_horizontal_fov / self.camera_width # The D455 has a horizontal FOV of 90 degrees

        message.time_increment = 0.0 # D455 takes all scans at once
        message.scan_time = 1/30 # 30Hz (time between scans)

        message.range_min = 0.3 # in meters
        message.range_max = 5.0  # in meters
        message.ranges = []
        for i in range(depth_data.shape[1]):
            distance = depth_frame.get_distance(i, depth_data.shape[0] // 2)
            message.ranges.append(distance)

        message.intensities = [] # TODO: Find method to get intensity data if needed

        self.scan_publisher.publish(message)
        self.msg_count += 1
        self.get_logger().info(f"LaserScan message #{self.msg_count} published at angle {angle}")


    def stop_pipeline(self):
        self.pipeline.stop()
    

def main(args=None):

    rclpy.init(args=args)

    camera_node = CameraNode()
    rclpy.spin(camera_node)

    camera_node.stop_pipeline()
    camera_node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()