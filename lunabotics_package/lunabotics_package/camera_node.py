#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Header

import pyrealsense2 as rs
import math

class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Camera Node Started')

        self.image_publisher = self.create_publisher(Image, 'camera/color/image_raw', 10)
        self.depth_publisher = self.create_publisher(Image, 'camera/depth/image_rect_raw', 10)
        self.scan_publisher = self.create_publisher(LaserScan, '/scan', 10)

        self.angle_min = -0.5
        self.angle_max = 0.5
        self.angle_increment = (self.angle_max - self.angle_min) / 640
        self.range_min = 0.1
        self.range_max = 10.0

        try:
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            self.pipeline.start(config)

            self.timer = self.create_timer(0.1, self.camera_callback)
            self.get_logger().info( "INTEL REALSENSE CONNECTED!" )
        except Exception as e:
            self.get_logger().error(f"{e}")

    
    def camera_callback(self):
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if not color_frame or not depth_frame:
            return

        

        message = LaserScan()

        # Create Header for message
        message.header = Header()
        message.header.frame_id = "laser_scan"
        message.header.stamp = self.get_clock().now().to_msg()

        # Set LaserScan message parameters
        message.angle_min = math.pi / 4 # TODO: make non-hardcoded
        message.angle_max = -math.pi / 4 # TODO: make non-hardcoded
        message.angle_increment = (math.pi / 2) / depth_frame.get_width() # The D455 has a horizontal FOV of 90 degrees

        message.time_increment = 0 # D455 takes all scans at once
        message.scan_time = 1/30 # 30Hz (time between scans)





        message.intensities = [] # TODO: Find method to get intensity data if needed

        print(f"width: {depth_frame.get_width()}, height: {depth_frame.get_height()}, Accessing through method (m): {depth_frame.get_distance(1,1)}, As array: {depth_frame[1][1]}")


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