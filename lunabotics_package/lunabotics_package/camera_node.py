#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

import pyrealsense2 as rs
import numpy as np
import math
import struct
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
            self.pipeline.start(config)

            self.get_logger().info( "INTEL REALSENSE CONNECTED!" )
        except Exception as e:
            self.get_logger().error(f"{e}")

        self.image_publisher = self.create_publisher(Image, 'D455/color/image_raw', 10)
        self.pointcloud_publisher = self.create_publisher(PointCloud2, 'D455/pointcloud', 10)
        self.scan_publisher = self.create_publisher(LaserScan, 'D455/scan', 10)

        self.timer = self.create_timer(0.1, self.camera_callback)

    def camera_callback(self):
        frames = self.pipeline.wait_for_frames()

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            self.get_logger().warning("Frames not received")
            return

        depth_data = np.asanyarray(depth_frame.get_data())
        color_data = np.asanyarray(color_frame.get_data())

        laserscan_message = LaserScan()

        # Create Header for Laserscan message
        laserscan_message.header = Header()
        laserscan_message.header.frame_id = "laser_scan" #TODO: Change to whatever frame id is needed
        laserscan_message.header.stamp = self.get_clock().now().to_msg()

        # Set LaserScan message parameters
        laserscan_message.angle_min = -self.depth_horizontal_fov / 2 # TODO: Make not hardcoded
        laserscan_message.angle_max = self.depth_horizontal_fov / 2  # TODO: Make not hardcoded
        laserscan_message.angle_increment = self.depth_horizontal_fov / self.camera_width # The D455 has a depth horizontal FOV of 90 degrees

        laserscan_message.time_increment = 0.0 # D455 takes all scans at once
        laserscan_message.scan_time = 1/30 # 30Hz (time between scans)

        laserscan_message.range_min = 0.3 # in meters
        laserscan_message.range_max = 5.0  # in meters

        laserscan_message.ranges = []
        for i in range(depth_data.shape[1]):
            distance = depth_frame.get_distance(i, depth_data.shape[0] // 2)
            laserscan_message.ranges.append(distance)

        laserscan_message.intensities = [] # TODO: Find method to get intensity data if needed

        # Publish LaserScan message
        self.scan_publisher.publish(laserscan_message)

        intrinsics: rs.intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics

        pointcloud_message = PointCloud2()

        pointcloud_message.header = Header()
        pointcloud_message.header.frame_id = "camera_link" # TODO: Change to whatever frame id is needed
        pointcloud_message.header.stamp = self.get_clock().now().to_msg()

        pointcloud_message.height = self.camera_height
        pointcloud_message.width = self.camera_width
        pointcloud_message.is_dense = False
        pointcloud_message.is_bigendian = False

        pointcloud_message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1)
            ]

        pointcloud_message.point_step = 16
        pointcloud_message.row_step = pointcloud_message.point_step * self.camera_width

        pointcloud_data = []

        for y in range(self.camera_height):
            for x in range(self.camera_width):
                depth = depth_frame.get_distance(x, y)
                if 0.3 < depth < 5.0:
                    point = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], depth)
                    r, g, b = color_frame.get_data()[y, x]
                    rgb = struct.unpack('I', struct.pack('BBB', b, g, r))[0]
                    pointcloud_data.append((*point, rgb))
                else:
                    pointcloud_data.append((float('nan'), float('nan'), float('nan'), 0))

        pointcloud_message.data = np.array(pointcloud_data, dtype=np.float32).tobytes()
        self.pointcloud_publisher.publish(pointcloud_message)

        self.publish_image(color_data)
        
        self.msg_count += 1
        
        self.get_logger().info(f"Camera Realsense D455 has published {self.msg_count} times")

    def publish_image(self, color_data: List[List[List[int]]]):
        img_message = Image()

        # Create Header for Image message
        img_message.header = Header()
        img_message.header.stamp = self.get_clock().now().to_msg()
        img_message.header.frame_id = "color_frame" #TODO: Change to whatever frame id is needed

        # Set Image message parameters
        img_message.height = color_data.shape[0]
        img_message.width = color_data.shape[1]
        img_message.encoding = 'bgr8'
        img_message.is_bigendian = False
        img_message.step = color_data.shape[1] * color_data.shape[2]
        img_message.data = color_data.tobytes()

        self.image_publisher.publish(img_message)


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