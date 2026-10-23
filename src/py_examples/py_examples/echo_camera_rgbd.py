#!/usr/bin/env python3
"""
Head depth camera multi-topic subscription example

Supports selecting the topic type to subscribe via startup parameter --ros-args -p topic_type:=<type>:
  - depth_image: Depth image (sensor_msgs/Image)
  - depth_camera_info: Camera intrinsic parameters (sensor_msgs/CameraInfo)
  - rgb_image: RGB image (sensor_msgs/Image)
  - rgb_image_compressed: RGB compressed image (sensor_msgs/CompressedImage)
  - rgb_camera_info: Camera intrinsic parameters (sensor_msgs/CameraInfo)

Example:
  ros2 run py_examples echo_camera_rgbd --ros-args -p topic_type:=rgb_image
  ros2 run py_examples echo_camera_rgbd --ros-args -p topic_type:=rgb_camera_info

Default topic_type is rgb_image
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from collections import deque
import cv2
import numpy as np


class CameraTopicEcho(Node):
    def __init__(self):
        super().__init__('camera_topic_echo')

        # Select the topic type to subscribe
        self.declare_parameter('topic_type', 'rgb_image')
        self.declare_parameter('dump_video_path', '')
        self.declare_parameter('dump_video_fps', 30.0)

        self.topic_type = self.get_parameter('topic_type').value
        self.dump_video_path = self.get_parameter('dump_video_path').value
        self.dump_video_fps = self.get_parameter('dump_video_fps').value
        self.video_writer = None
        self.frame_count = 0

        # SensorDataQoS: BEST_EFFORT + VOLATILE
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Create different subscribers based on topic_type
        if self.topic_type == "depth_image":
            self.topic_name = "/aima/hal/sensor/rgbd_head_front/depth_image"
            self.sub_image = self.create_subscription(
                Image, self.topic_name, self.cb_image, qos)
            self.get_logger().info(
                f"✅ Subscribing Depth Image: {self.topic_name}")

        elif self.topic_type == "rgb_image":
            self.topic_name = "/aima/hal/sensor/rgbd_head_front/rgb_image"
            self.sub_image = self.create_subscription(
                Image, self.topic_name, self.cb_image, qos)
            self.get_logger().info(
                f"✅ Subscribing RGB Image: {self.topic_name}")
            if self.dump_video_path:
                self.get_logger().info(
                    f"📝 Will dump received images to video: {self.dump_video_path}")

        elif self.topic_type == "rgb_image_compressed":
            self.topic_name = "/aima/hal/sensor/rgbd_head_front/rgb_image/compressed"
            self.sub_compressed = self.create_subscription(
                CompressedImage, self.topic_name, self.cb_compressed, qos)
            self.get_logger().info(
                f"✅ Subscribing CompressedImage: {self.topic_name}")

        elif self.topic_type == "rgb_camera_info":
            self.topic_name = "/aima/hal/sensor/rgbd_head_front/rgb_camera_info"
            # RGB-D CameraInfo subscriptions are different from other cameras. The best_effort + volatile QoS is sufficient for 30Hz rgb_camera_info
            self.sub_camerainfo = self.create_subscription(
                CameraInfo, self.topic_name, self.cb_camerainfo, qos)
            self.get_logger().info(
                f"✅ Subscribing RGB CameraInfo: {self.topic_name}")

        elif self.topic_type == "depth_camera_info":
            self.topic_name = "/aima/hal/sensor/rgbd_head_front/depth_camera_info"
            # RGB-D CameraInfo subscriptions are different from other cameras. The best_effort + volatile QoS is sufficient for 30Hz depth_camera_info
            self.sub_camerainfo = self.create_subscription(
                CameraInfo, self.topic_name, self.cb_camerainfo, qos)
            self.get_logger().info(
                f"✅ Subscribing Depth CameraInfo: {self.topic_name}")

        else:
            self.get_logger().error(f"Unknown topic_type: {self.topic_type}")
            raise ValueError("Unknown topic_type")

        # Internal state
        self.last_print = self.get_clock().now()
        self.print_allowed = False
        self.arrivals = deque()

    def update_arrivals(self):
        """Calculate received FPS"""
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

    def get_fps(self):
        """Get FPS"""
        return len(self.arrivals)

    def should_print(self, master=True):
        """Control print frequency"""
        if not master:
            return self.print_allowed
        now = self.get_clock().now()
        if (now - self.last_print).nanoseconds * 1e-9 >= 1.0:
            self.last_print = now
            self.print_allowed = True
        else:
            self.print_allowed = False
        return self.print_allowed

    def cb_image(self, msg: Image):
        """Image callback (Depth/RGB image)"""
        self.update_arrivals()

        if self.should_print():
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"📸 {self.topic_type} received\n"
                f"  • frame_id:        {msg.header.frame_id}\n"
                f"  • stamp (sec):     {stamp_sec:.6f}\n"
                f"  • encoding:        {msg.encoding}\n"
                f"  • size (WxH):      {msg.width} x {msg.height}\n"
                f"  • step (bytes/row):{msg.step}\n"
                f"  • is_bigendian:    {msg.is_bigendian}\n"
                f"  • recv FPS (1s):   {self.get_fps():.1f}"
            )

        # Only RGB image supports video dump
        if self.topic_type == "rgb_image" and self.dump_video_path:
            self.dump_image_to_video(msg)

    def cb_compressed(self, msg: CompressedImage):
        """CompressedImage callback"""
        self.update_arrivals()

        if self.should_print():
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"🗜️  CompressedImage received\n"
                f"  • frame_id:        {msg.header.frame_id}\n"
                f"  • stamp (sec):     {stamp_sec:.6f}\n"
                f"  • format:          {msg.format}\n"
                f"  • data size:       {len(msg.data)}\n"
                f"  • recv FPS (1s):   {self.get_fps():.1f}"
            )

    def cb_camerainfo(self, msg: CameraInfo):
        """CameraInfo callback (camera intrinsic parameters)"""
        # Camera info will only receive one frame, print it directly
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Format D array
        d_str = ", ".join([f"{d:.6f}" for d in msg.d])

        # Format K matrix
        k_str = ", ".join([f"{k:.6f}" for k in msg.k])

        # Format P matrix
        p_str = ", ".join([f"{p:.6f}" for p in msg.p])

        self.get_logger().info(
            f"📷 {self.topic_type} received\n"
            f"  • frame_id:        {msg.header.frame_id}\n"
            f"  • stamp (sec):     {stamp_sec:.6f}\n"
            f"  • width x height:  {msg.width} x {msg.height}\n"
            f"  • distortion_model:{msg.distortion_model}\n"
            f"  • D: [{d_str}]\n"
            f"  • K: [{k_str}]\n"
            f"  • P: [{p_str}]\n"
            f"  • binning_x: {msg.binning_x}\n"
            f"  • binning_y: {msg.binning_y}\n"
            f"  • roi: {{ x_offset: {msg.roi.x_offset}, y_offset: {msg.roi.y_offset}, height: {msg.roi.height}, width: {msg.roi.width}, do_rectify: {msg.roi.do_rectify} }}"
        )

    def dump_image_to_video(self, msg: Image):
        """Dump RGB image to video file using mp4v encoding"""
        if msg.encoding != "rgb8":
            self.get_logger().warn(
                f"image encoding not expected: {msg.encoding}")
            return

        np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3)
        cv_image = cv2.cvtColor(np_array, cv2.COLOR_RGB2BGR)

        if self.video_writer is None:
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            height, width = cv_image.shape[:2]
            self.video_writer = cv2.VideoWriter(
                self.dump_video_path, fourcc, self.dump_video_fps, (width, height))
            if not self.video_writer.isOpened():
                self.get_logger().error(
                    f"Failed to open video writer: {self.dump_video_path}")
                self.dump_video_path = ''
                return
            self.get_logger().info(
                f"📹 Video writer initialized: {width}x{height} @ {self.dump_video_fps}fps")

        self.video_writer.write(cv_image)
        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f"📹 Written {self.frame_count} frames to video")

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(
                f"📹 Video saved: {self.dump_video_path} ({self.frame_count} frames)")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraTopicEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
