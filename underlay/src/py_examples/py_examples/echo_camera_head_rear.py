#!/usr/bin/env python3
"""
Head rear monocular camera multi-topic subscription example

Supports selecting the topic type to subscribe via startup parameter --ros-args -p topic_type:=<type>:
  - rgb_image: RGB image (sensor_msgs/Image)
  - rgb_image_compressed: RGB compressed image (sensor_msgs/CompressedImage)
  - camera_info: Camera intrinsic parameters (sensor_msgs/CameraInfo)

Example:
  ros2 run py_examples echo_camera_head_rear --ros-args -p topic_type:=rgb_image
  ros2 run py_examples echo_camera_head_rear --ros-args -p topic_type:=rgb_image_compressed
  ros2 run py_examples echo_camera_head_rear --ros-args -p topic_type:=camera_info

Default topic_type is rgb_image

Note: Some rear cameras are rotated by 90 degrees. This example automatically
detects them and applies rotation compensation when needed.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, CameraInfo
from collections import deque
import os
import cv2
import numpy as np
import re

REAR_ROTATION_PATTERN = re.compile(r't3[d.]5', re.IGNORECASE)


class HeadRearCameraTopicEcho(Node):
    def __init__(self):
        super().__init__('head_rear_camera_topic_echo')

        self.declare_parameter('topic_type', 'rgb_image')
        self.declare_parameter('dump_video_path', '')
        self.declare_parameter('dump_video_fps', 10.0)
        self.declare_parameter('with_mask', False)
        self.declare_parameter('auto_rotate', True)

        self.topic_type = self.get_parameter('topic_type').value
        self.dump_video_path = self.get_parameter('dump_video_path').value
        self.dump_video_fps = self.get_parameter('dump_video_fps').value
        self.with_mask = self.get_parameter('with_mask').value
        self.auto_rotate = self.get_parameter('auto_rotate').value
        self.mask_image = None
        self.need_rotation = False
        self.hw_version = None
        self.video_writer = None
        self.frame_count = 0

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        if self.auto_rotate:
            self._detect_rotate_from_file()

        if self.with_mask and self.dump_video_path:
            mask_file = 'rgb_head_rear_mask_t35.png' if self.need_rotation else 'rgb_head_rear_mask.png'
            mask_path = os.path.join(
                os.path.dirname(__file__), 'data', mask_file)
            self.mask_image = cv2.imread(mask_path, cv2.IMREAD_GRAYSCALE)
            if self.mask_image is None:
                self.get_logger().error(
                    f"Failed to load mask file: {mask_path}")
                raise ValueError("Failed to load mask file")
            self.get_logger().info(f"🎭 Loaded mask: {mask_file}")

        if self.topic_type == "rgb_image":
            self.topic_name = "/aima/hal/sensor/rgb_head_rear/rgb_image"
            self.sub_image = self.create_subscription(
                Image, self.topic_name, self.cb_image, qos)
            self.get_logger().info(
                f"✅ Subscribing RGB Image: {self.topic_name}")
            if self.dump_video_path:
                mask_state = "with mask" if self.with_mask else "without mask"
                rotation_state = "with rotation compensation" if self.need_rotation else "without rotation"
                self.get_logger().info(
                    f"📝 Will dump received images {mask_state} {rotation_state} to video: {self.dump_video_path}")

        elif self.topic_type == "rgb_image_compressed":
            self.topic_name = "/aima/hal/sensor/rgb_head_rear/rgb_image/compressed"
            self.sub_compressed = self.create_subscription(
                CompressedImage, self.topic_name, self.cb_compressed, qos)
            self.get_logger().info(
                f"✅ Subscribing CompressedImage: {self.topic_name}")

        elif self.topic_type == "camera_info":
            self.topic_name = "/aima/hal/sensor/rgb_head_rear/camera_info"
            camera_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
            )
            self.sub_camerainfo = self.create_subscription(
                CameraInfo, self.topic_name, self.cb_camerainfo, camera_qos)
            self.get_logger().info(
                f"✅ Subscribing CameraInfo (with transient_local): {self.topic_name}")

        else:
            self.get_logger().error(f"Unknown topic_type: {self.topic_type}")
            raise ValueError("Unknown topic_type")

        self.last_print = self.get_clock().now()
        self.print_allowed = False
        self.arrivals = deque()

    def _detect_rotate_from_file(self):
        try:
            hw_file = '/agibot/data/info/hardware_version'
            if os.path.exists(hw_file):
                with open(hw_file, 'r') as f:
                    self.hw_version = f.read().strip()
                self.get_logger().info(
                    f"📋 Hardware version from file: '{self.hw_version}'")
                self._check_rotation(self.hw_version)
            else:
                self.get_logger().warn(
                    f"⚠️  Hardware version file not found: {hw_file}")
        except Exception as e:
            self.get_logger().warn(
                f"⚠️  Failed to read hardware version from file: {e}")

    def _check_rotation(self, version: str):
        if REAR_ROTATION_PATTERN.search(version):
            self.need_rotation = True
            self.get_logger().info(
                "🔄 Rotation detected: Rear camera rotation compensation enabled (90° counter-clockwise)"
            )
        else:
            self.get_logger().info(
                f"✅ No rotation detected ({version}): No rotation compensation needed"
            )

    def _rotate_image_90_ccw(self, image: np.ndarray) -> np.ndarray:
        return cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    def update_arrivals(self):
        now = self.get_clock().now()
        self.arrivals.append(now)
        while self.arrivals and (now - self.arrivals[0]).nanoseconds * 1e-9 > 1.0:
            self.arrivals.popleft()

    def get_fps(self):
        return len(self.arrivals)

    def should_print(self, master=True):
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
        self.update_arrivals()

        if self.should_print():
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            rotation_note = " (rotated 90° CCW)" if self.need_rotation else ""
            self.get_logger().info(
                f"📸 {self.topic_type} received{rotation_note}\n"
                f"  • frame_id:        {msg.header.frame_id}\n"
                f"  • stamp (sec):     {stamp_sec:.6f}\n"
                f"  • encoding:        {msg.encoding}\n"
                f"  • size (WxH):      {msg.width} x {msg.height}\n"
                f"  • step (bytes/row):{msg.step}\n"
                f"  • is_bigendian:    {msg.is_bigendian}\n"
                f"  • recv FPS (1s):   {self.get_fps():.1f}"
            )

        if self.topic_type == "rgb_image" and self.dump_video_path:
            self.dump_image_to_video(msg)

    def cb_compressed(self, msg: CompressedImage):
        self.update_arrivals()

        if self.should_print():
            stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.get_logger().info(
                f"🗜️  {self.topic_type} received\n"
                f"  • frame_id:        {msg.header.frame_id}\n"
                f"  • stamp (sec):     {stamp_sec:.6f}\n"
                f"  • format:          {msg.format}\n"
                f"  • data size:       {len(msg.data)}\n"
                f"  • recv FPS (1s):   {self.get_fps():.1f}"
            )

    def cb_camerainfo(self, msg: CameraInfo):
        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        d_str = ", ".join([f"{d:.6f}" for d in msg.d])
        k_str = ", ".join([f"{k:.6f}" for k in msg.k])
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
        try:
            if msg.encoding != "rgb8":
                self.get_logger().warn(
                    f"image encoding not expected: {msg.encoding}")
                return

            np_array = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                msg.height, msg.width, 3)
            cv_image = cv2.cvtColor(np_array, cv2.COLOR_RGB2BGR)

            if self.need_rotation:
                cv_image = self._rotate_image_90_ccw(cv_image)

            if self.mask_image is not None:
                mask_resized = cv2.resize(
                    self.mask_image, (cv_image.shape[1], cv_image.shape[0]))
                cv_image[mask_resized == 0] = 0

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

        except Exception as e:
            self.get_logger().error(f"Failed to process image for video: {e}")

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info(
                f"� Video saved: {self.dump_video_path} ({self.frame_count} frames)")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HeadRearCameraTopicEcho()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
