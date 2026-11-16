#!/usr/bin/python3
"""Measure received wall FPS, RGB layout, matching intrinsics and the complete TF graph."""
import argparse
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage


class Probe(Node):
    def __init__(self, args):
        super().__init__("auto_aim_ros_probe")
        self.args = args
        self.arrivals = []
        self.images = {}
        self.infos = {}
        self.edges = set()
        self.joints = False
        self.errors = set()
        self.matches = 0
        static_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, args.image, self.image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, args.info, self.info, qos_profile_sensor_data)
        self.create_subscription(JointState, args.joints, self.joint, 10)
        self.create_subscription(TFMessage, args.tf, self.transforms, 100)
        self.create_subscription(TFMessage, args.tf_static, self.transforms, static_qos)

    @staticmethod
    def stamp(message):
        return message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec

    def image(self, message):
        self.arrivals.append(time.monotonic())
        if message.encoding != "rgb8" or message.step != message.width * 3:
            self.errors.add("Invalid RGB8/row stride")
        if len(message.data) != message.height * message.step:
            self.errors.add("Invalid RGB byte count")
        self.images[self.stamp(message)] = (message.header.frame_id, message.width, message.height)
        self.match()

    def info(self, message):
        if message.k[0] <= 0 or message.k[4] <= 0:
            self.errors.add("Invalid focal length")
        self.infos[self.stamp(message)] = (message.header.frame_id, message.width, message.height)
        self.match()

    def match(self):
        for stamp in self.images.keys() & self.infos.keys():
            if self.images[stamp] != self.infos[stamp]:
                self.errors.add("Image/CameraInfo frame or dimensions mismatch")
            self.images.pop(stamp)
            self.infos.pop(stamp)
            self.matches += 1
        # A missed DDS frame must not create an unbounded verification queue.
        for queue in (self.images, self.infos):
            while len(queue) > 200:
                queue.pop(next(iter(queue)))

    def transforms(self, message):
        for transform in message.transforms:
            self.edges.add((transform.header.frame_id, transform.child_frame_id))

    def joint(self, message):
        self.joints = len(message.name) == 2 and len(message.position) == 2


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=10)
    parser.add_argument("--min-fps", type=float, default=100)
    parser.add_argument("--image", default="/image_raw")
    parser.add_argument("--info", default="/camera_info")
    parser.add_argument("--joints", default="/joint_states")
    parser.add_argument("--tf", default="/tf")
    parser.add_argument("--tf-static", default="/tf_static")
    parser.add_argument("--frames", nargs=7, default=["world", "odom", "yaw_link", "pitch_link",
                                                    "camera_link", "camera_optical_frame", "muzzle_link"])
    args = parser.parse_args()
    rclpy.init()
    node = Probe(args)
    deadline = time.monotonic() + args.seconds
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        count = len(node.arrivals)
        fps = (count - 1) / (node.arrivals[-1] - node.arrivals[0]) if count > 1 else 0
        world, base, yaw, pitch, camera, optical, muzzle = args.frames
        expected = {(world, base), (base, yaw), (yaw, pitch), (pitch, camera),
                    (camera, optical), (pitch, muzzle)}
        missing = expected - node.edges
        print(f"Received FPS: {fps:.2f}; frames: {count}; image/info matched: {node.matches}")
        print(f"TF missing: {sorted(missing)}; JointState: {node.joints}; errors: {sorted(node.errors)}")
        passed = fps >= args.min_fps and node.matches > 0 and not missing and node.joints and not node.errors
        print("PASS" if passed else "FAIL")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
