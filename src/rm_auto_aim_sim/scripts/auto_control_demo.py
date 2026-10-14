#!/usr/bin/python3
"""AUTO example: Float64 angle commands in rad; a Bool controls continuous fire."""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, Float64


class AutoControlDemo(Node):
    def __init__(self, args):
        super().__init__("auto_aim_control_demo")
        self.args = args
        self.pitch = self.create_publisher(Float64, args.pitch_topic, 1)
        self.yaw = self.create_publisher(Float64, args.yaw_topic, 1)
        self.fire = self.create_publisher(Bool, args.fire_topic, 1)
        self.start = time.monotonic()
        self.timer = self.create_timer(0.01, self.tick)

    def tick(self):
        elapsed = time.monotonic() - self.start
        self.pitch.publish(Float64(data=0.08 + 0.08 * math.sin(elapsed)))
        self.yaw.publish(Float64(data=0.4 * math.sin(0.5 * elapsed)))
        self.fire.publish(Bool(data=self.args.fire))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--fire", action="store_true", help="Continuously fire while this demo runs")
    parser.add_argument("--pitch-topic", default="/gimbal/target_pitch")
    parser.add_argument("--yaw-topic", default="/gimbal/target_yaw")
    parser.add_argument("--fire-topic", default="/gimbal/fire")
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args, signal_handler_options=SignalHandlerOptions.NO)
    node = AutoControlDemo(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.timer.cancel()
        if rclpy.ok():
            for _ in range(3):
                node.fire.publish(Bool(data=False))
                rclpy.spin_once(node, timeout_sec=0.02)
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
