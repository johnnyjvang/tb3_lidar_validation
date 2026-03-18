#!/usr/bin/env python3

"""
message_rate.py

Subscribe to the LaserScan topic and measure how consistently messages arrive.

Goal:
- Confirm the LIDAR topic is publishing
- Measure average message rate
- Measure min/max time between messages
- Save the result to the JSON results file
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from tb3_lidar_validation.result_utils import append_result


# ===== Test Settings =====
SCAN_TOPIC = '/scan'
TEST_DURATION = 10.0          # seconds to observe scan messages
EXPECTED_RATE = 5.0           # Hz, common TurtleBot3 LDS default
RATE_TOL = 1.5                # acceptable +/- Hz tolerance


class LidarMessageRate(Node):
    def __init__(self):
        super().__init__('message_rate')

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.first_msg_time = None
        self.last_msg_time = None
        self.msg_count = 0
        self.dt_list = []

        self.done = False
        self.finish_time = None

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting LIDAR message rate test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')

    def scan_cb(self, msg):
        now = time.time()

        if self.first_msg_time is None:
            self.first_msg_time = now
            self.get_logger().info('First scan message received')

        if self.last_msg_time is not None:
            dt = now - self.last_msg_time
            self.dt_list.append(dt)

        self.last_msg_time = now
        self.msg_count += 1

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.msg_count > 1 and self.first_msg_time is not None:
            active_time = self.last_msg_time - self.first_msg_time
            est_rate = (len(self.dt_list) / active_time) if active_time > 0 else 0.0
        else:
            est_rate = 0.0

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'messages: {self.msg_count} | est rate: {est_rate:.2f} Hz'
        )

    def finish_and_exit(self):
        if self.msg_count < 2 or len(self.dt_list) == 0:
            status = 'FAIL'
            measurement = '0.00 Hz'
            notes = f'No sufficient scan messages received on {SCAN_TOPIC}'
            self.get_logger().error('Test failed: insufficient scan messages received')
        else:
            elapsed = self.last_msg_time - self.first_msg_time
            avg_rate = len(self.dt_list) / elapsed if elapsed > 0.0 else 0.0
            min_dt = min(self.dt_list)
            max_dt = max(self.dt_list)
            avg_dt = sum(self.dt_list) / len(self.dt_list)

            rate_error = abs(avg_rate - EXPECTED_RATE)
            status = 'PASS' if rate_error <= RATE_TOL else 'FAIL'

            measurement = f'{avg_rate:.2f} Hz'
            notes = (
                f'rate={avg_rate:.1f}Hz, '
                f'dt_avg={avg_dt:.4f}s'
            )

            self.get_logger().info('=== LIDAR Message Rate Test Results ===')
            self.get_logger().info(f'Topic: {SCAN_TOPIC}')
            self.get_logger().info(f'Total messages: {self.msg_count}')
            self.get_logger().info(f'Elapsed time: {elapsed:.3f} s')
            self.get_logger().info(f'Average rate: {avg_rate:.2f} Hz')
            self.get_logger().info(f'Average dt: {avg_dt:.4f} s')
            self.get_logger().info(f'Min dt: {min_dt:.4f} s')
            self.get_logger().info(f'Max dt: {max_dt:.4f} s')
            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='message_rate',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting message_rate')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = LidarMessageRate()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()