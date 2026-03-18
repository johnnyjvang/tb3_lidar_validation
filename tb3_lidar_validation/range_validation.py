#!/usr/bin/env python3

"""
range_validation.py

Validate LaserScan ranges for correctness and usability.
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from tb3_lidar_validation.result_utils import append_result


SCAN_TOPIC = '/scan'
TEST_DURATION = 10.0
VALID_RATIO_TOL = 0.90


class RangeValidation(Node):
    def __init__(self):
        super().__init__('range_validation')

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.scan_count = 0
        self.total_beams = 0
        self.valid_beams = 0
        self.nan_count = 0
        self.inf_count = 0
        self.out_of_range_count = 0

        self.global_min_valid = None
        self.global_max_valid = None

        self.last_valid_ratio = 0.0
        self.last_range_min = None
        self.last_range_max = None

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info('Starting LIDAR range validation test')

    def scan_cb(self, msg):
        self.scan_count += 1
        self.last_range_min = msg.range_min
        self.last_range_max = msg.range_max

        scan_total = len(msg.ranges)
        scan_valid = 0

        for r in msg.ranges:
            self.total_beams += 1

            if math.isnan(r):
                self.nan_count += 1
                continue

            if math.isinf(r):
                self.inf_count += 1
                continue

            if r < msg.range_min or r > msg.range_max:
                self.out_of_range_count += 1
                continue

            self.valid_beams += 1
            scan_valid += 1

            if self.global_min_valid is None or r < self.global_min_valid:
                self.global_min_valid = r

            if self.global_max_valid is None or r > self.global_max_valid:
                self.global_max_valid = r

        if scan_total > 0:
            self.last_valid_ratio = scan_valid / scan_total

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time
        overall_valid = (
            self.valid_beams / self.total_beams if self.total_beams > 0 else 0.0
        )

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s | scans={self.scan_count} | '
            f'valid={overall_valid*100:.1f}%'
        )

    def finish_and_exit(self):
        if self.total_beams == 0:
            status = 'FAIL'
            measurement = '0%'
            notes = 'No scan data'
        else:
            valid_ratio = self.valid_beams / self.total_beams
            valid_percent = valid_ratio * 100.0

            status = 'PASS' if valid_ratio >= VALID_RATIO_TOL else 'FAIL'
            measurement = f'{valid_percent:.1f}% valid'

            notes = (
                f'nan={self.nan_count}, '
                f'inf={self.inf_count}, '
                f'out_of_range={self.out_of_range_count}'
            )

            self.get_logger().info('=== LIDAR Range Validation Results ===')
            self.get_logger().info(f'Valid: {valid_percent:.1f}%')
            self.get_logger().info(f'NaN: {self.nan_count}')
            self.get_logger().info(f'Inf: {self.inf_count}')
            self.get_logger().info(f'Out-of-range: {self.out_of_range_count}')

            if self.global_min_valid is not None:
                self.get_logger().info(
                    f'Observed range: {self.global_min_valid:.3f} → {self.global_max_valid:.3f} m'
                )

            self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='range_validation',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                rclpy.shutdown()
            return

        if time.time() - self.start_time >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = RangeValidation()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()