#!/usr/bin/env python3

"""
symmetry_test.py

Compare left and right LaserScan sectors to evaluate scan symmetry.

This version also checks local side-wall consistency so the result is more
trustworthy when the robot is slightly angled.

Goal:
- Confirm the LIDAR is behaving symmetrically
- Detect scene imbalance or mounting/alignment issues
- Check whether left and right side-wall readings are locally consistent
- Save the result to the JSON results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from tb3_lidar_validation.result_utils import append_result


# ===== Test Settings =====
SCAN_TOPIC = '/scan'
TEST_DURATION = 10.0

# Compare overall left/right side distance around ±90 deg
LEFT_CENTER_DEG = 90.0
RIGHT_CENTER_DEG = -90.0
SIDE_WINDOW_HALF_WIDTH_DEG = 10.0

# Local consistency windows to detect angle / non-flat side reading
LEFT_CHECK_1_DEG = 80.0
LEFT_CHECK_2_DEG = 100.0
RIGHT_CHECK_1_DEG = -80.0
RIGHT_CHECK_2_DEG = -100.0
CHECK_WINDOW_HALF_WIDTH_DEG = 5.0

SYMMETRY_TOL = 0.10              # meters
SIDE_ALIGNMENT_TOL = 0.05        # meters


class SymmetryTest(Node):
    def __init__(self):
        super().__init__('symmetry_test')

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.scan_count = 0

        self.left_measurements = []
        self.right_measurements = []
        self.diff_measurements = []

        self.left_alignment_diffs = []
        self.right_alignment_diffs = []

        self.last_left_avg = None
        self.last_right_avg = None
        self.last_symmetry_diff = None
        self.last_left_align_diff = None
        self.last_right_align_diff = None

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting LIDAR symmetry test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info(
            'Place the robot in a roughly centered hallway or symmetric scene and keep it still'
        )

    def sector_average(self, msg, center_deg, half_width_deg):
        center_rad = math.radians(center_deg)
        half_width_rad = math.radians(half_width_deg)

        values = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if abs(angle - center_rad) > half_width_rad:
                continue

            if math.isnan(r) or math.isinf(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            values.append(r)

        if not values:
            return None, 0

        return sum(values) / len(values), len(values)

    def scan_cb(self, msg):
        self.scan_count += 1

        left_avg, _ = self.sector_average(msg, LEFT_CENTER_DEG, SIDE_WINDOW_HALF_WIDTH_DEG)
        right_avg, _ = self.sector_average(msg, RIGHT_CENTER_DEG, SIDE_WINDOW_HALF_WIDTH_DEG)

        left_check_1, _ = self.sector_average(msg, LEFT_CHECK_1_DEG, CHECK_WINDOW_HALF_WIDTH_DEG)
        left_check_2, _ = self.sector_average(msg, LEFT_CHECK_2_DEG, CHECK_WINDOW_HALF_WIDTH_DEG)

        right_check_1, _ = self.sector_average(msg, RIGHT_CHECK_1_DEG, CHECK_WINDOW_HALF_WIDTH_DEG)
        right_check_2, _ = self.sector_average(msg, RIGHT_CHECK_2_DEG, CHECK_WINDOW_HALF_WIDTH_DEG)

        if left_avg is not None and right_avg is not None:
            symmetry_diff = abs(left_avg - right_avg)

            self.last_left_avg = left_avg
            self.last_right_avg = right_avg
            self.last_symmetry_diff = symmetry_diff

            self.left_measurements.append(left_avg)
            self.right_measurements.append(right_avg)
            self.diff_measurements.append(symmetry_diff)

        if left_check_1 is not None and left_check_2 is not None:
            left_align_diff = abs(left_check_1 - left_check_2)
            self.last_left_align_diff = left_align_diff
            self.left_alignment_diffs.append(left_align_diff)

        if right_check_1 is not None and right_check_2 is not None:
            right_align_diff = abs(right_check_1 - right_check_2)
            self.last_right_align_diff = right_align_diff
            self.right_alignment_diffs.append(right_align_diff)

    def compute_stddev(self, samples):
        if len(samples) < 2:
            return 0.0
        mean_val = sum(samples) / len(samples)
        variance = sum((x - mean_val) ** 2 for x in samples) / len(samples)
        return math.sqrt(variance)

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.diff_measurements:
            avg_diff = sum(self.diff_measurements) / len(self.diff_measurements)

            if self.left_alignment_diffs and self.right_alignment_diffs:
                avg_left_align = sum(self.left_alignment_diffs) / len(self.left_alignment_diffs)
                avg_right_align = sum(self.right_alignment_diffs) / len(self.right_alignment_diffs)

                self.get_logger().info(
                    f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                    f'scans: {self.scan_count} | '
                    f'left={self.last_left_avg:.3f} m | '
                    f'right={self.last_right_avg:.3f} m | '
                    f'sym_diff={avg_diff:.3f} m | '
                    f'left_align={avg_left_align:.3f} m | '
                    f'right_align={avg_right_align:.3f} m'
                )
            else:
                self.get_logger().info(
                    f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                    f'scans: {self.scan_count} | '
                    f'left={self.last_left_avg:.3f} m | '
                    f'right={self.last_right_avg:.3f} m | '
                    f'sym_diff={avg_diff:.3f} m'
                )
        else:
            self.get_logger().info(
                f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                f'scans: {self.scan_count} | waiting for valid left/right sectors'
            )

    def finish_and_exit(self):
        if not self.diff_measurements:
            status = 'FAIL'
            measurement = '0.000 m'
            notes = 'No valid left/right sector measurements collected'
            self.get_logger().error('Test failed: no valid left/right sector data')
        elif not self.left_alignment_diffs or not self.right_alignment_diffs:
            status = 'FAIL'
            measurement = '0.000 m'
            notes = 'No valid side alignment measurements collected'
            self.get_logger().error('Test failed: no valid side alignment data')
        else:
            avg_left = sum(self.left_measurements) / len(self.left_measurements)
            avg_right = sum(self.right_measurements) / len(self.right_measurements)
            avg_diff = sum(self.diff_measurements) / len(self.diff_measurements)
            diff_std = self.compute_stddev(self.diff_measurements)

            avg_left_align = sum(self.left_alignment_diffs) / len(self.left_alignment_diffs)
            avg_right_align = sum(self.right_alignment_diffs) / len(self.right_alignment_diffs)
            left_align_std = self.compute_stddev(self.left_alignment_diffs)
            right_align_std = self.compute_stddev(self.right_alignment_diffs)

            symmetry_ok = avg_diff <= SYMMETRY_TOL
            left_align_ok = avg_left_align <= SIDE_ALIGNMENT_TOL
            right_align_ok = avg_right_align <= SIDE_ALIGNMENT_TOL

            status = 'PASS' if (symmetry_ok and left_align_ok and right_align_ok) else 'FAIL'
            measurement = f'{avg_diff:.3f} m'
            notes = (
                f'left={avg_left:.3f}m, '
                f'right={avg_right:.3f}m, '
                f'left_align={avg_left_align:.3f}m, '
                f'right_align={avg_right_align:.3f}m'
            )

            self.get_logger().info('=== LIDAR Symmetry Test Results ===')
            self.get_logger().info(f'Total scans received: {self.scan_count}')
            self.get_logger().info(f'Valid paired symmetry measurements: {len(self.diff_measurements)}')
            self.get_logger().info(f'Average left distance: {avg_left:.3f} m')
            self.get_logger().info(f'Average right distance: {avg_right:.3f} m')
            self.get_logger().info(f'Average symmetry difference: {avg_diff:.3f} m')
            self.get_logger().info(f'Symmetry diff std dev: {diff_std:.4f} m')
            self.get_logger().info(f'Average left-side alignment diff: {avg_left_align:.3f} m')
            self.get_logger().info(f'Average right-side alignment diff: {avg_right_align:.3f} m')
            self.get_logger().info(f'Left-side alignment std dev: {left_align_std:.4f} m')
            self.get_logger().info(f'Right-side alignment std dev: {right_align_std:.4f} m')
            self.get_logger().info(f'Symmetry tolerance: <= {SYMMETRY_TOL:.3f} m')
            self.get_logger().info(f'Side alignment tolerance: <= {SIDE_ALIGNMENT_TOL:.3f} m')
            self.get_logger().info(f'Result: {status}')

            if not left_align_ok or not right_align_ok:
                self.get_logger().warn(
                    'Robot may be angled relative to side walls, or scene may not be locally flat/symmetric'
                )

        append_result(
            test_name='symmetry_test',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting symmetry_test')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = SymmetryTest()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()