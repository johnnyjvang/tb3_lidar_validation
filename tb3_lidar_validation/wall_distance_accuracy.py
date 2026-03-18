#!/usr/bin/env python3

"""
wall_distance_accuracy.py

Measure the distance to a flat wall using the center beams of the LaserScan,
and check whether the robot is approximately perpendicular to the wall.

Goal:
- Confirm the LIDAR reports a reasonable wall distance
- Confirm the robot is not significantly angled relative to the wall
- Save the result to the JSON results file
"""

import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from tb3_lidar_validation.result_utils import append_result


# ===== Default Test Settings =====
SCAN_TOPIC = '/scan'
TEST_DURATION = 10.0
CENTER_WINDOW_DEG = 10.0           # total center averaging window is +/- this value
ALIGNMENT_HALF_WINDOW_DEG = 10.0   # use left/right sectors around center for alignment
DEFAULT_EXPECTED_DISTANCE = 1.0    # meters
DISTANCE_TOL = 0.05                # meters
ALIGNMENT_TOL = 0.03               # meters difference between left/right averages


class WallDistanceAccuracy(Node):
    def __init__(self):
        super().__init__('wall_distance_accuracy')

        self.declare_parameter('expected_distance', DEFAULT_EXPECTED_DISTANCE)
        self.expected_distance = float(self.get_parameter('expected_distance').value)

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.scan_count = 0

        self.center_measurements = []
        self.left_measurements = []
        self.right_measurements = []
        self.alignment_diffs = []

        self.last_center_avg = None
        self.last_left_avg = None
        self.last_right_avg = None
        self.last_alignment_diff = None

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting wall distance accuracy test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Expected wall distance: {self.expected_distance:.3f} m')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Place the robot facing a flat wall and keep it still')

    def sector_average(self, msg, angle_min_deg, angle_max_deg):
        values = []

        min_rad = math.radians(angle_min_deg)
        max_rad = math.radians(angle_max_deg)

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if angle < min_rad or angle > max_rad:
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

        center_avg, center_count = self.sector_average(
            msg,
            -CENTER_WINDOW_DEG,
            CENTER_WINDOW_DEG
        )

        left_avg, left_count = self.sector_average(
            msg,
            1.0,
            ALIGNMENT_HALF_WINDOW_DEG
        )

        right_avg, right_count = self.sector_average(
            msg,
            -ALIGNMENT_HALF_WINDOW_DEG,
            -1.0
        )

        if center_avg is not None:
            self.last_center_avg = center_avg
            self.center_measurements.append(center_avg)

        if left_avg is not None:
            self.last_left_avg = left_avg
            self.left_measurements.append(left_avg)

        if right_avg is not None:
            self.last_right_avg = right_avg
            self.right_measurements.append(right_avg)

        if left_avg is not None and right_avg is not None:
            alignment_diff = abs(left_avg - right_avg)
            self.last_alignment_diff = alignment_diff
            self.alignment_diffs.append(alignment_diff)

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.center_measurements:
            running_center = sum(self.center_measurements) / len(self.center_measurements)
            dist_error = running_center - self.expected_distance

            if self.alignment_diffs:
                running_align = sum(self.alignment_diffs) / len(self.alignment_diffs)
                self.get_logger().info(
                    f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                    f'scans: {self.scan_count} | '
                    f'center avg: {running_center:.3f} m | '
                    f'dist error: {dist_error:+.3f} m | '
                    f'align diff: {running_align:.3f} m'
                )
            else:
                self.get_logger().info(
                    f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                    f'scans: {self.scan_count} | '
                    f'center avg: {running_center:.3f} m | '
                    f'dist error: {dist_error:+.3f} m | '
                    f'waiting for alignment data'
                )
        else:
            self.get_logger().info(
                f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                f'scans: {self.scan_count} | waiting for valid center beams'
            )

    def compute_stddev(self, samples):
        if len(samples) < 2:
            return 0.0
        mean_val = sum(samples) / len(samples)
        variance = sum((x - mean_val) ** 2 for x in samples) / len(samples)
        return math.sqrt(variance)

    def finish_and_exit(self):
        if not self.center_measurements:
            status = 'FAIL'
            measurement = '0.000 m'
            notes = 'No valid center-beam measurements collected'
            self.get_logger().error('Test failed: no valid center-beam measurements')
        elif not self.alignment_diffs:
            status = 'FAIL'
            measurement = '0.000 m'
            notes = 'No valid alignment measurements collected'
            self.get_logger().error('Test failed: no valid alignment measurements')
        else:
            avg_distance = sum(self.center_measurements) / len(self.center_measurements)
            distance_stddev = self.compute_stddev(self.center_measurements)
            distance_error = avg_distance - self.expected_distance
            abs_distance_error = abs(distance_error)

            avg_left = sum(self.left_measurements) / len(self.left_measurements) if self.left_measurements else 0.0
            avg_right = sum(self.right_measurements) / len(self.right_measurements) if self.right_measurements else 0.0
            avg_alignment_diff = sum(self.alignment_diffs) / len(self.alignment_diffs)
            alignment_stddev = self.compute_stddev(self.alignment_diffs)

            distance_ok = abs_distance_error <= DISTANCE_TOL
            alignment_ok = avg_alignment_diff <= ALIGNMENT_TOL

            status = 'PASS' if (distance_ok and alignment_ok) else 'FAIL'
            measurement = f'{avg_distance:.3f} m'
            notes = (
                f'expected={self.expected_distance:.3f}m, '
                f'dist_error={distance_error:+.3f}m, '
                f'align_diff={avg_alignment_diff:.3f}m'
            )

            self.get_logger().info('=== LIDAR Wall Distance Accuracy Results ===')
            self.get_logger().info(f'Total scans received: {self.scan_count}')
            self.get_logger().info(f'Valid center measurements: {len(self.center_measurements)}')
            self.get_logger().info(f'Expected distance: {self.expected_distance:.3f} m')
            self.get_logger().info(f'Measured average distance: {avg_distance:.3f} m')
            self.get_logger().info(f'Distance error: {distance_error:+.3f} m')
            self.get_logger().info(f'Distance std dev: {distance_stddev:.4f} m')
            self.get_logger().info(f'Average left-center distance: {avg_left:.3f} m')
            self.get_logger().info(f'Average right-center distance: {avg_right:.3f} m')
            self.get_logger().info(f'Average alignment diff: {avg_alignment_diff:.3f} m')
            self.get_logger().info(f'Alignment std dev: {alignment_stddev:.4f} m')
            self.get_logger().info(f'Distance tolerance: +/- {DISTANCE_TOL:.3f} m')
            self.get_logger().info(f'Alignment tolerance: <= {ALIGNMENT_TOL:.3f} m')
            self.get_logger().info(f'Result: {status}')

            if not alignment_ok:
                self.get_logger().warn(
                    'Robot may be angled relative to wall; left/right center sectors do not match well'
                )

        append_result(
            test_name='wall_distance_accuracy',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting wall_distance_accuracy')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = WallDistanceAccuracy()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()