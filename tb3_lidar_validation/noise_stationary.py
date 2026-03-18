#!/usr/bin/env python3

"""
noise_stationary.py

Keep the robot stationary and measure LaserScan range stability over time.

Goal:
- Confirm the LIDAR is stable while the robot is not moving
- Measure average beam noise across repeated scans
- Identify worst-case beam noise
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
TEST_DURATION = 10.0              # seconds
MIN_SAMPLES_PER_BEAM = 5
AVG_NOISE_TOL = 0.03              # meters
MAX_NOISE_WARN = 0.08             # meters


class NoiseStationary(Node):
    def __init__(self):
        super().__init__('noise_stationary')

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.scan_count = 0
        self.beam_samples = None

        self.valid_beam_count = 0
        self.avg_noise = None
        self.max_noise = None

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting LIDAR stationary noise test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info('Keep the robot completely still during this test')

    def scan_cb(self, msg):
        self.scan_count += 1

        if self.beam_samples is None:
            self.beam_samples = [[] for _ in range(len(msg.ranges))]
            self.get_logger().info(f'Initialized sample buffers for {len(msg.ranges)} beams')

        for i, r in enumerate(msg.ranges):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            self.beam_samples[i].append(r)

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time

        sampled_beams = 0
        if self.beam_samples is not None:
            sampled_beams = sum(1 for samples in self.beam_samples if len(samples) >= MIN_SAMPLES_PER_BEAM)

        self.get_logger().info(
            f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
            f'scans: {self.scan_count} | '
            f'beams with enough samples: {sampled_beams}'
        )

    def compute_stddev(self, samples):
        if len(samples) < 2:
            return None
        mean_val = sum(samples) / len(samples)
        variance = sum((x - mean_val) ** 2 for x in samples) / len(samples)
        return math.sqrt(variance)

    def finish_and_exit(self):
        if self.scan_count == 0 or self.beam_samples is None:
            status = 'FAIL'
            measurement = '0.0000 m'
            notes = 'No scan data received'
            self.get_logger().error('Test failed: no scan data received')
        else:
            beam_stddevs = []

            for samples in self.beam_samples:
                if len(samples) < MIN_SAMPLES_PER_BEAM:
                    continue

                stddev = self.compute_stddev(samples)
                if stddev is not None:
                    beam_stddevs.append(stddev)

            self.valid_beam_count = len(beam_stddevs)

            if self.valid_beam_count == 0:
                status = 'FAIL'
                measurement = '0.0000 m'
                notes = 'No beams had enough valid samples'
                self.get_logger().error('Test failed: no beams had enough valid samples')
            else:
                self.avg_noise = sum(beam_stddevs) / len(beam_stddevs)
                self.max_noise = max(beam_stddevs)

                status = 'PASS' if self.avg_noise <= AVG_NOISE_TOL else 'FAIL'
                measurement = f'{self.avg_noise:.4f} m'
                notes = (
                    f'max_noise={self.max_noise:.4f}m, '
                    f'valid_beams={self.valid_beam_count}'
                )

                self.get_logger().info('=== LIDAR Stationary Noise Test Results ===')
                self.get_logger().info(f'Total scans received: {self.scan_count}')
                self.get_logger().info(f'Beams with enough samples: {self.valid_beam_count}')
                self.get_logger().info(f'Average beam noise (std dev): {self.avg_noise:.4f} m')
                self.get_logger().info(f'Max beam noise (std dev): {self.max_noise:.4f} m')
                self.get_logger().info(f'Pass tolerance (avg): <= {AVG_NOISE_TOL:.4f} m')

                if self.max_noise > MAX_NOISE_WARN:
                    self.get_logger().warn(
                        f'Worst beam noise is high: {self.max_noise:.4f} m'
                    )

                self.get_logger().info(f'Result: {status}')

        append_result(
            test_name='noise_stationary',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting noise_stationary')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = NoiseStationary()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()