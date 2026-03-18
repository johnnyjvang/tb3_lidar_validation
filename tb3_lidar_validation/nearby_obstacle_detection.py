#!/usr/bin/env python3

"""
nearby_obstacle_detection.py

Scan the full LaserScan and detect nearby obstacle clusters around the robot.

Goal:
- Detect obstacles within a chosen radius around the robot
- Estimate how many nearby objects/clusters are present
- Report nearest obstacle distance
- Report approximate center angle and width of each cluster
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

DEFAULT_DETECTION_RADIUS = 0.9144   # 3 feet in meters
RANGE_JUMP_TOL = 0.10               # meters; split cluster if adjacent ranges jump too much
MIN_CLUSTER_BEAMS = 3               # ignore tiny noisy clusters
MAX_CLUSTERS_TO_REPORT = 8


class NearbyObstacleDetection(Node):
    def __init__(self):
        super().__init__('nearby_obstacle_detection')

        self.declare_parameter('detection_radius', DEFAULT_DETECTION_RADIUS)
        self.detection_radius = float(self.get_parameter('detection_radius').value)

        self.sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.scan_cb, 10)

        self.start_time = time.time()
        self.done = False
        self.finish_time = None

        self.scan_count = 0

        self.last_cluster_count = 0
        self.last_nearest_distance = None
        self.last_clusters = []

        self.cluster_count_history = []
        self.nearest_distance_history = []

        self.timer = self.create_timer(0.1, self.loop)
        self.progress_timer = self.create_timer(1.0, self.progress_update)

        self.get_logger().info(f'Starting nearby obstacle detection test on topic: {SCAN_TOPIC}')
        self.get_logger().info(f'Collecting data for {TEST_DURATION:.1f} seconds...')
        self.get_logger().info(f'Detecting obstacles within {self.detection_radius:.3f} m')

    def valid_points_within_radius(self, msg):
        points = []

        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if math.isnan(r) or math.isinf(r):
                continue

            if r < msg.range_min or r > msg.range_max:
                continue

            if r > self.detection_radius:
                continue

            points.append({
                'index': i,
                'angle_rad': angle,
                'distance': r
            })

        return points

    def cluster_points(self, points):
        if not points:
            return []

        clusters = []
        current_cluster = [points[0]]

        for prev, curr in zip(points[:-1], points[1:]):
            adjacent_beam = (curr['index'] == prev['index'] + 1)
            small_range_jump = abs(curr['distance'] - prev['distance']) <= RANGE_JUMP_TOL

            if adjacent_beam and small_range_jump:
                current_cluster.append(curr)
            else:
                clusters.append(current_cluster)
                current_cluster = [curr]

        clusters.append(current_cluster)

        filtered_clusters = [
            c for c in clusters if len(c) >= MIN_CLUSTER_BEAMS
        ]

        return filtered_clusters

    def summarize_cluster(self, cluster):
        distances = [p['distance'] for p in cluster]
        angles_deg = [math.degrees(p['angle_rad']) for p in cluster]

        min_distance = min(distances)
        mean_distance = sum(distances) / len(distances)
        center_angle_deg = sum(angles_deg) / len(angles_deg)
        width_deg = max(angles_deg) - min(angles_deg)

        return {
            'beam_count': len(cluster),
            'min_distance': min_distance,
            'mean_distance': mean_distance,
            'center_angle_deg': center_angle_deg,
            'width_deg': width_deg,
        }

    def scan_cb(self, msg):
        self.scan_count += 1

        points = self.valid_points_within_radius(msg)
        clusters = self.cluster_points(points)
        summaries = [self.summarize_cluster(c) for c in clusters]

        self.last_clusters = summaries
        self.last_cluster_count = len(summaries)

        if summaries:
            nearest = min(s['min_distance'] for s in summaries)
            self.last_nearest_distance = nearest
            self.nearest_distance_history.append(nearest)

        self.cluster_count_history.append(len(summaries))

    def compute_mean(self, values):
        if not values:
            return None
        return sum(values) / len(values)

    def progress_update(self):
        if self.done:
            return

        elapsed = time.time() - self.start_time

        if self.last_nearest_distance is not None:
            self.get_logger().info(
                f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                f'scans: {self.scan_count} | '
                f'clusters: {self.last_cluster_count} | '
                f'nearest: {self.last_nearest_distance:.3f} m'
            )
        else:
            self.get_logger().info(
                f'[Progress] {elapsed:.1f}s / {TEST_DURATION:.1f}s | '
                f'scans: {self.scan_count} | '
                f'clusters: {self.last_cluster_count} | '
                f'no nearby obstacles detected'
            )

    def finish_and_exit(self):
        avg_cluster_count = self.compute_mean(self.cluster_count_history)
        avg_nearest_distance = self.compute_mean(self.nearest_distance_history)

        if avg_cluster_count is None:
            status = 'FAIL'
            measurement = '0 clusters'
            notes = 'No scan data received'
            self.get_logger().error('Test failed: no scan data received')
        else:
            status = 'PASS'
            measurement = f'{avg_cluster_count:.1f} clusters'

            if avg_nearest_distance is None:
                notes = f'no obstacles within {self.detection_radius:.3f}m'
            else:
                notes = f'nearest={avg_nearest_distance:.3f}m within {self.detection_radius:.3f}m'

            self.get_logger().info('=== Nearby Obstacle Detection Results ===')
            self.get_logger().info(f'Total scans received: {self.scan_count}')
            self.get_logger().info(f'Detection radius: {self.detection_radius:.3f} m')
            self.get_logger().info(f'Average cluster count: {avg_cluster_count:.2f}')

            if avg_nearest_distance is not None:
                self.get_logger().info(f'Average nearest obstacle distance: {avg_nearest_distance:.3f} m')
            else:
                self.get_logger().info('Average nearest obstacle distance: none detected')

            if self.last_clusters:
                self.get_logger().info('Last scan cluster summary:')
                for idx, cluster in enumerate(self.last_clusters[:MAX_CLUSTERS_TO_REPORT], start=1):
                    self.get_logger().info(
                        f'  Cluster {idx}: '
                        f'angle={cluster["center_angle_deg"]:.1f} deg | '
                        f'min_dist={cluster["min_distance"]:.3f} m | '
                        f'mean_dist={cluster["mean_distance"]:.3f} m | '
                        f'width={cluster["width_deg"]:.1f} deg | '
                        f'beams={cluster["beam_count"]}'
                    )

                if len(self.last_clusters) > MAX_CLUSTERS_TO_REPORT:
                    self.get_logger().info(
                        f'  ... {len(self.last_clusters) - MAX_CLUSTERS_TO_REPORT} more cluster(s)'
                    )
            else:
                self.get_logger().info('Last scan cluster summary: no nearby obstacles detected')

            self.get_logger().info('Result: PASS')

        append_result(
            test_name='nearby_obstacle_detection',
            status=status,
            measurement=measurement,
            notes=notes
        )

        self.done = True
        self.finish_time = time.time()

    def loop(self):
        if self.done:
            if time.time() - self.finish_time > 0.5:
                self.get_logger().info('Exiting nearby_obstacle_detection')
                rclpy.shutdown()
            return

        elapsed = time.time() - self.start_time
        if elapsed >= TEST_DURATION:
            self.finish_and_exit()


def main(args=None):
    rclpy.init(args=args)
    node = NearbyObstacleDetection()
    rclpy.spin(node)
    node.destroy_node()


if __name__ == '__main__':
    main()