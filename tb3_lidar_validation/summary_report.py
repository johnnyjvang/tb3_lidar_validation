import csv
from pathlib import Path

import rclpy
from rclpy.node import Node

from tb3_lidar_validation.result_utils import RESULTS_FILE

TEST_ORDER = [
    'timed_forward',
    'timed_back',
    'odom_forward',
    'odom_back',
    'rotate_ccw',
    'rotate_cw',
]


class SummaryReport(Node):
    def __init__(self):
        super().__init__('summary_report')
        self.print_summary()

    def print_summary(self):
        results = {}

        if RESULTS_FILE.exists():
            with open(RESULTS_FILE, 'r', newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    results[row['test']] = row

        rows = []
        for test_name in TEST_ORDER:
            if test_name in results:
                row = results[test_name]
                rows.append([
                    row.get('test', test_name),
                    row.get('status', 'UNKNOWN'),
                    row.get('measurement', ''),
                    row.get('notes', '')
                ])
            else:
                rows.append([test_name, 'MISSING', '', 'no result found'])

        headers = ['Test', 'Status', 'Measurement', 'Notes']
        widths = [len(h) for h in headers]

        for row in rows:
            for i, cell in enumerate(row):
                widths[i] = max(widths[i], len(str(cell)))

        def format_row(row):
            return '| ' + ' | '.join(str(cell).ljust(widths[i]) for i, cell in enumerate(row)) + ' |'

        border = '+-' + '-+-'.join('-' * w for w in widths) + '-+'

        print()
        print(border)
        print(format_row(headers))
        print(border)
        for row in rows:
            print(format_row(row))
        print(border)
        print()


def main(args=None):
    rclpy.init(args=args)
    node = SummaryReport()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()