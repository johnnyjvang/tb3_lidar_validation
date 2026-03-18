from setuptools import find_packages, setup
import os
from glob import glob


package_name = 'tb3_lidar_validation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jvang',
    maintainer_email='johnnyjvang@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'message_rate = tb3_lidar_validation.message_rate:main',
            'range_validation = tb3_lidar_validation.range_validation:main',
            'noise_stationary = tb3_lidar_validation.noise_stationary:main',
            'wall_distance_accuracy = tb3_lidar_validation.wall_distance_accuracy:main',
            'symmetry_test = tb3_lidar_validation.symmetry_test:main',
            'front_obstacle_detection = tb3_lidar_validation.front_obstacle_detection:main',
            'nearby_obstacle_detection = tb3_lidar_validation.nearby_obstacle_detection:main',
            # Added to print and reset json output
            'reset_results = tb3_lidar_validation.reset_results:main',
            'summary_report = tb3_lidar_validation.summary_report:main',
        ],
    },
)
