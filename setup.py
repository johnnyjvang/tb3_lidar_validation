from setuptools import find_packages, setup

package_name = 'tb3_lidar_validation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'lidar_message_rate = tb3_lidar_validation.lidar_message_rate:main',
            # Added to print and reset json output
            'reset_results = tb3_lidar_validation.reset_results:main',
            'summary_report = tb3_lidar_validation.summary_report:main',
        ],
    },
)
