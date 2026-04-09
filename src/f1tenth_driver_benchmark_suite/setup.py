from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_driver_benchmark_suite'


setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'EVALUATION_MANUAL.md']),
        (os.path.join('share', package_name), ['generate_performance_report.py']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch', 'drivers'), glob('launch/drivers/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'drivers'), glob('config/drivers/*.yaml')),
        (os.path.join('share', package_name, 'config', 'scenarios'), glob('config/scenarios/*.yaml')),
        (os.path.join('share', package_name, 'config', 'tests'), glob('config/tests/*.yaml')),
        (os.path.join('share', package_name, 'assets', 'maps'), glob('assets/maps/*')),
        (os.path.join('share', package_name, 'assets', 'racelines'), glob('assets/racelines/*.csv')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml', 'matplotlib', 'psutil'],
    zip_safe=True,
    maintainer='anonymous',
    maintainer_email='anonymous@example.com',
    description='Unified benchmark suite for F1TENTH driver comparison',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_collector_node = f1tenth_driver_benchmark_suite.metrics_collector_node:main',
            'realtime_visualizer = f1tenth_driver_benchmark_suite.realtime_visualizer:main',
            'raceline_path_publisher = f1tenth_driver_benchmark_suite.raceline_path_publisher:main',
            'benchmark_runner = f1tenth_driver_benchmark_suite.benchmark_runner:main',
            'evaluation_runner = f1tenth_driver_benchmark_suite.evaluation_runner:main',
            'report_aggregator = f1tenth_driver_benchmark_suite.report_aggregator:main',
        ],
    },
)
