from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pp_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'racelines'), glob('racelines/*.csv')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='anonymous',
    maintainer_email='anonymous@example.com',
    description='Pure Pursuit driver for F1TENTH Gym ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pp_core = pp_core.pure_pursuit_node:main',
        ],
    },
)
