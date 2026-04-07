from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pp_adaptive'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'methods'), glob('config/methods/*.yaml')),
        (os.path.join('share', package_name, 'config', 'benchmarks', 'main'), glob('config/benchmarks/main/*.yaml')),
        (os.path.join('share', package_name, 'config', 'benchmarks', 'ablation'), glob('config/benchmarks/ablation/*.yaml')),
        (os.path.join('share', package_name, 'config', 'benchmarks', 'sensitivity'), glob('config/benchmarks/sensitivity/*.yaml')),
        (os.path.join('share', package_name, 'config', 'scenarios'), glob('config/scenarios/*.yaml')),
        (os.path.join('share', package_name, 'racelines'), glob('racelines/*.csv')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.png')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*.pgm')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools', 'numpy', 'pyyaml', 'pillow'],
    zip_safe=True,
    maintainer='jin',
    maintainer_email='jin@todo.todo',
    description='Curvature-Aware Adaptive Lookahead Pure Pursuit for F1TENTH — IEEE RA-L 2026',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pp_adaptive = pp_adaptive.pure_pursuit_node:main',
            'compute_lookahead_kw_sim = pp_adaptive.compute_adaptive_lookahead_kw2:main',
            'compute_lookahead_alg1_org = pp_adaptive.compute_adaptive_lookahead_org:main',
            'compute_lookahead_kv = pp_adaptive.compute_adaptive_lookahead_kv:main',
            'compute_lookahead_curv = pp_adaptive.compute_adaptive_lookahead_curv:main',
        ],
    },
)
