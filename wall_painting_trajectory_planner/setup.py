from setuptools import setup
import os
from glob import glob

package_name = 'wall_painting_trajectory_planner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'launch'), glob('launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    author='Carlo',
    author_email='carlo_wiesse@artc.a-star.edu.sg',
    zip_safe=True,
    description='Task planner for wall-painting demo',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'start = wall_painting_trajectory_planner.path_estimator:main',
        ],
    },
)
