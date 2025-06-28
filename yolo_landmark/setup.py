import os
from glob import glob
from setuptools import setup, find_packages

package_name = 'yolo_landmark'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'nav2_params'), glob('nav2_params/*')),
        (os.path.join('share', package_name, 'sdf'), glob('sdf/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ns',
    maintainer_email='nsukhov22@gmail.com',
    description='Automated landmark detection and annotation',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'yolo_node = yolo_landmark.yolo_node:main',
            'teleop = yolo_landmark.teleop:main'
        ],
    },
)

