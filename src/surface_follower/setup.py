from setuptools import find_packages, setup
import os

package_name = 'surface_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch file
        (os.path.join('share', package_name, 'launch'),
            ['launch/surface_follower.launch.py']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/surface_follower2.launch.py']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/surface_follower_servo.launch.py']),
        (os.path.join('share', package_name, 'launch'),
            ['launch/surface_follower_servo_pid.launch.py']),
        # config yaml
        (os.path.join('share', package_name, 'config'),
            ['config/surface_follower.yaml']),
        (os.path.join('share', package_name, 'config'),
            ['config/servo_panda.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daniel',
    maintainer_email='hhzhang2016@outlook.com',
    description='Surface follower with force compliance + pointcloud normal estimation',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "surface_follower_node = surface_follower.surface_follower_node:main",
            "surface_follower_node2 = surface_follower.surface_follower_node2:main",
            "surface_follower_node3 = surface_follower.surface_follower_node3:main",
            "surface_follower_node4 = surface_follower.surface_follower_node4:main",
            "dummy_pc = surface_follower.dummy_pc:main",
            "dummy_wrench = surface_follower.dummy_wrench:main",
        ],
    },
)
