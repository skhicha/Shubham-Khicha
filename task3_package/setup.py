from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task3_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'radius_publisher = task3_package.radius_publisher:main',
        'compute_ang_vel_server = task3_package.compute_ang_vel_server:main',
        'turtlebot_node = task3_package.turtlebot_node:main',
        'infinity_publisher = task3_package.infinity_publisher:main',
        ],
    },
)
