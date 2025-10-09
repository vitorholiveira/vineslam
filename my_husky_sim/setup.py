import os
from glob import glob
from setuptools import setup

package_name = 'my_husky_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vitor',
    maintainer_email='vitorh.magnus.oliveira@gmail.com',
    description='Launch file for spawning Husky in custom Gazebo world',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
