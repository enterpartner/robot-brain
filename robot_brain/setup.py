from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='epadmin',
    maintainer_email='epadmin@robot.local',
    description='Vision + LLM task planner using ZED camera and Ollama LLaVA',
    license='MIT',
    entry_points={
        'console_scripts': [
            'brain_node = robot_brain.brain_node:main',
            'brain_cli = robot_brain.cli_interface:main',
            'motion_executor = robot_brain.motion_executor:main',
        ],
    },
)
