import os
from glob import glob

from setuptools import setup

package_name = 'arm_task_server'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'robots'), glob('config/robots/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='multirobot_sim',
    maintainer_email='noreply@example.com',
    description="The robot's manipulator_action_server: named arm poses through move_group, harvest through cut_stem.",
    license='Proprietary',
    entry_points={
        'console_scripts': [
            'arm_task_server = arm_task_server.server:main',
        ],
    },
)
