from setuptools import setup
import os
from glob import glob

package_name = 'tb3_multi_goal'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wintunlatt',
    maintainer_email='you@example.com',
    description='Multi-goal navigation demo for TurtleBot3',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'multi_goal_nav = tb3_multi_goal.multi_goal_nav:main',
            'multi_goal_from_yaml = tb3_multi_goal.multi_goal_from_yaml:main',
            'multi_goal_from_click = tb3_multi_goal.multi_goal_from_click:main',
            'multi_goal_from_click_execute = tb3_multi_goal.multi_goal_from_click_execute:main',
        ],
    },
)
