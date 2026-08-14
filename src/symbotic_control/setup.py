import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'symbotic_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Symbotic Project',
    maintainer_email='maintainer@example.com',
    description='Keyboard, autonomous and validation nodes for the Symbotic robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'command_arbiter = symbotic_control.command_arbiter:main',
            'go_to_goal_server = symbotic_control.go_to_goal_server:main',
            'keyboard_teleop = symbotic_control.keyboard_teleop:main',
            'velocity_limiter = symbotic_control.velocity_limiter:main',
        ],
    },
)
