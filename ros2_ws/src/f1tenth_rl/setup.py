from setuptools import setup
import os
from glob import glob

package_name = 'f1tenth_rl'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'stable_baselines3'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='RL driver for F1TENTH',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_driver = f1tenth_rl.rl_driver:main',
            'hardware_bridge = f1tenth_rl.hardware_bridge:main',
        ],
    },
)
