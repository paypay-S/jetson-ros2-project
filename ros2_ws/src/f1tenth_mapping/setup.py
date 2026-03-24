from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'f1tenth_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch ファイル
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # 設定ファイル
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='F1TENTH User',
    maintainer_email='user@example.com',
    description='F1TENTH 実機用マップ作成ツール',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
