#!/usr/bin/env python

from setuptools import setup
import os
from glob import glob

package_name = 'vikit_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_dir={'': 'src'},  # 指定 Python 包的根目录
    data_files=[
        # 安装 package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 如果需要安装其他资源文件（如 launch 文件），可以在这里添加
        # (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools', 'pyyaml'],  # 更新依赖项
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='The vikit_py package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 在这里添加你的可执行脚本
            # 'your_script_name = vikit_py.your_module:main',
        ],
    },
)