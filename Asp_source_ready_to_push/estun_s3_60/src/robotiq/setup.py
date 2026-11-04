from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotiq'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加这行来安装msg文件
        (os.path.join('share', package_name, 'msg'), glob('msg/*.msg')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fury',
    maintainer_email='fury@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotiq_2f_gripper_node = robotiq.Robotiq2FGripperRtuNode:main',
            'gripper_simple_controller = robotiq.Robotiq2FGripperSimpleController:main',
        ],
    },
)
