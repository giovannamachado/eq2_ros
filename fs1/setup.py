from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'fs1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'), glob(os.path.join('launch','*launch.[pxy][yma]'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='phavm@softex.cin.ufpe.br',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'my_node = fs1.my_node:main',
            'hand_node = fs1.hand_node:main',
            'test_node = fs1.test_node:main',
            'gripper_client = fs1.gripper_client:main',
            'gripper_control = fs1.gripper_control:main',
            'joints_control = fs1.joints_control:main',
            'kinova_api = fs1.kinova_api:main',
            'servo_adapter = fs1.servo_adapter:main',
            'keyboard_teleop = fs1.keyboard_teleop:main'
        ],
    },
)