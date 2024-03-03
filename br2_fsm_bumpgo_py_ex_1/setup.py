from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'br2_fsm_bumpgo_py_ex_1'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ether',
    maintainer_email='mdether0056@gmail.com',
    description='Chapter 3 Exercise 1',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bump_go_main_ex_1 = br2_fsm_bumpgo_py_ex_1.bump_go_main_ex_1:main'
        ],
    },
)
