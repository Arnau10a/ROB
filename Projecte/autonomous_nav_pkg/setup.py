from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autonomous_nav_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arnau',
    maintainer_email='arnau@todo.todo',
    description='Autonomous Navigation project fulfilling Phase I, II, and III',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = autonomous_nav_pkg.mission_controller:main'
        ],
    },
)
