from setuptools import setup

package_name = 'lab2_pkg'

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
    maintainer='Arnau',
    maintainer_email='arnau@todo.com',
    description='Paquete para el Laboratorio 2 de ROB',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_scan = lab2_pkg.LIDAR_scan:main',
            'square_odom = lab2_pkg.SquareTrajectory_odom:main',
            'square_time = lab2_pkg.SquareTrajectory_time:main',
        ],
    },
)
