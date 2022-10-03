from setuptools import setup
import os
from glob import glob

package_name = 'tello_imu_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julen.cayero',
    maintainer_email='julen.cayero@upc.edu',
    description='Simple package to show how node parameters and launch files work',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'read_imu = tello_imu_control.read_imu:main'
        ],
    },
)
