from setuptools import setup

package_name = 'tello_interaction'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='julian.cayero',
    maintainer_email='julian.cayero@eurecat.org',
    description='Simple node that interacts with the Ryze tello drone though the tello_ros package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'interaction_client = tello_interaction.interaction:main'
        ],
    },
)
