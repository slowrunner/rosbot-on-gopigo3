from setuptools import setup

package_name = 'ros2_gopigo3_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        # (os.path.join('share', package_name), glob('launch/*.py)),
        # Include all msg files
        # (os.path.join('share', package_name), glob('msg/*)),
        # Include all service files
        # (os.path.join('share', package_name), glob('srv/*)),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='slowrunner',
    maintainer_email='slowrunner@users.noreply.github.com',
    description='ROS2 Version of https://github.com/ros-gopigo/gopigo3_node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'distance_sensor = ros2_gopigo3_node.distance_sensor:main',

        ],
    },
)
