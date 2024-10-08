from setuptools import find_packages, setup

package_name = 'turtlebot_basic_movements'

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
    maintainer='root',
    maintainer_email='tin.nguyen5499@gmail.com',
    description='TODO: Package description',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'square = turtlebot_basic_movements.square:main',
            'turn_left = turtlebot_basic_movements.turn_left:main',
            'turn_right = turtlebot_basic_movements.turn_right:main',
            'go_straight = turtlebot_basic_movements.go_straight:main',
            'stop = turtlebot_basic_movements.stop:main',
            'current_states = turtlebot_basic_movements.current_states:main',
            'square_using_yaw = turtlebot_basic_movements.square_using_yaw:main',
        ],
    },
)
