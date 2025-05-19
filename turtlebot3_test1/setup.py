from setuptools import find_packages, setup

package_name = 'turtlebot3_test1'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "my_first_node = turtlebot3_test1.my_first_node:main",
            "line_detector = turtlebot3_test1.line_detector:main",
            "line_follower = turtlebot3_test1.line_follower:main",
            "sign_detection = turtlebot3_test1.sign_detection:main",
            "trafficlight_detection = turtlebot3_test1.trafficlight_detection:main",
        ],
    },
)
