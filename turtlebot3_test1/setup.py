from setuptools import setup
import os
from glob import glob

package_name = 'turtlebot3_test1' 

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch-Dateien installieren
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Bilder-Verzeichnis installieren
        (os.path.join('share', package_name, 'images'), glob('share/images/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dein_name',
    maintainer_email='dein_email@example.com',
    description='Autorace Challenge Paket',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Hier werden deine Python-Skripte als ausführbare Nodes registriert
            'line_detector_node = turtlebot3_test1.line_detector_node:main',
            'line_follower_node = turtlebot3_test1.line_follower_node:main',
            'sign_detector_node = turtlebot3_test1.sign_detector_node:main',
        ],
    },
)