from setuptools import find_packages, setup
import os

package_name = 'aruco_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
       (os.path.join('share', package_name, 'launch'), [
            'launch/aruco_main.launch.py',
            'launch/aruco_tracker.launch.py'
        ]),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bitterbyte',
    maintainer_email='giabinhvungtau48@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detect_aruco = aruco_tracker.detect_aruco:main',
            'follow_aruco = aruco_tracker.follow_aruco:main',
            'debug_aruco = aruco_tracker.debug_aruco:main',
            'debug = aruco_tracker.debug:main',
        ],
    },

)
