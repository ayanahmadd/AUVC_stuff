from setuptools import find_packages, setup

package_name = 'bluerov2_controllers'

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
    maintainer='blue',
    maintainer_email='ayan.k.ahmaad01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pid_depth = bluerov2_controllers.PID_depth:main',
            'pid_heading = bluerov2_controllers.PID_heading:main',
            'pid_controller = bluerov2_controllers.PIDController:main',
            'pid_lane = bluerov2_controllers.PID_lane_following:main',
            'depth_publisher = bluerov2_node.DepthPublisher:main',
        ],
    },
)
