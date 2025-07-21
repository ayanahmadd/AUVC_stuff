from setuptools import find_packages, setup

package_name = 'bwsi_team'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/bwsi_team/launch', ['launch/example.launch.yaml']),
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
        'auv_movement = bwsi_team.auv_movement:main',
        'arm_disarm = bwsi_team.arm_disarm:main',
        'dance_movement = bwsi_team.dance_movement:main',
    ],
    },
)
