from setuptools import find_packages, setup

package_name = 'bluerov2_arm_disarm'

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
            'arm_disarm = bluerov2_arm_disarm.arm_disarm:main',
            'auv_movement = bluerov2_arm_disarm.auv_movement:main',
            'dance_movement = bluerov2_arm_disarm.dance_movement:main',
            'test_diagonal = bluerov2_arm_disarm.test_diagonal_movement:main',
        ],
    },
)
