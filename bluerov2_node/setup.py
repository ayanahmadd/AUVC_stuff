from setuptools import find_packages, setup

package_name = 'bluerov2_node'

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
            'arm_disarm = bluerov2_node.ArmDisarm:main',
            'convert_to_depth = bluerov2_node.PressureToDepth:main',
            'lane_scan = bluerov2_node.LaneScan:main',
        ],
    },
)
