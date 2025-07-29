import os
virtualenv_name = "bluecv"
home_path = os.path.expanduser("~")
executable_path = os.path.join(home_path, '.virtualenvs', virtualenv_name, 'bin', 'python')

from setuptools import find_packages, setup

package_name = 'bluerov2_bluecv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','dt_apriltags'],
    zip_safe=True,
    maintainer='zguo27',
    maintainer_email='zguo011235@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'virtualenvs = bluecv.virtualenvs:main',
            'april_tag = bluerov2_bluecv.ReadAprilTags:main',
            'tag_follow = bluerov2_bluecv.TagFollower:main',
            'tag_publish = bluerov2_bluecv.TagPublisher:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': executable_path,
        }
    },
)
