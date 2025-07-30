import os
virtualenv_name = "bluecv"
home_path = os.path.expanduser("~")
#executable_path = os.path.join(home_path, 'venvs', virtualenv_name, 'bin', 'python')
executable_path = os.path.join(home_path, '.virtualenvs', virtualenv_name, 'bin', 'python')

from setuptools import find_packages, setup

package_name = 'bluerov2_testers'

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
            'basic_cv = bluerov2_testers.BasicStrategyCV:main',
            'basic_tag = bluerov2_testers.BasicStrategyTag:main',
            'basic_tag_no_tilt = bluerov2_testers.BasicStrategyTagNoTilt:main',
        ],
    },
    options={
        'build_scripts': {
            'executable': executable_path,
        }
    },
)
