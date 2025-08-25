from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'movement_platform'

lib_files = glob(package_name+'/*.py')
# lib_files.remove(package_name+'/*.py')
lib_files.remove(package_name+'/__init__.py')

# print(f"debug iansun2 <{lib_files}>")

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*_launch.py')),
        ('lib/' + package_name, lib_files),
        # ('lib/' + package_name, [package_name+'/platform_audio.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros_dev',
    maintainer_email='iansun2004@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "platform=movement_platform.platform_controller:main",
            "command_sender=movement_platform.command_sender:main",
            "measure=movement_platform.measure:main"
        ],
    },
)
