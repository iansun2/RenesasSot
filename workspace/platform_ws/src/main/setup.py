from setuptools import find_packages, setup
from glob import glob

package_name = 'main'

lib_files = glob(package_name+'/*.py')
# lib_files.remove(package_name+'/*.py')
lib_files.remove(package_name+'/__init__.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ["main/platform_audio.py", "main/platform_button.py"]),
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
            'main=main.main:main',
            'main_color=main.main_color:main',
            'main_arm=main.main_arm:main'
        ],
    },
)
