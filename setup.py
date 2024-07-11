from setuptools import setup
import os
from glob import glob

package_name = 'ros2_rpi_as5048a'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[
        'ros2_rpi_as5048a.encoder_node',
    ],
    install_requires=[
        'setuptools',
        'spidev', 
    ],
    zip_safe=True,
    maintainer='Atar Babgei',
    maintainer_email='atarbabgei@gmail.com',
    description='ROS 2 package for AS5048A encoder',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'encoder_node = ros2_rpi_as5048a.encoder_node:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
)
