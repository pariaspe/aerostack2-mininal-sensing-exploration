"""setup.py."""

from glob import glob
import os

from setuptools import setup

package_name = 'as2_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch
        (os.path.join('share', package_name, 'launch'), glob(
            os.path.join('launch', '*launch.[pxy][yma]*'))),
        # Config
        (os.path.join('share', package_name, 'config'),
         ['config/' + 'as2_default.rviz']),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='CVAR-UPM',
    maintainer_email='cvar.upm3@gmail.com',
    description='Aerostack2 Visualization Tools',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'marker_publisher = as2_visualization.marker_publisher:main',
        ],
    },
)
