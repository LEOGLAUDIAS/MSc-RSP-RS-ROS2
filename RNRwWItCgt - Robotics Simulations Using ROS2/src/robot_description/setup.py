import os
from glob import glob
from setuptools import setup

package_name = 'robot_description'

setup(
    name=package_name,
    version='0.0.1',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install urdf files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Install world files (use one consistent folder: worlds)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='student@example.com',
    description='URDF and world description for two_wheeler_robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
