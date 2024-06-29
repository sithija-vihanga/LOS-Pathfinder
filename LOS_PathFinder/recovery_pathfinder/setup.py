from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'recovery_pathfinder'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    package_data={
        '': ['include/*'],  # Include all files within the 'include' directory
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sithija',
    maintainer_email='sithija.vihanga28@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pulse_monitor = recovery_pathfinder.pulseMonitor:main',
        ],
    },
)
