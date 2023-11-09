from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robile_safety_features'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # ... Other data files
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='shinas',
    maintainer_email='shinasshaji12@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "<EXECUTABLE_NAME = <PACKAGE_NAME>.<MODULE_NAME>:<METHOD>",
            'my_node = robile_safety_features.my_node:main',
            # 'safety_monitoring_BT = robile_safety_features.safety_monitoring_BT:main'
            # 'velcmd = robile_safety_features.VelCommand:main'
            'scanner = robile_safety_features.LaserScanner:main',
            # 'battmon = robile_safety_features.BatteryMonitor:main'
        ],
    },
)
