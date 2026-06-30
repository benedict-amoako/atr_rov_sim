from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'atr_rov_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benedict',
    maintainer_email='benedict.amoako@fortressaisolutions.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_driver = atr_rov_navigation.depth_driver:main',
            'dvl_driver = atr_rov_navigation.dvl_driver:main',
            'imu_republisher = atr_rov_navigation.imu_republisher:main',
            'sonar_republisher = atr_rov_navigation.sonar_republisher:main',
        ],
    },
)
