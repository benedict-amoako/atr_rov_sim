from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'atr_rov_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benedict',
    maintainer_email='benedict.amoako@fortressaisolutions.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            # 'setpoint_manager = atr_rov_control.setpoint_manager:main',
            # 'cascaded_pid = atr_rov_control.cascaded_pid:main',
            # 'wls_allocator = atr_rov_control.wls_allocator:main',
        ],
    },
)
