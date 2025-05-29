import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'drims2_motion_server'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools',
                      'drims2_msgs'],
    zip_safe=True,
    maintainer='kalman',
    maintainer_email='samuele.sandrini@polito.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'motion_server = {package_name}.motion_server:main',
            f'motion_client_test = examples.motion_client_test:main',
        ],
    },
)
