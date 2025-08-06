import os
from glob import glob
from setuptools import setup

package_name = 'dummy_publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shin-kyoto',
    maintainer_email='58775300+Shin-kyoto@users.noreply.github.com',
    description='AWSIM dummy topic publisher',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'awsim_topic_pub = dummy_publisher.awsim_topic_pub:main',
        ],
    },
)
