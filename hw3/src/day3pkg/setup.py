from glob import glob
from setuptools import find_packages, setup


package_name = 'day3pkg'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chichi',
    maintainer_email='chichizhang20000510@gmail.com',
    description='Project 3 Action client and server nodes.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'executive_node = day3pkg.executive_node:main',
            'driving_node = day3pkg.driving_node:main',
        ],
    },
)
