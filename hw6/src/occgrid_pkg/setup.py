from setuptools import setup

package_name = 'occgrid_pkg'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='team',
    maintainer_email='team@example.com',
    description='Occupancy grid mapping from Turtlebot LIDAR',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapping_node = occgrid_pkg.mapping_node:main',
            'offline_test = occgrid_pkg.offline_test:main',
        ],
    },
)
