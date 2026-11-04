from setuptools import setup

package_name = 'hw9pkg'

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
    description='HW9: find coloured traffic cones with the camera and lidar, then drive to them',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_finder = hw9pkg.cone_finder:main',
            'calibrate = hw9pkg.calibrate:main',
        ],
    },
)
