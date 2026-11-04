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
    description='HW9: find three coloured traffic cones and drive to the one the user names',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cone_hunter = hw9pkg.cone_hunter:main',
            'cone_calib = hw9pkg.cone_calib:main',
            'sim_test = hw9pkg.sim_test:main',
        ],
    },
)
