from setuptools import find_packages, setup

package_name = 'day2pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chichi',
    maintainer_email='chichizhang20000510@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'node1 = day2pkg.node1:main',
            'node2 = day2pkg.node2:main',
            'node3 = day2pkg.node3:main',
            'node4 = day2pkg.node4:main',
            'node5 = day2pkg.node5:main'
        ],
    },
)
