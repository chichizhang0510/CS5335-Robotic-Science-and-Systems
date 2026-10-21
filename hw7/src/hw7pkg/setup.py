from setuptools import setup

package_name = 'hw7pkg'

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
    description='HW7: two-way occupancy grids, landmark matching and A* planning',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mapping_node = hw7pkg.mapping_node:main',
            'landmarks = hw7pkg.landmarks:main',
            'plan_path = hw7pkg.plan_path:main',
            'simulate = hw7pkg.simulate:main',
        ],
    },
)
