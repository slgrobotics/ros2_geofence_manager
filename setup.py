from setuptools import find_packages, setup

package_name = 'geofence_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/geofence.launch.py']),
        ('share/' + package_name + '/config', ['config/geofence.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergei Grichine',
    maintainer_email='slg@quakemap.com',
    description='Geofence manager: geometry, validation, and status for outdoor robots',
    license='Apache License 2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'geofence_manager_node = geofence_manager.geofence_manager_node:main',
        ],
    },
)