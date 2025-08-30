from setuptools import find_packages, setup

package_name = 'gps_mqtt_bridge'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['gps_mqtt_node = gps_mqtt_bridge.gps_mqtt_node:main','hw_gps_node = gps_mqtt_bridge.hw_gps_node:main',

        ],
    },
)
