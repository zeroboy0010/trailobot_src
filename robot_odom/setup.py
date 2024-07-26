from setuptools import find_packages, setup

package_name = 'robot_odom'

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
    maintainer='zero',
    maintainer_email='you@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'odom_node = robot_odom.odom_node:main',
            # 'odom_vel_node = robot_odom.odom_node_velocity:main',
            'odom_pub_node = robot_odom.odom_sub:main',
            'odom_modify = robot_odom.odom_modify:main',
        ],
    },
)
