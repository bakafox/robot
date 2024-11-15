from setuptools import find_packages, setup

package_name = 'm6e1_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [
            'resource/' + package_name,
        ]),
        ('share/' + package_name, [
            'package.xml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/start.launch.py',
        ]),
        ('share/' + package_name + '/ros_gz', [
            'ros_gz/bridge_config.yaml',
        ]),
        ('share/' + package_name + '/rviz', [
            'rviz/urdf_config.rviz',
        ]),
        ('share/' + package_name + '/description', [
            'description/robot.gazebo.xacro',
            'description/robot.urdf.xacro',
            'description/lidar.urdf.xacro',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bakafox',
    maintainer_email='v.chekhovskii@g.nsu.ru',
    description='TODO: Package description',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
