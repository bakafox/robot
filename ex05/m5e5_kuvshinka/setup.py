from setuptools import find_packages, setup

package_name = 'm5e5_kuvshinka'

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
        ('share/' + package_name + '/src/description', [
            'src/description/kuvshinka.gazebo.xacro',
            'src/description/kuvshinka.urdf.xacro',
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
            'vosmerka = m5e5_kuvshinka.vosmerka:main',
        ],
    },
)
