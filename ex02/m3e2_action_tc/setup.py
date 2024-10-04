from setuptools import find_packages, setup

package_name = 'm3e2_action_tc' # action_turtle_commands

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
    maintainer='bakafox',
    maintainer_email='v.chekhovskii@g.nsu.ru',
    description='TODO: Package description',
    license='Unlicense',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_turtle_client = m3e2_action_tc.action_client:main',
            'action_turtle_server = m3e2_action_tc.action_server:main',
        ],
    },
)
