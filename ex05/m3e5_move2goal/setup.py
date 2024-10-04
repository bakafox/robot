from setuptools import find_packages, setup

package_name = 'm3e5_move2goal'

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
            'move_to_goal = m3e5_move2goal.move_to_goal:main',
        ],
    },
)
