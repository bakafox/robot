from setuptools import find_packages, setup

package_name = 'm4e2_carrot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'carrot.rviz']),
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
            'broadcaster = m4e2_carrot.broadcaster:main',
            'listener = m4e2_carrot.listener:main',
            'carrot = m4e2_carrot.carrot:main',
        ],
    },
)
