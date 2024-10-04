from setuptools import find_packages, setup

package_name = 'm3e1_service_fullname'

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
            'client_name = m3e1_service_fullname.client_name:main',
            'service_name = m3e1_service_fullname.service_name:main',
        ],
    },
)
