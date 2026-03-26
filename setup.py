from setuptools import find_packages, setup

package_name = 'walter_bot'

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
    maintainer='Amine Frioua',
    maintainer_email='aminefrioua@gmail.com',
    description='Walter Robot ROS 2 Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = walter_bot.bridge_node:main',
        ],
    },
)
