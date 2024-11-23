from setuptools import find_packages, setup

package_name = 'arduino_uno_serial'

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
    maintainer='Jacob Cohen',
    maintainer_email='thejacobcohen@gmail.com',
    description='serial connection for Aruino Uno as microROS not compatible with Arduino Uno',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arduino_uno_serialized = arduino_uno_serial.arduino_uno_serialized:main'
        ],
    },
)
