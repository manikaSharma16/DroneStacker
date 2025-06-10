from setuptools import setup

package_name = 'subscriber_serial_arduino'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Manika Sharma',
    maintainer_email='manika.sharma10000@gmail.com',
    description='ROS 2 subscriber node that sends serial commands to Arduino',
    license='MIT',
    entry_points={
        'console_scripts': [
            'serial_node = subscriber_serial_arduino.serial_node:main',
        ],
    },
)

