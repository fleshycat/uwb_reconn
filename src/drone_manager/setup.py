from setuptools import find_packages, setup

package_name = 'drone_manager'

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
    maintainer='user',
    maintainer_email='crummycat0217@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_manager = drone_manager.drone_manager:main',
            'motor_monitor = drone_manager.motor_monitor:main',
            'motor_publisher = drone_manager.motor_publisher:main',
            'target_monitor = drone_manager.target_monitor:main',
        ],
    },
)
