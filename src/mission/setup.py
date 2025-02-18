from setuptools import find_packages, setup

package_name = 'mission'

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
            'start_mission = mission.start_mission:main',
            'start_mission_hover = mission.start_mission_hover:main',
            'start_mission_corner = mission.start_mission_corner:main',
            'start_mission_straight = mission.start_mission_straight:main',
            'start_mission_circle = mission.start_mission_circle:main',
            'uwb_reconnaissance = mission.uwb_reconnaissance:main',
        ],
    },
)
