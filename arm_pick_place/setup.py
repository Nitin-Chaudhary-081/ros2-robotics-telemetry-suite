from setuptools import find_packages, setup

package_name = 'arm_pick_place'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/pick_place_demo.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='3-DOF Robotic Arm Kinematics and Pick-and-Place Controller',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_kinematics = arm_pick_place.arm_kinematics:main',
            'pick_place_controller = arm_pick_place.pick_place_controller:main',
        ],
    },
)
