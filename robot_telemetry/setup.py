from setuptools import find_packages, setup

package_name = 'robot_telemetry'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/record_bag.sh']),
        ('share/' + package_name + '/launch',
            ['launch/telemetry_recorder.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Telemetry logging and offline data analysis for robot simulations',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'telemetry_recorder = robot_telemetry.telemetry_recorder:main',
            'analyze_telemetry = robot_telemetry.analyze_telemetry:main',
        ],
    },
)
