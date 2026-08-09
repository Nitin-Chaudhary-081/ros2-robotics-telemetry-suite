from setuptools import find_packages, setup

package_name = 'basic_robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            ['launch/obstacle_avoidance.launch.py']),
        ('share/' + package_name + '/scripts',
            ['scripts/pybullet_sim.py']),
        ('share/' + package_name + '/urdf',
            ['urdf/robot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Obstacle-avoidance robot demo: custom kinematics/LiDAR '
                'simulation and avoider node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_mover = basic_robot_sim.robot_mover:main',
            'obstacle_avoider = basic_robot_sim.obstacle_avoider:main'
        ],
    },
)
