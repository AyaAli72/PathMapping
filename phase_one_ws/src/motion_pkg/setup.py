import glob
from setuptools import setup

package_name = 'motion_pkg'
launch_files = glob.glob('launch/*')

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mido',
    maintainer_email='mido@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speed_control = motion_pkg.speed_control:main',  # Added missing comma
            'points_sender = motion_pkg.points_sender:main',  # Added missing comma
            'random_points_generator= motion_pkg.random_points_generator:main',  # Added missing comma
            'current_position= motion_pkg.current_position:main',  # Added missing comma
            'mark_target= motion_pkg.mark_target:main',  # Added missing comma
            'lidar_sensor= motion_pkg.lidar_sensor:main',  # Added missing comma
            'path_planning_mariam_version = motion_pkg.path_planning_mariam_version:main',
            'path_planning_1 = motion_pkg.path_planning_1:main',
        ],
    },
)
