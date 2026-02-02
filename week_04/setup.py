from setuptools import find_packages, setup

package_name = 'week_04_slam_2d'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_04_slam_2d/launch', ['w04_exercises/launch/week4.launch.py', 'w04_exercises/launch/demo.launch.py']),
        ('share/week_04_slam_2d/config', ['w04_exercises/config/params.yaml', 'w04_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 04 - 2D SLAM with LiDAR',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'task1_occupancy_grid_solution = lab_solutions.task1_occupancy_grid_solution:main',
            'task2_icp_solution = lab_solutions.task2_icp_solution:main',
            'task3_motion_model_solution = lab_solutions.task3_motion_model_solution:main',
            'task4_scan_matching_localization_solution = lab_solutions.task4_scan_matching_localization_solution:main',
            'task5_occupancy_mapping_with_slam_solution = lab_solutions.task5_occupancy_mapping_with_slam_solution:main',
            'task6_loop_closure_solution = lab_solutions.task6_loop_closure_solution:main',
            'task7_full_slam_solution = lab_solutions.task7_full_slam_solution:main',
            'exercise1_node = w04_exercises.exercise1_node:main',
            'exercise2_node = w04_exercises.exercise2_node:main',
            'exercise3_node = w04_exercises.exercise3_node:main',
            'solution1_node = w04_solutions.solution1_node:main',
            'solution2_node = w04_solutions.solution2_node:main',
            'solution3_node = w04_solutions.solution3_node:main',
        
        ],
    },
)
