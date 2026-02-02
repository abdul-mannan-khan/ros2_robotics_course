from setuptools import setup, find_packages

package_name = 'week_09_ego_planner'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_09_ego_planner/launch', ['w09_exercises/launch/exercise1.launch.py', 'w09_exercises/launch/exercise2.launch.py', 'w09_exercises/launch/exercise3.launch.py', 'w09_exercises/launch/demo.launch.py']),
        ('share/week_09_ego_planner/config', ['w09_exercises/config/params.yaml', 'w09_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'scipy'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 9: EGO-Planner and Trajectory Optimization',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'task1_bspline = lab_exercises.task1_bspline_basics:main',
            'task2_trajectory = lab_exercises.task2_trajectory_representation:main',
            'task3_smoothness = lab_exercises.task3_smoothness_cost:main',
            'task4_collision = lab_exercises.task4_collision_avoidance:main',
            'task5_ego = lab_exercises.task5_full_ego_planner:main',
            'task6_multi = lab_exercises.task6_multi_drone:main',
            'task7_integration = lab_exercises.task7_full_integration:main',
            'sol1_bspline = lab_solutions.task1_solution:main',
            'sol2_trajectory = lab_solutions.task2_solution:main',
            'sol3_smoothness = lab_solutions.task3_solution:main',
            'sol4_collision = lab_solutions.task4_solution:main',
            'sol5_ego = lab_solutions.task5_solution:main',
            'sol6_multi = lab_solutions.task6_solution:main',
            'sol7_integration = lab_solutions.task7_solution:main',
            'exercise1_node = w09_exercises.exercise1_node:main',
            'exercise2_node = w09_exercises.exercise2_node:main',
            'exercise3_node = w09_exercises.exercise3_node:main',
            'solution1_node = w09_solutions.solution1_node:main',
            'solution2_node = w09_solutions.solution2_node:main',
            'solution3_node = w09_solutions.solution3_node:main',
        
        ],
    },
)
