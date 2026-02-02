from setuptools import setup, find_packages

package_name = 'week_08_3d_path_planning'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_08_3d_path_planning/launch', ['w08_exercises/launch/exercise1.launch.py', 'w08_exercises/launch/exercise2.launch.py', 'w08_exercises/launch/exercise3.launch.py', 'w08_exercises/launch/demo.launch.py']),
        ('share/week_08_3d_path_planning/config', ['w08_exercises/config/params.yaml', 'w08_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'scipy'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 8: 3D Path Planning for Drones',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'generate_env = lab_exercises.generate_3d_env:generate_environment',
            'task1_3d_astar = lab_exercises.task1_3d_astar:main',
            'task2_rrt = lab_exercises.task2_rrt:main',
            'task3_rrt_star = lab_exercises.task3_rrt_star:main',
            'task4_trajectory = lab_exercises.task4_trajectory_optimization:main',
            'task5_replanning = lab_exercises.task5_dynamic_replanning:main',
            'task6_energy = lab_exercises.task6_energy_aware:main',
            'task7_pipeline = lab_exercises.task7_full_planning:main',
            'sol1_3d_astar = lab_solutions.task1_solution:main',
            'sol2_rrt = lab_solutions.task2_solution:main',
            'sol3_rrt_star = lab_solutions.task3_solution:main',
            'sol4_trajectory = lab_solutions.task4_solution:main',
            'sol5_replanning = lab_solutions.task5_solution:main',
            'sol6_energy = lab_solutions.task6_solution:main',
            'sol7_pipeline = lab_solutions.task7_solution:main',
            'exercise1_node = w08_exercises.exercise1_node:main',
            'exercise2_node = w08_exercises.exercise2_node:main',
            'exercise3_node = w08_exercises.exercise3_node:main',
            'solution1_node = w08_solutions.solution1_node:main',
            'solution2_node = w08_solutions.solution2_node:main',
            'solution3_node = w08_solutions.solution3_node:main',
        
        ],
    },
)
