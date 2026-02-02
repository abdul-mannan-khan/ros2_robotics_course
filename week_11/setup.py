from setuptools import setup, find_packages

package_name = 'week_11_control_systems'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_11_control_systems/launch', ['w11_exercises/launch/exercise1.launch.py', 'w11_exercises/launch/exercise2.launch.py', 'w11_exercises/launch/exercise3.launch.py', 'w11_exercises/launch/demo.launch.py']),
        ('share/week_11_control_systems/config', ['w11_exercises/config/params.yaml', 'w11_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'scipy'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 11: Control Systems - PID to MPC',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'control_sim = lab_exercises.control_sim:main',
            'task1_pid = lab_exercises.task1_pid_analysis:main',
            'task2_state_space = lab_exercises.task2_state_space:main',
            'task3_pole_placement = lab_exercises.task3_pole_placement:main',
            'task4_lqr = lab_exercises.task4_lqr:main',
            'task5_mpc_basics = lab_exercises.task5_mpc_basics:main',
            'task6_mpc_drone = lab_exercises.task6_mpc_drone:main',
            'task7_comparison = lab_exercises.task7_full_comparison:main',
            'sol1_pid = lab_solutions.task1_solution:main',
            'sol2_state_space = lab_solutions.task2_solution:main',
            'sol3_pole_placement = lab_solutions.task3_solution:main',
            'sol4_lqr = lab_solutions.task4_solution:main',
            'sol5_mpc_basics = lab_solutions.task5_solution:main',
            'sol6_mpc_drone = lab_solutions.task6_solution:main',
            'sol7_comparison = lab_solutions.task7_solution:main',
            'exercise1_node = w11_exercises.exercise1_node:main',
            'exercise2_node = w11_exercises.exercise2_node:main',
            'exercise3_node = w11_exercises.exercise3_node:main',
            'solution1_node = w11_solutions.solution1_node:main',
            'solution2_node = w11_solutions.solution2_node:main',
            'solution3_node = w11_solutions.solution3_node:main',
        
        ],
    },
)
