from setuptools import setup, find_packages

package_name = 'week_06_quadrotor_dynamics'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_06_quadrotor_dynamics/launch', ['w06_exercises/launch/week6.launch.py', 'w06_exercises/launch/demo.launch.py']),
        ('share/week_06_quadrotor_dynamics/config', ['w06_exercises/config/params.yaml', 'w06_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 6: Quadrotor Dynamics and Attitude Control',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'task1_solution = lab_solutions.task1_solution:main',
            'task2_solution = lab_solutions.task2_solution:main',
            'task3_solution = lab_solutions.task3_solution:main',
            'task4_solution = lab_solutions.task4_solution:main',
            'task5_solution = lab_solutions.task5_solution:main',
            'task6_solution = lab_solutions.task6_solution:main',
            'task7_solution = lab_solutions.task7_solution:main',
            'exercise1_node = w06_exercises.exercise1_node:main',
            'exercise2_node = w06_exercises.exercise2_node:main',
            'exercise3_node = w06_exercises.exercise3_node:main',
            'solution1_node = w06_solutions.solution1_node:main',
            'solution2_node = w06_solutions.solution2_node:main',
            'solution3_node = w06_solutions.solution3_node:main',
        
        ],
    },
)
