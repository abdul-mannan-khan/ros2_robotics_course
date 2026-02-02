from setuptools import setup, find_packages

package_name = 'week_07_px4_integration'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_07_px4_integration/launch', ['w07_exercises/launch/exercise1.launch.py', 'w07_exercises/launch/exercise2.launch.py', 'w07_exercises/launch/exercise3.launch.py', 'w07_exercises/launch/demo.launch.py']),
        ('share/week_07_px4_integration/config', ['w07_exercises/config/params.yaml', 'w07_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 7: PX4/ArduPilot and ROS2 Integration',
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
            'exercise1_node = w07_exercises.exercise1_node:main',
            'exercise2_node = w07_exercises.exercise2_node:main',
            'exercise3_node = w07_exercises.exercise3_node:main',
            'solution1_node = w07_solutions.solution1_node:main',
            'solution2_node = w07_solutions.solution2_node:main',
            'solution3_node = w07_solutions.solution3_node:main',
        
        ],
    },
)
