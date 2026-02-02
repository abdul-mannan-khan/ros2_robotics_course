from setuptools import setup, find_packages

package_name = 'week_05_nav2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_05_nav2/launch', ['w05_exercises/launch/week5.launch.py', 'w05_exercises/launch/demo.launch.py']),
        ('share/week_05_nav2/config', ['w05_exercises/config/params.yaml', 'w05_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 5: ROS2 Navigation Stack (Nav2)',
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
            'exercise1_node = w05_exercises.exercise1_node:main',
            'exercise2_node = w05_exercises.exercise2_node:main',
            'exercise3_node = w05_exercises.exercise3_node:main',
            'solution1_node = w05_solutions.solution1_node:main',
            'solution2_node = w05_solutions.solution2_node:main',
            'solution3_node = w05_solutions.solution3_node:main',
        
        ],
    },
)
