from setuptools import find_packages, setup

package_name = 'week_03_ros2_fundamentals'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_03_ros2_fundamentals/launch', ['w03_exercises/launch/exercise1.launch.py', 'w03_exercises/launch/exercise2.launch.py', 'w03_exercises/launch/exercise3.launch.py', 'w03_exercises/launch/demo.launch.py']),
        ('share/week_03_ros2_fundamentals/config', ['w03_exercises/config/params.yaml', 'w03_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 03 - ROS2 Fundamentals and Node Architecture',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'task1_publisher_solution = lab_solutions.task1_publisher_solution:main',
            'task2_subscriber_solution = lab_solutions.task2_subscriber_solution:main',
            'task3_pub_sub_system_solution = lab_solutions.task3_pub_sub_system_solution:main',
            'task4_service_solution = lab_solutions.task4_service_solution:main',
            'task5_parameters_solution = lab_solutions.task5_parameters_solution:main',
            'task6_launch_system_solution = lab_solutions.task6_launch_system_solution:main',
            'task7_integration_solution = lab_solutions.task7_integration_solution:main',
            'exercise1_node = w03_exercises.exercise1_node:main',
            'exercise2_node = w03_exercises.exercise2_node:main',
            'exercise3_node = w03_exercises.exercise3_node:main',
            'solution1_node = w03_solutions.solution1_node:main',
            'solution2_node = w03_solutions.solution2_node:main',
            'solution3_node = w03_solutions.solution3_node:main',
        
        ],
    },
)
