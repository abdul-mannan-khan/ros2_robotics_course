from setuptools import find_packages, setup

package_name = 'week_02_sensor_fusion'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_02_sensor_fusion/launch', ['w02_exercises/launch/exercise1.launch.py', 'w02_exercises/launch/exercise2.launch.py', 'w02_exercises/launch/exercise3.launch.py', 'w02_exercises/launch/demo.launch.py']),
        ('share/week_02_sensor_fusion/config', ['w02_exercises/config/params.yaml', 'w02_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 02 - Sensor Fusion with Extended Kalman Filter',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'task1_prediction_solution = lab_solutions.task1_prediction_solution:main',
            'task2_encoder_update_solution = lab_solutions.task2_encoder_update_solution:main',
            'task3_gps_update_solution = lab_solutions.task3_gps_update_solution:main',
            'task4_jacobians_solution = lab_solutions.task4_jacobians_solution:main',
            'task5_covariance_tuning_solution = lab_solutions.task5_covariance_tuning_solution:main',
            'task6_full_ekf_solution = lab_solutions.task6_full_ekf_solution:main',
            'task7_evaluation_solution = lab_solutions.task7_evaluation_solution:main',
            'exercise1_node = w02_exercises.exercise1_node:main',
            'exercise2_node = w02_exercises.exercise2_node:main',
            'exercise3_node = w02_exercises.exercise3_node:main',
            'solution1_node = w02_solutions.solution1_node:main',
            'solution2_node = w02_solutions.solution2_node:main',
            'solution3_node = w02_solutions.solution3_node:main',
        
        ],
    },
)
