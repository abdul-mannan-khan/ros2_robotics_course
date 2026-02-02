from setuptools import setup, find_packages

package_name = 'week_12_capstone'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_12_capstone/launch', ['w12_exercises/launch/exercise1.launch.py', 'w12_exercises/launch/exercise2.launch.py', 'w12_exercises/launch/exercise3.launch.py', 'w12_exercises/launch/demo.launch.py']),
        ('share/week_12_capstone/config', ['w12_exercises/config/params.yaml', 'w12_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'scipy'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 12: Full System Integration & Capstone Project',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'capstone_sim = lab_exercises.capstone_sim:main',
            'task1_arch = lab_exercises.task1_system_architecture:main',
            'task2_fusion = lab_exercises.task2_sensor_fusion_integration:main',
            'task3_plan_ctrl = lab_exercises.task3_planning_control_integration:main',
            'task4_perception = lab_exercises.task4_perception_pipeline:main',
            'task5_mission = lab_exercises.task5_full_mission:main',
            'task6_stress = lab_exercises.task6_stress_testing:main',
            'task7_eval = lab_exercises.task7_evaluation:main',
            'sol1_arch = lab_solutions.task1_solution:main',
            'sol2_fusion = lab_solutions.task2_solution:main',
            'sol3_plan_ctrl = lab_solutions.task3_solution:main',
            'sol4_perception = lab_solutions.task4_solution:main',
            'sol5_mission = lab_solutions.task5_solution:main',
            'sol6_stress = lab_solutions.task6_solution:main',
            'sol7_eval = lab_solutions.task7_solution:main',
            'exercise1_node = w12_exercises.exercise1_node:main',
            'exercise2_node = w12_exercises.exercise2_node:main',
            'exercise3_node = w12_exercises.exercise3_node:main',
            'solution1_node = w12_solutions.solution1_node:main',
            'solution2_node = w12_solutions.solution2_node:main',
            'solution3_node = w12_solutions.solution3_node:main',
        
        ],
    },
)
