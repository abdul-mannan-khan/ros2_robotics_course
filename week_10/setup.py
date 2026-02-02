from setuptools import setup, find_packages

package_name = 'week_10_computer_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_10_computer_vision/launch', ['w10_exercises/launch/exercise1.launch.py', 'w10_exercises/launch/exercise2.launch.py', 'w10_exercises/launch/exercise3.launch.py', 'w10_exercises/launch/demo.launch.py']),
        ('share/week_10_computer_vision/config', ['w10_exercises/config/params.yaml', 'w10_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools', 'numpy', 'matplotlib', 'scipy'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 10: Computer Vision for Drones',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
'generate_vision_data = lab_exercises.generate_vision_data:main',
            'task1_camera = lab_exercises.task1_camera_model:main',
            'task2_features = lab_exercises.task2_feature_detection:main',
            'task3_homography = lab_exercises.task3_homography:main',
            'task4_detection = lab_exercises.task4_object_detection:main',
            'task5_stereo = lab_exercises.task5_stereo_depth:main',
            'task6_flow = lab_exercises.task6_optical_flow:main',
            'task7_pipeline = lab_exercises.task7_full_vision:main',
            'sol1_camera = lab_solutions.task1_solution:main',
            'sol2_features = lab_solutions.task2_solution:main',
            'sol3_homography = lab_solutions.task3_solution:main',
            'sol4_detection = lab_solutions.task4_solution:main',
            'sol5_stereo = lab_solutions.task5_solution:main',
            'sol6_flow = lab_solutions.task6_solution:main',
            'sol7_pipeline = lab_solutions.task7_solution:main',
            'exercise1_node = w10_exercises.exercise1_node:main',
            'exercise2_node = w10_exercises.exercise2_node:main',
            'exercise3_node = w10_exercises.exercise3_node:main',
            'solution1_node = w10_solutions.solution1_node:main',
            'solution2_node = w10_solutions.solution2_node:main',
            'solution3_node = w10_solutions.solution3_node:main',
        
        ],
    },
)
