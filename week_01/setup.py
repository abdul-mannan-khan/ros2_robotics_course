from setuptools import find_packages, setup

package_name = 'week_01_lidar_processing'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/week_01_lidar_processing/launch', ['w01_exercises/launch/exercise1.launch.py', 'w01_exercises/launch/exercise2.launch.py', 'w01_exercises/launch/exercise3.launch.py', 'w01_exercises/launch/demo.launch.py']),
        ('share/week_01_lidar_processing/config', ['w01_exercises/config/params.yaml', 'w01_exercises/config/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='Week 01 - LiDAR Point Cloud Processing',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
# Note: Week 01 solution scripts do not define a main() function.
            # They use if __name__ == "__main__" blocks directly.
            # Run them with: python3 -m week_01.lab_solutions.<script_name>
            'exercise1_node = w01_exercises.exercise1_node:main',
            'exercise2_node = w01_exercises.exercise2_node:main',
            'exercise3_node = w01_exercises.exercise3_node:main',
            'solution1_node = w01_solutions.solution1_node:main',
            'solution2_node = w01_solutions.solution2_node:main',
            'solution3_node = w01_solutions.solution3_node:main',
        
        ],
    },
)
