from setuptools import find_packages, setup

package_name = 'ros2_robotics_course'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dr. Abdul Manan Khan',
    maintainer_email='abdul.khan@example.com',
    description='ROS2 Robotics Course - Metapackage for all weekly modules',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [],
    },
)
