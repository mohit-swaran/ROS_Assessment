from setuptools import find_packages, setup
import glob 

package_name = 'cafe_robot_action_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf',glob.glob('robot_description/urdf/*')),
        ('share/' + package_name + '/launch',glob.glob('launch/*')),
        ('share/' + package_name + '/config',glob.glob('config/*')),
        ('share/' + package_name + '/worlds',glob.glob('worlds/*')),
        ('share/' + package_name + '/maps',glob.glob('maps/*')),
        
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mohit',
    maintainer_email='mohit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "robot_navigator=cafe_robot_action_planning.robot_navigator:main",
            "order_manager=cafe_robot_action_planning.order_manager:main",
        ],
    },
)
