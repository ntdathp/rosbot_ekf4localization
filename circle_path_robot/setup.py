from setuptools import find_packages, setup

package_name = 'circle_path_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dat',
    maintainer_email='manchesterizredd@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'circle_path_node = circle_path_robot.circle_path_node:main',
            'square_path_node = circle_path_robot.square_path_node:main',
            'odometry_to_path_node = circle_path_robot.odometry_to_path:main',
            'circle_path_closed_loop_node = circle_path_robot.circle_path_closed_loop_node:main',
            'minimal_publisher = circle_path_robot.minimal_publisher:main'

        ],
    },
)
