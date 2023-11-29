from setuptools import find_packages, setup

package_name = 'assignment_pkg'

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
    maintainer='dema',
    maintainer_email='5433737@studenti.unige.it',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'motor_control = assignment_pkg.motor_control:main',
                'robot_controller = assignment_pkg.robot_controller:main',
                'robot_revolute_node = assignment_pkg.robot_revolute_node:main',
        ],
    },
)
