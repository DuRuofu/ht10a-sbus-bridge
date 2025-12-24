from setuptools import find_packages, setup

package_name = 'rc_control_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='DuRuofu',
    maintainer_email='you@example.com',
    description='RC input to /cmd_vel bridge node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rc_control_node = rc_control_node.rc_control_node:main',
        ],
    },
)