from setuptools import setup

package_name = 'drivetrain'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='odroid',
    maintainer_email='odroid@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drivetrain = drivetrain.drivetrain:main',
            'safestop = drivetrain.safestop:main',
            'audioreceiver = drivetrain.audioin:main',
            'imureader = drivetrain.imureader:main',
            'camera_odom = drivetrain.camera_odom:main',
        ],
    },
)
