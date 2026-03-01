from setuptools import setup

package_name = 'unitree_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # package_data={package_name: ['lib/*.so']},
    # include_package_data=True,
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name + '/lib', [package_name + '/lib/libunitreeMotorSDK_Arm64.so']),
        ('lib/' + package_name + '/lib', [package_name + '/lib/libunitreeMotorSDK_Linux64.so']),
        ('lib/' + package_name + '/lib', [package_name + '/lib/unitree_actuator_sdk.cpython-38-aarch64-linux-gnu.so']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unitree_motor_node = unitree_control.motor_bridge_float64:main',
        ],
    },
)
