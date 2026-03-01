from setuptools import setup

package_name = 'py01_topic'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    # 新增 EDS 文件安装
    ('share/' + package_name + '/eds', [
        'py01_topic/CANedsGoldV002.eds'
    ]),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fit',
    maintainer_email='fit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_command_publisher = py01_topic.motor_command_publisher:main',
            'motor_command_sin = py01_topic.motor_command_sin:main',
            'motor_controller_node = py01_topic.motor_controller_node:main',
            'motor_command_listner = py01_topic.motor_command_listner:main',
            'testsyncnode = py01_topic.testsyncnode:main',
            'testsyncnoderemote = py01_topic.testsyncnoderemote:main',
            'canopen_motor_node = py01_topic.canopen_motor_node:main'             
        ],
    },
)
