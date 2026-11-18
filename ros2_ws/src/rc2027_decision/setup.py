from glob import glob

from setuptools import find_packages, setup

package_name = 'rc2027_decision'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config/flows', glob('config/flows/*.yaml')),
        ('share/' + package_name + '/config', ['config/spot_approaches.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc2027',
    maintainer_email='wdsj2009.2006101@gmail.com',
    description='决策层：Flow 引擎（学华工压栈流程）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'flow_runner = rc2027_decision.flow_runner_node:main',
            'field_state = rc2027_decision.field_state_node:main',
            'plan_random = rc2027_decision.plan_random:main',
            'mock_action_executor = rc2027_decision.mock_action_executor_node:main',
        ],
    },
)
