from glob import glob

from setuptools import find_packages, setup

package_name = 'rc2027_utils'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/scripts', glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc2027',
    maintainer_email='wdsj2009.2006101@gmail.com',
    description='标定脚本、调试工具、mock 节点（无硬件依赖的辅助工具集）',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'l2_cycle_test = rc2027_utils.l2_cycle_test:main',
            'calibrate_lidar_base = rc2027_utils.calibration.calibrate_lidar_base:main',
            'teach_point = rc2027_utils.teach_point.teach_point_node:main',
            'ground_truth_check = rc2027_utils.sim_check.ground_truth_check_node:main',
            'sim_lidar_range_check = rc2027_utils.sim_check.sim_lidar_range_check_node:main',
            'sim_climb_replay = rc2027_utils.sim_check.sim_climb_replay_node:main',
            'sim_l2_hop = rc2027_utils.sim_check.sim_l2_hop_node:main',
            'sim_blocks = rc2027_utils.sim_check.sim_blocks_node:main',
            'sim_tower = rc2027_utils.sim_check.sim_tower_node:main',
        ],
    },
)
