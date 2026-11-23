"""上 L2 的仿真落点。纯函数，不依赖 ROS。

爬 L2 的机构还没有图纸。仿真底盘也上不了楼梯（和 L1 那次一样，见 climb_replay.py）。
现在先把车直接放到 L2 台面上，用来试定位和后面的流程。图纸来了再换成沿楼梯的轨迹。

落在红侧刚跨上 L2 台面的地方。台沿在 |x| = 1.5，车半长约 0.15 m，
中心放在 x = −1.32，整车在台面上、贴着西沿，不浮在楼梯上。
中央柱在 (0, 0)、半径约 0.14 m，这个点离它 1.3 m。朝 +x，脸朝台面里面。
蓝方按场地镜像：x 取反，朝向 π − yaw。
"""

import math

# z 不写在这里：由场地网格现查台面。
L2_RED = (-1.32, 0.00, 0.0)
CLEARANCE = 0.01
L2_SPOTS = ((-1.25, 1.25), (1.25, 1.25), (-1.25, -1.25), (1.25, -1.25))


def hop_target(side='red'):
    """(x, y, yaw)。红方刚跨上西侧台沿、朝东；蓝方镜像。"""
    x, y, yaw = L2_RED
    if side == 'blue':
        x = -x
        yaw = math.atan2(math.sin(math.pi - yaw), math.cos(math.pi - yaw))
    return x, y, yaw


def footprint_z(surface_z, clearance=CLEARANCE):
    """base_footprint 的 z：贴着台面，高出 clearance，和爬楼梯回放一样。"""
    return float(surface_z) + clearance


def just_on_platform(x, y):
    """车心刚跨过台沿（|x| = 1.5），整车在台面上，还没靠近中央柱。"""
    return 1.20 <= abs(x) <= 1.38 and abs(y) <= 0.3
