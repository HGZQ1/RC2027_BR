"""示教录点的纯逻辑部分：多帧位姿求平均、读写录好的点文件。不依赖 ROS。

华工去年的 calc_mean_position.py 这部分数学是对的（角度不能直接算
算术平均——比如 359° 和 1° 平均不是 180°，而应该是 0°），原样照搬这个
思路，没有重新发明。ROS 节点薄壳在 teach_point_node.py，只做"连 TF/攒
样本/按键触发"这些 ROS 相关的事，这个模块只管数学和文件格式。
"""

import math
import os

import yaml


def yaw_from_quaternion(x, y, z, w):
    """四元数 -> 绕 z 轴的 yaw（弧度，范围 [-pi, pi]）。

    不用 tf_transformations.euler_from_quaternion：镜像里 apt 装的
    tf_transformations 依赖 python3-transforms3d 0.3.1，它用了 numpy 1.24
    已经删掉的 np.float，跟我们锁的 numpy 1.26.4 不兼容，import 就报错
    （2026-10-03 助教实测，见 0002）。录点只需要 yaw，一行公式就够，不值得为它
    引入依赖。结果跟 euler_from_quaternion(..., 'sxyz') 的第三个分量一致。
    """
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def circular_mean_pose(samples):
    """多帧 (x, y, theta) 采样求平均，theta 用圆均值（不能直接算术平均）。

    Args:
        samples: [(x, y, theta), ...]，至少 1 帧。

    Returns:
        (mean_x, mean_y, mean_theta)。
    """
    if not samples:
        raise ValueError("samples 不能是空的，没有数据求不出平均值")

    xs = [s[0] for s in samples]
    ys = [s[1] for s in samples]
    thetas = [s[2] for s in samples]

    mean_x = sum(xs) / len(xs)
    mean_y = sum(ys) / len(ys)

    # 圆均值：把每个角度看成单位圆上的向量 (cosθ, sinθ) 再平均，
    # 用 atan2 换回角度。直接对 θ 算术平均在跨 ±π 边界时是错的
    # （比如 179° 和 -179° 的算术平均是 0°，实际应该是 180°/-180°）
    mean_cos = sum(math.cos(t) for t in thetas) / len(thetas)
    mean_sin = sum(math.sin(t) for t in thetas) / len(thetas)
    mean_theta = math.atan2(mean_sin, mean_cos)

    return mean_x, mean_y, mean_theta


def load_points(path):
    """读取已经录好的点文件，文件不存在时返回空字典（还没录过点）。"""
    try:
        with open(path, encoding='utf-8') as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        return {}
    return data or {}


def save_point(path, name, pose):
    """把一个点写进文件（新增或覆盖同名点），保留文件里其它已有的点。

    Args:
        path: 点文件路径。
        name: 点的名字，比如 'l1_corner_red'。
        pose: (x, y, theta)。
    """
    points = load_points(path)
    x, y, theta = pose
    points[name] = {'x': round(x, 4), 'y': round(y, 4), 'theta': round(theta, 4)}
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as f:
        yaml.safe_dump(points, f, allow_unicode=True, sort_keys=True)
    return points
