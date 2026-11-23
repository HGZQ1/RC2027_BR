"""雷达到底盘旋转中心的外参标定——纯数学部分，不依赖 ROS。

标定方法（比华工去年 calibrate_chassis_center.py 的 2D 圆拟合更通用）：

把机器人绕一个固定在车身上的点（比如卡住一个脚轮、原地旋转）转起来，
雷达定位算法（Point-LIO）输出的是雷达在世界系下的位姿 (p_i, R_i)。
旋转中心这个点在雷达自身坐标系下的位置 b 是不变的（刚体假设），
而它在世界系下的位置 P 也是不变的（物理上压根没动）。于是对每一帧 i：

    P = p_i + R_i @ b

这是关于 (b, P) 的线性方程组（6 个未知数，每帧提供 3 个方程），
旋转越多样（不只是绕竖直轴转一圈），解出来的 b 越可靠。

注意：如果只做"原地平转一圈"这种绕竖直轴的旋转，b 的竖直分量
（雷达到旋转轴的高度差）是解不出来的——那一项和 P 的竖直分量
纠缠在一起、方程组在竖直方向秩不足。这个高度差本来就该用卷尺量，
不需要也不该指望标定算出来；脚本会用奇异值检测这种情况并提示。
"""

import numpy as np


def quaternion_to_matrix(x, y, z, w):
    """四元数转旋转矩阵，避免引入 scipy/tf_transformations 这类额外依赖。"""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        raise ValueError("四元数接近零向量，无法归一化")
    s = 2.0 / n
    return np.array([
        [1 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1 - s * (x * x + y * y)],
    ])


def solve_extrinsic(positions, rotations):
    """求解 b（旋转中心在雷达系下的坐标）和 P（旋转中心在世界系下的坐标）。

    Args:
        positions: (N, 3) 数组，雷达原点在世界系下的位置，N >= 3。
        rotations: (N, 3, 3) 数组，雷达姿态的旋转矩阵（world_R_lidar）。

    Returns:
        b: (3,) 旋转中心在雷达坐标系下的坐标。
        P: (3,) 旋转中心在世界坐标系下的坐标。
        singular_values: (6,) 最小二乘问题的奇异值，降序排列。
            最小奇异值远小于最大奇异值，说明旋转多样性不够、对应方向
            的解不可信（典型情况见模块说明里的竖直分量问题）。
    """
    positions = np.asarray(positions, dtype=float)
    rotations = np.asarray(rotations, dtype=float)

    if positions.ndim != 2 or positions.shape[1] != 3:
        raise ValueError(f"positions 应为 (N, 3)，实际是 {positions.shape}")
    if rotations.ndim != 3 or rotations.shape[1:] != (3, 3):
        raise ValueError(f"rotations 应为 (N, 3, 3)，实际是 {rotations.shape}")
    n = positions.shape[0]
    if rotations.shape[0] != n:
        raise ValueError("positions 和 rotations 的帧数对不上")
    if n < 3:
        raise ValueError("至少需要 3 帧数据才能求解，旋转角度差异越大越好")

    # 方程 p_i + R_i @ b = P，整理成 A @ [b; P] = y 的形式：
    #   [R_i | -I] @ [b; P] = -p_i
    a = np.zeros((3 * n, 6))
    y = np.zeros(3 * n)
    for i in range(n):
        a[3 * i:3 * i + 3, 0:3] = rotations[i]
        a[3 * i:3 * i + 3, 3:6] = -np.eye(3)
        y[3 * i:3 * i + 3] = -positions[i]

    x, _residuals, _rank, singular_values = np.linalg.lstsq(a, y, rcond=None)
    b = x[0:3]
    p_world = x[3:6]
    return b, p_world, singular_values


def observability_warning(singular_values, ratio_threshold=1e-3):
    """根据奇异值判断这批数据的旋转多样性够不够，不够就返回警告文字。

    最小奇异值相对最大奇异值的比例低于 ratio_threshold，说明存在一个
    解方向几乎不受数据约束（求解器会给出一个数，但它基本是任意的）。

    Returns:
        警告字符串；数据质量够的话返回 None。
    """
    singular_values = np.asarray(singular_values, dtype=float)
    largest = singular_values.max()
    if largest <= 0:
        return "奇异值全为 0，数据有问题（比如所有帧位姿完全相同）"
    ratio = singular_values.min() / largest
    if ratio < ratio_threshold:
        return (
            f"最小/最大奇异值比例为 {ratio:.2e}（阈值 {ratio_threshold:.0e}），"
            "旋转多样性不够，部分分量（典型情况是竖直方向）的解不可信，"
            "建议只采用水平分量、竖直偏移改用卷尺实测"
        )
    return None
