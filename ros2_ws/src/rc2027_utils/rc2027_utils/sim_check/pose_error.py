"""定位结果和真值之间的误差——纯数学，不依赖 ROS，方便单测。"""

import math


def wrap_angle(a):
    """把角度折回 [-pi, pi)。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def pose_error(estimate, truth):
    """两个平面位姿 (x, y, yaw) 的误差。

    Returns:
        (dx, dy, dist, dyaw)：estimate - truth，dyaw 已折回 [-pi, pi)，
        跨 ±180° 时不会算出接近 360° 的假误差。
    """
    dx = estimate[0] - truth[0]
    dy = estimate[1] - truth[1]
    return dx, dy, math.hypot(dx, dy), wrap_angle(estimate[2] - truth[2])


def interpolate_pose(history, t):
    """在按时间排好序的 [(t, x, y, yaw), ...] 里取 t 时刻的位姿（线性插值，yaw 走近路）。

    t 不在历史范围内返回 None——不外推，宁可这一秒不报也不报一个猜出来的值。
    """
    if not history or t < history[0][0] or t > history[-1][0]:
        return None
    # 历史只有几百条，线性扫一遍足够；从后往前找，要查的通常是最近的时刻
    for i in range(len(history) - 1, 0, -1):
        t0, x0, y0, yaw0 = history[i - 1]
        t1, x1, y1, yaw1 = history[i]
        if t0 <= t <= t1:
            a = 0.0 if t1 == t0 else (t - t0) / (t1 - t0)
            return (x0 + a * (x1 - x0),
                    y0 + a * (y1 - y0),
                    wrap_angle(yaw0 + a * wrap_angle(yaw1 - yaw0)))
    t0, x0, y0, yaw0 = history[0]
    return (x0, y0, yaw0)  # 只有一条且正好等于 t


def speed_over(history, window):
    """最近 window 秒内的平均平移速度（m/s），历史不够长返回 None。"""
    if len(history) < 2:
        return None
    t_end, x_end, y_end, _ = history[-1]
    for t, x, y, _ in history:
        if t_end - t <= window:
            if t_end == t:
                return None
            return math.hypot(x_end - x, y_end - y) / (t_end - t)
    return None


def yaw_rate_over(history, window):
    """最近 window 秒内的平均角速度大小（rad/s，按相邻采样累加，转过 ±180° 也对），不够长返回 None。"""
    recent = [h for h in history if history[-1][0] - h[0] <= window]
    if len(recent) < 2 or recent[-1][0] == recent[0][0]:
        return None
    turned = sum(abs(wrap_angle(b[3] - a[3])) for a, b in zip(recent, recent[1:]))
    return turned / (recent[-1][0] - recent[0][0])
