"""爬楼梯"运动学回放"的轨迹生成——纯 numpy，不依赖 ROS，host 上单测。

为什么要回放（2026-10-04）：仿真底盘是 planar_move（只管水平面、竖直速度每步清零），上不了
楼梯；BR 的爬楼梯机构还没图纸，也没法建模。但上位机要验证的是**高度变化后定位还准不准**
（Point-LIO 爬升中会不会发散、上了 L1 以后 ICP 能不能对上、L1 Retry 能不能用），不是爬升
物理。所以：按一条贴着地形的 3D 轨迹（含 z、俯仰、横滚），每个周期直接设置车的位姿和速度
（gazebo_ros_state 的 set_entity_state），Point-LIO / ICP 照常跑。

Gazebo 的 IMU 是对 link 速度做差分算加速度（再减世界重力，跟 link 自己开不开重力无关），
所以只要设进去的速度和位姿一致，IMU 读数就是对的。局限：没有真实的震动和冲击（台阶磕碰、
电机抖动），IMU 饱和这类问题只能真车测。

轨迹 = 水平路线（按段直线移动，车头朝向不变，舵轮横移）+ 地形：
  - 每个时刻在车体底面（chassis 0.3 x 0.3）每 2 cm 取一个点查表面高度
  - 用最小二乘平面拟合俯仰/横滚（楼梯上就是沿台阶前沿的斜面）
  - 时间上平滑，再把车体底面整体抬到所有点上方 clearance（不和台阶边缘相交）
  - 速度用中心差分（线速度、角速度都在世界系，跟 set_entity_state 的约定一致）
"""

import math

import numpy as np

# 红方 BR：起点（box_b）-> 楼梯底 -> 上楼梯（4 级，每级高 0.15 m、深 0.3 m，
# y −2.725 到 −1.825，地面 0.05 -> 平台 0.65）-> 楼梯顶平台 -> 上 L1 -> L1 Retry Zone。
# 车头一直朝 +y（90°），只平移（舵轮）。高度数据是 2026-10-04 对 Gazebo 场地网格实测的。
# 每段：('move', x, y, 速度 m/s)、('turn', 目标朝向 rad, 角速度 rad/s) 或 ('hold', 秒)
ROUTE_RED_STAIRS_TO_L1_RETRY = [
    ('hold', 2.0),
    ('move', -3.45, -3.30, 0.30),
    ('move', -3.45, -1.30, 0.15),   # 楼梯
    ('hold', 3.0),                  # 楼梯顶停一下（可以在这里 relocalize）
    ('move', -2.40, -1.30, 0.30),
    ('move', -2.40, 2.60, 0.30),
    ('move', -1.80, 2.60, 0.20),    # L1 Retry Zone 中心
    ('turn', -math.pi / 2, 0.8),    # 原地转到朝南（L1 Retry Zone 的摆法，用户 10-04 定）
    ('hold', 5.0),
]


def mirror_route(route):
    """红 -> 蓝：x 取反；朝向按 π - yaw 镜像（±90° 镜像后不变）。"""
    out = []
    for s in route:
        if s[0] == 'move':
            out.append((s[0], -s[1], s[2], s[3]))
        elif s[0] == 'turn':
            out.append((s[0], math.atan2(math.sin(math.pi - s[1]), math.cos(math.pi - s[1])), s[2]))
        else:
            out.append(s)
    return out


def build_xy_path(start, route, dt=0.02, accel=0.5):
    """水平路线 -> 等时间间隔的 (t, x, y, yaw) 数组，每段梯形速度（起停都平滑）。"""
    x, y, yaw = start
    out = [(0.0, x, y, yaw)]
    t = 0.0
    for seg in route:
        if seg[0] == 'hold':
            for _ in range(int(round(seg[1] / dt))):
                t += dt
                out.append((t, x, y, yaw))
            continue
        if seg[0] == 'turn':
            # 原地转，走近路，角速度梯形（同样的加速度数值当 rad/s²）
            delta = math.atan2(math.sin(seg[1] - yaw), math.cos(seg[1] - yaw))
            w = seg[2]
            t_acc = w / accel
            if accel * t_acc ** 2 > abs(delta):
                t_acc = math.sqrt(abs(delta) / accel)
                w = accel * t_acc
            t_flat = (abs(delta) - accel * t_acc ** 2) / w if w > 0 else 0.0
            total = 2 * t_acc + t_flat
            steps = max(1, int(math.ceil(total / dt)))
            for k in range(1, steps + 1):
                tau = min(total, k * dt)
                if tau < t_acc:
                    a = 0.5 * accel * tau ** 2
                elif tau < t_acc + t_flat:
                    a = 0.5 * accel * t_acc ** 2 + w * (tau - t_acc)
                else:
                    a = abs(delta) - 0.5 * accel * (total - tau) ** 2
                out.append((t + k * dt, x, y, yaw + math.copysign(a, delta)))
            t += steps * dt
            yaw = yaw + delta
            continue
        _, tx, ty, v = seg
        length = math.hypot(tx - x, ty - y)
        if length < 1e-9:
            continue
        # 梯形：加速段 / 匀速 / 减速段（太短就是三角形）
        t_acc = v / accel
        d_acc = 0.5 * accel * t_acc ** 2
        if 2 * d_acc > length:
            t_acc = math.sqrt(length / accel)
            d_acc = length / 2
            v = accel * t_acc
        t_flat = (length - 2 * d_acc) / v
        total = 2 * t_acc + t_flat
        steps = max(1, int(math.ceil(total / dt)))
        for k in range(1, steps + 1):
            tau = min(total, k * dt)
            if tau < t_acc:
                s = 0.5 * accel * tau ** 2
            elif tau < t_acc + t_flat:
                s = d_acc + v * (tau - t_acc)
            else:
                r = total - tau
                s = length - 0.5 * accel * r ** 2
            a = s / length
            out.append((t + k * dt, x + a * (tx - x), y + a * (ty - y), yaw))
        t += steps * dt
        x, y = tx, ty
    return np.array(out)


def _moving_average(a, n):
    if n <= 1:
        return a
    pad = n // 2
    p = np.pad(a, (pad, pad), mode='edge')
    return np.convolve(p, np.ones(n) / n, mode='valid')[:len(a)]


def _moving_max(a, n):
    if n <= 1:
        return a
    pad = n // 2
    p = np.pad(a, (pad, pad), mode='edge')
    return np.array([p[i:i + n].max() for i in range(len(a))])


def smooth_upper(a, n):
    """平滑，并且结果处处 >= 原值：先滑动最大、再滑动平均（窗口相同）。

    第 i 个点的平均窗口里每个最大值窗口都包含 i，所以平均值 >= a[i]。用在 z 上：
    "刚好不碰台阶"的高度在台阶边缘进入车体底面那一刻会跳一下，直接平滑会切进台阶，
    平滑后再取 max 又把跳变带回来（第一版就是这样，竖直速度算出来 2 m/s）。
    """
    return _moving_average(_moving_max(a, n), n)


def attach_terrain(path, height_fn, half_len=0.15, half_wid=0.15, spacing=0.02, clearance=0.01,
                   smooth_sec=0.5):
    """给水平路径加上 z / 横滚 / 俯仰：车体底面贴着地形、高出 clearance，不和台阶相交。

    返回 (N, 7)：t, x, y, z, roll, pitch, yaw（z 是 base_footprint 原点，即车体底面中心）。
    ROS 约定：pitch 为负 = 车头朝上；roll 为正 = 左侧朝上。
    """
    nu = int(round(2 * half_len / spacing)) + 1
    nv = int(round(2 * half_wid / spacing)) + 1
    ug, vg = np.meshgrid(np.linspace(-half_len, half_len, nu),
                         np.linspace(-half_wid, half_wid, nv))
    us, vs = ug.ravel(), vg.ravel()
    design = np.stack([np.ones_like(us), us, vs], 1)
    n = len(path)
    heights = np.zeros((n, len(us)))
    slopes = np.zeros((n, 2))
    for i, (_, x, y, yaw) in enumerate(path):
        c, s = math.cos(yaw), math.sin(yaw)
        wx, wy = x + c * us - s * vs, y + s * us + c * vs
        h = np.array([height_fn(px, py) for px, py in zip(wx, wy)])
        if np.isnan(h).any():
            raise ValueError(f'轨迹第 {i} 个点 ({x:.2f}, {y:.2f}) 下面有地方没有地面')
        # 每个采样点取它和上下左右邻点里最高的：台阶边缘落在两个采样点之间时，
        # 只看采样点会让车体底面切进台阶（单测查出来的）；台阶这种单调阶梯，膨胀一格
        # 以后能保证采样点之间也不相交
        g = h.reshape(nv, nu)
        p = np.pad(g, 1, mode='edge')
        dil = np.max([p[1 + dv:1 + dv + nv, 1 + du:1 + du + nu]
                      for dv in (-1, 0, 1) for du in (-1, 0, 1)], axis=0)
        heights[i] = dil.ravel()
        coef = np.linalg.lstsq(design, h, rcond=None)[0]
        slopes[i] = coef[1:]   # dz/du（前后）、dz/dv（左右）

    win = max(1, int(round(smooth_sec / (path[1, 0] - path[0, 0])))) if n > 1 else 1
    bu = _moving_average(slopes[:, 0], win)
    bv = _moving_average(slopes[:, 1], win)
    # 底面平面 z0 + bu*u + bv*v 要在每个采样点上方 clearance
    z0 = np.max(heights + clearance - bu[:, None] * us - bv[:, None] * vs, axis=1)
    z0 = smooth_upper(z0, win)   # 平滑、而且处处不低于"刚好不相交"的高度
    pitch = -np.arctan(bu)
    roll = np.arctan(bv)
    return np.column_stack([path[:, 0], path[:, 1], path[:, 2], z0, roll, pitch, path[:, 3]])


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


def _log_so3(r):
    """旋转矩阵 -> 旋转向量（轴 * 角度）。"""
    cos_a = max(-1.0, min(1.0, (np.trace(r) - 1) / 2))
    a = math.acos(cos_a)
    if a < 1e-9:
        return np.zeros(3)
    w = np.array([r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]])
    return w * a / (2 * math.sin(a))


def velocities(traj):
    """世界系线速度、角速度（中心差分），返回 (N, 6)：vx vy vz wx wy wz。"""
    n = len(traj)
    out = np.zeros((n, 6))
    rots = [rpy_to_matrix(*row[4:7]) for row in traj]
    for i in range(n):
        a, b = max(0, i - 1), min(n - 1, i + 1)
        dt = traj[b, 0] - traj[a, 0]
        if dt <= 0:
            continue
        out[i, :3] = (traj[b, 1:4] - traj[a, 1:4]) / dt
        out[i, 3:] = _log_so3(rots[b] @ rots[a].T) / dt
    return out


def com_velocity(v_base, omega, roll, pitch, yaw, com_in_body):
    """把 base_footprint 原点的线速度换成质心的线速度：v_com = v_base + ω × (R · r_com)。

    Gazebo 的 Model/Link::SetLinearVel 设的是**质心**速度（ODE 刚体的位置就是质心），IMU 的速度
    也是从质心速度推出来的。回放轨迹给的是 base_footprint 原点的速度，俯仰变化时（爬楼梯起止，
    角速度约 1 rad/s）两者差 ω × r，不换算的话 IMU 会看到一个假的加速度（2026-10-04 加）。
    """
    r = rpy_to_matrix(roll, pitch, yaw) @ np.asarray(com_in_body, float)
    return np.asarray(v_base, float) + np.cross(np.asarray(omega, float), r)
