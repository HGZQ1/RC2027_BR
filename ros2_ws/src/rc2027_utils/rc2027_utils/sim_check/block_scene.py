"""仿真摆方块的场景：场景文件 → 每块方块的真值位姿、Gazebo SDF、跟检测结果比对。纯 Python，不依赖 ROS。

场景文件（yaml）：

    z_offset: 0.05            # Gazebo 场地比规则整体高 5 cm
    stacks:                   # 一摞一摞地写，从下往上
      - region: spot_03       # spot_01..spot_10 / transfer_red / transfer_blue
        dx: 0.02              # 这摞相对区域中心的偏移（m）
        dy: -0.01
        yaw_deg: [3, -4, 10]  # 每层朝向（度）；写一个数 = 每层一样
        offsets: [[0, 0], [0.02, 0.01], [0, 0]]   # 可选：每层再错开多少（m）
        blocks: [ground, ground, sky]

区域位置、台面高度从 docs/rules/field_geometry.yaml 读（跟 block_detector 一样：L2 / L1 / 地面按 levels 判断）。
"""

import math

GROUND_SIZE = 0.35
SKY_SIZE = 0.20
SIZES = {'ground': GROUND_SIZE, 'sky': SKY_SIZE}
# 红方的地块是红的。天空块朝上的一面：红是我们的，蓝是对方的。Gazebo 里看着方便，雷达看不出颜色
COLORS = {'ground': (0.8, 0.15, 0.15), 'sky': (0.95, 0.95, 0.95)}
SKY_FACES = {'red': (0.9, 0.12, 0.12), 'blue': (0.15, 0.35, 0.9)}


def regions_from_geometry(geometry):
    """field_geometry.yaml 的 dict → {区域名: (cx, cy, 台面高度)}（规则里的高度，不含仿真偏移）。"""
    l1, l2 = geometry['levels']['l1'], geometry['levels']['l2']

    def inside(level, x, y):
        return abs(x) <= level['size_m'][0] / 2 and abs(y) <= level['size_m'][1] / 2

    out = {}
    for name, (x, y) in geometry['building_spots'].items():
        z = l2['height_m'] if inside(l2, x, y) else l1['height_m'] if inside(l1, x, y) else 0.0
        out[name] = (float(x), float(y), float(z))
    for side, cfg in (geometry.get('transfer_area') or {}).items():
        out[f'transfer_{side}'] = (float(cfg['center'][0]), float(cfg['center'][1]),
                                   float(cfg['z']))
    return out


def scene_blocks(scene, regions):
    """场景 → 方块列表，每块 dict：name region type x y z_bottom yaw layer size。"""
    z_offset = float(scene.get('z_offset', 0.0))
    blocks = []
    for i, stack in enumerate(scene.get('stacks') or []):
        region = stack['region']
        if region not in regions:
            raise ValueError(f'stacks[{i}]: 没有区域 {region}')
        cx, cy, cz = regions[region]
        types = stack['blocks']
        yaws = stack.get('yaw_deg', 0.0)
        yaws = yaws if isinstance(yaws, list) else [yaws] * len(types)
        offsets = stack.get('offsets') or [[0.0, 0.0]] * len(types)
        if len(yaws) != len(types) or len(offsets) != len(types):
            raise ValueError(f'stacks[{i}]: yaw_deg / offsets 的个数要跟 blocks 一样')
        z = cz + z_offset
        for layer, (kind, yaw, off) in enumerate(zip(types, yaws, offsets), start=1):
            if kind not in SIZES:
                raise ValueError(f'stacks[{i}]: 方块类型只有 ground / sky，写的是 {kind}')
            if kind == 'sky' and layer != len(types):
                raise ValueError(f'stacks[{i}]: 天空块只能在最上面')
            blocks.append({
                'name': f'blk_{len(blocks):03d}',
                'region': region,
                'type': kind,
                'x': cx + float(stack.get('dx', 0.0)) + float(off[0]),
                'y': cy + float(stack.get('dy', 0.0)) + float(off[1]),
                'z_bottom': z,
                'yaw': math.radians(float(yaw)),
                'layer': layer,
                'size': SIZES[kind],
            })
            z += SIZES[kind]
    return blocks


def block_sdf(block):
    """一块方块的 SDF（static：不受物理影响，车蹭到也不会动，测检测够用）。"""
    s = block['size']
    face = block.get('face')
    if face in SKY_FACES:
        r, g, b = SKY_FACES[face]
    else:
        r, g, b = COLORS[block['type']]
    geom = f'<geometry><box><size>{s} {s} {s}</size></box></geometry>'
    color = f'{r} {g} {b} 1'
    return (
        f'<?xml version="1.0"?><sdf version="1.6"><model name="{block["name"]}">'
        f'<static>true</static><link name="link">'
        f'<collision name="collision">{geom}</collision>'
        f'<visual name="visual">{geom}<material><ambient>{color}</ambient>'
        f'<diffuse>{color}</diffuse></material></visual>'
        f'</link></model></sdf>')


def block_center(block):
    """SDF 模型原点在方块中心：(x, y, z, yaw)。"""
    return block['x'], block['y'], block['z_bottom'] + block['size'] / 2, block['yaw']


def yaw_diff_mod90(a, b):
    """两个朝向在"模 90°"意义下的差（rad，[-pi/4, pi/4)）。方块四面一样，只能测到模 90°。"""
    d = (a - b) % (math.pi / 2)
    return d - math.pi / 2 if d >= math.pi / 4 else d


def compare(truth, detected, max_dist=0.15):
    """一个区域的真值方块 vs 检测到的方块。

    Args:
        truth: scene_blocks 里属于这个区域的那些。
        detected: [{'type': 'ground'/'sky'/'unknown', 'x', 'y', 'yaw', 'layer', 'reliable'}]。
        max_dist: 同一层里中心离多远以内算"同一块"。

    Returns:
        (rows, missed, extra)：rows 每块真值一行 (truth, det 或 None, 误差 cm, 朝向误差度)；
        missed = 没找到的真值数；extra = 多出来的检测数。
    """
    used = set()
    rows = []
    for t in truth:
        best, best_d = None, max_dist
        for i, d in enumerate(detected):
            if i in used or d['layer'] != t['layer']:
                continue
            dist = math.hypot(d['x'] - t['x'], d['y'] - t['y'])
            if dist < best_d:
                best, best_d = i, dist
        if best is None:
            rows.append((t, None, None, None))
            continue
        used.add(best)
        d = detected[best]
        rows.append((t, d, best_d * 100, math.degrees(yaw_diff_mod90(d['yaw'], t['yaw']))))
    missed = sum(1 for r in rows if r[1] is None)
    return rows, missed, len(detected) - len(used)


def center_stack(truth, region_xy):
    """离区域中心最近的那一摞的 (地块数, 有没有天空块)——跟 block_detector 的摘要口径一样。"""
    if not truth:
        return 0, False
    cx, cy = region_xy
    bottoms = [t for t in truth if t['layer'] == 1]
    base = min(bottoms, key=lambda t: math.hypot(t['x'] - cx, t['y'] - cy))
    # 同一摞 = 跟塔基同一个 stack（名字连号、xy 很近）
    stack = [t for t in truth if math.hypot(t['x'] - base['x'], t['y'] - base['y']) < 0.1]
    return sum(t['type'] == 'ground' for t in stack), any(t['type'] == 'sky' for t in stack)
