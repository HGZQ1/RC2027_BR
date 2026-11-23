"""仿真里摆方块（spawn 到 Gazebo），再自动开过去看、跟真值比——测雷达方块检测（block_detector）。

三种模式（参数 mode）：
    spawn（默认）  先删掉上次摆的（blk_000～blk_xxx），再按场景文件摆
    clear          只删
    check          按场景文件的 views 一个一个开过去（point_navigator 的 navigate_to_point），停稳后调
                   /block_detector/detect 看 look 里的区域，跟真值比，最后打一张表

用法见场景文件开头（默认 sim_ws/src/rc2027_sim_bridge/config/block_scenes/l1_red.yaml）。
方块是 static 的（不受物理影响），车蹭到也不会动。场景里的区域位置、台面高度从 field_geometry.yaml 读。
"""

import math
import time

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rc2027_navigation_msgs.action import NavigateToPoint
from rc2027_perception_msgs.srv import DetectBlocks
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import yaml

from rc2027_utils.sim_check.block_scene import (
    block_center, block_sdf, center_stack, compare, regions_from_geometry, scene_blocks)

DEFAULT_SCENE = '/workspace/sim_ws/src/rc2027_sim_bridge/config/block_scenes/l1_red.yaml'
TYPE_NAMES = {0: 'ground', 1: 'sky', 2: 'unknown'}


class SimBlocks(Node):

    def __init__(self):
        super().__init__('sim_blocks')
        self.mode = self.declare_parameter('mode', 'spawn').value
        scene_path = self.declare_parameter('scene', DEFAULT_SCENE).value
        geometry_path = self.declare_parameter(
            'field_geometry_path', '/workspace/docs/rules/field_geometry.yaml').value
        self.clear_max = self.declare_parameter('clear_max', 40).value
        self.settle = self.declare_parameter('settle_sec', 2.0).value
        with open(scene_path, encoding='utf-8') as f:
            self.scene = yaml.safe_load(f)
        with open(geometry_path, encoding='utf-8') as f:
            self.regions = regions_from_geometry(yaml.safe_load(f))
        self.blocks = scene_blocks(self.scene, self.regions)
        self.get_logger().info(f'场景 {scene_path}：{len(self.blocks)} 块')

    def call(self, client, request, timeout=10.0):
        """同步调服务（这个节点只在 main 里顺序跑，spin_until_future_complete 就够）。"""
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'服务 {client.srv_name} 不在')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if future.result() is None:
            raise RuntimeError(f'调 {client.srv_name} 超时')
        return future.result()

    # ── 摆 / 删 ──
    def clear(self):
        client = self.create_client(DeleteEntity, '/delete_entity')
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('/delete_entity 不在（Gazebo 没起？）')
        # 删不存在的实体有时会一直不返回（助教 10-05 用 ros2 service call 卡了 40 分钟）：单次 3 s 超时就当它不存在，
        # 连续 3 次超时就不再往下删
        removed, timeouts = 0, 0
        for i in range(self.clear_max):
            try:
                res = self.call(client, DeleteEntity.Request(name=f'blk_{i:03d}'), timeout=3.0)
            except RuntimeError:
                timeouts += 1
                if timeouts >= 3:
                    self.get_logger().warn(f'删 blk_{i:03d} 连续 {timeouts} 次超时，后面的不删了')
                    break
                continue
            timeouts = 0
            removed += bool(res.success)
        self.get_logger().info(f'删掉了 {removed} 块')

    def spawn(self):
        client = self.create_client(SpawnEntity, '/spawn_entity')
        for b in self.blocks:
            req = SpawnEntity.Request()
            req.name = b['name']
            req.xml = block_sdf(b)
            x, y, z, yaw = block_center(b)
            req.initial_pose.position.x = x
            req.initial_pose.position.y = y
            req.initial_pose.position.z = z
            req.initial_pose.orientation.z = math.sin(yaw / 2)
            req.initial_pose.orientation.w = math.cos(yaw / 2)
            res = self.call(client, req)
            if not res.success:
                self.get_logger().error(f'{b["name"]} 摆不上: {res.status_message}')
        for b in self.blocks:
            self.get_logger().info(
                f'{b["name"]} {b["region"]:13s} 第{b["layer"]}层 {b["type"]:6s} '
                f'({b["x"]:.3f}, {b["y"]:.3f}) yaw {math.degrees(b["yaw"]):.1f}°')

    # ── 开过去看 ──
    def go(self, x, y, yaw_deg):
        client = ActionClient(self, NavigateToPoint, 'navigate_to_point')
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('navigate_to_point 不在（导航没起？）')
        goal = NavigateToPoint.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(math.radians(yaw_deg) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(yaw_deg) / 2)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError(f'导航目标 ({x}, {y}) 被拒绝')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=120.0)
        outcome = result.result().result.outcome if result.result() else 'TIMEOUT'
        if outcome != 'ARRIVED':
            raise RuntimeError(f'没开到 ({x}, {y})：{outcome}')

    def check(self):
        detect = self.create_client(DetectBlocks, '/block_detector/detect')
        lines, totals = [], {'blocks': 0, 'missed': 0, 'extra': 0, 'layers_wrong': 0}
        errors = []
        for view in self.scene.get('views') or []:
            x, y, yaw = view['go']
            self.get_logger().info(f'开到 ({x}, {y}, {yaw}°)')
            self.go(x, y, yaw)
            if not view.get('look'):
                continue
            time.sleep(self.settle)   # 停稳、攒几帧点云
            for region in view['look']:
                res = self.call(detect, DetectBlocks.Request(region=region))
                if not res.success:
                    lines.append(f'{region}: 调用失败 {res.message}')
                    continue
                s = res.state
                truth = [b for b in self.blocks if b['region'] == region]
                det = [{'type': TYPE_NAMES[b.type], 'x': b.x, 'y': b.y, 'yaw': b.yaw,
                        'layer': b.layer, 'reliable': b.center_reliable} for b in s.blocks]
                rows, missed, extra = compare(truth, det)
                totals['blocks'] += len(truth)
                totals['missed'] += missed
                totals['extra'] += extra
                if region.startswith('spot_'):
                    # 层数摘要只对建造点有意义（"建造点中心那一摞"）。交接区里几摞离中心差不多远，
                    # 摘要挑哪一摞是随缘的（2026-10-05：两摞离中心都是 0.283 m，真值挑了 2 叠、检测挑了单块）
                    cx, cy, _ = self.regions[region]
                    want = center_stack(truth, (cx, cy))
                    got = (s.ground_layers, s.has_sky)
                    totals['layers_wrong'] += want != got
                    summary = (f'层数 真 {want[0]}{"+天" if want[1] else ""} / '
                               f'测 {got[0]}{"+天" if got[1] else ""}'
                               f'{"（顶部可能出视场）" if s.top_maybe_clipped else ""}'
                               f'{" ✗" if want != got else ""}')
                else:
                    summary = '（交接区只比每一块）'
                lines.append(f'{region}: {summary}，漏 {missed}，多 {extra}'
                             f'{"，" + s.note if s.note else ""}')
                for t, d, dist, dyaw in rows:
                    if d is None:
                        lines.append(f'    第{t["layer"]}层 {t["type"]:6s} 没找到')
                        continue
                    errors.append(dist)
                    flag = '' if d['type'] == t['type'] else f' 类型测成 {d["type"]} ✗'
                    lines.append(
                        f'    第{t["layer"]}层 {t["type"]:6s} 中心误差 {dist:4.1f} cm  '
                        f'朝向误差 {dyaw:+5.1f}°{"" if d["reliable"] else "（中心不可靠）"}{flag}')
        self.get_logger().info('\n'.join(['', '──── 方块检测 vs 真值 ────'] + lines + [
            f'合计 {totals["blocks"]} 块：漏 {totals["missed"]}，多 {totals["extra"]}，'
            f'层数不对的建造点 {totals["layers_wrong"]}；中心误差 '
            + (f'中位 {sorted(errors)[len(errors) // 2]:.1f} cm、最大 {max(errors):.1f} cm'
               if errors else '无')]))


def main(args=None):
    rclpy.init(args=args)
    node = SimBlocks()
    try:
        if node.mode in ('spawn', 'clear'):
            node.clear()
        if node.mode == 'spawn':
            node.spawn()
        elif node.mode == 'check':
            node.check()
        elif node.mode != 'clear':
            node.get_logger().error(f'mode 只有 spawn / clear / check，写的是 {node.mode}')
    except RuntimeError as exc:
        node.get_logger().error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
