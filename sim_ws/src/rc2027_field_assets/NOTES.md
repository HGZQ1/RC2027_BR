# 本地备注（非上游内容）

上游仓库：https://github.com/heartpain-kong/rc2027_field
Vendored 时的 commit：ef30cc2（2026-10-01 拉取，MIT 协议）
已移除 .git，当前按"直接拷贝源码"管理，不是 submodule（submodule vs 直接拷贝待两人确认，见 ARCHITECTURE.md）。

---

# rc2027_field_assets

存放 MuJoCo 场地模型数据：https://github.com/heartpain-kong/rc2027_field
（MIT 协议，几何经过和官方 CAD 比对验证，带 10 个建塔点的精确 <site> 坐标 +
53 个可动比赛道具的质量/碰撞参数）。

本包**不是用来跑仿真的**（MuJoCo 本身没接 ROS2，也没有雷达传感器），
用途是作为"坐标权威数据源"：

- 场地原点在场地中心，若用西南角原点，换算 corner = centred + 5.5（两轴都加）
- 10 个建塔点坐标直接从 field.xml 的 <site> 抄出，见 docs/rules/field_geometry.yaml
