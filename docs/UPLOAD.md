# 上传与交付范围

以 `rc2027/` 为交付根目录。交付内容包括运行源码、配置、测试、部署说明和必要设计决策。

## 保留内容

- 根 README、`docs/INTEGRATION.md` 和本文件。
- `ros2_ws/src/`、`external_ws/src/`、`sim_ws/src/`：源码、消息定义、launch、参数、测试、地图及扫描方向表。
- `docker/` 下的构建和启动文件、`.devcontainer/`、`.gitignore`、`.dockerignore`。
- `docs/rules/`、`docs/field_data/`：场地几何与示教点位。
- 决策0003、0004、0005：定位、串口和导航的详细设计参考。
- 决策0007、0008：短赛程与全场版的版本说明。默认采用全场版。
- 第三方许可证、来源说明及补丁说明。

## 留在本机

- 学习笔记、Cursor/Claude历史备份、参考工程及个人IDE配置。
- `docs/decisions/0002-session-roles.md`、`docs/decisions/test`、0006旧策略过程记录。必要设计原因已整理到对接文档。
- 各层 `build/`、`install/`、`log/`、Python缓存、录包与运行日志。
- `docker/docker_deps/`：作为依赖包单独交付，或随开发镜像提供。

上传采用筛选后的独立副本。文件包排除 `.git/`；需要云端仓库时从交付副本建立新仓库，使学习记录留在本机历史中。

## 新机器的依赖准备

当前 Dockerfile 读取以下本机目录：

```text
docker/docker_deps/Livox-SDK2/
docker/docker_deps/librealsense-2.56.4/
docker/docker_deps/nlohmann_json/    # 保留本地 Git 元数据，供构建时克隆
docker/docker_deps/whl/40/          # 默认 COPY 源，开启 PyTorch 时提供匹配 wheel
```

PyTorch 默认关闭，wheel目录仍参与COPY。上述目录通过依赖包补齐，或直接使用已构建的开发镜像。

MuJoCo 场地与道具 OBJ 随仓库提供，保留 heartpain-kong/rc2027_field 的 MIT 许可证。Gazebo 场地 STL 使用 YaoZA233/ABU-Robocon-2027-Gazebo 的固定版本，由 `sim_ws/src/robocon_2027_gazebo/sync_field_meshes.py` 补齐。原生启动和离线导入见[仿真说明](../sim_ws/src/rc2027_sim_bridge/README.md#原生-ros-2-环境)。

## 交付检查

1. 记录源码提交和随包参数。本机试验 F 的角速度3.0 rad/s尚未提交，交付时注明采用提交参数还是本机试验参数。
2. 核对地图、场地几何、点位、接口定义、`mid360.csv`和第三方许可证。
3. 按部署机器调整IP、串口设备和路径，清除个人凭据。
4. 在独立环境编译并运行单测，另外记录全场仿真结果。
