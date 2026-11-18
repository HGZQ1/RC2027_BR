# Docker 环境

给算法组用的统一开发/部署环境。设计目标：**两人机器上跑出来的东西必须一样**，不依赖"我电脑上能跑"。

## 为什么要分 TR / BR 两个容器

规则 11.10：TR 和 BR 比赛中禁止互相通信。ROS2 的 DDS 默认走局域网组播，两套程序即使代码里没写通信逻辑，只要在同一网段也有"意外通到对方"的风险。所以在 Docker 层面就把两者隔开，`docker-compose.yml` 里给 `tr`、`br` 两个服务分别设了不同的 `ROS_DOMAIN_ID`，从基础设施层面保证合规，不靠人工约定。

开发阶段先只用 `br` 这一个服务（你负责的雷达定位在 BR 上），`tr` 的服务定义先占位，等 TR 那边开始写代码再补内容。

## 现在的真实状态（而不是"打算做什么"）

| 东西 | 状态 |
|---|---|
| **雷达驱动**（livox_ros_driver2） | ✅ 真实源码（Mid-360S 专用版）+ 已移植 IMU 跟随外参旋转的 patch，Dockerfile 会真的编译它 |
| **Livox-SDK2**（驱动的底层依赖） | ✅ 真实源码，Dockerfile 会真的编译安装 |
| **定位算法**（point_lio） | ✅ 真实源码（华工去年 ROS2 版本，原样拷贝），Dockerfile 会真的编译它 |
| **EKF 融合**（robot_localization） | ✅ apt 装的二进制包，依赖由 apt 自动解析，不需要额外 pip |
| **RealSense SDK**（librealsense） | ✅ 从本地源码编译核心库；ROS2 的相机节点包（`realsense2_camera`）还没加 |
| **RViz2** | ✅ 装了，且 compose 配了 X11 转发，理论上能显示点云（没有实机验证过，见下文） |
| **PyTorch** | ⭕ 默认关闭（开关在，不是没做） |
| **Gazebo Classic / MuJoCo**（仿真，见"仿真"一节） | ✅ 2026-10-03 加进 `install_deps.sh`，没有实机验证过能不能跑起来 |
| **GPU 渲染直通**（见"GPU 直通"一节） | ⭕ 默认不启用，单独一个可选的 `docker-compose.gpu.yml`；助教确认了 `nvidia-smi` 能看到显卡、OpenGL 驱动库确实被注入，但还没有拿 RViz/Gazebo 实际渲染一次确认画面是真的在用 GPU |

下面按你问的几个问题逐个说。

## 1. 雷达驱动搞定没？接 RViz 能直接出点吗？

**驱动本身：搞定了。** `external_ws/src/livox_ros_driver2/` 现在是真实源码（去年广美整理的 Mid-360S 专用版），Dockerfile 会在构建时真的跑 `./build.sh humble` 编译它，不再是"代码到位后自动生效"的占位状态。还顺手把之前讨论过的"IMU 跟着外参一起旋转"的 patch 移植进去了（改动点见 `external_ws/src/livox_ros_driver2/NOTES.md`，三处都标了 `// RC2027 patch` 注释方便 diff）。

**RViz 能不能直接出点：代码链路上应该可以，但有两件事没有实机验证过**，如实说明：

1. **这个 patch 没有在真实硬件上跑过**——我是对照华工那边验证过的同款改法逐行搬过来的，结构确认一致，但"编译通过"不等于"运行时数值对不对"。车到了之后第一件事应该是验证静止时 IMU 读数（`ros2 topic echo /livox/imu --field linear_acceleration`，z 应该接近 1g）。
2. **RViz2 要显示窗口，需要 X11 转发**，`docker-compose.yml` 已经配了（`DISPLAY`、`QT_X11_NO_MITSHM`、挂载 `/tmp/.X11-unix`），但**启动容器前，宿主机要先执行一次**：

   ```bash
   xhost +local:docker
   ```

   不执行这步，`rviz2` 起来时会报连不上显示。另外现在走的是软件渲染（没配 NVIDIA runtime，CPU 画图），能用但会比有 GPU 直通慢，这个阶段先不管，能出图就行。

跑起来的命令大致是（等 point_lio 或先用 FAST-LIO 接上之后）：

```bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
rviz2   # Fixed Frame 选 livox_frame，添加 PointCloud2，话题选 /livox/lidar
```

## 2. EKF 依赖配齐没？

配齐了，而且**不需要 pip**。`robot_localization` 是通过 apt 装的二进制包（`install_deps.sh` 里的 `ros-humble-robot-localization`），它的依赖（Eigen、TF2、diagnostic_updater 等）全部由 apt 自动解析安装，不存在"进容器才发现还要 pip 装什么"的情况——这正是优先用 apt 装预编译包、而不是从源码编译的好处之一。

## 3. 英特尔红外相机（D435i）能用吗？

**分两部分看，SDK 能用，ROS 节点还没加：**

- **librealsense 核心库**：能用。Dockerfile 会从本地源码（`docker/docker_deps/librealsense-2.56.4`）编译安装核心库，容器起来后 `rs-enumerate-devices` 这类 SDK 自带的命令行工具应该能正常找到 USB 接的 D435i（前提是 compose 里 USB 直通配置对，现在是 `privileged: true` + `network_mode: host`，USB 设备本来就能访问到）。
- **ROS2 节点**（`realsense2_camera`，真正发布 `/camera/camera/color/image_raw` 这些话题的那个包）**还没加**。这是故意的：按你说的"暂不打算做视觉识别"，这个包属于"自己的 ROS 包"性质，等真要接的时候放进 `external_ws` 里编译。
- **一个要留意的坑**：等以后真加 `realsense2_camera` 的时候，**不要同时用 apt 装这个包**（`ros-humble-realsense2-camera`）——它会带一份自己的 librealsense 版本，跟我们现在从源码编译装在 `/usr/local` 的那份（2.56.4）产生版本冲突。去年的 Dockerfile 里恰好两个都装了（`install_deps.sh` 的 apt 列表里有 `ros-humble-realsense2-camera`，后面又从源码编译了一份 librealsense），这次我们还没走到那一步，到时候应该二选一：要么只用 apt 版（连库带 ROS 节点一起装，不单独编译 librealsense），要么把 `realsense2_camera` 也从源码编译、链接我们自己装的这份 librealsense。

## 4. numpy 锁的问题

这次按去年验证过有效的套路做了，而不是重新摸索：

1. `install_deps.sh` 里，**所有 apt 安装都做完之后**才处理 Python 依赖——因为 `python3-opencv`/`cv_bridge` 这些包会通过 apt 依赖关系顺带装一个系统版 `python3-numpy`，必须等它们都装完，才能一次性清掉、改用 pip 装的锁定版本，不然清了也白清。
2. 清理顺序：`rm -rf /usr/lib/python3/dist-packages/numpy*` → `pip install numpy==1.26.4`（固定值，和去年一致）→ 再装 `open3d`/`matplotlib`（这两个只要求 numpy 不低于某个版本，不会主动把它升级走）。
3. **PyTorch 那一层单独又锁了一次**：本地 wheel 目录里连 torch 自己的依赖（包括一份 `numpy==2.2.6`）都离线放着，装 torch 时会把 numpy 顺带升级成那份，所以装完 torch 立刻 `pip install --force-reinstall --no-deps numpy==1.26.4` 摁回去。这段只在 `INSTALL_PYTORCH=true` 时才会跑。

**别用 `tf_transformations`（2026-10-03 踩过）**：`ros-humble-tf-transformations` 依赖 apt 的 `python3-transforms3d 0.3.1`，里面用了 `np.float`（numpy 1.24 已删除），跟锁定的 numpy 1.26.4 不兼容，`import tf_transformations` 直接 `AttributeError`。镜像已经不装它了；Python 节点要四元数转 yaw，用 `rc2027_utils.teach_point.pose_stats.yaw_from_quaternion` 或者自己写 `atan2` 一行公式。真需要完整的 transforms3d，用 pip 装 `transforms3d>=0.4`（注意别让它顺带升级 numpy）。

去年反复被改坏的原因，大概率是"中间插了一个新 pip 包，没人记得要在那之后再补一次强制重锁"。这次把两处锁的位置和原因都写在了 Dockerfile/脚本的注释里，以后谁要加新的 pip 依赖，照着注释里说的"装完检查一下 numpy 版本，变了就在后面补一行强制重锁"操作，不要去改锁定的版本号迁就新包。

## 镜像分层

`Dockerfile` 按这个顺序分层构建（层与层之间尽量不互相影响，改上层不触发下层重新编译）：

1. ROS2 Humble 基础镜像 + 系统依赖/编译工具链（含 numpy 锁，见上文）
2. PyTorch（**默认关闭**，离线 wheel 安装）
3. librealsense（**默认开启**，从本地源码编译）
4. Livox-SDK2（独立 C++ 库，不进 colcon 构建）
5. `external_ws`：livox_ros_driver2（真实源码+patch）+ point_lio（真实源码）
6. `ros2_ws`：项目自己的代码（改动最频繁，放最后，吃 Docker 缓存）

## 本地开发 vs 正式构建

- **本地开发**：用 `docker-compose.yml` 里的 bind mount，把宿主机的 `ros2_ws/src`、`external_ws/src` 挂进容器，改代码不用重新 build 镜像，容器里手动 `colcon build --packages-select <改动的包>` 就行。
- **正式/上车部署**：建议打一个不挂载源码、把代码直接 COPY 进镜像的"生产版" Dockerfile（`Dockerfile.prod`，目前还没写，等代码稳定后再加），避免依赖宿主机文件系统。

## 快速开始

```bash
xhost +local:docker       # 允许容器连宿主机的 X11，rviz2 要用，见上文
cd rc2027/docker
docker compose build br
docker compose up br
```

进容器调试：

```bash
docker compose exec br bash
```

## VS Code：打开文件夹自动进容器

装 **Dev Containers** 扩展（`ms-vscode-remote.remote-containers`），用 VS Code 打开
`rc2027` 这个文件夹，右下角会提示"在容器中重新打开"，点一下就行；也可以命令面板
（`Ctrl+Shift+P`）里敲 `Dev Containers: Reopen in Container`。

配置在 [`.devcontainer/devcontainer.json`](../.devcontainer/devcontainer.json)，复用的
就是上面这同一个 `docker-compose.yml` 的 `br` 服务——不是另外单独维护一套，VS Code 里
跑起来的和命令行 `docker compose up br` 是**同一个容器定义**，镜像、依赖、patch 全一致。

几点说明：

- VS Code 里看到的文件树是整个 `rc2027` 仓库（对应 compose 里新加的 `..:/workspace`
  挂载），不只是 `ros2_ws/src`，在里面改代码跟直接改宿主机文件是同一份东西；
- 容器默认常驻（`command: sleep infinity`），VS Code 关掉窗口不会把它杀掉
  （`shutdownAction: none`），下次打开还是原来那个容器，`colcon build` 的产物还在；
- VS Code 内置终端会自动有 `ros2`/`colcon` 命令可用——这是靠 Dockerfile 最后把
  三行 `source` 写进了 `/root/.bashrc`（entrypoint 的 source 只对容器启动的第一个
  进程生效，VS Code/`docker exec` 连进来的是全新进程，不走 entrypoint，只能靠
  `.bashrc`）；
- 要用 `rviz2` 的话，**在宿主机的真实终端**（不是 VS Code 里面）执行一次
  `xhost +local:docker`，这件事 VS Code 没法替你做，容器每次启动时会在终端打印
  一句提醒。
- 目前只接了 `br`，`tr` 还没有自己的 devcontainer 配置，等 TR 那边开始写代码再加。
- **GPU 渲染直通**：有 NVIDIA 显卡/`nvidia-container-toolkit` 的机器，可以选
  [`.devcontainer/gpu/devcontainer.json`](../.devcontainer/gpu/devcontainer.json)
  这份配置（执行 `Dev Containers: Reopen in Container` 时会弹出选择列表，选
  "RC2027 (BR, GPU)"）。只比默认那份多叠加了 `docker-compose.gpu.yml`，其他
  完全一样。**单独放一份，没有改默认配置**：没有 NVIDIA 显卡的队友选默认的
  "RC2027 (BR)" 就行，两份互不影响，见 `docker-compose.gpu.yml` 开头注释。

## 文件说明

| 文件 | 作用 |
|---|---|
| `Dockerfile` | 镜像定义，按上面说的 6 层构建 |
| `docker-compose.yml` | 本地开发用，定义 tr/br 两个服务、设备挂载、X11 转发、网络模式、build args |
| `docker-compose.gpu.yml` | 可选覆盖文件，GPU 渲染直通，默认不启用，见上面专门一节 |
| `../.dockerignore`（注意，不在 `docker/` 下面） | 排除不需要进镜像构建上下文的文件，见下面单独一段解释为什么不放在这里 |
| `docker_deps/` | **不进 git**，librealsense / Livox-SDK2 / nlohmann_json 源码 + PyTorch wheel 离线缓存，每人自己机器上各放一份，见下方专门章节 |
| `scripts/install_deps.sh` | apt / pip 依赖安装（含 numpy 锁），镜像构建时调用 |
| `scripts/build_driver.sh` | 只编译 livox_ros_driver2，镜像构建时调用 |
| `scripts/build_point_lio.sh` | 只编译 point_lio，在 driver 编译完、装好之后才被 COPY 进去调用（原因见脚本开头注释：避免 driver 自带的裸 `colcon build` 误伤还没装好的 point_lio） |
| `scripts/entrypoint.sh` | 容器启动时执行：依次 source external_ws、ros2_ws，再执行传入的命令 |

## `.dockerignore` 为什么在仓库根目录，不在 `docker/` 下面

**踩过的坑，记一下**：这个文件之前被我放在 `docker/.dockerignore`，跟 `Dockerfile`
放一起，直觉上很合理——但 Docker 实际上是按**构建上下文的根目录**去找
`.dockerignore`，不是按 Dockerfile 所在的目录找。我们的构建上下文是
`docker-compose.yml` 里 `context: ..`，也就是仓库根目录 `rc2027/`，不是
`docker/`。结果是：`docker/.dockerignore` 放了很久，**其实从来没有被应用过**，
`librealsense-2.56.2`、`whl/50` 这些"应该被排除"的大文件其实一直都被完整打进了
build context（只是没造成构建错误，只是让每次 build 传输的数据比预期多了几个 G，
会拖慢速度）。

实测验证过（用临时目录 + 最小 Dockerfile 对比两种摆放方式）：放在 Dockerfile
旁边的 `.dockerignore` 完全不生效，放在仓库根目录才生效。现在已经挪到
`rc2027/.dockerignore`，往后**改忽略规则，去改根目录那份，不要在 `docker/` 下面
新建一份**（新建了也是摆设，不会报错，只会悄悄不起作用，很难发现）。

## 离线依赖缓存：`docker/docker_deps/`

体积大、没法在容器里联网下载的东西，统一手动放这个目录，**不进 git**（见 `.gitignore`），每个人在自己机器上各放一份：

```
docker/docker_deps/
├── Livox-SDK2/             # 雷达驱动的底层依赖，源码不大（2.4M），直接进 build context
├── librealsense-2.56.2/    # 旧版本，备用
├── librealsense-2.56.4/    # 当前默认用这个
├── nlohmann_json/          # librealsense 编译时需要的子依赖，故意保留了 .git（当本地 git clone 源用，
│                           #   见 Dockerfile 第 3 层注释），25M，直接进 build context
└── whl/
    ├── 40/                  # PyTorch 2.6.0+cu126 稳定版（py3.10），RTX 40系显卡 —— 当前默认
    └── 50/                  # PyTorch 2.12.0 nightly 开发版（cu128, py3.10），RTX 50系显卡 —— 备用
```

`40`/`50` 的命名延续去年 `Dockerfile.ros2.gpu` 的 `GPU_SERIES` 约定：50 系显卡当时刚出，PyTorch 对 sm120 架构支持很差，只有 nightly 能用，所以按显卡世代分了两套离线包。现在大多数人是 40 系显卡，默认用 `40`。PyTorch 更新之后如果要用新的稳定版，到时候重新下载替换 `whl/40` 里的内容就行，不急。

**根目录的 `.dockerignore`（见上一节为什么不在 `docker/` 下面）只放行默认用的那两份**
（`librealsense-2.56.4` 和 `whl/40`），另外两份会被排除出 Docker build context，不然每次
build 都要打包 6.7G 的无关文件。换默认版本时，`Dockerfile` 里的 `ARG` 默认值和
`.dockerignore` 里排除的那两行要一起改，不然会出现"改了 Dockerfile 默认值，但
`.dockerignore` 还在排除新选的那个版本"的坑。`nlohmann_json/` 不受这条规则影响，
两个版本都没有、也不需要备用版本。

### PyTorch（默认关闭，需要时手动打开）

```bash
docker compose build br --build-arg INSTALL_PYTORCH=true
```

不传这个参数（默认）就完全不碰 PyTorch，`pip` 也不会发起任何网络请求（用的是 `--no-index --find-links=本地目录`，纯离线安装）。换显卡世代：加 `--build-arg GPU_SERIES=50`（同时记得去 `.dockerignore` 把排除的那份换一下）。

**2026-10-03 助教那台机器是 RTX 5080 Laptop GPU，50 系**——以后在那台机器上
真要打开 `INSTALL_PYTORCH` 时必须记得传 `GPU_SERIES=50`，默认的 40 对应
cu126，不支持 sm_120（这台卡的架构），装上会用不了。

## 仿真：Gazebo / MuJoCo

`sim_ws` 下两个仿真资产（见 `ARCHITECTURE.md` 第四节）用的是两条独立的路线，
互不依赖：

- `robocon_2027_gazebo`：Gazebo Classic 11 + `gazebo_ros`，2026-10-03 加进
  `install_deps.sh`（`ros-humble-gazebo-ros-pkgs`）
- `rc2027_field_assets`：场地几何的权威数据源，`example_load.py` 需要
  `mujoco` 这个 pip 包，同一批加的

两个都是运行时依赖，不需要额外的 build arg，跟着默认的 `docker compose build br` 一起装。
没有 GPU 直通时 Gazebo 的 `gzclient` 是软件渲染，会比较卡，见下一节。

## GPU 直通（渲染用，跟 PyTorch 是两条独立的事）

上面 PyTorch 那条是**计算**用的 GPU（训练/推理，需要重新 build 镜像装 CUDA）。
这里说的是**渲染**用的 GPU（RViz2/Gazebo 的 OpenGL，不需要重新 build，
只是运行时往容器里注入驱动库），两者互不影响。

2026-10-03 助教在自己机器上实测过（RTX 5080 Laptop GPU，已装
`nvidia-container-toolkit`）：用现有镜像 `docker run --rm --gpus all
--entrypoint nvidia-smi rc2027:br-dev -L` 能看到显卡；加上
`NVIDIA_DRIVER_CAPABILITIES=all` 后，RViz/Gazebo 的 OpenGL 能走 NVIDIA，
不再是 `nouveau`/Mesa 软件渲染。

默认**不启用**（不是所有队友的机器都有 NVIDIA 显卡/toolkit，这段配置在没有的
机器上会导致容器直接起不来），单独放在 `docker-compose.gpu.yml` 里，需要的人
自己叠加：

```bash
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d br
```

## 构建验证

实际的 `docker build` / `docker compose build` 请在你们自己网络配置好的机器上跑——这边的沙箱环境网络受限，测不出真实结果，继续在这边跑只是浪费时间。Dockerfile / 脚本的改动我会做静态检查（语法、YAML 格式、ARG 声明是否匹配、路径对不对），但"跑不跑得通"以你们本机实跑的结果为准。

## 待确认事项

- [ ] `point_lio` 还没针对我们自己的雷达装车角度配 yaml（华工那几份标定值不能直接用，见其 `NOTES.md`）
- [x] 驱动的 IMU 旋转 patch：2026-10-02 实机验证通过，加速度和陀螺仪两部分都验证过，细节见 `external_ws/src/livox_ros_driver2/NOTES.md` 的"验证记录"
- [ ] 串口设备名（`/dev/chassis` 之类的 udev 规则）等接上真实下位机再确认
- [ ] `external_ws` 的源码管理方式（直接拷贝 vs git submodule）还没定，目前是直接拷贝
- [ ] 要不要同时装 `ros-humble-realsense2-camera`（apt 版 ROS 节点），见上文"librealsense 冲突"那一段，等真要用相机节点时再决定
- [ ] GPU 层目前没加（当前方案是 Point-LIO，纯 CPU）。如果以后要试 GLIM，再加一个 `Dockerfile.gpu` 分支，不要把 CUDA 依赖混进现在这个默认镜像，保持镜像精简
- [ ] Gazebo/MuJoCo 装上去之后没有实际跑过（`ros2 launch robocon_2027_gazebo ...`、`python3 example_load.py`），镜像能不能 build 过都还没验证
- [ ] GPU 渲染直通：`nvidia-smi` 和驱动库注入确认了，但没有拿 RViz2 或 `gzclient` 实际跑一次，确认画面渐染是不是真的在用 GPU（而不是巧合地还在用软件渲染）
- [ ] `.devcontainer/gpu/devcontainer.json` 这个多配置选择列表没有在真实 VS Code 里点过一遍，"Reopen in Container" 会不会正确弹出两个选项、选了之后两个 compose 文件是不是真的一起生效，都没有验证过
