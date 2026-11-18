#!/bin/bash
# 系统级依赖安装。对应之前讨论的 Docker 依赖表。
# 分成几个 apt 调用是为了后面改某一类依赖时，git diff 里看得清楚改的是哪一组。
#
# 关于"改这个文件要不要整个重新下载一遍"：Dockerfile 调用这个脚本的 RUN
# 指令挂了 BuildKit 的 cache mount（--mount=type=cache），apt 的下载缓存
# （/var/cache/apt）、索引（/var/lib/apt/lists）、pip 的下载缓存
# （/root/.cache/pip）本意是单独存在一块跨 build 持续保留的缓存里。
#
# 【2026-10-02 助教那边实测踩的坑，记一下】：光挂载 /var/cache/apt 不够用。
# Debian/Ubuntu 官方镜像（包括我们用的 ros:humble-ros-base-jammy）自带
# /etc/apt/apt.conf.d/docker-clean，内容是"每次 apt 装完就把 .deb 包删掉"，
# 这是 Docker 官方镜像为了"不用 cache mount 时也能保持镜像小"而加的默认行为，
# 但恰好跟我们现在要用的 cache mount 对着干——挂进来的缓存目录每次都被这个
# 配置自己清空，下一次 build 照样要全量重新下载。下面两行是 Docker 官方文档
# 给的标准解法：删掉这个自动清理配置，再显式告诉 apt "下载的包留着"。
set -euo pipefail

rm -f /etc/apt/apt.conf.d/docker-clean
echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' > /etc/apt/apt.conf.d/keep-cache

apt-get update

# ── 编译工具链 ──
apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    python3-dev \
    libomp-dev

# ── 数学 / 点云库 ──
apt-get install -y --no-install-recommends \
    libeigen3-dev \
    libpcl-dev

# ── yaml-cpp（field_localizer 读 field_geometry.yaml 用，见
#    rc2027_navigation 2026-10-03 改 C++ 的说明） ──
#   候选版本 0.7.0+dfsg-8build1，导出的 CMake 目标名是裸的 yaml-cpp，
#   在 host 上本地编译同版本验证过，不是猜的，见 CMakeLists.txt 注释
apt-get install -y --no-install-recommends \
    libyaml-cpp-dev

# ── point_lio 编译依赖 ──
#   只是缺头文件：point_lio 的 include/ivox/ivox3d.h 第 8 行 #include
#   <glog/logging.h>，唯一用到的 LOG(ERROR) 调用（约第 256 行）已经被
#   注释掉了，所以不需要链接 glog 库、只需要这个头文件能找到。
#   （2026-10-02 助教那边联调时发现，见 docs/decisions/0002-session-roles.md
#   的调试记录；选的是"加依赖"而不是"删 vendor 源码里那行 include"，
#   保持 point_lio 原样未改的约定）
apt-get install -y --no-install-recommends \
    libgoogle-glog-dev

# ── ROS2 通用包（robot_localization、TF、PCL桥接等） ──
#   nav-msgs / visualization-msgs / std-srvs 是 point_lio 的 package.xml 显式要的，
#   不确定会不会被别的包顺带装上，干脆直接列出来，不留侥幸
apt-get install -y --no-install-recommends \
    ros-humble-robot-localization \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-tf2-ros \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-nav-msgs \
    ros-humble-visualization-msgs \
    ros-humble-std-srvs \
    ros-humble-nav2-msgs
# nav2-msgs（2026-10-04 加）：只为了 NavigateToPose 这个 action 定义——导航是自研的
#   point_navigator（rc2027_navigation，见 docs/decisions/0005-navigation.md），
#   接口沿用 Nav2 的标准 action，跟 GAFA 去年一样。不装 Nav2 本体（navigation2 /
#   nav2-bringup），用不上
# 不装 ros-humble-tf-transformations（2026-10-03 去掉）：它依赖 apt 的
#   python3-transforms3d 0.3.1，里面用了 numpy 1.24 已删除的 np.float，
#   跟下面锁的 numpy 1.26.4 不兼容，import 直接报错。四元数/欧拉角转换
#   自己写一行公式（见 rc2027_utils/teach_point/pose_stats.py 的
#   yaw_from_quaternion），见 docker/README.md 第 4 节

# ── Gazebo Classic（没有真车真场地阶段的仿真，见 ARCHITECTURE.md 第四节） ──
#   sim_ws/src/robocon_2027_gazebo 的 package.xml 声明的 exec_depend 是
#   gazebo_ros，这个元包把 Gazebo Classic 11 本体 + gazebo_ros/gazebo_plugins
#   一起装上。2026-10-03 助教那边确认镜像里缺这个，加上。
#   没有 GPU 直通时 gzclient 是软件渲染会比较卡，GPU 直通是单独的 compose
#   覆盖文件（docker-compose.gpu.yml），跟这里装不装 Gazebo 无关
#   libboost-chrono-dev：sim_ws/src/livox_laser_simulation_ros2（Mid-360 仿真
#   插件）的 CMakeLists 里 find_package(Boost REQUIRED COMPONENTS chrono)，
#   不确定会不会被别的包顺带装上，显式列出来不留侥幸。插件其余依赖
#   （gazebo_dev/gazebo_plugins/gazebo11/libgazebo11-dev）2026-10-03 查过
#   gazebo_ros_pkgs 元包的 package.xml，确认会被下面这个包传递装上
#   teleop_twist_keyboard：仿真里键盘开车建图（发 cmd_vel，Gazebo 差速插件订阅）
apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs \
    libboost-chrono-dev \
    ros-humble-teleop-twist-keyboard

# ── auto_serial_bridge 编译依赖（下位机协议代码生成框架，见
#    ros2_ws/src/auto_serial_bridge/NOTES.md）──
#   package.xml 里的 <depend> 对应到的 apt 包名；asio 本身没单独列，
#   实测跟着 ros-humble-asio-cmake-module 一起装好了，不需要再单独加
#   （2026-10-02 助教一次性容器实测：装了下面三个之后 15.6s 编译通过）
apt-get install -y --no-install-recommends \
    ros-humble-io-context \
    ros-humble-asio-cmake-module \
    ros-humble-rclcpp-components

# ── 串口 / udev（容器内解析 udev 规则用） ──
#   socat 能建一对虚拟串口（/dev/pts/x <-> /dev/pts/y），没有下位机硬件时，
#   mock_stm32 脚本和 auto_serial_bridge 生成的 ROS 节点可以各接一头互相收发，
#   不用等真的 STM32 板子就能测通串口协议这条链路；python3-serial 是 mock
#   脚本读写串口用的（auto_serial_bridge 自己的 test_depend 里也要这两个）
apt-get install -y --no-install-recommends \
    udev \
    socat \
    python3-serial

# ── 调试工具 ──
apt-get install -y --no-install-recommends \
    tmux \
    ros-humble-rviz2

# ──────────────────────────────────────────────────────────────
# 相机 / 视觉（预留，暂不写视觉识别节点，但基础库和权限现在就备好）
#   - v4l-utils / usbutils：宿主机摄像头是 UVC 设备时排查用（lsusb、v4l2-ctl --list-devices）
#   - cv_bridge / image_transport：ROS2 <-> OpenCV 图像互转，之后接检测节点必需
#   - python3-opencv：轻量，先装上不费事
#   - libssl-dev / libusb-1.0-0-dev / libudev-dev / pkg-config：
#       编译 librealsense 源码需要的系统依赖（源码本身是离线放在
#       docker/docker_deps/ 的，见 Dockerfile 里"第1.6层"，不走 Intel 的 apt 源，
#       不需要额外联网加源）
# ──────────────────────────────────────────────────────────────
apt-get install -y --no-install-recommends \
    v4l-utils \
    usbutils \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-opencv \
    libssl-dev \
    libusb-1.0-0-dev \
    libudev-dev \
    pkg-config

# 容器目前以 root 运行，设备文件权限一般不是问题（root 不受 DAC 权限位限制）；
# 如果以后改成非 root 用户跑，记得把那个用户加进 video/plugdev 组：
#   usermod -aG video,plugdev <username>
#
# 真正要接 D435i 节点时，还需要 ROS2 的 realsense2_camera 包（链接刚编译好的
# librealsense），这个属于"自己的 ROS 包"性质，放进 external_ws 里编译，
# 不在这个系统依赖脚本里装。

# ──────────────────────────────────────────────────────────────
# Python 依赖 + numpy 锁定
#
#   放在这个文件最后（所有 apt 安装都做完之后）是故意的：上面 cv_bridge /
#   python3-opencv 这些包会通过 apt 依赖关系顺带装一个系统版 python3-numpy
#   （Ubuntu jammy 仓库版本，通常是 1.x 系列里比较老的一个小版本），
#   如果不清掉它、只靠 pip 装一份新的，两份 numpy 混在一起容易出诡异的
#   ABI 不匹配问题（典型症状：import 报错、或者看起来装成功了但某个库
#   运行时崩）。这是去年的 Dockerfile 踩出来的经验（见 docker 配置历史里
#   反复出现的"numpy 必须锁 1.26.4"注释），这次直接按那个套路来，不重新摸索：
#
#     1) 先彻底删掉 apt 装的系统版 numpy
#     2) pip 装一个锁定版本（1.26.4：numpy 1.x 系列最后一个版本，兼容面广）
#     3) 再装 open3d / matplotlib —— 这两个只要求 numpy>=某个较低版本，
#        不会主动把 1.26.4 升级上去，正常不会冲突
#
#   【重要】如果以后在别处（比如某个新加的 pip 包、或者上面的 PyTorch 层）
#   又把 numpy 版本带跑偏了，不要去改这里的版本号"将就"那个新包——
#   按 PyTorch 那一层的写法，在那个新装的东西后面加一行强制重锁
#   `pip3 install --force-reinstall --no-deps numpy==1.26.4`，
#   而不是反复改来改去最后谁也不知道当前到底锁的是哪个版本。
# ──────────────────────────────────────────────────────────────
# 直连 PyPI 对我们的网络环境不稳定，用清华源——这是去年验证过能稳定跑通的
# 镜像，要换源只用改这一个变量
PIP_MIRROR="https://pypi.tuna.tsinghua.edu.cn/simple"

apt-get install -y --no-install-recommends python3-pip
# 注意：这一步不能加 --break-system-packages——此时 apt 装的还是 jammy 自带的老 pip
# （不认识这个选项，加了反而报错 "no such option"）。先用它把 pip 自己升级上去，
# 之后 pip 版本够新了，才需要也才认识这个选项。
python3 -m pip install --upgrade pip setuptools wheel -i "${PIP_MIRROR}"

rm -rf /usr/lib/python3/dist-packages/numpy*

pip3 install numpy==1.26.4 -i "${PIP_MIRROR}" --break-system-packages
pip3 install open3d matplotlib -i "${PIP_MIRROR}" --break-system-packages

# mujoco：sim_ws/src/rc2027_field_assets/example_load.py 用它读取/检查场地模型
# （MuJoCo 坐标数据源，跟上面装的 Gazebo 是两条独立的仿真路线，互不依赖）。
# 2026-10-03 跟 Gazebo 一起加，不单独要求 numpy 版本，不会把上面锁的版本带跑偏
pip3 install mujoco -i "${PIP_MIRROR}" --break-system-packages

# 不手动 rm -rf /var/lib/apt/lists/*——这个目录现在是 cache mount 进来的，
# 不会被打进最终镜像层，留着比清掉更好（下次 build 还能用）

# ──────────────────────────────────────────────────────────────
# 可选：GPU 层（当前方案是 Point-LIO，纯 CPU，不需要）
# 如果以后要试 GLIM，另起一个 Dockerfile.gpu，
# 在那边装 nvidia-container-toolkit 对应的 CUDA 基础镜像，
# 不要往这个默认镜像里加，保持它精简、两人机器都能跑
# ──────────────────────────────────────────────────────────────
