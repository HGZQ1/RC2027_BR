# RC2027 算法组开发镜像
#
# 【这里原来有一行 `# syntax=docker/dockerfile:1`，删掉了】
# 那行的作用是让 BuildKit 去拉一个指定版本的"frontend"镜像来解析这个
# Dockerfile，但拉取本身要联网（会访问 registry-1.docker.io），我们的
# 网络环境这步经常直接 TLS 握手超时，反而成了构建失败的原因。
# 不写这行，BuildKit 会用 Docker 自己内置的 frontend（随 Docker Engine
# 版本一起装的，不用额外联网拉取），而 `--mount=type=cache` 这个功能
# 在内置 frontend 里早就支持了（稳定很多年了），不需要专门声明版本才能用。
# 除非以后要用什么非常新的 Dockerfile 语法特性，不然别加这行回来。
# 分层顺序：越底层改动越少，越上层改动越频繁 —— 保证日常改代码时能最大限度命中 Docker 缓存
#
# 构建：docker build -f docker/Dockerfile -t rc2027:dev .   （在仓库根目录执行，构建上下文需要能看到 ros2_ws / external_ws）
# 或者直接用 docker compose build（见 docker-compose.yml）

FROM ros:humble-ros-base-jammy AS base

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=humble \
    LANG=C.UTF-8

SHELL ["/bin/bash", "-c"]

# ──────────────────────────────────────────────────────────────
# 第 1 层：系统依赖 + 编译工具链
#   python3-dev 是 Point-LIO 编译的硬性依赖（CMakeLists 里
#   find_package(PythonLibs REQUIRED)，没装这个 cmake 配置就过不了）
#
#   三个 --mount=type=cache：apt 的下载缓存、apt 的索引、pip 的缓存，
#   各自独立存在 BuildKit 自己管理的一块地方，不是这一层镜像的一部分，
#   也就不受"改了 install_deps.sh、这一层缓存失效"的影响——
#   即使整个脚本重跑一遍，之前下载过的 .deb/.whl 还在，不用重新走网络。
#   以后改这个脚本加新包，直接改、直接 build 就行，不用额外做什么。
# ──────────────────────────────────────────────────────────────
COPY docker/scripts/install_deps.sh /tmp/install_deps.sh
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    --mount=type=cache,target=/root/.cache/pip \
    chmod +x /tmp/install_deps.sh && /tmp/install_deps.sh && rm /tmp/install_deps.sh

# ──────────────────────────────────────────────────────────────
# 第 2 层：PyTorch（可选，默认关闭）
#   实测直接 pip install torch 在我们的网络环境下根本下不动（体积太大、容易卡死/超时），
#   现在也不确定今年要不要跑视觉/AI，所以默认不装。
#
#   离线 wheel 缓存放在 docker/docker_deps/whl/<GPU_SERIES>/ 下（不进 git，见 .gitignore），
#   命名延续去年 Dockerfile.ros2.gpu 的 GPU_SERIES 约定（当时 50 系显卡刚出，
#   PyTorch 对 sm120 支持很差，只有 nightly 能用，所以按显卡世代分两套）：
#     40 = torch 2.6.0+cu126 稳定版，给 RTX 40 系显卡用（默认，大多数人是这个）
#     50 = torch 2.12.0 nightly 开发版（cu128），给 RTX 50 系显卡用，必要时再换新稳定版
#   换成 50：docker compose build br --build-arg INSTALL_PYTORCH=true --build-arg GPU_SERIES=50
#   同时要去 docker/.dockerignore 把排除的那份换一下（现在排除的是 whl/50）。
#
#   不启用时（默认 INSTALL_PYTORCH=false），COPY 进来的文件不会被安装，
#   也不会发起任何网络请求。
#
#   numpy 锁的坑（去年踩过，这次按同样的套路处理，不要图省事省掉任何一步）：
#   whl 目录里连 torch 自己的依赖（numpy/pillow/jinja2 等）都一起离线放着，
#   pip 装 torch 时会连带把 numpy 升级成 whl 目录里那份（比如 2.2.6），
#   这会跟 install_deps.sh 里锁的 numpy==1.26.4 冲突。所以这里"先随它装，
#   装完再把 numpy 强制摁回 1.26.4"——这是去年验证过有效的顺序，不要改成
#   "装 torch 时加 --no-deps"，那样 torch 其他依赖又会缺。
# ──────────────────────────────────────────────────────────────
ARG INSTALL_PYTORCH=false
ARG GPU_SERIES=40
COPY docker/docker_deps/whl/${GPU_SERIES} /tmp/pytorch_wheels
RUN if [ "$INSTALL_PYTORCH" = "true" ]; then \
        echo "[pytorch] 从本地 wheel(GPU_SERIES=${GPU_SERIES}) 安装 torch/torchvision/torchaudio ..." && \
        pip3 install --no-cache-dir --no-index --find-links=/tmp/pytorch_wheels \
            torch torchvision torchaudio --break-system-packages && \
        echo "[pytorch] 安装完成，把 numpy 摁回锁定版本 1.26.4（见上面注释）" && \
        pip3 install --no-cache-dir --force-reinstall --no-deps \
            numpy==1.26.4 --break-system-packages ; \
    else \
        echo "[pytorch] INSTALL_PYTORCH=false，跳过（默认行为）" ; \
    fi && \
    rm -rf /tmp/pytorch_wheels

# ──────────────────────────────────────────────────────────────
# 第 3 层：librealsense（D435i 用，默认直接编译进镜像）
#   "以防万一"不是指这个库本身——这个现在就装好；指的是"视觉识别的节点/逻辑"
#   暂时不写。源码同样是本地离线缓存（docker/docker_deps/，不进 git），
#   目前本地有 2.56.2 / 2.56.4 两个版本，默认用新的 2.56.4。
#   只编译核心 SDK（库 + 头文件），不编译示例/GUI viewer（不需要 GTK，少装一堆包）。
#   ROS2 的 realsense2_camera 包等真正要接相机节点时再加到 external_ws。
#
#   【2026-10-02 助教那边实测踩的坑】：librealsense 的 cmake 配置阶段会通过
#   CMake/external_json.cmake 在线 git clone nlohmann/json（硬编码在它自己
#   的 CMake/json-download.cmake.in 里，不是一个能从命令行关掉的 cmake 选项）。
#   源码本身已经离线了，这一个子依赖却漏网，构建时偶发 TLS 握手失败。
#
#   修法：vendor 一份 nlohmann/json v3.11.3 的本地 git 仓库（docker/docker_deps/
#   nlohmann_json，特意保留了 .git，别的 vendor 目录都是去掉 .git 的，这个不一样，
#   原因见下面），构建时用 `git config --global url.<本地路径>.insteadOf
#   <github地址>` 把"git clone github.com/nlohmann/json"这条命令**透明重定向**
#   到本地路径——librealsense 自己的 CMake 脚本完全不用改一个字，它还是在"clone"，
#   只是 clone 的源被 git 自己悄悄换掉了。这是标准的 git mirror 做法，不是我们
#   发明的奇技淫巧。
# ──────────────────────────────────────────────────────────────
ARG REALSENSE_VERSION=2.56.4
COPY docker/docker_deps/nlohmann_json /tmp/nlohmann_json_mirror
RUN git config --global url."file:///tmp/nlohmann_json_mirror".insteadOf \
        "https://github.com/nlohmann/json.git"

COPY docker/docker_deps/librealsense-${REALSENSE_VERSION} /tmp/librealsense
RUN cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=false \
        -DBUILD_GRAPHICAL_EXAMPLES=false \
        -DBUILD_PYTHON_BINDINGS=false \
        -DFORCE_RSUSB_BACKEND=true && \
    cmake --build /tmp/librealsense/build -j"$(nproc)" && \
    cmake --install /tmp/librealsense/build && \
    ldconfig && \
    rm -rf /tmp/librealsense /tmp/nlohmann_json_mirror

# ──────────────────────────────────────────────────────────────
# 第 4 层：Livox-SDK2（独立 C++ 库，不进 colcon 构建流程，livox_ros_driver2 编译时要链接它）
#   源码离线放在 docker/docker_deps/Livox-SDK2（不进 git，跟 librealsense 一个待遇）
# ──────────────────────────────────────────────────────────────
COPY docker/docker_deps/Livox-SDK2 /tmp/Livox-SDK2
RUN cmake -S /tmp/Livox-SDK2 -B /tmp/Livox-SDK2/build -DCMAKE_BUILD_TYPE=Release && \
    cmake --build /tmp/Livox-SDK2/build -j"$(nproc)" && \
    cmake --install /tmp/Livox-SDK2/build && \
    ldconfig && \
    rm -rf /tmp/Livox-SDK2

# ──────────────────────────────────────────────────────────────
# 第 5 层：外部工作空间（驱动 + 定位算法），改动频率低
#
#   故意拆成两个 COPY+RUN，不是一次性把 external_ws/src 整个拷进来——
#   livox_ros_driver2 自己的 build.sh 内部跑的是不带 --packages-select
#   的裸 colcon build（"把 src/ 下所有包都编译"）。如果 point_lio 这时候
#   已经在 src/ 里，会被这次裸 build 误伤，编译顺序/时机不可控。
#   所以先只拷 driver、编译、装好，这一步结束前 point_lio 压根不在磁盘上，
#   driver 不可能碰到它；确认 driver 装好之后，下一步才拷 point_lio 进来
#   单独编译。细节见 build_point_lio.sh 开头的注释。
# ──────────────────────────────────────────────────────────────
WORKDIR /opt/external_ws
COPY external_ws/src/livox_ros_driver2 ./src/livox_ros_driver2
COPY docker/scripts/build_driver.sh /tmp/build_driver.sh
RUN chmod +x /tmp/build_driver.sh && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    /tmp/build_driver.sh && \
    rm /tmp/build_driver.sh

COPY external_ws/src/point_lio ./src/point_lio
COPY docker/scripts/build_point_lio.sh /tmp/build_point_lio.sh
RUN chmod +x /tmp/build_point_lio.sh && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    /tmp/build_point_lio.sh && \
    rm /tmp/build_point_lio.sh

# ──────────────────────────────────────────────────────────────
# 第 6 层：项目工作空间（自己的代码），改动最频繁，放最后
#   本地开发时这一层会被 docker-compose 的 bind mount 覆盖，
#   这里的 COPY 主要是给"不挂载源码的部署场景"用
# ──────────────────────────────────────────────────────────────
WORKDIR /opt/ros2_ws
COPY ros2_ws/src ./src
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source /opt/external_ws/install/setup.bash && \
    colcon build --symlink-install

# ──────────────────────────────────────────────────────────────
# entrypoint：依次 source 两层工作空间
#
#   这个 entrypoint 只对"容器启动的第一个进程"生效（PID 1）。VS Code
#   Dev Containers 或者手动 `docker exec -it ... bash` 连进来的是一个
#   全新的进程，不会重新走 entrypoint，只会读 bash 自己的启动文件
#   （交互式非登录 shell 读的是 .bashrc）。所以下面单独把 source 三行
#   写进 ~/.bashrc，保证"exec 进去就能直接用 ros2 命令"，不依赖
#   entrypoint 有没有跑过。
# ──────────────────────────────────────────────────────────────
RUN { \
      echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; \
      echo '[ -f /opt/external_ws/install/setup.bash ] && source /opt/external_ws/install/setup.bash'; \
      echo '[ -f /opt/ros2_ws/install/setup.bash ] && source /opt/ros2_ws/install/setup.bash'; \
    } >> /root/.bashrc

COPY docker/scripts/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /opt/ros2_ws
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
