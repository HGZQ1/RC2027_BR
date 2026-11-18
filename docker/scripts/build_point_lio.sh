#!/bin/bash
# 编译 point_lio。
#
# 【为什么要跟 livox_ros_driver2 分成两个脚本/两个 Dockerfile 层，而不是
#  像之前那样一个 COPY external_ws/src 把两个包一起拷进去再依次编译】
#
# livox_ros_driver2 自己的 build.sh 内部跑的是不带 --packages-select 的
# 裸 `colcon build`（见它的源码），意思是"把 src/ 下能找到的所有包都编译"。
# 如果这时候 point_lio 已经跟它一起躺在 src/ 下，driver 的 build.sh 会
# 顺带把 point_lio 也拉进那一次 colcon build 里，而那个时间点 driver
# 自己可能都还没编译完/装好，point_lio 的 CMakeLists 又要
# find_package(livox_ros_driver2 REQUIRED)——两个包的编译顺序交给了
# 我们管不到的 driver 自带脚本，出没出问题不好说，排查起来还麻烦。
#
# 所以现在 Dockerfile 里分两层：先只 COPY driver、跑 build_driver.sh，
# 这一步结束时 point_lio 的源码压根还不在镜像里、不在磁盘上，driver
# 的裸 colcon build 不可能误伤到它。确认 driver 编译完、装好之后，
# 下一层才 COPY point_lio 进来，用这个脚本单独编译它。
set -euo pipefail

EXTERNAL_WS=/opt/external_ws

if [ ! -d "${EXTERNAL_WS}/src/point_lio" ]; then
    echo "[build_point_lio] 警告：external_ws/src/point_lio 不存在，跳过（这个不该发生，检查是不是手滑删了）。"
    exit 0
fi

# 【踩坑记录】colcon 生成的 setup.bash 内部直接引用 $COLCON_TRACE 等变量、
# 没有加默认值兜底，在 `set -u`（本脚本开头就开着）的环境下 source 它会
# 直接报 "unbound variable" 退出。这是 colcon 本身的已知行为，标准处理
# 方式就是 source 前后临时关掉/开回 -u。
set +u
source "${EXTERNAL_WS}/install/setup.bash"
set -u

echo "[build_point_lio] 编译 point_lio ..."
cd "${EXTERNAL_WS}"
colcon build --packages-select point_lio

echo "[build_point_lio] 完成"
