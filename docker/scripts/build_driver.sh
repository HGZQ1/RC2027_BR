#!/bin/bash
# 编译 livox_ros_driver2。只做这一件事，刻意跟 point_lio 分开（见
# build_point_lio.sh 开头的注释解释为什么必须分开）。
set -euo pipefail

EXTERNAL_WS=/opt/external_ws

echo "[build_driver] 编译 livox_ros_driver2 ..."

if [ ! -d "${EXTERNAL_WS}/src/livox_ros_driver2" ]; then
    echo "[build_driver] 错误：external_ws/src/livox_ros_driver2 不存在！"
    echo "[build_driver]   这个不该发生——源码应该已经被 vendor 进仓库了，检查是不是手滑删了。"
    exit 1
fi

pushd "${EXTERNAL_WS}/src/livox_ros_driver2" > /dev/null
./build.sh humble
popd > /dev/null

echo "[build_driver] 完成"
