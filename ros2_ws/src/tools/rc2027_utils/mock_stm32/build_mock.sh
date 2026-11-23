#!/bin/bash
# 从唯一的协议权威来源（auto_serial_bridge 的 config/protocol.yaml）重新生成
# protocol.h/.c，和 mock_stm32.c 一起编译成一个可执行文件。
#
# 不把生成的 protocol.h/.c 提交进仓库——跟 auto_serial_bridge 自己的约定一样
# （见它的 .gitignore 注释），改一次 protocol.yaml 就重跑一次这个脚本即可，
# 不存在"生成的代码和 yaml 对不上"的问题。
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ASB_DIR="${SCRIPT_DIR}/../../auto_serial_bridge"
PROTOCOL_YAML="${ASB_DIR}/config/protocol.yaml"
# codegen.py 自己会在给的 output_dir 下面再建一层 generated/ 子目录，
# 所以这里传 SCRIPT_DIR 本身，实际产物落在 ${SCRIPT_DIR}/generated/ 下
GEN_DIR="${SCRIPT_DIR}/generated"

python3 "${ASB_DIR}/scripts/codegen.py" "${PROTOCOL_YAML}" "${SCRIPT_DIR}"

gcc -Wall -Wextra -std=c11 \
    -I "${GEN_DIR}" \
    "${SCRIPT_DIR}/mock_stm32.c" \
    "${GEN_DIR}/protocol.c" \
    -o "${SCRIPT_DIR}/mock_stm32"

echo "build 完成: ${SCRIPT_DIR}/mock_stm32"
echo "用法: ${SCRIPT_DIR}/mock_stm32 <串口设备路径> [波特率，默认 115200]"
