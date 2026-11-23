/* mock_stm32 —— 假装自己是下位机（STM32），没有真实硬件时用来测串口链路。
 *
 * 框架部分（这个文件）已经完成；业务消息部分还没法写——今年和电控的协议
 * 还没定，config/protocol.yaml 里 messages 还是空的（见
 * ros2_ws/src/communication/auto_serial_bridge/NOTES.md）。等协议定了：
 *   1. 把业务消息加进 ../../auto_serial_bridge/config/protocol.yaml
 *   2. 重新跑 build_mock.sh（会重新生成 protocol.h/.c）
 *   3. 生成的 protocol.h 里会多出新的 on_receive_<Name>/send_<Name> 声明，
 *      在下面"业务消息回调"那一节照着实现就行，不用改这个文件其他地方
 *
 * 设计上没有用 Python 重新手搓一遍帧格式（旧版华工的 mock_stm32_ack.py 是这么
 * 做的），而是直接编译 auto_serial_bridge 生成的 protocol.c——这份 C 代码
 * 本来就是打算原样烧进真实 STM32 固件的，帧解析、校验、握手/心跳自动回包
 * 全部在里面实现好了（protocol_fsm_feed()），这个文件只需要实现一个
 * serial_write() 给它把字节发出去、再写个 main() 循环把收到的字节喂进去。
 * 这样协议细节只有一份来源（protocol.yaml + codegen.py），不会出现"文档和
 * 代码对不上"这种去年华工踩过的问题（见 0002 里助教那条记录）。
 */

/* cfmakeraw 是 glibc 扩展，-std=c11 严格模式下不会自动声明，得显式打开 */
#define _DEFAULT_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

#include "protocol.h"

static int serial_fd = -1;

/* protocol.c 里用到的外部依赖，由本文件实现 */
void serial_write(const uint8_t *data, uint16_t len)
{
    ssize_t written = write(serial_fd, data, len);
    if (written < 0 || (size_t)written != len) {
        fprintf(stderr, "[mock_stm32] serial_write 失败: %s\n", strerror(errno));
    }
}

/* ── 系统消息回调：握手/心跳的回包已经在 protocol_fsm_feed 里自动处理了，
 *    这几个钩子只是观察日志，不用也不应该在这里自己回包 ── */

void on_receive_Ack(const Packet_Ack *pkt)
{
    printf("[mock_stm32] 收到 Ack：acked_id=%u ack_seq=%u\n",
           (unsigned)pkt->acked_id, (unsigned)pkt->ack_seq);
    fflush(stdout);
}

void on_receive_Heartbeat(const Packet_Heartbeat *pkt)
{
    printf("[mock_stm32] 收到心跳 count=%u（已自动回包）\n", (unsigned)pkt->count);
    fflush(stdout);
}

void on_receive_Handshake(const Packet_Handshake *pkt)
{
    printf("[mock_stm32] 收到握手请求 protocol_hash=0x%08X（已自动回传本机哈希）\n",
           (unsigned)pkt->protocol_hash);
    fflush(stdout);
}

/* ── 业务消息回调：等协议定了，在这里加对应的 on_receive_<Name>/调用
 *    send_<Name>。目前 protocol.yaml 里 messages 是空的，所以生成的
 *    protocol.h 里除了上面三个系统消息，没有别的回调需要实现。 ── */

static int open_serial_port(const char *path, int baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY);
    if (fd < 0) {
        fprintf(stderr, "[mock_stm32] 打不开串口 %s: %s\n", path, strerror(errno));
        return -1;
    }

    speed_t speed;
    switch (baud) {
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 921600:  speed = B921600;  break;
        default:
            fprintf(stderr, "[mock_stm32] 不支持的波特率 %d（只支持标准档位）\n", baud);
            close(fd);
            return -1;
    }

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        fprintf(stderr, "[mock_stm32] tcgetattr 失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    cfmakeraw(&tty);
    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    /* 一次至少读 1 字节、不设超时：协议是逐字节喂进状态机的，阻塞读没问题 */
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        fprintf(stderr, "[mock_stm32] tcsetattr 失败: %s\n", strerror(errno));
        close(fd);
        return -1;
    }

    return fd;
}

int main(int argc, char **argv)
{
    const char *port = (argc > 1) ? argv[1] : "/dev/ttyACM0";
    int baud = (argc > 2) ? atoi(argv[2]) : 115200;

    serial_fd = open_serial_port(port, baud);
    if (serial_fd < 0) {
        return 1;
    }

    printf("[mock_stm32] 已打开 %s @ %d，等待上位机数据...\n", port, baud);
    fflush(stdout);

    uint8_t byte;
    ssize_t n;
    while ((n = read(serial_fd, &byte, 1)) >= 0) {
        if (n == 1) {
            protocol_fsm_feed(byte);
        }
    }

    fprintf(stderr, "[mock_stm32] 串口读取结束: %s\n", strerror(errno));
    close(serial_fd);
    return 1;
}
