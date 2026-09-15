#include <stdio.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <cstring>
#include <linux/input.h>
#include <linux/joystick.h>
#include <iostream>
#include <poll.h>
#include <cmath>
#include <cerrno>
#include <time.h>

// 说明：
//  - 为了支持手柄热插拔，js 设备断开时 read/poll 可能返回 EIO/ENODEV 或者 POLLHUP/POLLERR。
//  - 这里做“最小改动”：
//      1) 不再死等 read 负值（避免拔掉后一直卡住）。
//      2) 断开时自动 close，并周期性尝试重新 open（支持重新插回）。
//      3) 断开期间把 map 置为“中立”（避免继续沿用旧数据导致一直输出）。
//  - update() 对外依旧是阻塞式（内部用 poll + 轮询重连）。

using namespace std;

#define XBOX_TYPE_BUTTON 0x01
#define XBOX_TYPE_AXIS 0x02
#define XBOX_BUTTON_A 0x00
#define XBOX_BUTTON_B 0x01
#define XBOX_BUTTON_X 0x02
#define XBOX_BUTTON_Y 0x03
#define XBOX_BUTTON_LB 0x04
#define XBOX_BUTTON_RB 0x05
#define XBOX_BUTTON_START 0x07
#define XBOX_BUTTON_BACK 0x06
#define XBOX_BUTTON_HOME 0x08
#define XBOX_BUTTON_LO 0x09 /* 左摇杆按键 */
#define XBOX_BUTTON_RO 0x0a /* 右摇杆按键 */
#define XBOX_BUTTON_ON 0x01
#define XBOX_BUTTON_OFF 0x00

#define XBOX_AXIS_LX 0x00 /* 左摇杆X轴 */
#define XBOX_AXIS_LY 0x01 /* 左摇杆Y轴 */
#define XBOX_AXIS_RX 0x03 /* 右摇杆X轴 */
#define XBOX_AXIS_RY 0x04 /* 右摇杆Y轴 */
#define XBOX_AXIS_LT 0x02
#define XBOX_AXIS_RT 0x05
#define XBOX_AXIS_XX 0x06 /* 方向键X轴 */
#define XBOX_AXIS_YY 0x07 /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP -32767
#define XBOX_AXIS_VAL_DOWN 32767
#define XBOX_AXIS_VAL_LEFT -32767
#define XBOX_AXIS_VAL_RIGHT 32767

#define XBOX_AXIS_VAL_MIN -32767
#define XBOX_AXIS_VAL_MAX 32767
#define XBOX_AXIS_VAL_MID 0x00

typedef struct
{
    int time;
    int a;
    int b;
    int x;
    int y;
    int lb;
    int rb;
    int start;
    int back;
    int home;
    int lo;
    int ro;

    int lx;
    int ly;
    int rx;
    int ry;
    int lt;
    int rt;
    int xx;
    int yy;
} xbox_map_t;

class Xbox360Joystick
{
public:
    Xbox360Joystick() {};
    ~Xbox360Joystick()
    {
        closeDevice_();
    };

    bool initialize(void)
    {
        closeDevice_();

        xbox_map_t tmp;
        setNeutral_(tmp);

        if (!openAnyDevice_())
        {
            std::cout << "not found joystick!" << std::endl;
            return false;
        }

        std::cout << "has found joystick!" << std::endl;
        return true;
    }

    bool isConnected() const
    {
        return xbox_fd >= 0;
    }

    struct js_event update(xbox_map_t &joystick_map)
    {
        struct js_event js{};

        // 确保设备连接
        if (!ensureConnected_())
        {
            setNeutral_(joystick_map);
            return js;
        }

        struct pollfd pfd;
        pfd.fd = xbox_fd;
        pfd.events = POLLIN | POLLERR | POLLHUP;
        pfd.revents = 0;

        // 使用合理的超时时间
        int pr = poll(&pfd, 1, 1000);

        // 超时，没有新事件
        if (pr == 0)
        {
            return js;
        }

        if (pr < 0)
        {
            if (errno == EINTR)
                return js;

            closeDevice_();
            setNeutral_(joystick_map);
            return js;
        }

        if (pfd.revents & (POLLHUP | POLLERR | POLLNVAL))
        {
            closeDevice_();
            setNeutral_(joystick_map);
            return js;
        }

        // 有数据可读
        if (pfd.revents & POLLIN)
        {
            int rc = xbox_map_read(xbox_fd, &joystick_map, js);
            if (rc >= 0)
            {
                return js;
            }

            if (isDisconnectErrno_(errno))
            {
                closeDevice_();
                setNeutral_(joystick_map);
            }
        }

        return js;
    }

    void printXboxMap(const xbox_map_t &map)
    {
        std::cout << "=== Xbox Controller State ===" << std::endl;
        std::cout << "time:\t" << map.time << std::endl;
        std::cout << "a:\t" << map.a << std::endl;
        std::cout << "b:\t" << map.b << std::endl;
        std::cout << "x:\t" << map.x << std::endl;
        std::cout << "y:\t" << map.y << std::endl;
        std::cout << "lb:\t" << map.lb << std::endl;
        std::cout << "rb:\t" << map.rb << std::endl;
        std::cout << "start:\t" << map.start << std::endl;
        std::cout << "back:\t" << map.back << std::endl;
        std::cout << "home:\t" << map.home << std::endl;
        std::cout << "lo:\t" << map.lo << std::endl;
        std::cout << "ro:\t" << map.ro << std::endl;
        std::cout << "lx:\t" << map.lx << std::endl;
        std::cout << "ly:\t" << map.ly << std::endl;
        std::cout << "rx:\t" << map.rx << std::endl;
        std::cout << "ry:\t" << map.ry << std::endl;
        std::cout << "lt:\t" << map.lt << std::endl;
        std::cout << "rt:\t" << map.rt << std::endl;
        std::cout << "xx:\t" << map.xx << std::endl;
        std::cout << "yy:\t" << map.yy << std::endl;
    }

private:
    static bool isDisconnectErrno_(int e)
    {
        // 不同内核/驱动在拔插时可能返回不同 errno
        return (e == ENODEV) || (e == ENXIO) || (e == EIO) || (e == EBADF);
    }

    static void setNeutral_(xbox_map_t &m)
    {
        m.lt = -32767;
        m.rt = -32767;
        std::memset(&m, 0, sizeof(xbox_map_t));
    }

    void closeDevice_()
    {
        if (xbox_fd >= 0)
        {
            close(xbox_fd);
            xbox_fd = -1;
        }
    }

    bool openAnyDevice_()
    {
        for (int i = 0; i < 8; ++i)
        {
            char path[64];
            std::snprintf(path, sizeof(path), "/dev/input/js%d", i);
            int fd = xbox_open(path);
            if (fd >= 0)
            {
                xbox_fd = fd;
                return true;
            }
        }
        return false;
    }

    bool ensureConnected_()
    {
        if (xbox_fd >= 0)
            return true;

        // 设备断开时立即尝试重连
        return openAnyDevice_();
    }

    static uint64_t nowMs_()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000ULL + static_cast<uint64_t>(ts.tv_nsec) / 1000000ULL;
    }

    int xbox_open(const char *file_name)
    {
        int fd = open(file_name, O_RDONLY | O_NONBLOCK);
        if (fd < 0)
            return -1;
        return fd;
    }

    int xbox_map_read(int xbox_fd, xbox_map_t *map, struct js_event &js)
    {
        int len, type, number, value;

        len = read(xbox_fd, &js, sizeof(struct js_event));
        if (len < 0)
        {
            return -1;
        }

        type = js.type;
        number = js.number;
        value = js.value;

        map->time = js.time;

        if (type == JS_EVENT_BUTTON)
        {
            switch (number)
            {
            case XBOX_BUTTON_A:
                map->a = value;
                break;

            case XBOX_BUTTON_B:
                map->b = value;
                break;

            case XBOX_BUTTON_X:
                map->x = value;
                break;

            case XBOX_BUTTON_Y:
                map->y = value;
                break;

            case XBOX_BUTTON_LB:
                map->lb = value;
                break;

            case XBOX_BUTTON_RB:
                map->rb = value;
                break;

            case XBOX_BUTTON_START:
                map->start = value;
                break;

            case XBOX_BUTTON_BACK:
                map->back = value;
                break;

            case XBOX_BUTTON_HOME:
                map->home = value;
                break;

            case XBOX_BUTTON_LO:
                map->lo = value;
                break;

            case XBOX_BUTTON_RO:
                map->ro = value;
                break;

            default:
                break;
            }
        }
        else if (type == JS_EVENT_AXIS)
        {
            switch (number)
            {
            case XBOX_AXIS_LX:
                map->lx = value;
                break;

            case XBOX_AXIS_LY:
                map->ly = value;
                break;

            case XBOX_AXIS_RX:
                map->rx = value;
                break;

            case XBOX_AXIS_RY:
                map->ry = value;
                break;

            case XBOX_AXIS_LT:
                map->lt = value;
                break;

            case XBOX_AXIS_RT:
                map->rt = value;
                break;

            case XBOX_AXIS_XX:
                map->xx = value;
                break;

            case XBOX_AXIS_YY:
                map->yy = value;
                break;

            default:
                break;
            }
        }

        return len;
    }

private:
    int xbox_fd = -1;
};