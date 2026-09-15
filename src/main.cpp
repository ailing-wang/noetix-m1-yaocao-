#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <cstring>
#include <atomic>
#include <thread>
#include <functional>
#include <csignal>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <array>
#include <queue>
#include <iomanip>
#include <fstream>
#include <unistd.h>
#include <arpa/inet.h>
#include <modbus/modbus.h>
#include "PeriodicTask.h"
#include "Xbox360.hpp"
#include "Serializer.h"
#include "Modbus.h"
#include "version_lib.hpp"
#include <stdexcept>
#include "Logger.h"
#include "Timer.h"
#include "Keybuffer.h"
#include "Myutility.hpp"

// 全局变量
static std::atomic<bool> g_is_killed{false};

// 方向向量配置
static double g_direction_vector[] =
    {1, 1, 1, 1, 1, 1, 1,
     1, 1, 1, 1, 1, 1, 1};

int32_t twosComplementToInt(uint32_t value, int bits)
{
    // 参数检查（可选）
    if (bits <= 0 || bits > 32)
    {
        return 0; // 或抛出异常
    }

    // 提取低 bits 位的有效数值
    uint32_t mask = (bits == 32) ? 0xFFFFFFFF : (1u << bits) - 1;
    uint32_t low = value & mask;

    // 检查最高位（符号位）
    if (low >> (bits - 1))
    {
        // 负数：低 bits 位的值减去 2^bits
        // 使用 unsigned long long 避免移位溢出（1ULL << bits 在 bits≤32 时安全）
        unsigned long long full = 1ULL << bits;
        return static_cast<int32_t>(low - full);
    }
    else
    {
        // 正数直接返回
        return static_cast<int32_t>(low);
    }
}
static double
mapToPiRangeFast(int value)
{
    return value * M_PI / 32767.0;
}

static double mapToPiRange(int value)
{
    const int MULTI_LEVEL_MIN = 0;
    const int MULTI_LEVEL_MAX = 4095;
    const int MULTI_LEVEL_CENTER = 2048;
    const double PI = 3.14159265358979323846;

    // 确保输入值在有效范围内
    int clamped_value = value;
    if (clamped_value < MULTI_LEVEL_MIN)
        clamped_value = MULTI_LEVEL_MIN;
    if (clamped_value > MULTI_LEVEL_MAX)
        clamped_value = MULTI_LEVEL_MAX;

    // 将值转换为相对于中心点的偏移量
    double offset = clamped_value - MULTI_LEVEL_CENTER;

    // 计算映射比例：整个范围对应 2*pi
    double scale = (2.0 * PI) / (MULTI_LEVEL_MAX - MULTI_LEVEL_MIN);

    // 返回映射后的角度值
    return offset * scale;
}

// UDP客户端类
class UDPClient
{
private:
    int client_sockfd_;
    struct sockaddr_in server_addr_;
    std::atomic<bool> running_;

    static constexpr const char *SERVER_IP = "192.168.127.50";
    static constexpr int PORT = 8888;

public:
    UDPClient() : client_sockfd_(-1), running_(true)
    {
        memset(&server_addr_, 0, sizeof(server_addr_));
    }

    ~UDPClient()
    {
        cleanup();
    }

    bool initialize()
    {
        // 创建UDP socket
        client_sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (client_sockfd_ < 0)
        {
            std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
            return false;
        }

        // 设置服务器地址
        server_addr_.sin_family = AF_INET;
        server_addr_.sin_port = htons(PORT);

        if (inet_pton(AF_INET, SERVER_IP, &server_addr_.sin_addr) <= 0)
        {
            close(client_sockfd_);
            client_sockfd_ = -1;
            LogError("UDP Client initialized Error. Invalid address: {}.", SERVER_IP);
            return false;
        }

        LogWarn("UDP Client initialized. Server: {} : {}", SERVER_IP, PORT);
        return true;
    }

    void sendData(const uint8_t *data, size_t size)
    {
        if (client_sockfd_ < 0)
        {
            std::cerr << "Socket not initialized" << std::endl;
            return;
        }

        ssize_t sent_len = sendto(client_sockfd_, data, size, 0,
                                  reinterpret_cast<const sockaddr *>(&server_addr_),
                                  sizeof(server_addr_));

        if (sent_len < 0)
        {
            std::cerr << "Sendto failed: " << strerror(errno) << std::endl;
        }
    }

    void cleanup()
    {
        running_ = false;

        if (client_sockfd_ >= 0)
        {
            close(client_sockfd_);
            client_sockfd_ = -1;
        }

        LogError("Client cleaned up");
    }

    bool isRunning() const { return running_; }
};

// 速度控制参数
struct VelocityControlParams
{
    int32_t hertz = 20;        // 控制周期 hz
    int32_t delay = 100;       // 控制延时 ms
    float max_linear = 0.25f;  // 最大线速度
    float max_angular = 0.25f; // 最大角速度
    float zero_time = 1.5f;    // 减速归零时间s
};

// 摇杆状态结构体
struct JoystickState
{
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float button_stop = 0.0f;
};

class VelocityController
{
private:
    VelocityControlParams params_;
    float current_linear_ = 0.0f;
    float current_angular_ = 0.0f;
    bool is_running_ = false;

public:
    VelocityController(const VelocityControlParams &params = {})
        : params_(params) {}

    void setVelocity(float linear, float angular)
    {
        current_linear_ = linear;
        current_angular_ = angular;
    }

    void getCurrentVelocity(float &linear, float &angular) const
    {
        linear = current_linear_;
        angular = current_angular_;
    }

    void handleJoystickInput(const JoystickState &joystick)
    {
        float linear_x = joystick.y * 0.00001f; // 前后移动
        float linear_y = joystick.x * 0.00001f; // 左右移动
        float angular = joystick.z * 0.00001f;  // 旋转

        // 限制最大速度
        if (linear_x > params_.max_linear)
        {
            linear_x = params_.max_linear;
        }
        else if (linear_x < -params_.max_linear)
        {
            linear_x = -params_.max_linear;
        }

        if (linear_y > params_.max_linear)
        {
            linear_y = params_.max_linear;
        }
        else if (linear_y < -params_.max_linear)
        {
            linear_y = -params_.max_linear;
        }

        if (angular > params_.max_angular)
        {
            angular = params_.max_angular;
        }
        else if (angular < -params_.max_angular)
        {
            angular = -params_.max_angular;
        }

        // printf("摇杆状态: X=%.2f, Y=%.2f, Z=%.2f\n", linear_x, linear_y, angular);
    }

    void startControlLoop()
    {
        is_running_ = true;
    }

    void stopControlLoop()
    {
        is_running_ = false;
    }

    bool isRunning() const
    {
        return is_running_;
    }
};

// 机器人控制器
class RoboticArmController
{
private:
    KeyBuffer key_buffer_;
    VelocityControlParams params_;
    VelocityController controller_;
    Xbox360Joystick joystick_;
    xbox_map_t joystick_map_;

    ModbusManager modbus_mgr_;

    ModbusBus *bus0_ptr_, *bus1_ptr_, *bus2_ptr_, *bus3_ptr_;
    bool bus0_initialized_, bus1_initialized_, bus2_initialized_, bus3_initialized_;
    bool bus0_running, bus1_running, bus2_running, bus3_running;

    UDPClient udp_client_;
    static bool joystick_error_;

    static RoboticArmController *instance_;

    PeriodicTaskManager task_manager_;
    PeriodicFunction modbus_bus1_task_, modbus_bus2_task_, modbus_bus3_task_, modbus_bus4_task_;
    PeriodicFunction joystick_task_;
    PeriodicFunction upd_task_;

    static noetix_upd_in_t control_data_;
    uint64_t seq = 0u;
    static bool prev_home_pressed;
    static std::atomic<bool> zero_in_progress;

public:
    RoboticArmController() : controller_(params_),

                             bus0_ptr_(nullptr), bus1_ptr_(nullptr), bus2_ptr_(nullptr), bus3_ptr_(nullptr),

                             bus0_initialized_(false), bus1_initialized_(false), bus2_initialized_(false), bus3_initialized_(false),

                             modbus_bus1_task_(&task_manager_, 0.004f, "modbus1", []()
                                               { RoboticArmController::staticUpdateModbus1(); }, 99, 0),

                             modbus_bus2_task_(&task_manager_, 0.004f, "modbus2", []()
                                               { RoboticArmController::staticUpdateModbus2(); }, 99, 1),

                             modbus_bus3_task_(&task_manager_, 0.004f, "modbus3", []()
                                               { RoboticArmController::staticUpdateModbus3(); }, 99, 2),

                             modbus_bus4_task_(&task_manager_, 0.004f, "modbus4", []()
                                               { RoboticArmController::staticUpdateModbus4(); }, 99, 3),

                             joystick_task_(&task_manager_, 0.002f, "joystickcontrol", []()
                                            { RoboticArmController::staticJoystickControl(); }, 99, 4),

                             upd_task_(&task_manager_, 0.002f, "updclient", []()
                                       { RoboticArmController::staticSendControlData(); }, 99, 5)

    {
        instance_ = this;

        memset(&joystick_map_, 0, sizeof(joystick_map_));
    }

    static RoboticArmController *getInstance()
    {
        return instance_;
    }

    bool initializeJoystick()
    {
        return joystick_.initialize();
    }

    auto safeAddBus(const SerialConfig &cfg)
    {
        while (true)
        {
            // 尝试打开设备文件，确认可以访问
            int fd = open(cfg.port.c_str(), O_RDWR | O_NOCTTY);
            if (fd >= 0)
            {
                close(fd); // 成功打开，关闭后再交给 addBus
                try
                {
                    auto bus_id = modbus_mgr_.addBus(cfg);
                    return bus_id;
                }
                catch (const std::exception &e)
                {
                    LogError("Failed to add bus {}: {}, retrying in 1s...", cfg.port, e.what());
                    std::cerr << "Failed to add bus " << cfg.port << ": " << e.what() << ", retrying in 1s..." << std::endl;
                }
            }
            else
            {
                LogError("Device {} not ready: {}, retrying in 1s...", cfg.port, strerror(errno));
                std::cerr << "Device " << cfg.port << " not ready, retrying in 1s..." << std::endl;
            }
            sleep(1);
        }
    }

    void initModbusBuses()
    {
        safeAddBus(SerialConfig{.port = "/dev/noetix_arm_l1", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});

        safeAddBus(SerialConfig{.port = "/dev/noetix_arm_l2", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});

        safeAddBus(SerialConfig{.port = "/dev/noetix_arm_r1", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});

        safeAddBus(SerialConfig{.port = "/dev/noetix_arm_r2", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});
    }

    bool initialize()
    {
        // 检查版本库
        VersionSpace::VersionLib versionLib;
        VersionSpace::VersionLib::printInfo();

        if (!versionLib.isAvailable())
        {
            std::cerr << "Library is not available!" << std::endl;
            return false;
        }

        LogInfo("Library is available and working!");

        LogWarn("initialize start!");

        initModbusBuses();

        sleep(5);

#if 1
        // bus0
        auto id0 = modbus_mgr_.addBus(SerialConfig{.port = "/dev/noetix_arm_l1", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});
        auto id1 = modbus_mgr_.addBus(SerialConfig{.port = "/dev/noetix_arm_l2", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});
        auto id2 = modbus_mgr_.addBus(SerialConfig{.port = "/dev/noetix_arm_r1", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});
        auto id3 = modbus_mgr_.addBus(SerialConfig{.port = "/dev/noetix_arm_r2", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});

        LogWarn("open start!");

        bus0_ptr_ = &(modbus_mgr_.bus(id0));
        bus1_ptr_ = &(modbus_mgr_.bus(id1));
        bus2_ptr_ = &(modbus_mgr_.bus(id2));
        bus3_ptr_ = &(modbus_mgr_.bus(id3));

        // 初始化bus0
        if (bus0_ptr_)
        {
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 1, .start_address = 0x101, .register_count = 1, .description = "Left arm1"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 2, .start_address = 0x101, .register_count = 1, .description = "Left arm2"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 3, .start_address = 0x101, .register_count = 1, .description = "Left arm3"});
            bus0_ptr_->finalizeBuffer();
            bus0_initialized_ = true;
            bus0_running = true;
        }
        else
        {
            LogWarn("Failed to initialize bus0!");
            return false;
        }
        // 初始化bus1
        if (bus1_ptr_)
        {
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 4, .start_address = 0x101, .register_count = 1, .description = "Left arm4"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 5, .start_address = 0x101, .register_count = 1, .description = "Left arm5"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 6, .start_address = 0x101, .register_count = 1, .description = "Left arm6"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 7, .start_address = 0x101, .register_count = 1, .description = "Left arm7"});
            bus1_ptr_->finalizeBuffer();
            bus1_initialized_ = true;
            bus1_running = true;
        }
        else
        {
            LogWarn("Failed to initialize bus1!");
            return false;
        }

        // // 初始化bus2
        if (bus2_ptr_)
        {
            bus2_ptr_->addDevice(DeviceConfig{.slave_id = 1, .start_address = 0x101, .register_count = 1, .description = "Right arm1"});
            bus2_ptr_->addDevice(DeviceConfig{.slave_id = 2, .start_address = 0x101, .register_count = 1, .description = "Right arm2"});
            bus2_ptr_->addDevice(DeviceConfig{.slave_id = 3, .start_address = 0x101, .register_count = 1, .description = "Right arm3"});
            bus2_ptr_->finalizeBuffer();
            bus2_initialized_ = true;
            bus2_running = true;
        }
        else
        {
            LogWarn("Failed to initialize bus2!");
            return false;
        }

        // 初始化bus3
        if (bus3_ptr_)
        {
            bus3_ptr_->addDevice(DeviceConfig{.slave_id = 4, .start_address = 0x101, .register_count = 1, .description = "Right arm4"});
            bus3_ptr_->addDevice(DeviceConfig{.slave_id = 5, .start_address = 0x101, .register_count = 1, .description = "Right arm5"});
            bus3_ptr_->addDevice(DeviceConfig{.slave_id = 6, .start_address = 0x101, .register_count = 1, .description = "Right arm6"});
            bus3_ptr_->addDevice(DeviceConfig{.slave_id = 7, .start_address = 0x101, .register_count = 1, .description = "Right arm7"});
            bus3_ptr_->finalizeBuffer();
            bus3_initialized_ = true;
            bus3_running = true;
        }
        else
        {
            LogWarn("Failed to initialize bus3!");
            return false;
        }
#else
        auto id0 = modbus_mgr_.addBus(SerialConfig{.port = "/dev/ttyUSB0", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});
        auto id1 = modbus_mgr_.addBus(SerialConfig{.port = "/dev/ttyUSB1", .baudrate = 115200, .parity = 'N', .data_bit = 8, .stop_bit = 1});
        bus0_ptr_ = &(modbus_mgr_.bus(id0));
        bus1_ptr_ = &(modbus_mgr_.bus(id1));

        // 初始化bus0
        if (bus0_ptr_)
        {
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 1, .start_address = 0x101, .register_count = 1, .description = "Left arm1"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 2, .start_address = 0x101, .register_count = 1, .description = "Left arm2"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 3, .start_address = 0x101, .register_count = 1, .description = "Left arm3"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 4, .start_address = 0x101, .register_count = 1, .description = "Left arm4"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 5, .start_address = 0x101, .register_count = 1, .description = "Left arm5"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 6, .start_address = 0x101, .register_count = 1, .description = "Left arm6"});
            bus0_ptr_->addDevice(DeviceConfig{.slave_id = 7, .start_address = 0x101, .register_count = 1, .description = "Left arm7"});
            bus0_ptr_->finalizeBuffer();
            bus0_initialized_ = true;
        }
        else
        {
            std::cerr << "Failed to initialize bus0!" << std::endl;
            return false;
        }

        // 初始化bus1
        if (bus1_ptr_)
        {
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 1, .start_address = 0x101, .register_count = 1, .description = "Right arm1"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 2, .start_address = 0x101, .register_count = 1, .description = "Right arm2"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 3, .start_address = 0x101, .register_count = 1, .description = "Right arm3"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 4, .start_address = 0x101, .register_count = 1, .description = "Right arm4"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 5, .start_address = 0x101, .register_count = 1, .description = "Right arm5"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 6, .start_address = 0x101, .register_count = 1, .description = "Right arm6"});
            bus1_ptr_->addDevice(DeviceConfig{.slave_id = 7, .start_address = 0x101, .register_count = 1, .description = "Right arm7"});

            bus1_ptr_->finalizeBuffer();
            bus1_initialized_ = true;
        }
        else
        {
            std::cerr << "Failed to initialize bus1!" << std::endl;
            return false;
        }
#endif

        // 初始化UDP客户端
        if (!udp_client_.initialize())
        {
            LogError("Failed to initialize UDP client!");
            return false;
        }

        // 初始化手柄
        if (!joystick_.initialize())
        {
            RoboticArmController::joystick_error_ = true;
            LogError("Failed to initialize joystick!");
            return false;
        }

        // 启动速度控制器
        controller_.startControlLoop();

        LogWarn("initialize finished!");

        return true;
    }

    void joystickControl()
    {
        struct js_event js = joystick_.update(joystick_map_);

        // 检查连接状态
        if (!joystick_.isConnected())
        {
            LogError("The joystick has disconnected and is currently being reconnected.");
            if (joystick_.initialize())
            {
                LogWarn("The joystick has been successfully reconnected.");
                RoboticArmController::joystick_error_ = false;
            }
            else
            {
                LogError("Failed to reconnect the joystick.");
                RoboticArmController::joystick_error_ = true;
            }
        }

        // 处理有效事件
        if (js.type != 0)
        {
            // using CI = RoboticArmController::ControlIndex;
            // joystick_.printXboxMap(joystick_map_);
            mapJoystickToBuffer(js);
        }
    }

    void updateModbus1()
    {
        // 统计每秒调用次数
        static int call_count = 0;
        static auto last_time = std::chrono::steady_clock::now();

        ++call_count;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed_ms >= 1000)
        {
            // std::cout << "[updateModbus1] calls in last 1s: " << call_count << std::endl;
            call_count = 0;
            last_time = now;
        }

        if (!bus0_running)
        {
            return;
        }

        if (bus0_initialized_ && bus0_ptr_)
        {
            bus0_ptr_->readAll();
        }

        int i = 0;
        for (i = 0; i < bus0_ptr_->deviceCount(); i++)
        {
            if (!bus0_ptr_->buffer_state_[i])
            {
                bus0_ptr_->bus_state_ = false;
                break;
            }
        }
        if (i == bus0_ptr_->deviceCount())
        {
            bus0_ptr_->bus_state_ = true;
        }
        // auto data_pair = bus0_ptr_->view(0);
        // auto data_ptr = data_pair.first;
        // auto data_size = data_pair.second;
        // logger_yc_curveTar1.append(*data_ptr);
        // std::cout << twosComplementToInt(*data_ptr, 16) << std::endl;
    }

    void updateModbus2()
    {
        // 统计每秒调用次数
        static int call_count = 0;
        static auto last_time = std::chrono::steady_clock::now();

        ++call_count;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed_ms >= 1000)
        {
            // std::cout << "[updateModbus2] calls in last 1s: " << call_count << std::endl;
            call_count = 0;
            last_time = now;
        }

        if (!bus1_running)
        {
            return;
        }

        if (bus1_initialized_ && bus1_ptr_)
        {
            bus1_ptr_->readAll();
        }

        int i = 0;
        for (i = 0; i < bus1_ptr_->deviceCount(); i++)
        {
            if (!bus1_ptr_->buffer_state_[i])
            {
                bus1_ptr_->bus_state_ = false;
                break;
            }
        }
        if (i == bus1_ptr_->deviceCount())
        {
            bus1_ptr_->bus_state_ = true;
        }
        // auto data_pair = bus0_ptr_->view(0);
        // auto data_ptr = data_pair.first;
        // auto data_size = data_pair.second;
        // logger_yc_curveTar1.append(*data_ptr);
        // std::cout << twosComplementToInt(*data_ptr, 16) << std::endl;
    }

    void updateModbus3()
    {
        // 统计每秒调用次数
        static int call_count = 0;
        static auto last_time = std::chrono::steady_clock::now();

        ++call_count;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed_ms >= 1000)
        {
            // std::cout << "[updateModbus3] calls in last 1s: " << call_count << std::endl;
            call_count = 0;
            last_time = now;
        }

        if (!bus2_running)
        {
            return;
        }

        if (bus2_initialized_ && bus2_ptr_)
        {
            bus2_ptr_->readAll();
        }

        int i = 0;
        for (i = 0; i < bus2_ptr_->deviceCount(); i++)
        {
            if (!bus2_ptr_->buffer_state_[i])
            {
                bus2_ptr_->bus_state_ = false;
                break;
            }
        }
        if (i == bus2_ptr_->deviceCount())
        {
            bus2_ptr_->bus_state_ = true;
        }
    }

    void updateModbus4()
    {
        // 统计每秒调用次数
        static int call_count = 0;
        static auto last_time = std::chrono::steady_clock::now();

        ++call_count;
        auto now = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_time).count();

        if (elapsed_ms >= 1000)
        {
            // std::cout << "[updateModbus4] calls in last 1s: " << call_count << std::endl;
            call_count = 0;
            last_time = now;
        }

        if (!bus3_running)
        {
            return;
        }

        if (bus3_initialized_ && bus3_ptr_)
        {
            bus3_ptr_->readAll();
        }

        int i = 0;
        for (i = 0; i < bus3_ptr_->deviceCount(); i++)
        {
            if (!bus3_ptr_->buffer_state_[i])
            {
                bus3_ptr_->bus_state_ = false;
                break;
            }
        }
        if (i == bus3_ptr_->deviceCount())
        {
            bus3_ptr_->bus_state_ = true;
        }
    }

    void buildAndSendControlPacket(uint64_t seq, enum SerializationType type)
    {
        control_data_.sequence_number = seq;
        control_data_.type = type; // 设置机械臂位置
        for (int i = 0; i < 3; ++i)
        {
            auto [data_ptr, data_size] = bus0_ptr_->view(i);
            int value = twosComplementToInt(*data_ptr, 16);
            control_data_.left_arm_pos[i] = mapToPiRange(value) * g_direction_vector[i];
        }
        for (int i = 0; i < 4; ++i)
        {
            auto [data_ptr, data_size] = bus1_ptr_->view(i);
            int value = twosComplementToInt(*data_ptr, 16);
            control_data_.left_arm_pos[i + 3] = mapToPiRange(value) * g_direction_vector[i + 3];
        }
        for (int i = 0; i < 3; ++i)
        {
            auto [data_ptr, data_size] = bus2_ptr_->view(i);
            int value = twosComplementToInt(*data_ptr, 16);
            control_data_.right_arm_pos[i] = mapToPiRange(value) * g_direction_vector[i + 7];
        }
        for (int i = 0; i < 4; ++i)
        {
            auto [data_ptr, data_size] = bus3_ptr_->view(i);
            int value = twosComplementToInt(*data_ptr, 16);
            control_data_.right_arm_pos[i + 3] = mapToPiRange(value) * g_direction_vector[i + 10];
        }
        // 设置手柄数据 - 需要手动复制字段
        control_data_.map.time = joystick_map_.time;
        control_data_.map.a = joystick_map_.a;
        control_data_.map.b = joystick_map_.b;
        control_data_.map.x = joystick_map_.x;
        control_data_.map.y = joystick_map_.y;
        control_data_.map.lb = joystick_map_.lb;
        control_data_.map.rb = joystick_map_.rb;
        control_data_.map.start = joystick_map_.start;
        control_data_.map.back = joystick_map_.back;
        control_data_.map.home = joystick_map_.home;
        control_data_.map.lo = joystick_map_.lo;
        control_data_.map.ro = joystick_map_.ro;
        control_data_.map.lx = joystick_map_.lx;
        control_data_.map.ly = joystick_map_.ly;
        control_data_.map.rx = joystick_map_.rx;
        control_data_.map.ry = joystick_map_.ry;
        control_data_.map.lt = joystick_map_.lt;
        control_data_.map.rt = joystick_map_.rt;
        control_data_.map.xx = joystick_map_.xx;
        control_data_.map.yy = joystick_map_.yy;

        // 添加夹爪控制
        control_data_.left_arm_pos[7] = mapToPiRangeFast(joystick_map_.ly);
        control_data_.right_arm_pos[7] = mapToPiRangeFast(joystick_map_.ry);

        // 打印调试信息
        printNoetixUpdIn(&control_data_);

        // 序列化并发送
        size_t serialized_size;
        uint8_t *serialized_data = NoetixSerializer::serialize(&control_data_, serialized_size);
        if (serialized_data)
        {
            udp_client_.sendData(serialized_data, NoetixSerializer::serializedSize());
            delete[] serialized_data;
        }
        else
        {
            LogError("Failed to serialize control data!");
        }
    }

    void zeroPosition()
    {
        constexpr int delay_us = 1250;

        auto SetBus = [&](ModbusBus *bus_ptr,
                          const std::string &bus_name,
                          uint16_t reg_addr,
                          uint16_t value)
        {
            if (!bus_ptr)
                return;

            for (std::size_t i = 0; i < bus_ptr->deviceCount(); ++i)
            {
                try
                {
                    uint16_t readback = 0;

                    // 写入停止值
                    bus_ptr->writeDevice(i, reg_addr, &value, 1, true);
                    usleep(delay_us);

                    // 回读同一寄存器
                    bus_ptr->readRegister(i, reg_addr, &readback, 1, true);

                    std::cout << bus_name
                              << " device_index: " << i
                              << " readback[0x" << std::hex << reg_addr << std::dec << "] = "
                              << readback
                              << std::endl;
                }
                catch (const std::exception &e)
                {
                    std::cerr << "Error zeroing " << bus_name
                              << " device_index " << i
                              << ": " << e.what() << std::endl;
                }
            }
        };
        SetBus(bus0_ptr_, "bus0", 0x11, 4);
        SetBus(bus1_ptr_, "bus1", 0x11, 4);
        SetBus(bus2_ptr_, "bus2", 0x11, 4);
        SetBus(bus3_ptr_, "bus3", 0x11, 4);
        usleep(500000);

        SetBus(bus0_ptr_, "bus0", 0x12, 1);
        SetBus(bus1_ptr_, "bus1", 0x12, 1);
        SetBus(bus2_ptr_, "bus2", 0x12, 1);
        SetBus(bus3_ptr_, "bus3", 0x12, 1);
        usleep(500000);

        SetBus(bus0_ptr_, "bus0", 0x80, 2048);
        SetBus(bus1_ptr_, "bus1", 0x80, 2048);
        SetBus(bus2_ptr_, "bus2", 0x80, 2048);
        SetBus(bus3_ptr_, "bus3", 0x80, 2048);
        usleep(500000);

        SetBus(bus0_ptr_, "bus0", 0x80, 2048);
        SetBus(bus1_ptr_, "bus1", 0x80, 2048);
        SetBus(bus2_ptr_, "bus2", 0x80, 2048);
        SetBus(bus3_ptr_, "bus3", 0x80, 2048);
        usleep(500000);

        SetBus(bus0_ptr_, "bus0", 0x81, 0);
        SetBus(bus1_ptr_, "bus1", 0x81, 0);
        SetBus(bus2_ptr_, "bus2", 0x81, 0);
        SetBus(bus3_ptr_, "bus3", 0x81, 0);
        usleep(500000);
    }

    void run()
    {
        zeroPosition();

        modbus_bus1_task_.start();
        modbus_bus2_task_.start();
        modbus_bus3_task_.start();
        modbus_bus4_task_.start();
        usleep(10000 * 100);

        joystick_task_.start();
        usleep(10000 * 100);

        upd_task_.start();

        while (!g_is_killed)
        {
            usleep(1000 * 500);
            task_manager_.printStatusOfSlowTasks();
        }
    }

    static bool modbus_error_check(void)
    {
        if (instance_->bus0_ptr_->error_ || instance_->bus1_ptr_->error_ || instance_->bus2_ptr_->error_ || instance_->bus3_ptr_->error_)
        {
            ModbusBus::modbus_error_ = true;
        }
        else
        {
            ModbusBus::modbus_error_ = false;
        }
        return ModbusBus::modbus_error_;
    }

    void cleanup()
    {
        udp_client_.cleanup();
    }

    static void staticUpdateModbus1()
    {
        if (instance_)
        {
            instance_->updateModbus1();
        }
    }

    static void staticUpdateModbus2()
    {
        if (instance_)
        {
            instance_->updateModbus2();
        }
    }

    static void staticUpdateModbus3()
    {
        if (instance_)
        {
            instance_->updateModbus3();
        }
    }

    static void staticUpdateModbus4()
    {
        if (instance_)
        {
            instance_->updateModbus4();
        }
    }

    static void staticJoystickControl()
    {
        if (RoboticArmController::joystick_error_)
        {
            if (instance_ && instance_->initializeJoystick())
            {
                RoboticArmController::joystick_error_ = false;
            }
            else
            {
                std::this_thread::sleep_for(std::chrono::seconds(2));
            }
        }

        if (instance_ && !RoboticArmController::joystick_error_)
        {
            instance_->joystickControl();
        }
    }

    static void staticSendControlData()
    {
        // ---- Home 按钮状态 ----
        bool cur_home_pressed = (control_data_.map.home == 1);
        static SerializationType current_send_type = SERIAL_HEARTBEAT;

        // ---- 检查组合按键 ----
        bool home_rt_combo = (cur_home_pressed && control_data_.map.rt == 32767);
        bool home_lt_combo = (cur_home_pressed && control_data_.map.lt == 32767);

        // ---- 组合按键触发逻辑 ----
        if (home_rt_combo && current_send_type != SERIAL_DATA)
        {
            current_send_type = SERIAL_DATA; // Home + RT
            LogWarn("Home + RT long press detected: locking send type to SERIAL_DATA");

            // ---- 启动归零线程 ----
            if (!zero_in_progress.load())
            {
                zero_in_progress.store(true);
                instance_->bus0_running = false;
                instance_->bus1_running = false;
                instance_->bus2_running = false;
                instance_->bus3_running = false;

                sleep(1); // 确保正在进行的Modbus操作有时间完成

                std::thread([inst = instance_, &zero_in_progress]()
                            {
                            while (inst->bus0_running || inst->bus1_running ||
                                   inst->bus2_running || inst->bus3_running)
                            {
                                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            }

                            inst->zeroPosition();

                            inst->bus0_running = true;
                            inst->bus1_running = true;
                            inst->bus2_running = true;
                            inst->bus3_running = true;

                            LogInfo("Reset sequence completed in background thread.");
                            zero_in_progress.store(false); })
                    .detach();
            }
        }
        else if (home_lt_combo && current_send_type != SERIAL_ERROR)
        {
            current_send_type = SERIAL_ERROR; // Home + LT
            LogWarn("Home + LT long press detected: locking send type to SERIAL_ERROR");

            // ---- 启动归零线程 ----
            if (!zero_in_progress.load())
            {
                zero_in_progress.store(true);
                instance_->bus0_running = false;
                instance_->bus1_running = false;
                instance_->bus2_running = false;
                instance_->bus3_running = false;

                sleep(1); // 确保正在进行的Modbus操作有时间完成

                std::thread([inst = instance_, &zero_in_progress]()
                            {
                            while (inst->bus0_running || inst->bus1_running ||
                                   inst->bus2_running || inst->bus3_running)
                            {
                                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                            }

                            inst->zeroPosition();

                            inst->bus0_running = true;
                            inst->bus1_running = true;
                            inst->bus2_running = true;
                            inst->bus3_running = true;

                            LogInfo("Reset sequence completed in background thread.");
                            zero_in_progress.store(false); })
                    .detach();
            }
        }
        // ---- 没有组合按键，保持当前发送类型不变 ----

        prev_home_pressed = cur_home_pressed;

        // ---- 控制数据发送 ----
        if (instance_)
        {
            if (joystick_error_ || modbus_error_check())
            {
                instance_->buildAndSendControlPacket(instance_->seq, SERIAL_ERROR);
            }
            else
            {
                if (zero_in_progress.load())
                {
                    instance_->buildAndSendControlPacket(instance_->seq, SERIAL_ERROR);
                }
                else
                {
                    switch (current_send_type)
                    {
                    case SERIAL_HEARTBEAT:
                        instance_->buildAndSendControlPacket(instance_->seq, SERIAL_HEARTBEAT);
                        break;
                    case SERIAL_DATA:
                        instance_->buildAndSendControlPacket(instance_->seq, SERIAL_DATA);
                        break;
                    case SERIAL_ERROR:
                        instance_->buildAndSendControlPacket(instance_->seq, SERIAL_ERROR);
                        break;
                    }
                }
            }
        }

        instance_->seq++;
    }

    void
    syncJoystickMapToBuffer()
    {
        // 同步所有按钮状态
        key_buffer_.buttons.a = joystick_map_.a ? 1 : 0;
        key_buffer_.buttons.b = joystick_map_.b ? 1 : 0;
        key_buffer_.buttons.x = joystick_map_.x ? 1 : 0;
        key_buffer_.buttons.y = joystick_map_.y ? 1 : 0;
        key_buffer_.buttons.lb = joystick_map_.lb ? 1 : 0;
        key_buffer_.buttons.rb = joystick_map_.rb ? 1 : 0;
        key_buffer_.buttons.start = joystick_map_.start ? 1 : 0;
        key_buffer_.buttons.back = joystick_map_.back ? 1 : 0;
        key_buffer_.buttons.home = joystick_map_.home ? 1 : 0;
        key_buffer_.buttons.lo = joystick_map_.lo ? 1 : 0;
        key_buffer_.buttons.ro = joystick_map_.ro ? 1 : 0;

        // 同步所有摇杆状态
        key_buffer_.sticks.lx = joystick_map_.lx;
        key_buffer_.sticks.ly = joystick_map_.ly;
        key_buffer_.sticks.rx = joystick_map_.rx;
        key_buffer_.sticks.ry = joystick_map_.ry;

        // 同步所有扳机/方向键状态
        key_buffer_.triggers.lt = joystick_map_.lt;
        key_buffer_.triggers.rt = joystick_map_.rt;
        key_buffer_.triggers.xx = joystick_map_.xx;
        key_buffer_.triggers.yy = joystick_map_.yy;
    }

    // 映射手柄事件到缓冲区
    void mapJoystickToBuffer(const js_event &js)
    {
        if (js.type == JS_EVENT_BUTTON)
        {
            bool pressed = (js.value == 1);
            switch (js.number)
            {
            case 0:
                key_buffer_.buttons.a = pressed ? 1 : 0;
                break;
            case 1:
                key_buffer_.buttons.b = pressed ? 1 : 0;
                break;
            case 2:
                key_buffer_.buttons.x = pressed ? 1 : 0;
                break;
            case 3:
                key_buffer_.buttons.y = pressed ? 1 : 0;
                break;
            case 4:
                key_buffer_.buttons.lb = pressed ? 1 : 0;
                break;
            case 5:
                key_buffer_.buttons.rb = pressed ? 1 : 0;
                break;
            case 6:
                key_buffer_.buttons.back = pressed ? 1 : 0;
                break;
            case 7:
                key_buffer_.buttons.start = pressed ? 1 : 0;
                break;
            case 8:
                key_buffer_.buttons.home = pressed ? 1 : 0;
                break;
            case 9:
                key_buffer_.buttons.lo = pressed ? 1 : 0;
                break;
            case 10:
                key_buffer_.buttons.ro = pressed ? 1 : 0;
                break;
            default:
                break;
            }
        }
        else if (js.type == JS_EVENT_AXIS)
        {
            int value = js.value;
            switch (js.number)
            {
            case 0:
                key_buffer_.sticks.lx = value;
                break;
            case 1:
                key_buffer_.sticks.ly = value;
                break;
            case 2:
                key_buffer_.triggers.lt = value;
                break;
            case 3:
                key_buffer_.sticks.rx = value;
                break;
            case 4:
                key_buffer_.sticks.ry = value;
                break;
            case 5:
                key_buffer_.triggers.rt = value;
                break;
            case 6:
                key_buffer_.triggers.xx = value;
                break;
            case 7:
                key_buffer_.triggers.yy = value;
                break;
            default:
                break;
            }
        }
    }

    // 重置所有控制
    void resetControl()
    {
        key_buffer_.resetAll();
    }
    int getControlValue(ControlIndex index) const
    {
        return key_buffer_.getValueByIndex(index);
    }

    bool isControlButtonPressed(ControlIndex index) const
    {
        return key_buffer_.isButtonPressed(index);
    }

private:
    void printNoetixUpdIn(const noetix_upd_in_t *data)
    {
        static ThrottledErrorReporter print_reporter(10000);

        static long long count = 0;
        print_reporter.report([&]()
                              {
            // 获取当前时间的时间戳
            std::time_t now = std::time(nullptr);

            // 转换为本地时间
            std::tm *local_time = std::localtime(&now);

            // 打印时间戳（默认格式）
            LogWarn("Current local time: {}, Package serial number : {}, type: {} ", std::asctime(local_time), count++, data->type);

            std::ostringstream oss_l, oss_r;
            oss_l << "left_arm_pos: ";
            for (int i = 0; i < 8; ++i)
            {
                oss_l << data->left_arm_pos[i] << " ";
            }
            LogWarn("{}", oss_l.str());
            oss_r << "right_arm_pos: ";
            for (int i = 0; i < 8; ++i)
            {
                oss_r << data->right_arm_pos[i] << " ";
            }
            LogWarn("{}", oss_r.str()); });
    }
};

// 静态成员定义
noetix_upd_in_t RoboticArmController::control_data_ = {};
RoboticArmController *RoboticArmController::instance_ = nullptr;
bool RoboticArmController::joystick_error_ = false;
bool RoboticArmController::prev_home_pressed = false;
std::atomic<bool> RoboticArmController::zero_in_progress(false);

// 信号处理函数
void signalCallbackHandler(int signum)
{
    LogError("Caught kill signal: {}", signum);

    auto controller = RoboticArmController::getInstance();
    if (controller)
    {
        controller->cleanup();
    }
    g_is_killed = true;
    exit(signum);
}

int main()
{
    signal(SIGINT, signalCallbackHandler);
    signal(SIGTERM, signalCallbackHandler);
    signal(SIGQUIT, signalCallbackHandler);
    signal(SIGHUP, signalCallbackHandler);
    signal(SIGSEGV, signalCallbackHandler);
    signal(SIGABRT, signalCallbackHandler);
    signal(SIGFPE, signalCallbackHandler);

    LogWarn("Compiled date: {}", __DATE__);
    LogWarn("Compiled time: {}", __TIME__);
    LogWarn("Starting Robotic Arm Controller...");

    try
    {
        DirectionConfig cfg("/home/noetix/workspace/noetix-m1-yaocao/cfg.txt");
        auto vec = cfg.getVector();

        for (int i = 0; i < 14; i++)
        {
            g_direction_vector[i] = vec[i];
            LogWarn("Direction {}: {}", i, vec[i]);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error loading config: " << e.what() << "\n";
        return 1;
    }

    try
    {
        RoboticArmController controller;
        if (!controller.initialize())
        {
            std::cerr << "Failed to initialize controller" << std::endl;
            return EXIT_FAILURE;
        }

        controller.run();
    }
    catch (const std::exception &e)
    {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
