#pragma once
#include <modbus/modbus.h>

#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>
#include <utility>

struct SerialConfig
{
    std::string port;
    int baudrate = 115200;
    char parity = 'N';
    int data_bit = 8;
    int stop_bit = 1;

    // timeouts
    int resp_timeout_sec = 1;
    int resp_timeout_usec = 0;
    int byte_timeout_sec = 0;
    int byte_timeout_usec = 500000; // 500ms
};

struct DeviceConfig
{
    int slave_id = 1;
    std::uint16_t start_address = 0;
    std::uint16_t register_count = 1;
    std::string description;
};

enum class ModbusDeviceState
{
    OK = 0,
    FAIL
};

class ModbusPort
{
public:
    explicit ModbusPort(SerialConfig cfg);
    ~ModbusPort();

    ModbusPort(const ModbusPort &) = delete;
    ModbusPort &operator=(const ModbusPort &) = delete;
    ModbusPort(ModbusPort &&) noexcept;
    ModbusPort &operator=(ModbusPort &&) noexcept;

    modbus_t *ctx() const noexcept { return ctx_; }
    const std::string &portName() const noexcept { return cfg_.port; }

    // 断线/热插拔后重连：重新创建 ctx 并 modbus_connect。
    // 成功返回 true；失败返回 false（ctx_ 会被清理为 nullptr）。
    bool reconnect() noexcept;

private:
    void cleanup() noexcept;

    SerialConfig cfg_;
    modbus_t *ctx_ = nullptr;

    // 重连节流：避免在 2ms/4ms 循环里疯狂 close/open
    std::int64_t last_reconnect_try_ms_ = 0;
    int reconnect_interval_ms_ = 500;
};

class ModbusBus
{
public:
    explicit ModbusBus(SerialConfig port_cfg);

    // 添加设备：内部会计算该设备在总线buffer中的 offset
    void addDevice(DeviceConfig dev);

    // 一次性完成 buffer 分配（建议在 addDevice 全部完成后调用）
    void finalizeBuffer();

    // 读单设备，返回实际读取寄存器数量（失败抛异常或你也可改成返回bool）
    int readDevice(std::size_t device_index, bool verbose);

    // 读全设备
    void readAll();

    int readRegister(std::size_t device_index, std::uint16_t address, std::uint16_t *dst, std::size_t count, bool verbose);

    int writeDevice(std::size_t device_index, const std::uint16_t addr, const std::uint16_t *values, std::size_t count, bool verbose);

    // 获取某个设备的"视图"（指向内部buffer的一段连续区间）
    std::pair<const std::uint16_t *, std::size_t> view(std::size_t device_index) const;

    std::size_t deviceCount() const noexcept { return devices_.size(); }

    static bool modbus_error_;

    bool error_;
private:
    struct DeviceEntry
    {
        DeviceConfig cfg;
        std::size_t offset = 0; // 在 buffer_ 中的起始位置
    };

    ModbusPort port_;
    std::vector<DeviceEntry> devices_;
    std::vector<std::uint16_t> buffer_; // 总线级连续缓存（一次分配复用）

    bool buffer_ready_ = false;

public:
    bool bus_state_ = false;
    bool buffer_state_[7] = {false, false, false, false, false, false, false};
};

class ModbusManager
{
public:
    // 多总线管理
    std::size_t addBus(SerialConfig cfg);
    ModbusBus &bus(std::size_t idx);
    const ModbusBus &bus(std::size_t idx) const;

private:
    std::vector<ModbusBus> buses_;
};