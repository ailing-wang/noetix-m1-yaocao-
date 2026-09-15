#include "Modbus.h"
#include <cerrno>
#include <cstring>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <unistd.h>

#include "Logger.h"

bool ModbusBus::modbus_error_ = false;

static inline std::int64_t now_ms()
{
    using namespace std::chrono;
    return duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

static std::runtime_error make_modbus_error(const std::string &prefix)
{
    return std::runtime_error(prefix + ": " + modbus_strerror(errno));
}

static modbus_t *init_ctx_from_cfg(const SerialConfig &cfg)
{
    modbus_t *ctx = modbus_new_rtu(cfg.port.c_str(), cfg.baudrate, cfg.parity, cfg.data_bit, cfg.stop_bit);
    if (!ctx)
        return nullptr;

    modbus_set_response_timeout(ctx, cfg.resp_timeout_sec, cfg.resp_timeout_usec);
    modbus_set_byte_timeout(ctx, cfg.byte_timeout_sec, cfg.byte_timeout_usec);

    // 默认 slave，真正读之前会 set_slave
    modbus_set_slave(ctx, 1);
    return ctx;
}

static bool is_reconnect_errno(int e)
{
    // RTU 常见：ETIMEDOUT/EIO/EBADF/ENODEV；也兼容一些平台上的 ECONNRESET/ENXIO
    switch (e)
    {
    case ETIMEDOUT:
    case EIO:
    case EBADF:
    case ENODEV:
    case ENOENT:
    case EPIPE:
    case ECONNRESET:
    case ENXIO: // duplicate harmless
        return true;
    default:
        return false;
    }
}

/* ---------------- ModbusPort ---------------- */

ModbusPort::ModbusPort(SerialConfig cfg) : cfg_(std::move(cfg))
{
    ctx_ = init_ctx_from_cfg(cfg_);
    if (!ctx_)
    {
        throw std::runtime_error("modbus_new_rtu failed for " + cfg_.port);
    }

    if (modbus_connect(ctx_) == -1)
    {
        cleanup();
        throw make_modbus_error("modbus_connect failed for " + cfg_.port);
    }
}

ModbusPort::~ModbusPort()
{
    cleanup();
}

ModbusPort::ModbusPort(ModbusPort &&other) noexcept
    : cfg_(std::move(other.cfg_)), ctx_(other.ctx_)
{
    other.ctx_ = nullptr;
}

ModbusPort &ModbusPort::operator=(ModbusPort &&other) noexcept
{
    if (this != &other)
    {
        cleanup();
        cfg_ = std::move(other.cfg_);
        ctx_ = other.ctx_;
        other.ctx_ = nullptr;
    }
    return *this;
}

void ModbusPort::cleanup() noexcept
{
    if (ctx_)
    {
        modbus_close(ctx_);
        modbus_free(ctx_);
        ctx_ = nullptr;
    }
}

bool ModbusPort::reconnect() noexcept
{
    // 2ms/4ms 控制循环里，USB 掉线期间避免疯狂重连
    const std::int64_t t = now_ms();
    if (last_reconnect_try_ms_ != 0 && (t - last_reconnect_try_ms_) < reconnect_interval_ms_)
    {
        return false;
    }
    last_reconnect_try_ms_ = t;

    cleanup();

    ctx_ = init_ctx_from_cfg(cfg_);
    if (!ctx_)
    {
        return false;
    }

    if (modbus_connect(ctx_) == -1)
    {
        cleanup();
        return false;
    }
    return true;
}
/* ---------------- ModbusBus ---------------- */

ModbusBus::ModbusBus(SerialConfig port_cfg)
    : port_(std::move(port_cfg)) {}

void ModbusBus::addDevice(DeviceConfig dev)
{
    // 创建 DeviceEntry 并分别设置成员
    DeviceEntry entry;
    entry.cfg = std::move(dev);
    entry.offset = 0; // 临时值，finalizeBuffer 会重新计算
    devices_.push_back(std::move(entry));
    buffer_ready_ = false;
}

void ModbusBus::finalizeBuffer()
{
    // 如果已经初始化过了，直接返回
    if (buffer_ready_)
    {
        return;
    }

    // 计算每个设备在连续buffer内的 offset，并一次性分配总长度
    std::size_t total_regs = 0;
    for (auto &d : devices_)
    {
        d.offset = total_regs;
        total_regs += static_cast<std::size_t>(d.cfg.register_count);
    }

    buffer_.assign(total_regs, 0);
    buffer_ready_ = true;
}

int ModbusBus::readRegister(std::size_t device_index,
                            std::uint16_t address,
                            std::uint16_t *dst,
                            std::size_t count,
                            bool verbose)
{
    static ThrottledErrorReporter error_reporter(10000);

    if (device_index >= devices_.size())
    {
        std::cout << "device_index:" << device_index << std::endl;
        std::cout << "devices_.size:" << devices_.size() << std::endl;
        throw std::out_of_range("device_index out of range");
    }

    if (dst == nullptr)
    {
        throw std::invalid_argument("dst is null");
    }

    if (count == 0)
    {
        throw std::invalid_argument("count must be > 0");
    }

    auto &dev = devices_[device_index];

    if (!port_.ctx())
    {
        if (!port_.reconnect())
        {
            error_reporter.report([&]()
                                  { LogError("Failed to reconnect Modbus port: {}, device: {}", port_.portName(), dev.cfg.description); });
        }
    }

    if (modbus_set_slave(port_.ctx(), dev.cfg.slave_id) == -1)
    {
        throw make_modbus_error("modbus_set_slave failed (slave " + std::to_string(dev.cfg.slave_id) + ")");
    }

    int rc = modbus_read_registers(
        port_.ctx(),
        address,
        static_cast<int>(count),
        dst);
    usleep(1250);

    if (rc == -1)
    {
        const int e = errno;

        if (is_reconnect_errno(e))
        {
            const bool ok = port_.reconnect();
            if (ok)
            {
                if (modbus_set_slave(port_.ctx(), dev.cfg.slave_id) != -1)
                {
                    rc = modbus_read_registers(
                        port_.ctx(),
                        address,
                        static_cast<int>(count),
                        dst);
                    usleep(1250);
                }
            }
        }

        if (rc == -1)
        {
            errno = e;
            throw make_modbus_error("Read register failed for " + dev.cfg.description +
                                    " (slave " + std::to_string(dev.cfg.slave_id) +
                                    ", addr " + std::to_string(address) + ")");
        }
    }

    if (verbose)
    {
        std::cout << "=== Modbus Read Register Result ===" << std::endl;
        std::cout << "Device: " << dev.cfg.description << std::endl;
        std::cout << "Slave ID: " << dev.cfg.slave_id << std::endl;
        std::cout << "Read Address: 0x" << std::hex << address << std::dec << std::endl;
        std::cout << "Register Count: " << rc << std::endl;
        std::cout << "Values:" << std::endl;

        for (int i = 0; i < rc; ++i)
        {
            std::cout << "  [" << std::setw(4) << (address + i) << "] "
                      << "DEC: " << std::setw(5) << dst[i] << "  "
                      << "HEX: 0x" << std::hex << std::setw(4) << std::setfill('0') << dst[i]
                      << std::dec << std::setfill(' ') << std::endl;
        }
        std::cout << "==================================" << std::endl;
    }

    return rc;
}

int ModbusBus::readDevice(std::size_t device_index, bool verbose)
{
    static ThrottledErrorReporter error_reporter(5000);

    if (!buffer_ready_)
    {
        throw std::runtime_error("ModbusBus buffer not ready: call finalizeBuffer() after adding devices.");
    }
    if (device_index >= devices_.size())
    {
        std::cout << "device_index:" << device_index << std::endl;
        std::cout << "devices_.size:" << devices_.size() << std::endl;

        throw std::out_of_range("device_index out of range");
    }

    auto &dev = devices_[device_index];

    // USB 掉线/重连失败后，ctx_ 可能被清理为 nullptr；此时必须先重连
    if (!port_.ctx())
    {
        if (!port_.reconnect())
        {
            error_reporter.report([&]()
                                  { LogError("Failed to reconnect Modbus port: {}, device: {}", port_.portName(), dev.cfg.description); });
            return -1;
        }
        else
        {
            error_ = false; // 重连成功，恢复 false
            LogWarn("Successfully reconnected Modbus port: {}, device: {}", port_.portName(), dev.cfg.description);
        }
    }

    // 设置从站地址
    if (modbus_set_slave(port_.ctx(), dev.cfg.slave_id) == -1)
    {
        throw make_modbus_error("modbus_set_slave failed (slave " + std::to_string(dev.cfg.slave_id) + ")");
    }

    std::uint16_t *dst = buffer_.data() + dev.offset;
    const int count = static_cast<int>(dev.cfg.register_count);

    int rc = modbus_read_registers(
        port_.ctx(),
        dev.cfg.start_address,
        count,
        dst);

    usleep(1250);

    if (rc == -1)
    {
        const int e = errno;

        // 断线/热插拔：尝试重连并重试一次（最小化修改，不影响外部调用）
        if (is_reconnect_errno(e))
        {
            const bool ok = port_.reconnect();
            if (ok)
            {
                // 重设 slave 并重试
                if (modbus_set_slave(port_.ctx(), dev.cfg.slave_id) != -1)
                {
                    rc = modbus_read_registers(port_.ctx(), dev.cfg.start_address, count, dst);
                    if (rc != -1)
                    {
                        error_ = false; // 重连成功
                        LogWarn("Successfully reconnected Modbus port: {}, device: {}", port_.portName(), dev.cfg.description);
                    }
                    usleep(1250);
                }
            }
        }

        if (rc == -1)
        {
            errno = e; // 保留第一次失败的 errno，便于定位根因
            throw make_modbus_error("Read failed for " + dev.cfg.description +
                                    " (slave " + std::to_string(dev.cfg.slave_id) + ")");
        }
    }
    else
    {
        error_ = false; // 成功读取，清除错误标志
        LogInfo("Successfully read from device: {}", dev.cfg.description);
    }

    // 根据verbose参数控制输出详细程度
    if (verbose)
    {
        std::cout << "=== Modbus Read Result ===" << std::endl;
        std::cout << "Device: " << dev.cfg.description << std::endl;
        std::cout << "Slave ID: " << dev.cfg.slave_id << std::endl;
        std::cout << "Start Address: " << dev.cfg.start_address << std::endl;
        std::cout << "Register Count: " << rc << std::endl;
        std::cout << "Values:" << std::endl;

        for (int i = 0; i < rc; ++i)
        {
            std::cout << "  [" << std::setw(4) << (dev.cfg.start_address + i) << "] "
                      << "DEC: " << std::setw(5) << dst[i] << "  "
                      << "HEX: 0x" << std::hex << std::setw(4) << std::setfill('0') << dst[i]
                      << std::dec << std::endl;
        }
        std::cout << "=========================" << std::endl;
    }

    return rc;
}
void ModbusBus::readAll()
{
    if (!buffer_ready_)
        finalizeBuffer();

    for (std::size_t i = 0; i < devices_.size(); ++i)
    {
        try
        {
            int rc = readDevice(i, false);
            if (rc != (int)devices_[i].cfg.register_count)
            {
                static ThrottledErrorReporter error_reporter(5000);
                error_reporter.report([&]()
                                      { LogError("[modbus] WARN slave={} addr={} rc={}",
                                                 devices_[i].cfg.slave_id, devices_[i].cfg.start_address, rc); });
            }
            buffer_state_[i] = true;
        }
        catch (const std::exception &e)
        {
            static ThrottledErrorReporter error_reporter(5000);
            error_reporter.report([&]()
                                  { LogError("[modbus] ERROR {}", e.what()); });
            buffer_state_[i] = false;
            error_ = true;
        }
    }
}

int ModbusBus::writeDevice(std::size_t device_index, const std::uint16_t addr, const std::uint16_t *values, std::size_t count, bool verbose)
{
    static ThrottledErrorReporter error_reporter(500);

    // 1. 前置检查：缓冲区必须已 finalize（设备配置已完成）
    if (!buffer_ready_)
    {
        throw std::runtime_error("ModbusBus buffer not ready: call finalizeBuffer() first.");
    }
    // 2. 检查设备索引是否有效
    if (device_index >= devices_.size())
    {
        std::cout << "device_index:" << device_index << std::endl;
        std::cout << "devices_.size:" << devices_.size() << std::endl;
        throw std::out_of_range("device_index out of range");
    }

    auto &dev = devices_[device_index];

    // 3.新加：检查请求写入的寄存器数量是否超过设备配置的数量
    if (count > static_cast<std::size_t>(dev.cfg.register_count))
    {
        throw std::invalid_argument("write count exceeds device register_count");
    }

    // 4. 确保 Modbus 端口在线，若 ctx 为空则尝试重连
    if (!port_.ctx())
    {
        if (!port_.reconnect())
        {
            error_reporter.report([&]()
                                  { LogError("Failed to reconnect Modbus port: {}, device: {}", port_.portName(), dev.cfg.description); });
        }
    }

    // 5. 设置从站地址
    if (modbus_set_slave(port_.ctx(), dev.cfg.slave_id) == -1)
    {
        throw make_modbus_error("modbus_set_slave failed (slave " + std::to_string(dev.cfg.slave_id) + ")");
    }

    // 6. 执行 Modbus 写入（多个寄存器）
    int rc = modbus_write_registers(port_.ctx(), addr, 1, values);
    // 7. 错误处理与重试
    if (rc == -1)
    {
        const int e = errno;

        // 若错误属于可重连类型，则尝试重连并重试一次
        if (is_reconnect_errno(e))
        {
            const bool ok = port_.reconnect();
            if (ok)
            {
                if (modbus_set_slave(port_.ctx(), dev.cfg.slave_id) != -1)
                {
                    rc = modbus_write_registers(port_.ctx(), addr, count, values);
                }
            }
        }

        // 若重试后仍然失败，则恢复 errno 并抛出异常
        if (rc == -1)
        {
            errno = e;
            throw make_modbus_error("Write failed for " + dev.cfg.description +
                                    " (slave " + std::to_string(dev.cfg.slave_id) + ")");
        }
    }

    // 8. 可选详细输出（类似 readDevice 的打印风格）
    if (verbose)
    {
        std::cout << "=== Modbus Write Result ===" << std::endl;
        std::cout << "Device: " << dev.cfg.description << std::endl;
        std::cout << "Slave ID: " << dev.cfg.slave_id << std::endl;
        std::cout << "Start Address (configured): " << dev.cfg.start_address << std::endl;
        std::cout << "Actual Write Address: 0x80" << std::endl; // 添加了实际写入地址的提示
        std::cout << "Register Count: " << rc << std::endl;
        std::cout << "Values:" << std::endl;
        for (std::size_t i = 0; i < count; ++i)
        {
            std::cout << "  [" << std::setw(4) << (dev.cfg.start_address + i) << "] "
                      << "DEC: " << std::setw(5) << values[i] << "  "
                      << "HEX: 0x" << std::hex << std::setw(4) << std::setfill('0') << values[i]
                      << std::dec << std::endl;
        }
        std::cout << "===========================" << std::endl;
    }

    // 9. 返回实际写入的寄存器数量（成功时等于 count）
    return rc;
}

std::pair<const std::uint16_t *, std::size_t> ModbusBus::view(std::size_t device_index) const
{
    if (!buffer_ready_)
    {
        throw std::runtime_error("buffer not ready");
    }
    if (device_index >= devices_.size())
    {
        std::cout << "device_index:" << device_index << std::endl;
        std::cout << "devices_.size:" << devices_.size() << std::endl;
        throw std::out_of_range("device_index out of range");
    }
    const auto &dev = devices_[device_index];
    return {buffer_.data() + dev.offset, static_cast<std::size_t>(dev.cfg.register_count)};
}

/* ---------------- ModbusManager ---------------- */

std::size_t ModbusManager::addBus(SerialConfig cfg)
{
    buses_.emplace_back(std::move(cfg));
    return buses_.size() - 1;
}

ModbusBus &ModbusManager::bus(std::size_t idx)
{
    return buses_.at(idx);
}

const ModbusBus &ModbusManager::bus(std::size_t idx) const
{
    return buses_.at(idx);
}
