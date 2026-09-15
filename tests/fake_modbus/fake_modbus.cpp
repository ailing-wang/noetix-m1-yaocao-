#include <modbus/modbus.h>

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <fcntl.h>
#include <mutex>
#include <string>
#include <unistd.h>

namespace
{
struct FakeMotor
{
    int position = 2048;
    int target = 2048;
    bool torque = false;
    std::uint16_t acceleration = 2;
    std::uint16_t speed = 2;
    std::uint16_t torque_limit = 100;
    std::uint16_t status_fault_bits = 0;
    std::uint16_t voltage = 239;
    std::uint16_t temperature = 40;
    std::uint16_t current = 0;
    std::uint16_t position_p = 32;
    std::uint16_t position_d = 32;
    std::uint16_t position_i = 0;
    std::uint16_t lock_flag = 1;
};

std::mutex g_mutex;
std::array<std::array<FakeMotor, 8>, 4> g_motors{};
std::array<bool, 4> g_disconnected{{false, false, false, false}};
int g_target_write_count = 0;

void resetLocked()
{
    for (std::size_t bus = 0; bus < g_motors.size(); ++bus)
    {
        for (int id = 1; id <= 7; ++id)
        {
            FakeMotor &motor = g_motors[bus][static_cast<std::size_t>(id)];
            motor = FakeMotor{};
            motor.position = 1800 + static_cast<int>(bus) * 300 + id * 20;
            motor.target = motor.position;
            motor.temperature = static_cast<std::uint16_t>(35 + id);
        }
    }
    g_disconnected = {{false, false, false, false}};
    g_target_write_count = 0;
}

std::uint16_t readValue(int bus, int slave, int address)
{
    FakeMotor &motor = g_motors[static_cast<std::size_t>(bus)]
                                [static_cast<std::size_t>(slave)];
    switch (address)
    {
    case 0x0A: return static_cast<std::uint16_t>(slave);
    case 0x0D: return 0;
    case 0x0E: return 4095;
    case 0x10: return 0;
    case 0x11: return motor.position_p;
    case 0x12: return motor.position_d;
    case 0x13: return motor.position_i;
    case 0x80: return static_cast<std::uint16_t>(motor.target);
    case 0x81: return motor.torque ? 1 : 0;
    case 0x82: return motor.acceleration;
    case 0x83: return motor.speed;
    case 0x84: return motor.torque_limit;
    case 0x85: return motor.lock_flag;
    case 0x100:
        return static_cast<std::uint16_t>(motor.status_fault_bits |
                                          (motor.torque ? 0x10 : 0));
    case 0x101: return static_cast<std::uint16_t>(motor.position);
    case 0x102: return 0;
    case 0x103: return 0;
    case 0x104: return motor.voltage;
    case 0x105: return motor.temperature;
    case 0x106: return 0;
    case 0x107: return motor.current;
    default: return 0;
    }
}
} // namespace

struct _modbus
{
    int fd = -1;
    int master_fd = -1;
    int slave = 1;
    int bus_index = 0;
};

extern "C" void fake_modbus_reset()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    resetLocked();
}

extern "C" void fake_modbus_disconnect_bus(int bus_index)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus_index >= 0 && bus_index < static_cast<int>(g_disconnected.size()))
        g_disconnected[static_cast<std::size_t>(bus_index)] = true;
}

extern "C" void fake_modbus_set_position_on_bus(int bus, int slave,
                                                   int position)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus >= 0 && bus < static_cast<int>(g_motors.size()) &&
        slave >= 1 && slave <= 7)
    {
        g_motors[static_cast<std::size_t>(bus)]
                [static_cast<std::size_t>(slave)]
                    .position = position;
    }
}

extern "C" void fake_modbus_set_temperature(int slave, int temperature)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (slave >= 1 && slave <= 7)
    {
        const std::size_t bus = slave <= 3 ? 0 : 1;
        g_motors[bus][static_cast<std::size_t>(slave)].temperature =
            static_cast<std::uint16_t>(temperature);
    }
}

extern "C" int fake_modbus_torque(int slave)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (slave < 1 || slave > 7)
        return -1;
    const std::size_t bus = slave <= 3 ? 0 : 1;
    return g_motors[bus][static_cast<std::size_t>(slave)].torque ? 1 : 0;
}

extern "C" void fake_modbus_set_temperature_on_bus(int bus, int slave,
                                                     int temperature)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus >= 0 && bus < static_cast<int>(g_motors.size()) &&
        slave >= 1 && slave <= 7)
        g_motors[static_cast<std::size_t>(bus)]
                [static_cast<std::size_t>(slave)].temperature =
            static_cast<std::uint16_t>(temperature);
}

extern "C" int fake_modbus_torque_on_bus(int bus, int slave)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus < 0 || bus >= static_cast<int>(g_motors.size()) ||
        slave < 1 || slave > 7)
        return -1;
    return g_motors[static_cast<std::size_t>(bus)]
                   [static_cast<std::size_t>(slave)].torque
               ? 1
               : 0;
}

extern "C" void fake_modbus_set_runtime_on_bus(int bus, int slave,
                                                 int acceleration, int speed,
                                                 int torque_limit)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus < 0 || bus >= static_cast<int>(g_motors.size()) ||
        slave < 1 || slave > 7)
        return;
    FakeMotor &motor = g_motors[static_cast<std::size_t>(bus)]
                                [static_cast<std::size_t>(slave)];
    motor.acceleration = static_cast<std::uint16_t>(acceleration);
    motor.speed = static_cast<std::uint16_t>(speed);
    motor.torque_limit = static_cast<std::uint16_t>(torque_limit);
}

extern "C" void fake_modbus_set_pid_on_bus(int bus, int slave,
                                             int position_p, int position_d,
                                             int position_i, int lock_flag)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus < 0 || bus >= static_cast<int>(g_motors.size()) ||
        slave < 1 || slave > 7)
        return;
    FakeMotor &motor = g_motors[static_cast<std::size_t>(bus)]
                                [static_cast<std::size_t>(slave)];
    motor.position_p = static_cast<std::uint16_t>(position_p);
    motor.position_d = static_cast<std::uint16_t>(position_d);
    motor.position_i = static_cast<std::uint16_t>(position_i);
    motor.lock_flag = static_cast<std::uint16_t>(lock_flag);
}

extern "C" int fake_modbus_pid_value_on_bus(int bus, int slave, int field)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (bus < 0 || bus >= static_cast<int>(g_motors.size()) ||
        slave < 1 || slave > 7)
        return -1;
    const FakeMotor &motor = g_motors[static_cast<std::size_t>(bus)]
                                     [static_cast<std::size_t>(slave)];
    if (field == 0)
        return static_cast<int>(motor.position_p);
    if (field == 1)
        return static_cast<int>(motor.position_d);
    if (field == 2)
        return static_cast<int>(motor.position_i);
    if (field == 3)
        return static_cast<int>(motor.lock_flag);
    return -1;
}

extern "C" int fake_modbus_target_write_count()
{
    std::lock_guard<std::mutex> lock(g_mutex);
    return g_target_write_count;
}

extern "C" modbus_t *modbus_new_rtu(const char *device, int, char, int, int)
{
    modbus_t *context = new modbus_t;
    const std::string path = device ? device : "";
    if (path.find("l2") != std::string::npos)
        context->bus_index = 1;
    else if (path.find("r1") != std::string::npos)
        context->bus_index = 2;
    else if (path.find("r2") != std::string::npos)
        context->bus_index = 3;
    else
        context->bus_index = 0;
    context->master_fd = posix_openpt(O_RDWR | O_NOCTTY);
    if (context->master_fd >= 0 && grantpt(context->master_fd) == 0 &&
        unlockpt(context->master_fd) == 0)
    {
        const char *slave_path = ptsname(context->master_fd);
        if (slave_path)
            context->fd = open(slave_path, O_RDWR | O_NOCTTY);
    }
    return context;
}

extern "C" int modbus_set_response_timeout(modbus_t *, std::uint32_t, std::uint32_t)
{
    return 0;
}

extern "C" int modbus_set_byte_timeout(modbus_t *, std::uint32_t, std::uint32_t)
{
    return 0;
}

extern "C" int modbus_set_slave(modbus_t *context, int slave)
{
    context->slave = slave;
    return 0;
}

extern "C" int modbus_connect(modbus_t *)
{
    return 0;
}

extern "C" int modbus_get_socket(modbus_t *context)
{
    return context->fd;
}

extern "C" int modbus_read_registers(modbus_t *context, int address, int count,
                                      std::uint16_t *values)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_disconnected[static_cast<std::size_t>(context->bus_index)])
    {
        errno = ETIMEDOUT;
        return -1;
    }
    for (int index = 0; index < count; ++index)
        values[index] = readValue(context->bus_index, context->slave, address + index);
    return count;
}

extern "C" int modbus_write_register(modbus_t *context, int address, int value)
{
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_disconnected[static_cast<std::size_t>(context->bus_index)])
    {
        errno = ETIMEDOUT;
        return -1;
    }
    FakeMotor &motor = g_motors[static_cast<std::size_t>(context->bus_index)]
                                [static_cast<std::size_t>(context->slave)];
    switch (address)
    {
    case 0x80:
        ++g_target_write_count;
        motor.target = value;
        motor.position = value;
        break;
    case 0x81: motor.torque = value != 0; break;
    case 0x82: motor.acceleration = static_cast<std::uint16_t>(value); break;
    case 0x83: motor.speed = static_cast<std::uint16_t>(value); break;
    case 0x84: motor.torque_limit = static_cast<std::uint16_t>(value); break;
    case 0x11: motor.position_p = static_cast<std::uint16_t>(value); break;
    case 0x12: motor.position_d = static_cast<std::uint16_t>(value); break;
    case 0x13: motor.position_i = static_cast<std::uint16_t>(value); break;
    case 0x85: motor.lock_flag = static_cast<std::uint16_t>(value); break;
    default: break;
    }
    return 1;
}

extern "C" const char *modbus_strerror(int error_number)
{
    return std::strerror(error_number);
}

extern "C" void modbus_close(modbus_t *context)
{
    if (context->fd >= 0)
    {
        close(context->fd);
        context->fd = -1;
    }
    if (context->master_fd >= 0)
    {
        close(context->master_fd);
        context->master_fd = -1;
    }
}

extern "C" void modbus_free(modbus_t *context)
{
    delete context;
}

namespace
{
struct Initializer
{
    Initializer()
    {
        std::lock_guard<std::mutex> lock(g_mutex);
        resetLocked();
    }
} g_initializer;
} // namespace
