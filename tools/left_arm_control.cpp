#include <modbus/modbus.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <poll.h>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace
{
constexpr std::uint16_t kDeviceId = 0x0A;
constexpr std::uint16_t kMinimumAngleLimit = 0x0D;
constexpr std::uint16_t kMaximumAngleLimit = 0x0E;
constexpr std::uint16_t kOperatingMode = 0x10;

constexpr std::uint16_t kTargetPosition = 0x80;
constexpr std::uint16_t kTorqueEnable = 0x81;
constexpr std::uint16_t kAcceleration = 0x82;
constexpr std::uint16_t kSpeed = 0x83;
constexpr std::uint16_t kTorqueLimit = 0x84;

constexpr std::uint16_t kStatusStart = 0x100;
constexpr int kStatusRegisterCount = 8;
constexpr std::uint16_t kTorqueEnabledStatusBit = 0x10;

constexpr std::uint16_t kSafeAcceleration = 2;  // 200 steps/s^2
constexpr std::uint16_t kSafeSpeed = 2;         // 100 steps/s
constexpr std::uint16_t kSafeTorqueLimit = 100; // 10.0%

constexpr double kStepsPerRevolution = 4095.0;
constexpr double kSingleMoveMaximumDegrees = 1.0;
constexpr double kGroupMoveMaximumDegrees = 0.5;
constexpr double kWorkspaceDegrees = 3.0;
constexpr double kMaximumTemperatureC = 65.0;
constexpr double kMaximumCurrentA = 0.8;
constexpr double kMinimumVoltageV = 8.0;
constexpr double kMaximumVoltageV = 26.0;
constexpr int kPositionToleranceSteps = 3;
constexpr int kMaximumEnableJumpSteps = 8;

volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int)
{
    g_stop_requested = 1;
}

int degreesToSteps(double degrees)
{
    return static_cast<int>(std::lround(degrees * kStepsPerRevolution / 360.0));
}

double stepsToDegrees(int steps)
{
    return static_cast<double>(steps) * 360.0 / kStepsPerRevolution;
}

bool positionIsSingleTurn(int position)
{
    return position >= 0 && position <= 4095;
}

std::string hexAddress(std::uint16_t address)
{
    std::ostringstream output;
    output << std::uppercase << std::hex << std::setw(4)
           << std::setfill('0') << address;
    return output.str();
}

bool isAllDigits(const char *text)
{
    if (!text || !*text)
        return false;
    for (const char *p = text; *p; ++p)
    {
        if (*p < '0' || *p > '9')
            return false;
    }
    return true;
}

std::string canonicalPath(const std::string &path)
{
    char resolved[PATH_MAX]{};
    if (!realpath(path.c_str(), resolved))
        return {};
    return resolved;
}

std::vector<int> findProcessesHoldingDevice(const std::string &device)
{
    std::vector<int> holders;
    const std::string canonical_device = canonicalPath(device);
    if (canonical_device.empty())
        return holders;

    DIR *proc = opendir("/proc");
    if (!proc)
        return holders;

    while (dirent *process_entry = readdir(proc))
    {
        if (!isAllDigits(process_entry->d_name))
            continue;
        const int pid = std::atoi(process_entry->d_name);
        if (pid <= 0 || pid == static_cast<int>(getpid()))
            continue;

        const std::string fd_dir = std::string("/proc/") + process_entry->d_name + "/fd";
        DIR *fds = opendir(fd_dir.c_str());
        if (!fds)
            continue;

        while (dirent *fd_entry = readdir(fds))
        {
            if (!isAllDigits(fd_entry->d_name))
                continue;
            const std::string fd_path = fd_dir + "/" + fd_entry->d_name;
            char target[PATH_MAX]{};
            const ssize_t length = readlink(fd_path.c_str(), target, sizeof(target) - 1);
            if (length <= 0)
                continue;
            target[length] = '\0';
            if (canonicalPath(target) == canonical_device)
            {
                holders.push_back(pid);
                break;
            }
        }
        closedir(fds);
    }
    closedir(proc);
    return holders;
}

bool loadLeftArmDirections(const std::string &path, std::array<int, 7> &directions)
{
    std::ifstream input(path);
    if (!input)
    {
        std::cerr << "Cannot open direction config: " << path << '\n';
        return false;
    }

    std::array<bool, 7> seen{};
    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t separator = line.find('=');
        if (separator == std::string::npos)
            continue;
        try
        {
            const int index = std::stoi(line.substr(0, separator));
            const int value = std::stoi(line.substr(separator + 1));
            if (index >= 0 && index < 7)
            {
                if (value != -1 && value != 1)
                {
                    std::cerr << "Direction " << index << " must be -1 or 1.\n";
                    return false;
                }
                directions[static_cast<std::size_t>(index)] = value;
                seen[static_cast<std::size_t>(index)] = true;
            }
        }
        catch (const std::exception &)
        {
            std::cerr << "Invalid direction config line: " << line << '\n';
            return false;
        }
    }

    for (std::size_t i = 0; i < seen.size(); ++i)
    {
        if (!seen[i])
        {
            std::cerr << "Direction config is missing index " << i << ".\n";
            return false;
        }
    }
    return true;
}

struct ServoState
{
    std::uint16_t status_word = 0;
    int position = 0;
    int speed_raw = 0;
    int pwm_raw = 0;
    double voltage_v = 0.0;
    double temperature_c = 0.0;
    bool moving = false;
    double current_a = 0.0;
    int target_position = 0;
    bool torque_enabled = false;
    std::uint16_t acceleration = 0;
    std::uint16_t speed_limit = 0;
    std::uint16_t torque_limit = 0;
};

class BusPort
{
public:
    explicit BusPort(std::string device) : device_(std::move(device)) {}

    ~BusPort()
    {
        closePort();
    }

    BusPort(const BusPort &) = delete;
    BusPort &operator=(const BusPort &) = delete;

    bool connectExclusive()
    {
        const auto holders = findProcessesHoldingDevice(device_);
        if (!holders.empty())
        {
            std::cerr << "Refusing to open " << device_ << ": held by PID";
            for (const int pid : holders)
                std::cerr << ' ' << pid;
            std::cerr << '\n';
            return false;
        }

        ctx_ = modbus_new_rtu(device_.c_str(), 115200, 'N', 8, 1);
        if (!ctx_)
        {
            std::cerr << "modbus_new_rtu failed for " << device_ << '\n';
            return false;
        }
        modbus_set_response_timeout(ctx_, 0, 250000);
        modbus_set_byte_timeout(ctx_, 0, 50000);

        if (modbus_connect(ctx_) == -1)
        {
            reportError("connect");
            closePort();
            return false;
        }
        connected_ = true;

        const int fd = modbus_get_socket(ctx_);
        if (fd < 0 || flock(fd, LOCK_EX | LOCK_NB) == -1)
        {
            std::cerr << "Could not lock " << device_ << " exclusively.\n";
            closePort();
            return false;
        }
        if (ioctl(fd, TIOCEXCL) == -1)
        {
            std::cerr << "Could not set TIOCEXCL on " << device_ << ": "
                      << std::strerror(errno) << '\n';
            closePort();
            return false;
        }

        const auto holders_after_open = findProcessesHoldingDevice(device_);
        if (!holders_after_open.empty())
        {
            std::cerr << "Another process opened " << device_ << " during startup.\n";
            closePort();
            return false;
        }

        healthy_ = true;
        return true;
    }

    bool readRegisters(int slave_id, std::uint16_t address, int count,
                       std::uint16_t *destination)
    {
        if (!selectSlave(slave_id))
            return false;
        const int rc = modbus_read_registers(ctx_, address, count, destination);
        transactionDelay();
        if (rc != count)
        {
            reportError("read slave " + std::to_string(slave_id) + " register 0x" +
                        hexAddress(address));
            healthy_ = false;
            return false;
        }
        return true;
    }

    bool readOne(int slave_id, std::uint16_t address, std::uint16_t &value)
    {
        return readRegisters(slave_id, address, 1, &value);
    }

    bool writeVerified(int slave_id, std::uint16_t address, std::uint16_t value,
                       const std::string &description)
    {
        if (!selectSlave(slave_id))
            return false;
        const int rc = modbus_write_register(ctx_, address, value);
        transactionDelay();
        if (rc != 1)
        {
            reportError("write slave " + std::to_string(slave_id) + " " + description);
            healthy_ = false;
            return false;
        }

        std::uint16_t readback = 0;
        if (!readOne(slave_id, address, readback))
            return false;
        if (readback != value)
        {
            std::cerr << device_ << " ID " << slave_id << " readback mismatch for "
                      << description << ": wrote " << value << ", read " << readback << '\n';
            healthy_ = false;
            return false;
        }
        return true;
    }

    bool bestEffortWrite(int slave_id, std::uint16_t address, std::uint16_t value) noexcept
    {
        if (!ctx_ || !connected_)
            return false;
        if (modbus_set_slave(ctx_, slave_id) == -1)
            return false;
        const bool ok = modbus_write_register(ctx_, address, value) == 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return ok;
    }

    bool healthy() const { return healthy_; }
    const std::string &device() const { return device_; }

private:
    bool selectSlave(int slave_id)
    {
        if (!ctx_ || !connected_)
            return false;
        if (modbus_set_slave(ctx_, slave_id) == -1)
        {
            reportError("select slave " + std::to_string(slave_id));
            healthy_ = false;
            return false;
        }
        return true;
    }

    void reportError(const std::string &operation) const
    {
        std::cerr << device_ << ' ' << operation << " failed: "
                  << modbus_strerror(errno) << '\n';
    }

    static void transactionDelay()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    void closePort() noexcept
    {
        if (ctx_)
        {
            if (connected_)
                modbus_close(ctx_);
            modbus_free(ctx_);
            ctx_ = nullptr;
        }
        connected_ = false;
    }

    std::string device_;
    modbus_t *ctx_ = nullptr;
    bool connected_ = false;
    bool healthy_ = false;
};

struct Joint
{
    int number = 0;
    int slave_id = 0;
    int bus_index = 0;
    int direction = 1;
    int initial_position = 0;
    int target_position = 0;
    int workspace_min = 0;
    int workspace_max = 4095;
    bool enabled = false;
    bool qualified = false;
};

enum class ControlMode
{
    Observe,
    SingleJoint,
    Group
};

class LeftArmController
{
public:
    explicit LeftArmController(const std::array<int, 7> &directions)
        : buses_{{BusPort("/dev/noetix_arm_l1"), BusPort("/dev/noetix_arm_l2")}}
    {
        for (int i = 0; i < 7; ++i)
        {
            Joint &joint = joints_[static_cast<std::size_t>(i)];
            joint.number = i + 1;
            joint.slave_id = i + 1;
            joint.bus_index = i < 3 ? 0 : 1;
            joint.direction = directions[static_cast<std::size_t>(i)];
        }
    }

    ~LeftArmController()
    {
        if (anyEnabled())
            emergencyDisableAll();
    }

    bool connectAndPrepare()
    {
        if (!buses_[0].connectExclusive())
            return false;
        if (!buses_[1].connectExclusive())
        {
            emergencyDisableAll();
            return false;
        }

        // Remove any stale SRAM torque-enable state before the slower
        // identity/configuration scan begins. The arm must be bench-supported.
        emergencyDisableAll();

        for (Joint &joint : joints_)
        {
            if (!prepareJoint(joint))
            {
                std::cerr << "Left-arm preparation failed at J" << joint.number
                          << "/ID " << joint.slave_id << ".\n";
                emergencyDisableAll();
                return false;
            }
        }

        mode_ = ControlMode::Observe;
        std::cout << "All seven joints prepared with torque OFF. Mode=OBSERVE.\n";
        return printAllStates();
    }

    bool printAllStates()
    {
        std::cout << "joint bus id status  pos  delta_deg target moving torque temp  voltage current\n";
        for (Joint &joint : joints_)
        {
            ServoState state;
            if (!readState(joint, state))
                return false;
            joint.enabled = state.torque_enabled;
            if (!validateState(joint, state, "status"))
                return false;
            const bool should_be_enabled =
                mode_ == ControlMode::Group ||
                (mode_ == ControlMode::SingleJoint && joint.slave_id == active_joint_id_);
            if (state.torque_enabled != should_be_enabled)
            {
                std::cerr << "Unexpected torque state at J" << joint.number << ".\n";
                return false;
            }
            std::cout << 'J' << joint.number << "    L" << (joint.bus_index + 1)
                      << "  " << joint.slave_id << "  0x" << std::hex << std::setw(4)
                      << std::setfill('0') << state.status_word << std::dec << std::setfill(' ')
                      << "  " << std::setw(4) << state.position
                      << "  " << std::fixed << std::setprecision(3) << std::setw(9)
                      << (stepsToDegrees(state.position - joint.initial_position) * joint.direction)
                      << "  " << std::setw(4) << state.target_position
                      << "  " << (state.moving ? "yes" : " no")
                      << "    " << (state.torque_enabled ? "ON " : "OFF")
                      << "    " << std::setw(4) << state.temperature_c
                      << "  " << std::setw(6) << state.voltage_v
                      << "  " << state.current_a << '\n';
        }
        printModeAndQualification();
        return true;
    }

    bool selectSingleJoint(int slave_id)
    {
        if (mode_ != ControlMode::Observe)
        {
            std::cerr << "Select rejected: execute 'stop' before selecting another joint.\n";
            return false;
        }
        Joint *joint = findJoint(slave_id);
        if (!joint)
        {
            std::cerr << "Select rejected: ID must be 1..7.\n";
            return false;
        }

        if (!preflightAll(false))
            return false;

        ServoState before;
        if (!readState(*joint, before) || !validateState(*joint, before, "before enable"))
            return false;

        joint->target_position = before.position;
        if (!busFor(*joint).writeVerified(joint->slave_id, kTargetPosition,
                                          static_cast<std::uint16_t>(before.position),
                                          "target=current"))
            return false;

        joint->enabled = true; // Conservative if the write response/readback is lost.
        if (!busFor(*joint).writeVerified(joint->slave_id, kTorqueEnable, 1, "torque on"))
        {
            emergencyDisableAll();
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ServoState after;
        if (!readState(*joint, after) || !validateState(*joint, after, "after enable") ||
            !after.torque_enabled ||
            std::abs(after.position - before.position) > kMaximumEnableJumpSteps)
        {
            std::cerr << "J" << joint->number << " enable verification failed.\n";
            emergencyDisableAll();
            return false;
        }

        mode_ = ControlMode::SingleJoint;
        active_joint_id_ = slave_id;
        std::cout << "SINGLE mode: J" << joint->number << "/ID " << slave_id
                  << " enabled at raw " << after.position << ". Other joints remain OFF.\n";
        return true;
    }

    bool moveSingle(int slave_id, double logical_degrees)
    {
        if (mode_ != ControlMode::SingleJoint || active_joint_id_ != slave_id)
        {
            std::cerr << "Move rejected: first use 'select " << slave_id << "'.\n";
            return false;
        }
        if (!validMove(logical_degrees, kSingleMoveMaximumDegrees))
        {
            std::cerr << "Single-joint move magnitude must be 0.1.."
                      << kSingleMoveMaximumDegrees << " degrees.\n";
            return false;
        }

        Joint &joint = *findJoint(slave_id);
        ServoState state;
        if (!readState(joint, state) || !validateState(joint, state, "single move preflight"))
        {
            controlledStopAll();
            return false;
        }
        if (!state.torque_enabled)
        {
            std::cerr << "Move rejected: selected joint torque is OFF.\n";
            return false;
        }

        const int raw_delta = joint.direction * degreesToSteps(logical_degrees);
        const int requested = state.position + raw_delta;
        if (!targetInsideWorkspace(joint, requested))
            return false;

        joint.target_position = requested;
        if (!busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                         static_cast<std::uint16_t>(requested),
                                         "single small target"))
            return false;

        std::array<bool, 7> active{};
        active[static_cast<std::size_t>(joint.number - 1)] = true;
        std::cout << "J" << joint.number << " logical move " << logical_degrees
                  << " deg -> raw target " << requested << ".\n";
        return waitForTargets(active, std::chrono::seconds(5));
    }

    bool qualifySingle(int slave_id)
    {
        if (mode_ != ControlMode::SingleJoint || active_joint_id_ != slave_id)
        {
            std::cerr << "Pass rejected: the same ID must be active in SINGLE mode.\n";
            return false;
        }
        Joint &joint = *findJoint(slave_id);
        ServoState state;
        if (!readState(joint, state) || !validateState(joint, state, "qualification"))
        {
            controlledStopAll();
            return false;
        }
        if (!state.torque_enabled || state.moving ||
            std::abs(state.position - joint.target_position) > kPositionToleranceSteps)
        {
            std::cerr << "Pass rejected: joint is not stably holding its target.\n";
            return false;
        }

        if (!controlledStopAll())
            return false;
        joint.qualified = true;
        std::cout << "J" << joint.number << "/ID " << slave_id
                  << " marked qualified for this process session.\n";
        printModeAndQualification();
        return true;
    }

    bool enableGroup(const std::string &confirmation)
    {
        if (mode_ != ControlMode::Observe)
        {
            std::cerr << "Group enable rejected: execute 'stop' first.\n";
            return false;
        }
        for (const Joint &joint : joints_)
        {
            if (!joint.qualified)
            {
                std::cerr << "Group enable rejected: J" << joint.number
                          << " has not passed this session.\n";
                return false;
            }
        }
        if (confirmation != "LEFT_ARM")
        {
            std::cerr << "Group enable rejected: type 'enable-all LEFT_ARM'.\n";
            return false;
        }
        if (!preflightAll(false))
            return false;

        std::array<int, 7> positions{};
        for (Joint &joint : joints_)
        {
            ServoState state;
            if (!readState(joint, state) || !validateState(joint, state, "group enable preflight"))
                return false;
            positions[static_cast<std::size_t>(joint.number - 1)] = state.position;
        }
        for (Joint &joint : joints_)
        {
            const int position = positions[static_cast<std::size_t>(joint.number - 1)];
            joint.target_position = position;
            if (!busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                             static_cast<std::uint16_t>(position),
                                             "group target=current"))
            {
                emergencyDisableAll();
                return false;
            }
        }
        for (Joint &joint : joints_)
        {
            joint.enabled = true;
            if (!busFor(joint).writeVerified(joint.slave_id, kTorqueEnable, 1,
                                             "group torque on"))
            {
                emergencyDisableAll();
                return false;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        for (Joint &joint : joints_)
        {
            ServoState state;
            const int before = positions[static_cast<std::size_t>(joint.number - 1)];
            if (!readState(joint, state) || !validateState(joint, state, "after group enable") ||
                !state.torque_enabled ||
                std::abs(state.position - before) > kMaximumEnableJumpSteps)
            {
                std::cerr << "Group enable verification failed at J" << joint.number << ".\n";
                emergencyDisableAll();
                return false;
            }
        }

        mode_ = ControlMode::Group;
        active_joint_id_ = 0;
        std::cout << "GROUP mode enabled. All targets were initialized from current positions.\n";
        return true;
    }

    bool moveGroup(const std::array<double, 7> &logical_degrees)
    {
        if (mode_ != ControlMode::Group)
        {
            std::cerr << "Group move rejected: arm is not in GROUP mode.\n";
            return false;
        }

        std::array<int, 7> targets{};
        for (Joint &joint : joints_)
        {
            const std::size_t index = static_cast<std::size_t>(joint.number - 1);
            const double degrees = logical_degrees[index];
            if (degrees != 0.0 && !validMove(degrees, kGroupMoveMaximumDegrees))
            {
                std::cerr << "Group move rejected at J" << joint.number
                          << ": each non-zero magnitude must be 0.1.."
                          << kGroupMoveMaximumDegrees << " deg.\n";
                return false;
            }

            ServoState state;
            if (!readState(joint, state) || !validateState(joint, state, "group move preflight") ||
                !state.torque_enabled)
            {
                std::cerr << "Group move preflight failed at J" << joint.number << ".\n";
                controlledStopAll();
                return false;
            }
            targets[index] = state.position +
                             joint.direction * degreesToSteps(degrees);
            if (!targetInsideWorkspace(joint, targets[index]))
                return false;
        }

        bool first_write = true;
        std::chrono::steady_clock::time_point first_time{};
        std::chrono::steady_clock::time_point last_time{};
        for (Joint &joint : joints_)
        {
            const std::size_t index = static_cast<std::size_t>(joint.number - 1);
            const auto before_write = std::chrono::steady_clock::now();
            if (first_write)
            {
                first_time = before_write;
                first_write = false;
            }
            if (!busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                             static_cast<std::uint16_t>(targets[index]),
                                             "group small target"))
            {
                controlledStopAll();
                return false;
            }
            joint.target_position = targets[index];
            last_time = std::chrono::steady_clock::now();
        }

        const auto skew = std::chrono::duration_cast<std::chrono::milliseconds>(
                              last_time - first_time)
                              .count();
        std::cout << "Seven targets accepted sequentially; measured command span="
                  << skew << " ms (not atomic synchronization).\n";

        std::array<bool, 7> active{};
        active.fill(true);
        return waitForTargets(active, std::chrono::seconds(7));
    }

    bool holdActive()
    {
        std::array<bool, 7> active{};
        if (mode_ == ControlMode::SingleJoint)
            active[static_cast<std::size_t>(active_joint_id_ - 1)] = true;
        else if (mode_ == ControlMode::Group)
            active.fill(true);
        else
        {
            std::cout << "OBSERVE mode already has all torque OFF.\n";
            return true;
        }

        std::array<int, 7> positions{};
        for (Joint &joint : joints_)
        {
            const std::size_t index = static_cast<std::size_t>(joint.number - 1);
            if (!active[index])
                continue;
            ServoState state;
            if (!readState(joint, state))
            {
                controlledStopAll();
                return false;
            }
            positions[index] = state.position;
        }
        for (Joint &joint : joints_)
        {
            const std::size_t index = static_cast<std::size_t>(joint.number - 1);
            if (!active[index])
                continue;
            joint.target_position = positions[index];
            if (!busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                             static_cast<std::uint16_t>(positions[index]),
                                             "HOLD target"))
            {
                controlledStopAll();
                return false;
            }
        }
        std::cout << "HOLD targets captured for active joint(s).\n";
        return waitUntilStopped(active, std::chrono::seconds(2));
    }

    bool controlledStopAll()
    {
        bool hold_ok = true;
        std::array<int, 7> positions{};
        std::array<bool, 7> captured{};

        for (Joint &joint : joints_)
        {
            if (!joint.enabled)
                continue;
            ServoState state;
            if (readState(joint, state) && positionIsSingleTurn(state.position))
            {
                const std::size_t index = static_cast<std::size_t>(joint.number - 1);
                positions[index] = state.position;
                captured[index] = true;
            }
            else
            {
                hold_ok = false;
            }
        }

        for (Joint &joint : joints_)
        {
            const std::size_t index = static_cast<std::size_t>(joint.number - 1);
            if (!captured[index])
                continue;
            joint.target_position = positions[index];
            if (!busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                             static_cast<std::uint16_t>(positions[index]),
                                             "stop HOLD target"))
                hold_ok = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        bool torque_off_ok = true;
        for (Joint &joint : joints_)
        {
            if (busFor(joint).writeVerified(joint.slave_id, kTorqueEnable, 0,
                                            "torque off"))
                joint.enabled = false;
            else
                torque_off_ok = false;
        }

        if (torque_off_ok)
        {
            mode_ = ControlMode::Observe;
            active_joint_id_ = 0;
            std::cout << "STOP ALL complete; torque OFF confirmed for IDs 1..7.\n";
        }
        else
        {
            std::cerr << "Torque OFF could not be confirmed for every joint. "
                         "Use physical power isolation.\n";
        }
        return hold_ok && torque_off_ok;
    }

    bool monitor()
    {
        for (Joint &joint : joints_)
        {
            ServoState state;
            if (!readState(joint, state))
            {
                std::cerr << "COMMUNICATION LOST at J" << joint.number
                          << ". No reconnect or command replay.\n";
                emergencyDisableAll();
                return false;
            }
            if (!validateState(joint, state, "monitor"))
            {
                std::cerr << "Safety fault at J" << joint.number << ". Stopping all joints.\n";
                controlledStopAll();
                return false;
            }
            joint.enabled = state.torque_enabled;

            const bool should_be_enabled =
                mode_ == ControlMode::Group ||
                (mode_ == ControlMode::SingleJoint && joint.slave_id == active_joint_id_);
            if (state.torque_enabled != should_be_enabled)
            {
                std::cerr << "Unexpected torque state at J" << joint.number
                          << ". Stopping all joints.\n";
                controlledStopAll();
                return false;
            }
        }
        return true;
    }

    bool allBusesHealthy() const
    {
        return buses_[0].healthy() && buses_[1].healthy();
    }

    bool anyEnabled() const
    {
        return std::any_of(joints_.begin(), joints_.end(),
                           [](const Joint &joint) { return joint.enabled; });
    }

    void printHelp() const
    {
        std::cout
            << "Commands:\n"
            << "  status                         Read all seven joints\n"
            << "  select ID                      Enable only ID (1..7), all others remain OFF\n"
            << "  move ID DEG                    SINGLE logical move, magnitude 0.1..1.0 deg\n"
            << "  pass ID                        Confirm physical observation, stop, mark ID qualified\n"
            << "  qualified                      Show per-session qualification\n"
            << "  enable-all LEFT_ARM            Enable all only after IDs 1..7 pass\n"
            << "  move-all D1 D2 D3 D4 D5 D6 D7 Logical group deltas; each 0 or 0.1..0.5 deg\n"
            << "  hold                           Capture current position for active joint(s)\n"
            << "  stop                           HOLD active joints, torque OFF all IDs\n"
            << "  disconnect-test                STOP ALL, then safely test adapter unplug\n"
            << "  help                           Show commands\n"
            << "  quit                           STOP ALL and exit\n";
    }

    void printModeAndQualification() const
    {
        std::cout << "mode=";
        if (mode_ == ControlMode::Observe)
            std::cout << "OBSERVE";
        else if (mode_ == ControlMode::SingleJoint)
            std::cout << "SINGLE(ID " << active_joint_id_ << ')';
        else
            std::cout << "GROUP";
        std::cout << " qualified=";
        for (const Joint &joint : joints_)
            std::cout << 'J' << joint.number << ':' << (joint.qualified ? "yes" : "no") << ' ';
        std::cout << '\n';
    }

private:
    BusPort &busFor(const Joint &joint)
    {
        return buses_[static_cast<std::size_t>(joint.bus_index)];
    }

    Joint *findJoint(int slave_id)
    {
        if (slave_id < 1 || slave_id > 7)
            return nullptr;
        return &joints_[static_cast<std::size_t>(slave_id - 1)];
    }

    bool prepareJoint(Joint &joint)
    {
        BusPort &bus = busFor(joint);
        std::uint16_t reported_id = 0;
        std::uint16_t configured_minimum = 0;
        std::uint16_t configured_maximum = 0;
        std::uint16_t configuration[4]{};

        if (!bus.readOne(joint.slave_id, kDeviceId, reported_id) ||
            !bus.readOne(joint.slave_id, kMinimumAngleLimit, configured_minimum) ||
            !bus.readOne(joint.slave_id, kMaximumAngleLimit, configured_maximum) ||
            !bus.readRegisters(joint.slave_id, kOperatingMode, 4, configuration))
            return false;

        if (reported_id != static_cast<std::uint16_t>(joint.slave_id) ||
            configured_minimum >= configured_maximum || configured_maximum > 4095 ||
            configuration[0] != 0)
        {
            std::cerr << "J" << joint.number << " invalid identity/configuration: reported ID="
                      << reported_id << " limits=[" << configured_minimum << ','
                      << configured_maximum << "] mode=" << configuration[0] << '\n';
            return false;
        }

        ServoState state;
        if (!readState(joint, state))
            return false;
        joint.enabled = state.torque_enabled;
        if (!validateState(joint, state, "preparation") ||
            state.position < configured_minimum || state.position > configured_maximum)
            return false;

        if (!bus.writeVerified(joint.slave_id, kTargetPosition,
                               static_cast<std::uint16_t>(state.position), "target=current") ||
            !bus.writeVerified(joint.slave_id, kTorqueEnable, 0, "torque off") ||
            !bus.writeVerified(joint.slave_id, kAcceleration, kSafeAcceleration,
                               "safe acceleration") ||
            !bus.writeVerified(joint.slave_id, kSpeed, kSafeSpeed, "safe speed") ||
            !bus.writeVerified(joint.slave_id, kTorqueLimit, kSafeTorqueLimit,
                               "safe torque limit"))
            return false;
        joint.enabled = false;

        if (!readState(joint, state) || state.torque_enabled ||
            std::abs(state.target_position - state.position) > kPositionToleranceSteps)
            return false;

        joint.initial_position = state.position;
        joint.target_position = state.position;
        joint.workspace_min = std::max(static_cast<int>(configured_minimum),
                                       state.position - degreesToSteps(kWorkspaceDegrees));
        joint.workspace_max = std::min(static_cast<int>(configured_maximum),
                                       state.position + degreesToSteps(kWorkspaceDegrees));
        std::cout << "Prepared J" << joint.number << " on " << bus.device()
                  << " ID=" << joint.slave_id << " dir=" << joint.direction
                  << " pos=" << state.position << " workspace=[" << joint.workspace_min
                  << ',' << joint.workspace_max << "] mode=" << configuration[0]
                  << " P=" << configuration[1] << " D=" << configuration[2]
                  << " I=" << configuration[3] << " torque=OFF\n";
        return true;
    }

    bool readState(Joint &joint, ServoState &state)
    {
        std::uint16_t feedback[kStatusRegisterCount]{};
        std::uint16_t runtime[5]{};
        BusPort &bus = busFor(joint);
        if (!bus.readRegisters(joint.slave_id, kStatusStart, kStatusRegisterCount, feedback) ||
            !bus.readRegisters(joint.slave_id, kTargetPosition, 5, runtime))
            return false;

        state.status_word = feedback[0];
        state.position = static_cast<std::int16_t>(feedback[1]);
        state.speed_raw = static_cast<std::int16_t>(feedback[2]);
        state.pwm_raw = static_cast<std::int16_t>(feedback[3]);
        state.voltage_v = static_cast<double>(feedback[4]) * 0.1;
        state.temperature_c = static_cast<double>(feedback[5]);
        state.moving = feedback[6] != 0;
        state.current_a = static_cast<double>(feedback[7]) * 0.0065;
        state.target_position = static_cast<std::int16_t>(runtime[0]);
        state.torque_enabled = runtime[1] != 0;
        state.acceleration = runtime[2];
        state.speed_limit = runtime[3];
        state.torque_limit = runtime[4];
        return true;
    }

    bool validateState(const Joint &joint, const ServoState &state,
                       const std::string &phase) const
    {
        const std::uint16_t fault_bits =
            static_cast<std::uint16_t>(state.status_word & ~kTorqueEnabledStatusBit);
        if (fault_bits != 0 || !positionIsSingleTurn(state.position) ||
            state.temperature_c >= kMaximumTemperatureC ||
            state.voltage_v < kMinimumVoltageV || state.voltage_v > kMaximumVoltageV ||
            std::abs(state.current_a) >= kMaximumCurrentA)
        {
            std::cerr << phase << " J" << joint.number << " unsafe: status=0x"
                      << std::hex << state.status_word << std::dec
                      << " pos=" << state.position << " temp=" << state.temperature_c
                      << "C voltage=" << state.voltage_v << "V current="
                      << state.current_a << "A\n";
            return false;
        }
        return true;
    }

    bool preflightAll(bool expected_enabled)
    {
        for (Joint &joint : joints_)
        {
            ServoState state;
            if (!readState(joint, state) || !validateState(joint, state, "all-joint preflight"))
                return false;
            joint.enabled = state.torque_enabled;
            if (state.torque_enabled != expected_enabled)
            {
                std::cerr << "Preflight torque mismatch at J" << joint.number << ".\n";
                return false;
            }
        }
        return true;
    }

    static bool validMove(double degrees, double maximum)
    {
        return std::isfinite(degrees) && std::abs(degrees) >= 0.1 &&
               std::abs(degrees) <= maximum;
    }

    bool targetInsideWorkspace(const Joint &joint, int target) const
    {
        if (target < joint.workspace_min || target > joint.workspace_max)
        {
            std::cerr << "J" << joint.number << " target " << target
                      << " outside startup workspace [" << joint.workspace_min
                      << ',' << joint.workspace_max << "].\n";
            return false;
        }
        return true;
    }

    bool waitForTargets(const std::array<bool, 7> &active,
                        std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!g_stop_requested && std::chrono::steady_clock::now() < deadline)
        {
            bool all_reached = true;
            for (Joint &joint : joints_)
            {
                const std::size_t index = static_cast<std::size_t>(joint.number - 1);
                if (!active[index])
                    continue;
                ServoState state;
                if (!readState(joint, state) ||
                    !validateState(joint, state, "during movement") ||
                    !state.torque_enabled)
                {
                    controlledStopAll();
                    return false;
                }
                const int error = std::abs(state.position - joint.target_position);
                if (state.moving || error > kPositionToleranceSteps)
                    all_reached = false;
            }
            if (all_reached)
            {
                std::cout << "Active target(s) reached within "
                          << kPositionToleranceSteps << " steps.\n";
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        std::cerr << "Movement timeout or stop signal; stopping all joints.\n";
        controlledStopAll();
        return false;
    }

    bool waitUntilStopped(const std::array<bool, 7> &active,
                          std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!g_stop_requested && std::chrono::steady_clock::now() < deadline)
        {
            bool all_stopped = true;
            for (Joint &joint : joints_)
            {
                const std::size_t index = static_cast<std::size_t>(joint.number - 1);
                if (!active[index])
                    continue;
                ServoState state;
                if (!readState(joint, state) ||
                    !validateState(joint, state, "during HOLD") ||
                    !state.torque_enabled)
                {
                    controlledStopAll();
                    return false;
                }
                if (state.moving)
                    all_stopped = false;
            }
            if (all_stopped)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    }

    void emergencyDisableAll() noexcept
    {
        for (Joint &joint : joints_)
        {
            if (busFor(joint).bestEffortWrite(joint.slave_id, kTorqueEnable, 0))
                joint.enabled = false;
        }
    }

    std::array<BusPort, 2> buses_;
    std::array<Joint, 7> joints_{};
    ControlMode mode_ = ControlMode::Observe;
    int active_joint_id_ = 0;
};

bool parseGroupDeltas(std::istringstream &stream, std::array<double, 7> &deltas)
{
    for (double &delta : deltas)
    {
        if (!(stream >> delta))
            return false;
    }
    std::string extra;
    return !(stream >> extra);
}

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " --bench-confirmed [--direction-config cfg.txt]\n";
}
} // namespace

int main(int argc, char **argv)
{
    bool bench_confirmed = false;
    std::string direction_config = "cfg.txt";
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--bench-confirmed")
            bench_confirmed = true;
        else if (argument == "--direction-config" && i + 1 < argc)
            direction_config = argv[++i];
        else if (argument == "--help")
        {
            printUsage(argv[0]);
            return 0;
        }
        else
        {
            printUsage(argv[0]);
            return 2;
        }
    }

    if (!bench_confirmed)
    {
        printUsage(argv[0]);
        return 2;
    }
    if (std::system("test \"$(systemctl is-active startup.service 2>/dev/null)\" = inactive") != 0)
    {
        std::cerr << "startup.service is not confirmed inactive.\n";
        return 2;
    }

    std::array<int, 7> directions{};
    if (!loadLeftArmDirections(direction_config, directions))
        return 2;

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "LEFT-ARM SEVEN-JOINT BENCH CONTROL (NO UDP)\n"
              << "Topology: L1 IDs 1,2,3; L2 IDs 4,5,6,7.\n"
              << "Limits: speed=100 steps/s, acceleration=200 steps/s^2, torque=10%, "
                 "workspace=startup +/-3 deg.\n"
              << "All RTU operations are serialized; no automatic reconnect/replay.\n"
              << "Logical directions:";
    for (std::size_t i = 0; i < directions.size(); ++i)
        std::cout << " J" << (i + 1) << '=' << directions[i];
    std::cout << '\n';

    LeftArmController controller(directions);
    if (!controller.connectAndPrepare())
        return 1;
    controller.printHelp();

    bool prompt_visible = false;
    bool disconnect_test_armed = false;
    auto next_monitor = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);

    while (!g_stop_requested)
    {
        if (!prompt_visible)
        {
            std::cout << "left-arm> " << std::flush;
            prompt_visible = true;
        }

        pollfd input{};
        input.fd = STDIN_FILENO;
        input.events = POLLIN | POLLHUP | POLLERR;
        const int poll_result = poll(&input, 1, 100);
        if (poll_result < 0 && errno != EINTR)
        {
            std::cerr << "stdin poll failed: " << std::strerror(errno) << '\n';
            break;
        }

        if (poll_result > 0 && (input.revents & POLLIN))
        {
            std::string line;
            if (!std::getline(std::cin, line))
                break;
            prompt_visible = false;
            std::istringstream stream(line);
            std::string command;
            stream >> command;
            if (command.empty())
                continue;

            if (command == "status")
            {
                if (!controller.printAllStates())
                    break;
            }
            else if (command == "select")
            {
                int id = 0;
                if (!(stream >> id))
                    std::cerr << "Usage: select ID\n";
                else
                    controller.selectSingleJoint(id);
            }
            else if (command == "move")
            {
                int id = 0;
                double degrees = 0.0;
                if (!(stream >> id >> degrees))
                    std::cerr << "Usage: move ID DEG\n";
                else
                    controller.moveSingle(id, degrees);
            }
            else if (command == "pass")
            {
                int id = 0;
                if (!(stream >> id))
                    std::cerr << "Usage: pass ID\n";
                else
                    controller.qualifySingle(id);
            }
            else if (command == "qualified")
            {
                controller.printModeAndQualification();
            }
            else if (command == "enable-all")
            {
                std::string confirmation;
                stream >> confirmation;
                controller.enableGroup(confirmation);
            }
            else if (command == "move-all")
            {
                std::array<double, 7> deltas{};
                if (!parseGroupDeltas(stream, deltas))
                    std::cerr << "Usage: move-all D1 D2 D3 D4 D5 D6 D7\n";
                else
                    controller.moveGroup(deltas);
            }
            else if (command == "hold")
            {
                controller.holdActive();
            }
            else if (command == "stop")
            {
                controller.controlledStopAll();
            }
            else if (command == "disconnect-test")
            {
                if (controller.controlledStopAll())
                {
                    disconnect_test_armed = true;
                    std::cout << "Torque OFF confirmed for all IDs. Unplug only one selected "
                                 "left-arm adapter; the controller must report loss and exit.\n";
                }
            }
            else if (command == "help")
            {
                controller.printHelp();
            }
            else if (command == "quit" || command == "exit")
            {
                break;
            }
            else
            {
                std::cerr << "Unknown command. Type 'help'.\n";
            }

            if (!controller.allBusesHealthy())
            {
                std::cerr << "A Modbus transaction failed; leaving control mode.\n";
                break;
            }
        }
        else if (poll_result > 0 && (input.revents & (POLLHUP | POLLERR)))
        {
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now >= next_monitor)
        {
            if (!controller.monitor())
            {
                if (disconnect_test_armed)
                    std::cout << "Safe disconnect detection passed; no reconnect attempted.\n";
                break;
            }
            next_monitor = now + std::chrono::milliseconds(500);
        }
    }

    if (controller.anyEnabled())
        controller.controlledStopAll();
    std::cout << "Exiting left-arm bench control.\n";
    return g_stop_requested ? 130 : 0;
}
