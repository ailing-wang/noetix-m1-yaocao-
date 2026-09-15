#include "left_arm_local/Controller.h"

#include <modbus/modbus.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <dirent.h>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <limits.h>
#include <map>
#include <mutex>
#include <sstream>
#include <string>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <utility>
#include <vector>

namespace noetix::left_arm_local
{
namespace
{
using Clock = std::chrono::steady_clock;

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

// The dual-arm options carry per-joint safe limits as std::array while the
// single-arm build stores them as scalars; these overloads let the shared
// fade-stop path read either shape without #ifdeffery.
template <typename T>
std::enable_if_t<std::is_arithmetic_v<T>, std::uint16_t>
safeTorqueLimitAt(const T &value, std::size_t)
{
    return static_cast<std::uint16_t>(value);
}

template <typename T, std::size_t N>
std::uint16_t safeTorqueLimitAt(const std::array<T, N> &value, std::size_t index)
{
    return static_cast<std::uint16_t>(value[index]);
}

constexpr double kStepsPerRevolution = 4095.0;
constexpr double kMaximumTemperatureC = 65.0;
constexpr double kMaximumCurrentA = 0.8;
constexpr double kMinimumVoltageV = 8.0;
constexpr double kMaximumVoltageV = 26.0;
constexpr int kPositionToleranceSteps = 3;
constexpr int kMaximumEnableJumpSteps = 8;
constexpr std::size_t kMaximumStoredResults = 64;

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

bool startupServiceIsInactive(const std::string &service)
{
    if (service.empty())
        return false;
    for (const char character : service)
    {
        const bool safe = (character >= 'a' && character <= 'z') ||
                          (character >= 'A' && character <= 'Z') ||
                          (character >= '0' && character <= '9') ||
                          character == '-' || character == '_' || character == '.' ||
                          character == '@';
        if (!safe)
            return false;
    }
    const std::string command = "test \"$(systemctl is-active " + service +
                                " 2>/dev/null)\" = inactive";
    return std::system(command.c_str()) == 0;
}

bool positionIsSingleTurn(int position)
{
    return position >= 0 && position <= 4095;
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

struct Joint
{
    int number = 0;
    int slave_id = 0;
    int bus_index = 0;
    int direction = 1;
    int configured_minimum = 0;
    int configured_maximum = 4095;
    int initial_position = 0;
    int target_position = 0;
    int workspace_minimum = 0;
    int workspace_maximum = 4095;
    bool enabled = false;
    bool prepared = false;
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

    bool connectExclusive(std::string &error)
    {
        const auto holders = findProcessesHoldingDevice(device_);
        if (!holders.empty())
        {
            std::ostringstream message;
            message << "refusing to open " << device_ << ": held by PID";
            for (const int pid : holders)
                message << ' ' << pid;
            error = message.str();
            return false;
        }

        ctx_ = modbus_new_rtu(device_.c_str(), 115200, 'N', 8, 1);
        if (!ctx_)
        {
            error = "modbus_new_rtu failed for " + device_;
            return false;
        }
        modbus_set_response_timeout(ctx_, 0, 250000);
        modbus_set_byte_timeout(ctx_, 0, 50000);

        if (modbus_connect(ctx_) == -1)
        {
            error = errorText("connect");
            closePort();
            return false;
        }
        connected_ = true;

        const int fd = modbus_get_socket(ctx_);
        if (fd < 0 || flock(fd, LOCK_EX | LOCK_NB) == -1)
        {
            error = "could not lock " + device_ + " exclusively";
            closePort();
            return false;
        }
        if (ioctl(fd, TIOCEXCL) == -1)
        {
            error = "could not set TIOCEXCL on " + device_ + ": " +
                    std::strerror(errno);
            closePort();
            return false;
        }

        const auto holders_after_open = findProcessesHoldingDevice(device_);
        if (!holders_after_open.empty())
        {
            error = "another process opened " + device_ + " during startup";
            closePort();
            return false;
        }

        healthy_ = true;
        return true;
    }

    bool readRegisters(int slave_id, std::uint16_t address, int count,
                       std::uint16_t *destination, std::string &error)
    {
        if (!selectSlave(slave_id, error))
            return false;
        const int rc = modbus_read_registers(ctx_, address, count, destination);
        transactionDelay();
        if (rc != count)
        {
            error = errorText("read slave " + std::to_string(slave_id) +
                              " register 0x" + hexAddress(address));
            healthy_ = false;
            return false;
        }
        return true;
    }

    bool readOne(int slave_id, std::uint16_t address, std::uint16_t &value,
                 std::string &error)
    {
        return readRegisters(slave_id, address, 1, &value, error);
    }

    bool writeVerified(int slave_id, std::uint16_t address, std::uint16_t value,
                       const std::string &description, std::string &error)
    {
        if (!selectSlave(slave_id, error))
            return false;
        const int rc = modbus_write_register(ctx_, address, value);
        transactionDelay();
        if (rc != 1)
        {
            error = errorText("write slave " + std::to_string(slave_id) + ' ' + description);
            healthy_ = false;
            return false;
        }

        std::uint16_t readback = 0;
        if (!readOne(slave_id, address, readback, error))
            return false;
        if (readback != value)
        {
            std::ostringstream message;
            message << device_ << " ID " << slave_id << " readback mismatch for "
                    << description << ": wrote " << value << ", read " << readback;
            error = message.str();
            healthy_ = false;
            return false;
        }
        return true;
    }

    bool bestEffortWrite(int slave_id, std::uint16_t address,
                         std::uint16_t value) noexcept
    {
        if (!ctx_ || !connected_)
            return false;
        if (modbus_set_slave(ctx_, slave_id) == -1)
            return false;
        const bool ok = modbus_write_register(ctx_, address, value) == 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return ok;
    }

    bool healthy() const noexcept { return healthy_; }
    const std::string &device() const noexcept { return device_; }

private:
    bool selectSlave(int slave_id, std::string &error)
    {
        if (!ctx_ || !connected_)
        {
            error = device_ + " is not connected";
            healthy_ = false;
            return false;
        }
        if (modbus_set_slave(ctx_, slave_id) == -1)
        {
            error = errorText("select slave " + std::to_string(slave_id));
            healthy_ = false;
            return false;
        }
        return true;
    }

    std::string errorText(const std::string &operation) const
    {
        return device_ + ' ' + operation + " failed: " + modbus_strerror(errno);
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

class Hardware
{
public:
    explicit Hardware(const ControllerOptions &options)
        : options_(options),
          buses_{{BusPort(options.l1_device), BusPort(options.l2_device)}}
    {
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            Joint &joint = joints_[index];
            joint.number = static_cast<int>(index + 1);
            joint.slave_id = static_cast<int>(index + 1);
            joint.bus_index = index < 3 ? 0 : 1;
            joint.direction = options.directions[index];
        }
    }

    ~Hardware()
    {
        if (anyEnabled())
            emergencyDisableAll();
    }

    bool connectAndPrepare(std::array<ServoState, kJointCount> &states,
                           std::string &error)
    {
        if (!buses_[0].connectExclusive(error))
            return false;
        if (!buses_[1].connectExclusive(error))
        {
            emergencyDisableAll();
            return false;
        }

        emergencyDisableAll();
        for (Joint &joint : joints_)
        {
            if (!prepareJoint(joint, error))
            {
                error = "J" + std::to_string(joint.number) + " preparation failed: " + error;
                emergencyDisableAll();
                return false;
            }
        }
        if (!readAll(states, error) || !validateAll(states, false, "startup", error))
        {
            emergencyDisableAll();
            return false;
        }
        return true;
    }

    bool readAll(std::array<ServoState, kJointCount> &states, std::string &error)
    {
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (!readState(joints_[index], states[index], error))
                return false;
            joints_[index].enabled = states[index].torque_enabled;
        }
        return true;
    }

    bool validateAll(const std::array<ServoState, kJointCount> &states,
                     bool expected_torque, const std::string &phase,
                     std::string &error) const
    {
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (!validateState(joints_[index], states[index], phase, error))
                return false;
            if (states[index].torque_enabled != expected_torque)
            {
                error = phase + " J" + std::to_string(index + 1) +
                        " unexpected torque state";
                return false;
            }
        }
        return true;
    }

    bool enterControl(std::array<ServoState, kJointCount> &states,
                      long &command_span_ms, std::string &error)
    {
        if (!readAll(states, error) || !validateAll(states, false, "control preflight", error))
            return false;

        std::array<int, kJointCount> positions{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
            positions[index] = states[index].position;

        if (!writeTargets(positions, command_span_ms, error))
        {
            emergencyDisableAll();
            return false;
        }

        const auto first_time = Clock::now();
        auto last_time = first_time;
        for (Joint &joint : joints_)
        {
            joint.enabled = true;
            if (!busFor(joint).writeVerified(joint.slave_id, kTorqueEnable, 1,
                                             "control torque on", error))
            {
                emergencyDisableAll();
                return false;
            }
            last_time = Clock::now();
        }
        command_span_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              last_time - first_time)
                              .count();

        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        if (!readAll(states, error) || !validateAll(states, true, "after control enable", error))
        {
            emergencyDisableAll();
            return false;
        }
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (std::abs(states[index].position - positions[index]) > kMaximumEnableJumpSteps)
            {
                error = "J" + std::to_string(index + 1) +
                        " moved too far while enabling";
                emergencyDisableAll();
                return false;
            }
        }
        return true;
    }

    bool writeTargets(const std::array<int, kJointCount> &targets,
                      long &command_span_ms, std::string &error)
    {
        const auto first_time = Clock::now();
        auto last_time = first_time;
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            Joint &joint = joints_[index];
            if (!targetInsideLimits(joint, targets[index], error))
                return false;
            if (!busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                             static_cast<std::uint16_t>(targets[index]),
                                             "absolute trajectory target", error))
                return false;
            joint.target_position = targets[index];
            last_time = Clock::now();
        }
        command_span_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              last_time - first_time)
                              .count();
        return true;
    }

    bool hold(std::array<ServoState, kJointCount> &states,
              long &command_span_ms, std::string &error)
    {
        if (!readAll(states, error) || !validateAll(states, true, "HOLD preflight", error))
            return false;
        std::array<int, kJointCount> positions{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
            positions[index] = states[index].position;
        if (!writeTargets(positions, command_span_ms, error))
            return false;
        return waitStopped(states, std::chrono::seconds(2), error);
    }

    bool stopAll(std::array<ServoState, kJointCount> *states,
                 std::string &error)
    {
        bool hold_ok = true;
        std::array<int, kJointCount> positions{};
        std::array<bool, kJointCount> captured{};
        std::array<ServoState, kJointCount> local_states{};

        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            Joint &joint = joints_[index];
            if (!joint.enabled)
                continue;
            ServoState state;
            std::string read_error;
            if (readState(joint, state, read_error) && positionIsSingleTurn(state.position))
            {
                positions[index] = state.position;
                captured[index] = true;
            }
            else
            {
                hold_ok = false;
                if (error.empty())
                    error = read_error;
            }
        }

        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (!captured[index])
                continue;
            Joint &joint = joints_[index];
            std::string write_error;
            if (busFor(joint).writeVerified(joint.slave_id, kTargetPosition,
                                            static_cast<std::uint16_t>(positions[index]),
                                            "stop HOLD target", write_error))
                joint.target_position = positions[index];
            else
            {
                hold_ok = false;
                if (error.empty())
                    error = write_error;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

        bool torque_off_ok = true;
        for (Joint &joint : joints_)
        {
            std::string write_error;
            if (busFor(joint).writeVerified(joint.slave_id, kTorqueEnable, 0,
                                            "torque off", write_error))
                joint.enabled = false;
            else
            {
                torque_off_ok = false;
                if (error.empty())
                    error = write_error;
            }
        }

        if (states && busesHealthy())
        {
            std::string read_error;
            if (readAll(local_states, read_error))
                *states = local_states;
            else if (error.empty())
                error = read_error;
        }
        if (!hold_ok && error.empty())
            error = "could not capture every enabled joint before stopping";
        if (!torque_off_ok && error.empty())
            error = "could not confirm torque OFF for every joint";
        return hold_ok && torque_off_ok;
    }

    // Ramped torque-down variant of stopAll: hold current pose, then lower the
    // torque-limit register (0x84) in kFadeSteps steps over `fade`, and only
    // then switch torque off.  Lets an operator take over the arm's weight
    // smoothly during a control-source switch (no gravity sag jump).
    bool fadeStopAll(std::chrono::milliseconds fade,
                     std::array<ServoState, kJointCount> *states,
                     std::string &error)
    {
        constexpr int kFadeSteps = 10;
        std::array<int, kJointCount> positions{};
        std::array<bool, kJointCount> captured{};
        std::array<ServoState, kJointCount> local_states{};

        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            Joint &joint = joints_[index];
            if (!joint.enabled)
                continue;
            ServoState state;
            std::string read_error;
            if (readState(joint, state, read_error) &&
                positionIsSingleTurn(state.position))
            {
                positions[index] = state.position;
                captured[index] = true;
            }
            else if (error.empty())
                error = read_error;
        }
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (!captured[index])
                continue;
            Joint &joint = joints_[index];
            std::string write_error;
            if (!busFor(joint).writeVerified(
                    joint.slave_id, kTargetPosition,
                    static_cast<std::uint16_t>(positions[index]),
                    "fade stop HOLD target", write_error) &&
                error.empty())
                error = write_error;
        }

        std::array<std::uint16_t, kJointCount> start_limits{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            Joint &joint = joints_[index];
            start_limits[index] = safeTorqueLimitAt(options_.safe_torque_limit, index);
            if (!joint.enabled)
                continue;
            ServoState state;
            std::string read_error;
            if (readState(joint, state, read_error) && state.torque_limit > 0)
                start_limits[index] = state.torque_limit;
        }

        const auto step_delay = fade / kFadeSteps;
        for (int step = 1; step <= kFadeSteps; ++step)
        {
            for (std::size_t index = 0; index < joints_.size(); ++index)
            {
                if (!captured[index])
                    continue;
                Joint &joint = joints_[index];
                const std::uint32_t scaled =
                    static_cast<std::uint32_t>(start_limits[index]) *
                    static_cast<std::uint32_t>(kFadeSteps - step) / kFadeSteps;
                const std::uint16_t value = static_cast<std::uint16_t>(
                    std::max<std::uint32_t>(
                        scaled, safeTorqueLimitAt(options_.safe_torque_limit,
                                                  index)));
                std::string write_error;
                if (!busFor(joint).writeVerified(joint.slave_id, kTorqueLimit,
                                                 value, "torque fade",
                                                 write_error) &&
                    error.empty())
                    error = write_error;
            }
            if (step < kFadeSteps)
                std::this_thread::sleep_for(step_delay);
        }

        bool torque_off_ok = true;
        for (Joint &joint : joints_)
        {
            std::string write_error;
            if (busFor(joint).writeVerified(joint.slave_id, kTorqueEnable, 0,
                                            "torque off", write_error))
                joint.enabled = false;
            else
            {
                torque_off_ok = false;
                if (error.empty())
                    error = write_error;
            }
        }
        if (states && busesHealthy())
        {
            std::string read_error;
            if (readAll(local_states, read_error))
                *states = local_states;
        }
        if (!torque_off_ok && error.empty())
            error = "could not confirm torque OFF for every joint";
        return torque_off_ok && error.empty();
    }

    bool waitStopped(std::array<ServoState, kJointCount> &states,
                     std::chrono::milliseconds timeout, std::string &error)
    {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline)
        {
            if (!readAll(states, error) || !validateAll(states, true, "waiting for HOLD", error))
                return false;
            const bool stopped = std::all_of(states.begin(), states.end(),
                                             [](const ServoState &state)
                                             { return !state.moving; });
            if (stopped)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        error = "HOLD stop timeout";
        return false;
    }

    bool targetsReached(const std::array<ServoState, kJointCount> &states,
                        const std::array<int, kJointCount> &targets) const
    {
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            if (states[index].moving ||
                std::abs(states[index].position - targets[index]) > kPositionToleranceSteps)
                return false;
        }
        return true;
    }

    bool targetInsideLimits(const Joint &joint, int target, std::string &error) const
    {
        if (target < joint.configured_minimum || target > joint.configured_maximum ||
            target < joint.workspace_minimum || target > joint.workspace_maximum)
        {
            std::ostringstream message;
            message << "J" << joint.number << " target " << target
                    << " outside hardware/session limits ["
                    << std::max(joint.configured_minimum, joint.workspace_minimum)
                    << ',' << std::min(joint.configured_maximum, joint.workspace_maximum)
                    << ']';
            error = message.str();
            return false;
        }
        return true;
    }

    const std::array<Joint, kJointCount> &joints() const noexcept { return joints_; }
    bool busesHealthy() const noexcept { return buses_[0].healthy() && buses_[1].healthy(); }
    bool anyEnabled() const noexcept
    {
        return std::any_of(joints_.begin(), joints_.end(),
                           [](const Joint &joint) { return joint.enabled; });
    }

    void emergencyDisableAll() noexcept
    {
        for (Joint &joint : joints_)
        {
            if (busFor(joint).bestEffortWrite(joint.slave_id, kTorqueEnable, 0))
                joint.enabled = false;
        }
    }

private:
    BusPort &busFor(const Joint &joint)
    {
        return buses_[static_cast<std::size_t>(joint.bus_index)];
    }

    bool prepareJoint(Joint &joint, std::string &error)
    {
        BusPort &bus = busFor(joint);
        std::uint16_t reported_id = 0;
        std::uint16_t configured_minimum = 0;
        std::uint16_t configured_maximum = 0;
        std::uint16_t configuration[4]{};

        if (!bus.readOne(joint.slave_id, kDeviceId, reported_id, error) ||
            !bus.readOne(joint.slave_id, kMinimumAngleLimit, configured_minimum, error) ||
            !bus.readOne(joint.slave_id, kMaximumAngleLimit, configured_maximum, error) ||
            !bus.readRegisters(joint.slave_id, kOperatingMode, 4, configuration, error))
            return false;

        if (reported_id != static_cast<std::uint16_t>(joint.slave_id) ||
            configured_minimum >= configured_maximum || configured_maximum > 4095 ||
            configuration[0] != 0)
        {
            std::ostringstream message;
            message << "identity/config invalid: reported ID=" << reported_id
                    << " limits=[" << configured_minimum << ',' << configured_maximum
                    << "] mode=" << configuration[0];
            error = message.str();
            return false;
        }

        ServoState state;
        if (!readState(joint, state, error) ||
            !validateState(joint, state, "preparation", error) ||
            state.position < configured_minimum || state.position > configured_maximum)
            return false;

        joint.enabled = state.torque_enabled;
        if (!bus.writeVerified(joint.slave_id, kTargetPosition,
                               static_cast<std::uint16_t>(state.position),
                               "target=current", error) ||
            !bus.writeVerified(joint.slave_id, kTorqueEnable, 0,
                               "torque off", error) ||
            !bus.writeVerified(joint.slave_id, kAcceleration, options_.safe_acceleration,
                               "safe acceleration", error) ||
            !bus.writeVerified(joint.slave_id, kSpeed, options_.safe_speed,
                               "safe speed", error) ||
            !bus.writeVerified(joint.slave_id, kTorqueLimit, options_.safe_torque_limit,
                               "safe torque limit", error))
            return false;
        joint.enabled = false;

        if (!readState(joint, state, error) || state.torque_enabled ||
            std::abs(state.target_position - state.position) > kPositionToleranceSteps)
        {
            if (error.empty())
                error = "target/current or torque verification failed";
            return false;
        }

        const int workspace_steps = static_cast<int>(std::lround(
            options_.session_workspace_degrees * kStepsPerRevolution / 360.0));
        joint.configured_minimum = configured_minimum;
        joint.configured_maximum = configured_maximum;
        joint.initial_position = state.position;
        joint.target_position = state.position;
        joint.workspace_minimum = std::max(static_cast<int>(configured_minimum),
                                           state.position - workspace_steps);
        joint.workspace_maximum = std::min(static_cast<int>(configured_maximum),
                                           state.position + workspace_steps);
        joint.prepared = true;
        return true;
    }

    bool readState(Joint &joint, ServoState &state, std::string &error)
    {
        std::uint16_t feedback[kStatusRegisterCount]{};
        std::uint16_t runtime[5]{};
        BusPort &bus = busFor(joint);
        if (!bus.readRegisters(joint.slave_id, kStatusStart,
                               kStatusRegisterCount, feedback, error) ||
            !bus.readRegisters(joint.slave_id, kTargetPosition, 5, runtime, error))
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
                       const std::string &phase, std::string &error) const
    {
        const std::uint16_t fault_bits =
            static_cast<std::uint16_t>(state.status_word & ~kTorqueEnabledStatusBit);
        const bool outside_session = joint.prepared &&
                                     (state.position < joint.workspace_minimum ||
                                      state.position > joint.workspace_maximum);
        if (fault_bits != 0 || !positionIsSingleTurn(state.position) || outside_session ||
            state.position < joint.configured_minimum ||
            state.position > joint.configured_maximum ||
            state.temperature_c >= kMaximumTemperatureC ||
            state.voltage_v < kMinimumVoltageV || state.voltage_v > kMaximumVoltageV ||
            std::abs(state.current_a) >= kMaximumCurrentA)
        {
            std::ostringstream message;
            message << phase << " J" << joint.number << " unsafe: status=0x"
                    << std::hex << state.status_word << std::dec
                    << " pos=" << state.position << " temp=" << state.temperature_c
                    << "C voltage=" << state.voltage_v << "V current="
                    << state.current_a << 'A';
            error = message.str();
            return false;
        }
        return true;
    }

    ControllerOptions options_;
    std::array<BusPort, 2> buses_;
    std::array<Joint, kJointCount> joints_{};
};

enum class CommandKind
{
    Control,
    Trajectory,
    Hold,
    Heartbeat,
    Stop,
    Reset,
    FadeStop
};

struct Command
{
    CommandKind kind = CommandKind::Stop;
    std::uint64_t sequence = 0;
    Clock::time_point accepted_at{};
    Clock::time_point deadline{};
    std::chrono::milliseconds duration{0};
    std::array<double, kJointCount> targets{};
};

struct ProcessOutcome
{
    ResultCode code = ResultCode::Completed;
    std::string message;
    bool latch_fault = false;
};
} // namespace

std::int64_t monotonicMilliseconds() noexcept
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               Clock::now().time_since_epoch())
        .count();
}

double rawToAbsoluteDegrees(int raw, int direction) noexcept
{
    return static_cast<double>(raw - 2048) * 360.0 / kStepsPerRevolution *
           static_cast<double>(direction);
}

int absoluteDegreesToRaw(double degrees, int direction) noexcept
{
    return 2048 + static_cast<int>(std::lround(
                      degrees * kStepsPerRevolution / 360.0 *
                      static_cast<double>(direction)));
}

const char *toString(Mode mode) noexcept
{
    switch (mode)
    {
    case Mode::Starting: return "STARTING";
    case Mode::Observe: return "OBSERVE";
    case Mode::Control: return "CONTROL";
    case Mode::Fault: return "FAULT";
    case Mode::Stopped: return "STOPPED";
    }
    return "UNKNOWN";
}

const char *toString(ResultCode code) noexcept
{
    switch (code)
    {
    case ResultCode::Completed: return "COMPLETED";
    case ResultCode::Rejected: return "REJECTED";
    case ResultCode::Expired: return "EXPIRED";
    case ResultCode::Cancelled: return "CANCELLED";
    case ResultCode::Faulted: return "FAULTED";
    }
    return "UNKNOWN";
}

class Controller::Impl
{
public:
    explicit Impl(ControllerOptions options) : options_(std::move(options))
    {
        snapshot_.mode = Mode::Starting;
        snapshot_.monotonic_timestamp_ms = monotonicMilliseconds();
    }

    ~Impl()
    {
        shutdown();
    }

    bool start(std::string &error)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (worker_.joinable() || start_attempted_)
            {
                error = "controller start may only be called once";
                return false;
            }
            start_attempted_ = true;
        }

        for (const int direction : options_.directions)
        {
            if (direction != -1 && direction != 1)
            {
                error = "every joint direction must be -1 or 1";
                return false;
            }
        }
        if (options_.session_workspace_degrees <= 0.0 ||
            options_.maximum_command_delta_degrees <= 0.0 ||
            options_.maximum_trajectory_speed_degrees_per_second <= 0.0 ||
            options_.trajectory_tick.count() <= 0 ||
            options_.observe_period.count() <= 0 ||
            options_.control_period.count() <= 0 ||
            options_.maximum_queue_depth == 0)
        {
            error = "invalid controller options";
            return false;
        }

        worker_ = std::thread(&Impl::workerMain, this);
        std::unique_lock<std::mutex> lock(mutex_);
        const bool finished = initialization_cv_.wait_for(
            lock, std::chrono::seconds(30), [this] { return initialization_done_; });
        if (!finished)
        {
            error = "controller initialization timed out";
            shutdown_requested_.store(true);
            abort_requested_.store(true);
            command_cv_.notify_all();
            lock.unlock();
            if (worker_.joinable())
                worker_.join();
            return false;
        }
        if (!initialization_success_)
        {
            error = initialization_error_;
            lock.unlock();
            if (worker_.joinable())
                worker_.join();
            return false;
        }
        return true;
    }

    void shutdown()
    {
        shutdown_requested_.store(true);
        abort_requested_.store(true);
        command_cv_.notify_all();
        if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id())
            worker_.join();
    }

    SubmitResult submitControl(std::uint64_t sequence,
                               std::chrono::milliseconds ttl)
    {
        if (ttl < std::chrono::seconds(1) || ttl > std::chrono::seconds(30))
            return rejectedSubmission(sequence, "CONTROL TTL must be 1000..30000 ms");
        Command command;
        command.kind = CommandKind::Control;
        command.sequence = sequence;
        command.duration = ttl;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitTrajectory(
        std::uint64_t sequence,
        std::chrono::milliseconds ttl,
        std::chrono::milliseconds duration,
        const std::array<double, kJointCount> &absolute_degrees)
    {
        if (duration < std::chrono::milliseconds(200) ||
            duration > std::chrono::seconds(5))
            return rejectedSubmission(sequence, "trajectory duration must be 200..5000 ms");
        if (ttl < duration + std::chrono::milliseconds(500) ||
            ttl > std::chrono::seconds(30))
            return rejectedSubmission(sequence,
                                      "trajectory TTL must cover duration + 500 ms and be <=30000 ms");
        if (!std::all_of(absolute_degrees.begin(), absolute_degrees.end(),
                         [](double value) { return std::isfinite(value); }))
            return rejectedSubmission(sequence, "all absolute targets must be finite");

        Command command;
        command.kind = CommandKind::Trajectory;
        command.sequence = sequence;
        command.duration = duration;
        command.targets = absolute_degrees;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitHold(std::uint64_t sequence,
                            std::chrono::milliseconds ttl)
    {
        if (ttl < std::chrono::milliseconds(500) || ttl > std::chrono::seconds(30))
            return rejectedSubmission(sequence, "HOLD TTL must be 500..30000 ms");
        Command command;
        command.kind = CommandKind::Hold;
        command.sequence = sequence;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitHeartbeat(std::uint64_t sequence,
                                 std::chrono::milliseconds ttl)
    {
        if (ttl < std::chrono::seconds(1) || ttl > std::chrono::seconds(30))
            return rejectedSubmission(sequence, "heartbeat TTL must be 1000..30000 ms");
        Command command;
        command.kind = CommandKind::Heartbeat;
        command.sequence = sequence;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitStop(std::uint64_t sequence)
    {
        Command command;
        command.kind = CommandKind::Stop;
        command.sequence = sequence;
        abort_requested_.store(true);
        SubmitResult result = submit(std::move(command), std::chrono::seconds(5), true);
        if (!result.accepted &&
            result.message == "sequence must be non-zero and strictly increasing")
        {
            command_cv_.notify_all();
            const auto now = Clock::now();
            result.accepted = true;
            result.accepted_at_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        now.time_since_epoch())
                                        .count();
            result.deadline_at_ms = 0;
            result.message =
                "out-of-band safety STOP requested; stale sequence was not queued and "
                "will not produce a sequenced RESULT";
        }
        return result;
    }

    SubmitResult submitFadeStop(std::uint64_t sequence,
                                std::chrono::milliseconds fade)
    {
        if (fade < std::chrono::milliseconds(100) ||
            fade > std::chrono::seconds(10))
            return rejectedSubmission(sequence,
                                      "FADE_STOP fade must be 100..10000 ms");
        Command command;
        command.kind = CommandKind::FadeStop;
        command.sequence = sequence;
        command.duration = fade;
        abort_requested_.store(true);
        SubmitResult result = submit(std::move(command),
                                     fade + std::chrono::seconds(5), true);
        if (!result.accepted &&
            result.message == "sequence must be non-zero and strictly increasing")
        {
            command_cv_.notify_all();
            const auto now = Clock::now();
            result.accepted = true;
            result.accepted_at_ms =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch())
                    .count();
            result.deadline_at_ms = 0;
            result.message =
                "out-of-band FADE_STOP requested; stale sequence was not queued "
                "and will not produce a sequenced RESULT";
        }
        return result;
    }

    SubmitResult submitReset(std::uint64_t sequence,
                             std::chrono::milliseconds ttl)
    {
        if (ttl < std::chrono::milliseconds(500) || ttl > std::chrono::seconds(30))
            return rejectedSubmission(sequence, "RESET TTL must be 500..30000 ms");
        Command command;
        command.kind = CommandKind::Reset;
        command.sequence = sequence;
        return submit(std::move(command), ttl, false);
    }

    Snapshot snapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        Snapshot copy = snapshot_;
        copy.queue_depth = command_queue_.size();
        return copy;
    }

    bool waitForResult(std::uint64_t sequence,
                       std::chrono::milliseconds timeout,
                       CommandResult &result) const
    {
        std::unique_lock<std::mutex> lock(mutex_);
        const bool available = result_cv_.wait_for(lock, timeout, [this, sequence]
                                                   { return results_.count(sequence) != 0; });
        if (!available)
            return false;
        result = results_.at(sequence);
        return true;
    }

private:
    SubmitResult rejectedSubmission(std::uint64_t sequence,
                                    const std::string &message) const
    {
        SubmitResult result;
        result.sequence = sequence;
        result.message = message;
        return result;
    }

    SubmitResult submit(Command command, std::chrono::milliseconds ttl,
                        bool high_priority_stop)
    {
        const auto accepted_at = Clock::now();
        command.accepted_at = accepted_at;
        command.deadline = accepted_at + ttl;

        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialization_success_ || shutdown_requested_.load() ||
            snapshot_.mode == Mode::Stopped)
            return rejectedSubmission(command.sequence, "controller is not running");
        if (command.sequence == 0 || command.sequence <= snapshot_.last_submitted_sequence)
            return rejectedSubmission(command.sequence,
                                      "sequence must be non-zero and strictly increasing");

        if (high_priority_stop)
        {
            for (const Command &queued : command_queue_)
                recordResultLocked(queued.sequence, ResultCode::Cancelled,
                                   "cancelled by STOP sequence " +
                                       std::to_string(command.sequence));
            command_queue_.clear();
            command_queue_.push_front(command);
        }
        else
        {
            if (command_queue_.size() >= options_.maximum_queue_depth)
                return rejectedSubmission(command.sequence, "command queue is full");
            command_queue_.push_back(command);
        }

        snapshot_.last_submitted_sequence = command.sequence;
        snapshot_.queue_depth = command_queue_.size();
        command_cv_.notify_all();

        SubmitResult result;
        result.accepted = true;
        result.sequence = command.sequence;
        result.accepted_at_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    accepted_at.time_since_epoch())
                                    .count();
        result.deadline_at_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                    command.deadline.time_since_epoch())
                                    .count();
        result.message = high_priority_stop ? "STOP accepted with queue preemption"
                                            : "command accepted";
        return result;
    }

    void workerMain()
    {
        std::string error;
        if (options_.require_startup_inactive &&
            !startupServiceIsInactive(options_.startup_service))
        {
            finishInitialization(false,
                                 options_.startup_service + " is not confirmed inactive");
            return;
        }

        Hardware hardware(options_);
        std::array<ServoState, kJointCount> states{};
        if (!hardware.connectAndPrepare(states, error))
        {
            finishInitialization(false, error);
            return;
        }

        mode_ = Mode::Observe;
        publishSnapshot(hardware, states);
        finishInitialization(true, {});

        auto next_monitor = Clock::now() + options_.observe_period;
        while (!shutdown_requested_.load())
        {
            Command command;
            bool have_command = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                command_cv_.wait_until(lock, next_monitor, [this]
                                       {
                                           return shutdown_requested_.load() ||
                                                  abort_requested_.load() ||
                                                  !command_queue_.empty();
                                       });
                if (shutdown_requested_.load())
                    break;
                if (!command_queue_.empty())
                {
                    command = command_queue_.front();
                    command_queue_.pop_front();
                    snapshot_.queue_depth = command_queue_.size();
                    snapshot_.active_sequence = command.sequence;
                    have_command = true;
                }
            }

            if (have_command)
            {
                ProcessOutcome outcome = processCommand(hardware, states, command);
                if (outcome.latch_fault)
                    latchFault(hardware, states, outcome.message, command.sequence);
                recordResult(command.sequence, outcome.code, outcome.message);
                publishSnapshot(hardware, states);
                next_monitor = Clock::now() + monitorPeriod();
                continue;
            }

            if (abort_requested_.load())
            {
                std::string stop_error;
                hardware.stopAll(&states, stop_error);
                if (mode_ != Mode::Fault)
                    mode_ = Mode::Observe;
                abort_requested_.store(false);
                publishSnapshot(hardware, states);
                next_monitor = Clock::now() + monitorPeriod();
                continue;
            }

            if (Clock::now() >= next_monitor)
            {
                monitorHardware(hardware, states);
                publishSnapshot(hardware, states);
                next_monitor = Clock::now() + monitorPeriod();
            }
        }

        abort_requested_.store(true);
        std::string stop_error;
        hardware.stopAll(&states, stop_error);
        mode_ = Mode::Stopped;
        publishSnapshot(hardware, states);
        cancelQueuedCommands("controller shutdown");
    }

    void finishInitialization(bool success, const std::string &error)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        initialization_success_ = success;
        initialization_done_ = true;
        initialization_error_ = error;
        if (!success)
        {
            snapshot_.mode = Mode::Stopped;
            snapshot_.initialized = false;
            snapshot_.fault_reason = error;
            snapshot_.monotonic_timestamp_ms = monotonicMilliseconds();
        }
        initialization_cv_.notify_all();
    }

    std::chrono::milliseconds monitorPeriod() const
    {
        return mode_ == Mode::Control ? options_.control_period : options_.observe_period;
    }

    ProcessOutcome processCommand(Hardware &hardware,
                                  std::array<ServoState, kJointCount> &states,
                                  const Command &command)
    {
        if (command.kind != CommandKind::Stop &&
            command.kind != CommandKind::FadeStop &&
            Clock::now() > command.deadline)
            return {ResultCode::Expired, "command deadline expired", true};

        switch (command.kind)
        {
        case CommandKind::Control:
            return processControl(hardware, states, command);
        case CommandKind::Trajectory:
            return processTrajectory(hardware, states, command);
        case CommandKind::Hold:
            return processHold(hardware, states, command);
        case CommandKind::Heartbeat:
            return processHeartbeat(hardware, states, command);
        case CommandKind::Stop:
            return processStop(hardware, states);
        case CommandKind::FadeStop:
            return processFadeStop(hardware, states, command);
        case CommandKind::Reset:
            return processReset(hardware, states, command);
        }
        return {ResultCode::Rejected, "unknown command", false};
    }

    ProcessOutcome processControl(Hardware &hardware,
                                  std::array<ServoState, kJointCount> &states,
                                  const Command &command)
    {
        if (mode_ != Mode::Observe)
            return {ResultCode::Rejected, "CONTROL requires OBSERVE mode", false};
        long span = 0;
        std::string error;
        if (!hardware.enterControl(states, span, error))
            return {ResultCode::Faulted, "CONTROL failed: " + error, true};
        mode_ = Mode::Control;
        control_lease_deadline_ = command.deadline;
        setLastCommandSpan(span);
        abort_requested_.store(false);
        return {ResultCode::Completed, "CONTROL enabled with targets=current", false};
    }

    ProcessOutcome processTrajectory(Hardware &hardware,
                                     std::array<ServoState, kJointCount> &states,
                                     const Command &command)
    {
        if (mode_ != Mode::Control)
            return {ResultCode::Rejected, "TARGET requires CONTROL mode", false};

        std::string error;
        if (!hardware.readAll(states, error) ||
            !hardware.validateAll(states, true, "trajectory preflight", error))
            return {ResultCode::Faulted, "trajectory preflight failed: " + error, true};

        std::array<int, kJointCount> start_raw{};
        std::array<int, kJointCount> final_raw{};
        const auto &joints = hardware.joints();
        const double duration_seconds =
            static_cast<double>(command.duration.count()) / 1000.0;
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            start_raw[index] = states[index].position;
            final_raw[index] = absoluteDegreesToRaw(command.targets[index],
                                                    joints[index].direction);
            if (!hardware.targetInsideLimits(joints[index], final_raw[index], error))
                return {ResultCode::Rejected, error, false};

            const double current_degrees = rawToAbsoluteDegrees(
                start_raw[index], joints[index].direction);
            const double delta = std::abs(command.targets[index] - current_degrees);
            if (delta > options_.maximum_command_delta_degrees + 1e-9)
            {
                std::ostringstream message;
                message << "J" << (index + 1) << " absolute target delta " << delta
                        << " deg exceeds " << options_.maximum_command_delta_degrees;
                return {ResultCode::Rejected, message.str(), false};
            }
            if (delta / duration_seconds >
                options_.maximum_trajectory_speed_degrees_per_second + 1e-9)
            {
                std::ostringstream message;
                message << "J" << (index + 1) << " trajectory speed "
                        << delta / duration_seconds << " deg/s exceeds "
                        << options_.maximum_trajectory_speed_degrees_per_second;
                return {ResultCode::Rejected, message.str(), false};
            }
        }

        control_lease_deadline_ = command.deadline;
        const auto trajectory_start = Clock::now();
        const auto trajectory_end = trajectory_start + command.duration;
        auto next_tick = trajectory_start;
        bool final_target_sent = false;

        while (!final_target_sent)
        {
            if (shutdown_requested_.load() || abort_requested_.load())
            {
                std::string stop_error;
                hardware.stopAll(&states, stop_error);
                mode_ = Mode::Observe;
                return {ResultCode::Cancelled, "trajectory interrupted by STOP", false};
            }
            if (Clock::now() > command.deadline || Clock::now() > control_lease_deadline_)
                return {ResultCode::Expired, "trajectory or control lease expired", true};

            const auto now = Clock::now();
            const double elapsed = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(now - trajectory_start)
                    .count());
            const double total = static_cast<double>(
                std::chrono::duration_cast<std::chrono::microseconds>(command.duration).count());
            const double alpha = std::clamp(total > 0.0 ? elapsed / total : 1.0, 0.0, 1.0);
            std::array<int, kJointCount> waypoint{};
            for (std::size_t index = 0; index < kJointCount; ++index)
            {
                waypoint[index] = static_cast<int>(std::lround(
                    static_cast<double>(start_raw[index]) +
                    alpha * static_cast<double>(final_raw[index] - start_raw[index])));
            }
            if (alpha >= 1.0)
            {
                waypoint = final_raw;
                final_target_sent = true;
            }

            long span = 0;
            if (!hardware.writeTargets(waypoint, span, error))
                return {ResultCode::Faulted, "trajectory write failed: " + error, true};
            setLastCommandSpan(span);
            if (!hardware.readAll(states, error) ||
                !hardware.validateAll(states, true, "during trajectory", error))
                return {ResultCode::Faulted, "trajectory monitor failed: " + error, true};
            publishSnapshot(hardware, states);

            if (!final_target_sent)
            {
                next_tick += options_.trajectory_tick;
                if (next_tick > trajectory_end)
                    next_tick = trajectory_end;
                waitUntilInterruptible(next_tick);
            }
        }

        const auto arrival_deadline = std::min(command.deadline, Clock::now() +
                                                                   std::chrono::seconds(2));
        while (Clock::now() < arrival_deadline)
        {
            if (shutdown_requested_.load() || abort_requested_.load())
            {
                std::string stop_error;
                hardware.stopAll(&states, stop_error);
                mode_ = Mode::Observe;
                return {ResultCode::Cancelled, "trajectory arrival interrupted by STOP", false};
            }
            if (!hardware.readAll(states, error) ||
                !hardware.validateAll(states, true, "trajectory arrival", error))
                return {ResultCode::Faulted, "arrival verification failed: " + error, true};
            publishSnapshot(hardware, states);
            if (hardware.targetsReached(states, final_raw))
                return {ResultCode::Completed, "absolute trajectory target reached", false};
            waitUntilInterruptible(Clock::now() + std::chrono::milliseconds(50));
        }
        return {ResultCode::Faulted, "absolute trajectory arrival timeout", true};
    }

    ProcessOutcome processHold(Hardware &hardware,
                               std::array<ServoState, kJointCount> &states,
                               const Command &command)
    {
        if (mode_ != Mode::Control)
            return {ResultCode::Rejected, "HOLD requires CONTROL mode", false};
        long span = 0;
        std::string error;
        if (!hardware.hold(states, span, error))
            return {ResultCode::Faulted, "HOLD failed: " + error, true};
        control_lease_deadline_ = command.deadline;
        setLastCommandSpan(span);
        return {ResultCode::Completed, "HOLD captured all current positions", false};
    }

    ProcessOutcome processHeartbeat(Hardware &hardware,
                                    std::array<ServoState, kJointCount> &states,
                                    const Command &command)
    {
        if (mode_ != Mode::Control)
            return {ResultCode::Rejected, "HEARTBEAT requires CONTROL mode", false};
        std::string error;
        if (!hardware.readAll(states, error) ||
            !hardware.validateAll(states, true, "heartbeat", error))
            return {ResultCode::Faulted, "heartbeat validation failed: " + error, true};
        control_lease_deadline_ = command.deadline;
        return {ResultCode::Completed, "control lease renewed", false};
    }

    ProcessOutcome processStop(Hardware &hardware,
                               std::array<ServoState, kJointCount> &states)
    {
        std::string error;
        const bool stopped = hardware.stopAll(&states, error);
        abort_requested_.store(false);
        if (mode_ != Mode::Fault)
            mode_ = Mode::Observe;
        if (!stopped)
            return {ResultCode::Faulted, "STOP could not be fully confirmed: " + error, true};
        return {ResultCode::Completed, "STOP ALL confirmed torque OFF", false};
    }

    ProcessOutcome processFadeStop(Hardware &hardware,
                                   std::array<ServoState, kJointCount> &states,
                                   const Command &command)
    {
        std::string error;
        const bool faded = hardware.fadeStopAll(command.duration, &states, error);
        abort_requested_.store(false);
        if (mode_ != Mode::Fault)
            mode_ = Mode::Observe;
        if (!faded)
            return {ResultCode::Faulted,
                    "FADE_STOP could not be fully confirmed: " + error, true};
        return {ResultCode::Completed, "FADE_STOP confirmed torque OFF", false};
    }

    ProcessOutcome processReset(Hardware &hardware,
                                std::array<ServoState, kJointCount> &states,
                                const Command &)
    {
        if (mode_ != Mode::Fault)
            return {ResultCode::Rejected, "RESET requires FAULT mode", false};
        if (!hardware.busesHealthy())
            return {ResultCode::Rejected,
                    "RESET rejected: communication is terminally unhealthy; restart required",
                    false};

        std::string error;
        hardware.stopAll(&states, error);
        if (!hardware.readAll(states, error) ||
            !hardware.validateAll(states, false, "fault reset", error))
            return {ResultCode::Rejected, "RESET safety check failed: " + error, false};
        mode_ = Mode::Observe;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.fault_latched = false;
            snapshot_.fault_reason.clear();
        }
        return {ResultCode::Completed, "FAULT reset to OBSERVE", false};
    }

    void monitorHardware(Hardware &hardware,
                         std::array<ServoState, kJointCount> &states)
    {
        if (mode_ == Mode::Fault && !hardware.busesHealthy())
            return;
        if (mode_ == Mode::Control && Clock::now() > control_lease_deadline_)
        {
            latchFault(hardware, states, "control lease expired", 0);
            return;
        }

        std::string error;
        if (!hardware.readAll(states, error))
        {
            latchFault(hardware, states, "communication lost: " + error, 0);
            return;
        }
        const bool expected_torque = mode_ == Mode::Control;
        if (!hardware.validateAll(states, expected_torque, "periodic monitor", error))
            latchFault(hardware, states, error, 0);
    }

    void latchFault(Hardware &hardware,
                    std::array<ServoState, kJointCount> &states,
                    const std::string &reason, std::uint64_t source_sequence)
    {
        if (mode_ != Mode::Fault)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.fault_reason = reason +
                                     (source_sequence == 0
                                          ? std::string()
                                          : " [sequence " + std::to_string(source_sequence) + ']');
            snapshot_.fault_latched = true;
        }
        mode_ = Mode::Fault;
        abort_requested_.store(false);
        cancelQueuedCommands("cancelled by latched FAULT");
        std::string stop_error;
        hardware.stopAll(&states, stop_error);
    }

    void publishSnapshot(const Hardware &hardware,
                         const std::array<ServoState, kJointCount> &states)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.mode = mode_;
        snapshot_.initialized = mode_ != Mode::Starting && mode_ != Mode::Stopped;
        snapshot_.monotonic_timestamp_ms = monotonicMilliseconds();
        snapshot_.queue_depth = command_queue_.size();
        snapshot_.control_lease_remaining_ms = mode_ == Mode::Control
                                                   ? std::max<std::int64_t>(
                                                         0,
                                                         std::chrono::duration_cast<
                                                             std::chrono::milliseconds>(
                                                             control_lease_deadline_ - Clock::now())
                                                             .count())
                                                   : 0;
        const auto &joints = hardware.joints();
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            JointSnapshot &destination = snapshot_.joints[index];
            const ServoState &source = states[index];
            destination.joint = joints[index].number;
            destination.slave_id = joints[index].slave_id;
            destination.direction = joints[index].direction;
            destination.raw_position = source.position;
            destination.raw_target = source.target_position;
            destination.absolute_degrees = rawToAbsoluteDegrees(
                source.position, joints[index].direction);
            destination.target_degrees = rawToAbsoluteDegrees(
                source.target_position, joints[index].direction);
            destination.status_word = source.status_word;
            destination.speed_raw = source.speed_raw;
            destination.voltage_v = source.voltage_v;
            destination.temperature_c = source.temperature_c;
            destination.current_a = source.current_a;
            destination.moving = source.moving;
            destination.torque_enabled = source.torque_enabled;
        }
    }

    void setLastCommandSpan(long span)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.last_command_span_ms = span;
    }

    void recordResult(std::uint64_t sequence, ResultCode code,
                      const std::string &message)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        recordResultLocked(sequence, code, message);
        snapshot_.active_sequence = 0;
        result_cv_.notify_all();
    }

    void recordResultLocked(std::uint64_t sequence, ResultCode code,
                            const std::string &message)
    {
        CommandResult result;
        result.sequence = sequence;
        result.code = code;
        result.message = message;
        result.finished_at_ms = monotonicMilliseconds();
        results_[sequence] = result;
        while (results_.size() > kMaximumStoredResults)
            results_.erase(results_.begin());
        snapshot_.last_result_sequence = sequence;
        snapshot_.last_result_code = code;
        snapshot_.last_result_message = message;
    }

    void cancelQueuedCommands(const std::string &reason)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const Command &queued : command_queue_)
            recordResultLocked(queued.sequence, ResultCode::Cancelled, reason);
        command_queue_.clear();
        snapshot_.queue_depth = 0;
        result_cv_.notify_all();
    }

    void waitUntilInterruptible(Clock::time_point deadline)
    {
        std::unique_lock<std::mutex> lock(mutex_);
        command_cv_.wait_until(lock, deadline, [this]
                               {
                                   return shutdown_requested_.load() ||
                                          abort_requested_.load();
                               });
    }

    ControllerOptions options_;
    mutable std::mutex mutex_;
    mutable std::condition_variable result_cv_;
    std::condition_variable command_cv_;
    std::condition_variable initialization_cv_;
    std::thread worker_;
    std::deque<Command> command_queue_;
    std::map<std::uint64_t, CommandResult> results_;
    Snapshot snapshot_;
    Mode mode_ = Mode::Starting;
    Clock::time_point control_lease_deadline_{};
    std::atomic<bool> shutdown_requested_{false};
    std::atomic<bool> abort_requested_{false};
    bool start_attempted_ = false;
    bool initialization_done_ = false;
    bool initialization_success_ = false;
    std::string initialization_error_;
};

Controller::Controller(ControllerOptions options)
    : impl_(std::make_unique<Impl>(std::move(options)))
{
}

Controller::~Controller() = default;

bool Controller::start(std::string &error)
{
    return impl_->start(error);
}

void Controller::shutdown()
{
    impl_->shutdown();
}

SubmitResult Controller::submitControl(std::uint64_t sequence,
                                       std::chrono::milliseconds ttl)
{
    return impl_->submitControl(sequence, ttl);
}

SubmitResult Controller::submitTrajectory(
    std::uint64_t sequence,
    std::chrono::milliseconds ttl,
    std::chrono::milliseconds duration,
    const std::array<double, kJointCount> &absolute_degrees)
{
    return impl_->submitTrajectory(sequence, ttl, duration, absolute_degrees);
}

SubmitResult Controller::submitHold(std::uint64_t sequence,
                                    std::chrono::milliseconds ttl)
{
    return impl_->submitHold(sequence, ttl);
}

SubmitResult Controller::submitHeartbeat(std::uint64_t sequence,
                                         std::chrono::milliseconds ttl)
{
    return impl_->submitHeartbeat(sequence, ttl);
}

SubmitResult Controller::submitStop(std::uint64_t sequence)
{
    return impl_->submitStop(sequence);
}

SubmitResult Controller::submitFadeStop(std::uint64_t sequence,
                                        std::chrono::milliseconds fade)
{
    return impl_->submitFadeStop(sequence, fade);
}

SubmitResult Controller::submitReset(std::uint64_t sequence,
                                     std::chrono::milliseconds ttl)
{
    return impl_->submitReset(sequence, ttl);
}

Snapshot Controller::snapshot() const
{
    return impl_->snapshot();
}

bool Controller::waitForResult(std::uint64_t sequence,
                               std::chrono::milliseconds timeout,
                               CommandResult &result) const
{
    return impl_->waitForResult(sequence, timeout, result);
}
} // namespace noetix::left_arm_local
