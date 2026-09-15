#include "dual_arm_local/Controller.h"
#include "dual_arm_local/Profile.h"

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
#include <fstream>
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

namespace noetix::dual_arm_local
{
namespace
{
using Clock = std::chrono::steady_clock;

constexpr std::uint16_t kDeviceId = 0x0A;
constexpr std::uint16_t kMinimumAngleLimit = 0x0D;
constexpr std::uint16_t kMaximumAngleLimit = 0x0E;
constexpr std::uint16_t kOperatingMode = 0x10;
constexpr std::uint16_t kPositionP = 0x11;
constexpr std::uint16_t kPositionD = 0x12;
constexpr std::uint16_t kPositionI = 0x13;
constexpr std::uint16_t kTargetPosition = 0x80;
constexpr std::uint16_t kTorqueEnable = 0x81;
constexpr std::uint16_t kAcceleration = 0x82;
constexpr std::uint16_t kSpeed = 0x83;
constexpr std::uint16_t kTorqueLimit = 0x84;
constexpr std::uint16_t kPidLock = 0x85;
constexpr std::uint16_t kStatusStart = 0x100;
constexpr int kStatusRegisterCount = 8;
constexpr std::uint16_t kTorqueEnabledStatusBit = 0x10;

constexpr double kStepsPerRevolution = 4095.0;
constexpr double kSpeedRegisterStepsPerSecond = 50.0;
constexpr double kMaximumTemperatureC = 65.0;
constexpr double kMaximumCurrentA = 1.5;
constexpr double kMinimumVoltageV = 8.0;
constexpr double kMaximumVoltageV = 26.0;
constexpr int kPositionToleranceSteps = 3;
// Ignore sub-degree command noise while a streamed joint is stationary.  The
// source is encoder-derived, so one or two raw counts otherwise rewrite the
// target and wake HOLD -> MOVE on every few frames.  Three counts are about
// 0.26 degrees and still allow slow motion once the accumulated displacement
// crosses the deadband.
constexpr int kStreamTargetDeadbandSteps = 3;
// The loaded arm can settle a few encoder counts away from the commanded
// register while reporting zero speed. Ten counts are about 0.88 degrees and
// match the measured full-arm steady-state behavior. Actual position must
// still remain inside the calibrated soft limit.
constexpr int kTrajectoryArrivalToleranceSteps = 10;
constexpr int kMaximumEnableJumpSteps = 8;
constexpr std::size_t kMaximumStoredResults = 64;
constexpr auto kMaximumCommandTtl = std::chrono::seconds(120);
constexpr auto kMaximumTrajectoryDuration = std::chrono::seconds(60);
constexpr auto kTrajectoryArrivalAllowance = std::chrono::milliseconds(2500);

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

bool safeSystemdUnitName(const std::string &service)
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
    return true;
}

bool currentProcessBelongsToService(const std::string &service)
{
    if (!safeSystemdUnitName(service))
        return false;

    std::ifstream cgroup("/proc/self/cgroup");
    if (!cgroup)
        return false;

    const std::string suffix = "/" + service;
    std::string line;
    while (std::getline(cgroup, line))
    {
        const std::size_t first_colon = line.find(':');
        if (first_colon == std::string::npos)
            continue;
        const std::size_t second_colon = line.find(':', first_colon + 1);
        if (second_colon == std::string::npos)
            continue;

        const std::string path = line.substr(second_colon + 1);
        if (path.size() >= suffix.size() &&
            path.compare(path.size() - suffix.size(), suffix.size(), suffix) == 0)
            return true;
    }
    return false;
}

bool startupServiceDoesNotConflict(const std::string &service)
{
    if (!safeSystemdUnitName(service))
        return false;

    // A controller launched by this unit must not reject its own service as a
    // competing owner. Manual launches retain the original safety rule: the
    // configured service has to be inactive before serial devices are opened.
    if (currentProcessBelongsToService(service))
        return true;

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
    std::uint16_t position_p = 0;
    std::uint16_t position_d = 0;
    std::uint16_t position_i = 0;
    std::uint16_t pid_lock = 0;
};

struct Joint
{
    int number = 0;
    int slave_id = 0;
    int bus_index = 0;
    int direction = 1;
    int zero_raw = 2048;
    bool soft_limits_calibrated = false;
    int soft_minimum_raw = 0;
    int soft_maximum_raw = 4095;
    int configured_minimum = 0;
    int configured_maximum = 4095;
    int initial_position = 0;
    int target_position = 0;
    int workspace_minimum = 0;
    int workspace_maximum = 4095;
    bool enabled = false;
    bool prepared = false;
    bool original_pid_known = false;
    bool runtime_pid_active = false;
    std::uint16_t original_position_p = 0;
    std::uint16_t original_position_d = 0;
    std::uint16_t original_position_i = 0;
    std::uint16_t original_pid_lock = 1;
    std::uint16_t expected_position_p = 32;
    std::uint16_t expected_position_d = 32;
    std::uint16_t expected_position_i = 0;
};

double rawToJointDegrees(int raw, const Joint &joint) noexcept
{
    return static_cast<double>(raw - joint.zero_raw) * 360.0 /
           kStepsPerRevolution * static_cast<double>(joint.direction);
}

int jointDegreesToRaw(double degrees, const Joint &joint) noexcept
{
    return joint.zero_raw + static_cast<int>(std::lround(
                                degrees * kStepsPerRevolution / 360.0 *
                                static_cast<double>(joint.direction)));
}

std::string jointLabel(const Joint &joint)
{
    const char arm = joint.number <= static_cast<int>(kArmJointCount) ? 'L' : 'R';
    const int arm_joint = ((joint.number - 1) % static_cast<int>(kArmJointCount)) + 1;
    return std::string(1, arm) + 'J' + std::to_string(arm_joint);
}

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
        std::lock_guard<std::recursive_mutex> lock(io_mutex_);
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
        std::lock_guard<std::recursive_mutex> lock(io_mutex_);
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
        std::lock_guard<std::recursive_mutex> lock(io_mutex_);
        if (!ctx_ || !connected_)
            return false;
        if (modbus_set_slave(ctx_, slave_id) == -1)
            return false;
        const bool ok = modbus_write_register(ctx_, address, value) == 1;
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        return ok;
    }

    // Fire-and-forget write without readback.  Intended for the high-rate
    // stream path where freshness matters more than per-write verification:
    // the per-bus status reader (4 ms) and the control-loop validateAll
    // passes confirm writes within a few tens of milliseconds.  Failures
    // still latch the bus unhealthy and report an error immediately.
    bool writeOnly(int slave_id, std::uint16_t address, std::uint16_t value,
                   const std::string &description, std::string &error)
    {
        std::lock_guard<std::recursive_mutex> lock(io_mutex_);
        if (!selectSlave(slave_id, error))
            return false;
        const int rc = modbus_write_register(ctx_, address, value);
        transactionDelay();
        if (rc != 1)
        {
            error = errorText("write slave " + std::to_string(slave_id) + ' ' +
                              description);
            healthy_ = false;
            return false;
        }
        return true;
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
        /* Removed: fixed 2 ms pacing per Modbus transaction limited the
         * data bus to ~17 fps per arm.  libmodbus response timeouts
         * (250 ms) now guard against slave turnaround stalls; if the
         * fieldbus proves unreliable, restore a small non-zero delay. */
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
    std::recursive_mutex io_mutex_;
};

class Hardware
{
public:
    explicit Hardware(const ControllerOptions &options)
        : options_(options),
          buses_{{BusPort(options.devices[0]), BusPort(options.devices[1]),
                  BusPort(options.devices[2]), BusPort(options.devices[3])}}
    {
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            Joint &joint = joints_[index];
            joint.number = static_cast<int>(index + 1);
            const std::size_t arm_index = index % kArmJointCount;
            joint.slave_id = static_cast<int>(arm_index + 1);
            const int arm_bus_offset = index < kArmJointCount ? 0 : 2;
            joint.bus_index = arm_bus_offset + (arm_index < 3 ? 0 : 1);
            joint.direction = options.directions[index];
            joint.zero_raw = options.engineering_profiles[index].zero_raw;
            joint.soft_limits_calibrated =
                options.engineering_profiles[index].soft_limits_calibrated;
        }
    }

    ~Hardware()
    {
        const bool torque_may_be_enabled = anyEnabled();
        if (torque_may_be_enabled)
            emergencyDisableAll();
        // Never change PID after an unverified emergency torque-off attempt.
        // lock=1 guarantees that a power cycle restores the saved parameters.
        if (!torque_may_be_enabled)
            restoreOriginalPidsBestEffort();
    }

    bool connectAndPrepare(std::array<ServoState, kJointCount> &states,
                           std::string &error)
    {
        for (BusPort &bus : buses_)
        {
            if (!bus.connectExclusive(error))
            {
                emergencyDisableAll();
                restoreOriginalPidsBestEffort();
                return false;
            }
        }

        emergencyDisableAll();
        for (Joint &joint : joints_)
        {
            if (!prepareJoint(joint, error))
            {
                error = jointLabel(joint) + " preparation failed: " + error;
                emergencyDisableAll();
                restoreOriginalPidsBestEffort();
                return false;
            }
        }
        if (!readAll(states, error) ||
            !validateAll(states, 0, false, "startup", error))
        {
            emergencyDisableAll();
            restoreOriginalPidsBestEffort();
            return false;
        }
        return true;
    }

    bool readAll(std::array<ServoState, kJointCount> &states, std::string &error)
    {
        std::array<bool, 4> bus_ok{};
        std::array<std::string, 4> bus_errors{};
        std::atomic<bool> abort_flag{false};
        const unsigned int bus_count =
            static_cast<unsigned int>(buses_.size());
        std::vector<std::thread> workers;
        workers.reserve(bus_count);

        for (unsigned int b = 0; b < bus_count; ++b)
        {
            workers.emplace_back([&, b]
                                 {
                                     for (std::size_t index = 0;
                                          index < joints_.size() && !abort_flag.load(); ++index)
                                     {
                                         if (joints_[index].bus_index != static_cast<int>(b))
                                             continue;
                                         if (!readState(joints_[index], states[index],
                                                        bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                         joints_[index].enabled =
                                             states[index].torque_enabled;
                                     }
                                     bus_ok[b] = true;
                                 });
        }
        for (std::thread &worker : workers)
            worker.join();
        for (unsigned int b = 0; b < bus_count; ++b)
        {
            if (!bus_ok[b])
            {
                error = bus_errors[b];
                return false;
            }
        }
        return true;
    }

    bool validateAll(const std::array<ServoState, kJointCount> &states,
                     JointMask expected_torque_mask,
                     bool enforce_selected_workspace,
                     const std::string &phase,
                     std::string &error) const
    {
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            const bool selected = jointSelected(expected_torque_mask, index);
            if (!validateState(joints_[index], states[index],
                               enforce_selected_workspace && selected,
                               phase, error))
                return false;
            if (states[index].torque_enabled != selected)
            {
                error = phase + ' ' + jointLabel(joints_[index]) +
                        " unexpected torque state";
                return false;
            }
            if (selected &&
                states[index].target_position != joints_[index].target_position)
            {
                std::ostringstream message;
                message << phase << ' ' << jointLabel(joints_[index])
                        << " unexpected target register value: expected="
                        << joints_[index].target_position
                        << " read=" << states[index].target_position
                        << " position=" << states[index].position;
                error = message.str();
                return false;
            }
        }
        return true;
    }

    bool enterControl(JointMask selection,
                      std::array<ServoState, kJointCount> &states,
                      long &command_span_ms, std::string &error)
    {
        if (!readAll(states, error) ||
            !validateAll(states, 0, false, "control preflight", error))
            return false;

        std::array<int, kJointCount> positions{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            positions[index] = states[index].position;
        }

        std::array<double, kJointCount> current_degrees{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
            current_degrees[index] = rawToJointDegrees(states[index].position,
                                                       joints_[index]);
        if (!applyScheduledGains(selection, states, current_degrees, false, error))
        {
            emergencyDisableAll();
            return false;
        }

        const auto first_time = Clock::now();
        std::array<bool, 4> bus_ok{};
        std::array<std::string, 4> bus_errors{};
        std::atomic<bool> abort_flag{false};
        std::vector<std::thread> workers;
        workers.reserve(buses_.size());

        // The 8890 owner still executes exactly one command at a time.  Inside
        // CONTROL, each physical RTU bus gets one worker; joints on the same
        // bus remain strictly serialized while the four independent adapters
        // can prepare in parallel.  This preserves single-owner-per-port
        // semantics without making a dual-arm enable wait for 14 serial chains.
        for (unsigned int b = 0; b < buses_.size(); ++b)
        {
            workers.emplace_back([&, b]
                                 {
                                     for (std::size_t index = 0;
                                          index < joints_.size() &&
                                          !abort_flag.load(); ++index)
                                     {
                                         if (!jointSelected(selection, index))
                                             continue;
                                         Joint &joint = joints_[index];
                                         if (joint.bus_index != static_cast<int>(b))
                                             continue;

                                         // Re-read immediately before enabling:
                                         // a torque-off shoulder may keep drifting
                                         // while PID gains are being prepared.
                                         ServoState latest;
                                         if (!readState(joint, latest, bus_errors[b]) ||
                                             !validateState(
                                                 joint, latest, false,
                                                 "control per-joint pre-enable",
                                                 bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                         if (latest.torque_enabled)
                                         {
                                             bus_errors[b] =
                                                 "control per-joint pre-enable " +
                                                 jointLabel(joint) +
                                                 " unexpectedly has torque enabled";
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                         positions[index] = latest.position;
                                         if (!busFor(joint).writeVerified(
                                                 joint.slave_id, kTargetPosition,
                                                 static_cast<std::uint16_t>(
                                                     positions[index]),
                                                 "control target=current",
                                                 bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                         joint.target_position = positions[index];
                                         if (!busFor(joint).writeVerified(
                                                 joint.slave_id, kTorqueEnable, 1,
                                                 "control torque on", bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                         joint.enabled = true;
                                     }
                                     bus_ok[b] = true;
                                 });
        }
        for (std::thread &worker : workers)
            worker.join();
        for (unsigned int b = 0; b < buses_.size(); ++b)
        {
            if (!bus_ok[b])
            {
                error = bus_errors[b].empty()
                            ? "control enable aborted on another bus"
                            : bus_errors[b];
                emergencyDisableAll();
                return false;
            }
        }
        command_span_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              Clock::now() - first_time)
                              .count();

        std::this_thread::sleep_for(std::chrono::milliseconds(120));
        if (!readAll(states, error) ||
            !validateAll(states, selection, true, "after control enable", error))
        {
            emergencyDisableAll();
            return false;
        }
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (!jointSelected(selection, index))
                continue;
            if (std::abs(states[index].position - positions[index]) > kMaximumEnableJumpSteps)
            {
                error = jointLabel(joints_[index]) +
                        " moved too far while enabling";
                emergencyDisableAll();
                return false;
            }
        }
        return true;
    }

    bool applyScheduledGains(
        JointMask selection,
        std::array<ServoState, kJointCount> &states,
        const std::array<double, kJointCount> &target_degrees,
        bool moving, std::string &error)
    {
        std::array<bool, 4> bus_ok{};
        std::array<std::string, 4> bus_errors{};
        std::atomic<bool> abort_flag{false};
        std::vector<std::thread> workers;
        workers.reserve(buses_.size());

        // PID ramps can include multiple verified writes and configured
        // inter-step delays.  Run one ramp worker per independent adapter;
        // BusPort still serializes every operation on that adapter.
        for (unsigned int b = 0; b < buses_.size(); ++b)
        {
            workers.emplace_back([&, b]
                                 {
                                     for (std::size_t index = 0;
                                          index < joints_.size() &&
                                          !abort_flag.load(); ++index)
                                     {
                                         if (!jointSelected(selection, index))
                                             continue;
                                         Joint &joint = joints_[index];
                                         if (joint.bus_index != static_cast<int>(b))
                                             continue;
                                         const JointEngineeringProfile &profile =
                                             options_.engineering_profiles[index];
                                         if (!profile.pid_schedule.enabled)
                                             continue;

                                         const double current_degrees =
                                             rawToJointDegrees(
                                                 states[index].position, joint);
                                         const bool joint_is_moving =
                                             moving && pidScheduleRequiresMove(
                                                           current_degrees,
                                                           target_degrees[index]);
                                         PidGain desired;
                                         if (joint_is_moving)
                                             desired = scheduledPidGain(
                                                 profile, current_degrees, true);
                                         else if (moving)
                                             desired = scheduledSupportPidGain(
                                                 profile, current_degrees);
                                         else
                                             desired = scheduledPidGain(
                                                 profile, current_degrees, false);
                                         if (joint_is_moving)
                                         {
                                             const PidGain target_gain =
                                                 scheduledPidGain(
                                                     profile,
                                                     target_degrees[index], true);
                                             desired.p = std::max(
                                                 desired.p, target_gain.p);
                                             desired.d = std::max(
                                                 desired.d, target_gain.d);
                                             desired.i = std::max(
                                                 desired.i, target_gain.i);
                                         }
                                         if (!applyPidGainPair(
                                                 joint, states[index], desired,
                                                 profile, bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                     }
                                     bus_ok[b] = true;
                                 });
        }
        for (std::thread &worker : workers)
            worker.join();
        for (unsigned int b = 0; b < buses_.size(); ++b)
        {
            if (!bus_ok[b])
            {
                error = bus_errors[b].empty()
                            ? "PID schedule aborted on another bus"
                            : bus_errors[b];
                return false;
            }
        }
        return true;
    }

    // STREAM_TARGET has a per-joint motion state.  A joint whose command is
    // unchanged must keep its HOLD gain even while another selected joint is
    // moving.  This avoids turning encoder noise on an idle joint into a
    // high-stiffness SUPPORT loop.  Moving joints use the larger gain required
    // at the current/target endpoints, matching the absolute-trajectory path.
    bool applyStreamScheduledGains(
        JointMask selection, JointMask moving_selection,
        std::array<ServoState, kJointCount> &states,
        const std::array<double, kJointCount> &target_degrees,
        std::string &error)
    {
        // STREAM_TARGET is latency-sensitive.  The engineering trajectory
        // path deliberately ramps PID over many 25 ms steps, but doing that
        // in the stream owner thread blocks target writes for several seconds
        // when both arms change phase.  Apply the already range-checked
        // MOVE/HOLD pair once per phase transition, in parallel per physical
        // bus.  Unchanged gains generate no Modbus traffic.
        std::array<bool, 4> bus_ok{};
        std::array<std::string, 4> bus_errors{};
        std::atomic<bool> abort_flag{false};
        std::vector<std::thread> workers;
        workers.reserve(buses_.size());

        for (unsigned int b = 0; b < buses_.size(); ++b)
        {
            workers.emplace_back([&, b]
                                 {
                                     for (std::size_t index = 0;
                                          index < joints_.size() &&
                                          !abort_flag.load(); ++index)
                                     {
                                         Joint &joint = joints_[index];
                                         if (joint.bus_index != static_cast<int>(b) ||
                                             !jointSelected(selection, index))
                                             continue;
                                         const JointEngineeringProfile &profile =
                                             options_.engineering_profiles[index];
                                         if (!profile.pid_schedule.enabled)
                                             continue;

                                         const double current_degrees =
                                             rawToJointDegrees(states[index].position,
                                                               joint);
                                         const bool moving =
                                             jointSelected(moving_selection, index);
                                         PidGain desired = scheduledPidGain(
                                             profile, current_degrees, moving);
                                         if (moving)
                                         {
                                             const PidGain target_gain =
                                                 scheduledPidGain(
                                                     profile,
                                                     target_degrees[index], true);
                                             desired.p = std::max(desired.p,
                                                                  target_gain.p);
                                             desired.d = std::max(desired.d,
                                                                  target_gain.d);
                                             desired.i = std::max(desired.i,
                                                                  target_gain.i);
                                         }
                                         if (!applyPidGainPairImmediate(
                                                 joint, states[index], desired,
                                                 bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                     }
                                     bus_ok[b] = true;
                                 });
        }
        for (std::thread &worker : workers)
            worker.join();
        for (unsigned int b = 0; b < buses_.size(); ++b)
        {
            if (!bus_ok[b])
            {
                error = bus_errors[b];
                return false;
            }
        }
        return true;
    }

    bool writeTargets(const std::array<int, kJointCount> &targets,
                      JointMask selection,
                      long &command_span_ms, std::string &error)
    {
        const auto first_time = Clock::now();

        std::array<bool, 4> bus_ok{};
        std::array<std::string, 4> bus_errors{};
        std::atomic<bool> abort_flag{false};
        const unsigned int bus_count =
            static_cast<unsigned int>(buses_.size());
        std::vector<std::thread> workers;
        workers.reserve(bus_count);

        for (unsigned int b = 0; b < bus_count; ++b)
        {
            workers.emplace_back([&, b]
                                 {
                                     for (std::size_t index = 0;
                                          index < joints_.size() && !abort_flag.load(); ++index)
                                     {
                                         if (joints_[index].bus_index != static_cast<int>(b))
                                             continue;
                                         if (!jointSelected(selection, index))
                                             continue;
                                         Joint &joint = joints_[index];
                                         if (joint.target_position == targets[index])
                                             continue;
                                         if (!busFor(joint).writeVerified(
                                                 joint.slave_id, kTargetPosition,
                                                 static_cast<std::uint16_t>(targets[index]),
                                                 "absolute trajectory target", bus_errors[b]))
                                         {
                                             bus_ok[b] = false;
                                             abort_flag.store(true);
                                             return;
                                         }
                                         joint.target_position = targets[index];
                                     }
                                     bus_ok[b] = true;
                                 });
        }
        for (std::thread &worker : workers)
            worker.join();
        for (unsigned int b = 0; b < bus_count; ++b)
        {
            if (!bus_ok[b])
            {
                error = bus_errors[b];
                return false;
            }
        }

        command_span_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              Clock::now() - first_time)
                              .count();
        return true;
    }

    bool writeLegacyZeroTargets(long &command_span_ms, std::string &error)
    {
        const auto first_time = Clock::now();
        auto last_time = first_time;
        for (Joint &joint : joints_)
        {
            if (joint.zero_raw < joint.configured_minimum ||
                joint.zero_raw > joint.configured_maximum ||
                (joint.soft_limits_calibrated &&
                 (joint.zero_raw < joint.soft_minimum_raw ||
                  joint.zero_raw > joint.soft_maximum_raw)))
            {
                error = jointLabel(joint) +
                        " legacy zero is outside configured limits";
                return false;
            }
            if (!busFor(joint).writeVerified(
                    joint.slave_id, kTargetPosition,
                    static_cast<std::uint16_t>(joint.zero_raw),
                    "legacy zero target", error))
                return false;
            joint.target_position = joint.zero_raw;
            last_time = Clock::now();
        }
        command_span_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                              last_time - first_time)
                              .count();
        return true;
    }

    bool legacyZeroReached(const std::array<ServoState, kJointCount> &states,
                           int tolerance_steps) const
    {
        for (std::size_t index = 0; index < joints_.size(); ++index)
        {
            if (states[index].moving ||
                std::abs(states[index].position - joints_[index].zero_raw) >
                    tolerance_steps)
                return false;
        }
        return true;
    }

    bool hold(JointMask selection,
              std::array<ServoState, kJointCount> &states,
              long &command_span_ms, std::string &error)
    {
        if (!readAll(states, error) ||
            !validateAll(states, selection, true, "HOLD preflight", error))
            return false;
        std::array<int, kJointCount> positions{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
            positions[index] = states[index].position;
        if (!writeTargets(positions, selection, command_span_ms, error))
            return false;
        return waitStopped(selection, states, std::chrono::seconds(2), error);
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

    // Ramped torque-down variant of stopAll: hold the current pose, then lower
    // the torque-limit register (0x84) in kFadeSteps steps over `fade`, and
    // only then switch torque off.  Lets an operator take over the arm's
    // weight smoothly during a control-source switch (no gravity sag jump).
    bool fadeStopAll(std::chrono::milliseconds fade,
                     std::array<ServoState, kJointCount> *states,
                     std::string &error)
    {
        constexpr int kFadeSteps = 10;
        std::array<int, kJointCount> positions{};
        std::array<bool, kJointCount> captured{};
        std::array<ServoState, kJointCount> local_states{};

        // One worker owns each physical adapter for the duration of a phase.
        // The command remains serialized by the controller owner thread, and
        // joints sharing an adapter remain strictly ordered.  Returning false
        // from an operation records the failure but does not prevent the
        // remaining joints from reaching the final torque-off phase.
        const auto runPerBus = [&](auto &&operation)
        {
            std::array<bool, 4> bus_ok{{true, true, true, true}};
            std::array<std::string, 4> bus_errors{};
            std::vector<std::thread> workers;
            workers.reserve(buses_.size());
            for (unsigned int b = 0; b < buses_.size(); ++b)
            {
                workers.emplace_back([&, b]
                                     {
                                         for (std::size_t index = 0;
                                              index < joints_.size(); ++index)
                                         {
                                             if (joints_[index].bus_index !=
                                                 static_cast<int>(b))
                                                 continue;
                                             if (!operation(index, bus_errors[b]))
                                                 bus_ok[b] = false;
                                         }
                                     });
            }
            for (std::thread &worker : workers)
                worker.join();
            bool all_ok = true;
            for (unsigned int b = 0; b < buses_.size(); ++b)
            {
                if (bus_ok[b])
                    continue;
                all_ok = false;
                if (error.empty())
                    error = bus_errors[b].empty()
                                ? "fade stop bus operation failed"
                                : bus_errors[b];
            }
            return all_ok;
        };

        bool operation_ok = runPerBus(
            [&](std::size_t index, std::string &bus_error)
            {
                Joint &joint = joints_[index];
                if (!joint.enabled)
                    return true;
                ServoState state;
                if (!readState(joint, state, bus_error) ||
                    !positionIsSingleTurn(state.position))
                {
                    if (bus_error.empty())
                        bus_error = jointLabel(joint) +
                                    " invalid position during fade capture";
                    return false;
                }
                positions[index] = state.position;
                captured[index] = true;
                return true;
            });

        operation_ok = runPerBus(
                           [&](std::size_t index, std::string &bus_error)
                           {
                               if (!captured[index])
                                   return true;
                               Joint &joint = joints_[index];
                               if (!busFor(joint).writeVerified(
                                       joint.slave_id, kTargetPosition,
                                       static_cast<std::uint16_t>(positions[index]),
                                       "fade stop HOLD target", bus_error))
                                   return false;
                               joint.target_position = positions[index];
                               return true;
                           }) &&
                       operation_ok;

        std::array<std::uint16_t, kJointCount> start_limits{};
        for (std::size_t index = 0; index < joints_.size(); ++index)
            start_limits[index] = options_.safe_torque_limit[index];
        operation_ok = runPerBus(
                           [&](std::size_t index, std::string &bus_error)
                           {
                               Joint &joint = joints_[index];
                               if (!joint.enabled)
                                   return true;
                               ServoState state;
                               if (!readState(joint, state, bus_error))
                                   return false;
                               if (state.torque_limit > 0)
                                   start_limits[index] = state.torque_limit;
                               return true;
                           }) &&
                       operation_ok;

        const auto step_delay = fade / kFadeSteps;
        for (int step = 1; step <= kFadeSteps; ++step)
        {
            operation_ok = runPerBus(
                               [&](std::size_t index, std::string &bus_error)
                               {
                                   if (!captured[index])
                                       return true;
                                   Joint &joint = joints_[index];
                                   const std::uint32_t scaled =
                                       static_cast<std::uint32_t>(
                                           start_limits[index]) *
                                       static_cast<std::uint32_t>(
                                           kFadeSteps - step) /
                                       kFadeSteps;
                                   const std::uint16_t value =
                                       static_cast<std::uint16_t>(
                                           std::max<std::uint32_t>(
                                               scaled,
                                               options_.safe_torque_limit[index]));
                                   return busFor(joint).writeVerified(
                                       joint.slave_id, kTorqueLimit, value,
                                       "torque fade", bus_error);
                               }) &&
                           operation_ok;
            if (step < kFadeSteps)
                std::this_thread::sleep_for(step_delay);
        }

        // Re-capture settled positions so the final target registers match
        // where the arm actually came to rest — gravity creep during the ramp
        // would otherwise make the next CONTROL trip over stale targets.
        operation_ok = runPerBus(
                           [&](std::size_t index, std::string &bus_error)
                           {
                               if (!captured[index])
                                   return true;
                               Joint &joint = joints_[index];
                               ServoState state;
                               if (!readState(joint, state, bus_error) ||
                                   !positionIsSingleTurn(state.position))
                                   return false;
                               if (!busFor(joint).writeVerified(
                                       joint.slave_id, kTargetPosition,
                                       static_cast<std::uint16_t>(state.position),
                                       "fade stop final target", bus_error))
                                   return false;
                               joint.target_position = state.position;
                               return true;
                           }) &&
                       operation_ok;

        const bool torque_off_ok = runPerBus(
            [&](std::size_t index, std::string &bus_error)
            {
                Joint &joint = joints_[index];
                if (!busFor(joint).writeVerified(joint.slave_id, kTorqueEnable,
                                                 0, "torque off", bus_error))
                    return false;
                joint.enabled = false;
                return true;
            });
        if (states && busesHealthy())
        {
            std::string read_error;
            if (readAll(local_states, read_error))
                *states = local_states;
        }
        if (!torque_off_ok && error.empty())
            error = "could not confirm torque OFF for every joint";
        return operation_ok && torque_off_ok && error.empty();
    }

    bool restoreOriginalPids(std::array<ServoState, kJointCount> *states,
                             std::string &error)
    {
        if (anyEnabled())
        {
            error = "PID restore rejected because torque OFF is not confirmed";
            return false;
        }
        bool all_ok = true;
        for (Joint &joint : joints_)
        {
            if (!joint.original_pid_known || !joint.runtime_pid_active)
                continue;

            BusPort &bus = busFor(joint);
            std::string write_error;
            if (!bus.writeVerified(joint.slave_id, kPidLock, 1,
                                   "runtime-only PID lock before restore", write_error))
            {
                all_ok = false;
                if (error.empty())
                    error = jointLabel(joint) + " PID restore failed: " + write_error;
                continue;
            }

            bool joint_ok = true;
            const auto restore_value = [&](std::uint16_t address, std::uint16_t value,
                                           const std::string &description)
            {
                std::string value_error;
                if (!bus.writeVerified(joint.slave_id, address, value,
                                       description, value_error))
                {
                    joint_ok = false;
                    all_ok = false;
                    if (error.empty())
                        error = jointLabel(joint) + " PID restore failed: " + value_error;
                }
            };
            restore_value(kPositionD, joint.original_position_d,
                          "original position D");
            restore_value(kPositionI, joint.original_position_i,
                          "original position I");
            restore_value(kPositionP, joint.original_position_p,
                          "original position P");
            if (joint_ok && joint.original_pid_lock != 1)
                restore_value(kPidLock, joint.original_pid_lock,
                              "original PID lock");
            if (joint_ok)
                joint.runtime_pid_active = false;
        }

        if (states && busesHealthy())
        {
            std::array<ServoState, kJointCount> restored_states{};
            std::string read_error;
            if (readAll(restored_states, read_error))
                *states = restored_states;
            else
            {
                all_ok = false;
                if (error.empty())
                    error = "PID restore readback failed: " + read_error;
            }
        }
        return all_ok;
    }

    bool waitStopped(JointMask selection,
                     std::array<ServoState, kJointCount> &states,
                     std::chrono::milliseconds timeout, std::string &error)
    {
        const auto deadline = Clock::now() + timeout;
        while (Clock::now() < deadline)
        {
            if (!readAll(states, error) ||
                !validateAll(states, selection, true, "waiting for HOLD", error))
                return false;
            bool stopped = true;
            for (std::size_t index = 0; index < states.size(); ++index)
                stopped = stopped &&
                          (!jointSelected(selection, index) || !states[index].moving);
            if (stopped)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        error = "HOLD stop timeout";
        return false;
    }

    bool targetsReached(JointMask selection,
                        const std::array<ServoState, kJointCount> &states,
                        const std::array<int, kJointCount> &targets) const
    {
        for (std::size_t index = 0; index < states.size(); ++index)
        {
            if (!jointSelected(selection, index))
                continue;
            if (states[index].moving ||
                std::abs(states[index].position - targets[index]) >
                    kTrajectoryArrivalToleranceSteps)
                return false;
        }
        return true;
    }

    bool targetInsideLimits(const Joint &joint, int target, std::string &error) const
    {
        // When soft limits are not calibrated, skip the hardware limits check
        // so that the controller can still take control and move joints back
        // to a safe position.
        if (!joint.soft_limits_calibrated)
            return true;
        if (target < joint.configured_minimum || target > joint.configured_maximum ||
            target < joint.workspace_minimum || target > joint.workspace_maximum)
        {
            std::ostringstream message;
            message << jointLabel(joint) << " target " << target
                    << " outside hardware/"
                    << (options_.position_limit_policy ==
                                PositionLimitPolicy::CalibratedSoftOnly
                            ? "calibrated-soft"
                            : "session/calibrated-soft")
                    << " limits ["
                    << std::max(joint.configured_minimum, joint.workspace_minimum)
                    << ',' << std::min(joint.configured_maximum, joint.workspace_maximum)
                    << ']';
            error = message.str();
            return false;
        }
        return true;
    }

    std::chrono::milliseconds pidScheduleSettleDelay(JointMask selection) const
    {
        std::chrono::milliseconds delay{0};
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            if (jointSelected(selection, index) &&
                options_.engineering_profiles[index].pid_schedule.enabled)
                delay = std::max(
                    delay,
                    options_.engineering_profiles[index].pid_schedule.settle_delay);
        }
        return delay;
    }

    const std::array<Joint, kJointCount> &joints() const noexcept { return joints_; }
    BusPort &bus(std::size_t index) noexcept { return buses_[index]; }
    const std::array<BusPort, 4> &allBuses() const noexcept { return buses_; }
    bool busesHealthy() const noexcept
    {
        return std::all_of(buses_.begin(), buses_.end(),
                           [](const BusPort &bus) { return bus.healthy(); });
    }
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

    void restoreOriginalPidsBestEffort() noexcept
    {
        for (Joint &joint : joints_)
        {
            if (!joint.original_pid_known || !joint.runtime_pid_active)
                continue;
            BusPort &bus = busFor(joint);
            if (!bus.bestEffortWrite(joint.slave_id, kPidLock, 1))
                continue;
            const bool restored =
                bus.bestEffortWrite(joint.slave_id, kPositionD,
                                    joint.original_position_d) &&
                bus.bestEffortWrite(joint.slave_id, kPositionI,
                                    joint.original_position_i) &&
                bus.bestEffortWrite(joint.slave_id, kPositionP,
                                    joint.original_position_p);
            if (restored && joint.original_pid_lock != 1)
                (void)bus.bestEffortWrite(joint.slave_id, kPidLock,
                                          joint.original_pid_lock);
            if (restored)
                joint.runtime_pid_active = false;
        }
    }

private:
    BusPort &busFor(const Joint &joint)
    {
        return buses_[static_cast<std::size_t>(joint.bus_index)];
    }

    bool applyPidGainPair(Joint &joint, ServoState &state,
                          const PidGain &desired,
                          const JointEngineeringProfile &profile,
                          std::string &error)
    {
        if (joint.expected_position_p == desired.p &&
            joint.expected_position_d == desired.d &&
            joint.expected_position_i == desired.i)
            return true;

        const PidGain previous{joint.expected_position_p,
                               joint.expected_position_d,
                               joint.expected_position_i};

        BusPort &bus = busFor(joint);
        if (!bus.writeVerified(joint.slave_id, kPidLock, 1,
                               "scheduled runtime-only PID lock", error))
        {
            error = jointLabel(joint) + " PID schedule failed: " + error;
            return false;
        }

        auto approach = [](std::uint16_t current, std::uint16_t target,
                           std::uint16_t maximum_step)
        {
            if (current < target)
                return static_cast<std::uint16_t>(
                    std::min<unsigned int>(target, current + maximum_step));
            if (current > target)
                return static_cast<std::uint16_t>(
                    std::max<int>(target, static_cast<int>(current) - maximum_step));
            return current;
        };

        while (joint.expected_position_p != desired.p ||
               joint.expected_position_d != desired.d ||
               joint.expected_position_i != desired.i)
        {
            const std::uint16_t next_p = approach(
                joint.expected_position_p, desired.p,
                profile.pid_schedule.maximum_p_step);
            const std::uint16_t next_d = approach(
                joint.expected_position_d, desired.d,
                profile.pid_schedule.maximum_d_step);
            const std::uint16_t next_i = approach(
                joint.expected_position_i, desired.i,
                profile.pid_schedule.maximum_i_step);
            const bool increasing_stiffness =
                next_p > joint.expected_position_p ||
                next_i > joint.expected_position_i;

            const auto write_d = [&]()
            {
                if (next_d == joint.expected_position_d)
                    return true;
                return bus.writeVerified(joint.slave_id, kPositionD, next_d,
                                         "scheduled position D", error);
            };
            const auto write_p = [&]()
            {
                if (next_p == joint.expected_position_p)
                    return true;
                return bus.writeVerified(joint.slave_id, kPositionP, next_p,
                                         "scheduled position P", error);
            };
            const auto write_i = [&]()
            {
                if (next_i == joint.expected_position_i)
                    return true;
                return bus.writeVerified(joint.slave_id, kPositionI, next_i,
                                         "scheduled position I", error);
            };

            // Increasing stiffness installs damping first and integral action
            // last. Decreasing stiffness removes integral action first. No
            // target write can interleave on the controller command thread.
            const bool written = increasing_stiffness
                                     ? (write_d() && write_p() && write_i())
                                     : (write_i() && write_p() && write_d());
            if (!written)
            {
                error = jointLabel(joint) + " PID pair write failed: " + error;
                return false;
            }

            ServoState verified;
            if (!readState(joint, verified, error) ||
                verified.position_p != next_p || verified.position_d != next_d ||
                verified.position_i != next_i ||
                verified.pid_lock != 1)
            {
                if (error.empty())
                {
                    std::ostringstream message;
                    message << jointLabel(joint)
                            << " PID pair readback mismatch expected="
                            << next_p << '/' << next_d << '/' << next_i;
                    error = message.str();
                }
                return false;
            }
            joint.expected_position_p = next_p;
            joint.expected_position_d = next_d;
            joint.expected_position_i = next_i;
            state = verified;

            if (next_p != desired.p || next_d != desired.d || next_i != desired.i)
                std::this_thread::sleep_for(profile.pid_schedule.step_interval);
        }
        std::cout << "PID_SCHEDULE " << jointLabel(joint) << " P/D/I "
                  << previous.p << '/' << previous.d << '/' << previous.i << " -> "
                  << desired.p << '/' << desired.d << '/' << desired.i
                  << " at " << std::fixed << std::setprecision(3)
                  << rawToJointDegrees(state.position, joint) << " deg\n";
        return true;
    }

    bool applyPidGainPairImmediate(Joint &joint, ServoState &state,
                                   const PidGain &desired,
                                   std::string &error)
    {
        if (joint.expected_position_p == desired.p &&
            joint.expected_position_d == desired.d &&
            joint.expected_position_i == desired.i)
            return true;

        BusPort &bus = busFor(joint);
        if (state.pid_lock != 1 &&
            !bus.writeVerified(joint.slave_id, kPidLock, 1,
                               "stream runtime-only PID lock", error))
        {
            error = jointLabel(joint) + " stream PID lock failed: " + error;
            return false;
        }

        const bool increasing_stiffness =
            desired.p > joint.expected_position_p ||
            desired.i > joint.expected_position_i;
        const auto write_d = [&]()
        {
            return desired.d == joint.expected_position_d ||
                   bus.writeVerified(joint.slave_id, kPositionD, desired.d,
                                     "stream position D", error);
        };
        const auto write_p = [&]()
        {
            return desired.p == joint.expected_position_p ||
                   bus.writeVerified(joint.slave_id, kPositionP, desired.p,
                                     "stream position P", error);
        };
        const auto write_i = [&]()
        {
            return desired.i == joint.expected_position_i ||
                   bus.writeVerified(joint.slave_id, kPositionI, desired.i,
                                     "stream position I", error);
        };
        const bool written = increasing_stiffness
                                 ? (write_d() && write_p() && write_i())
                                 : (write_i() && write_p() && write_d());
        if (!written)
        {
            error = jointLabel(joint) + " stream PID write failed: " + error;
            return false;
        }

        ServoState verified;
        if (!readState(joint, verified, error) ||
            verified.position_p != desired.p ||
            verified.position_d != desired.d ||
            verified.position_i != desired.i || verified.pid_lock != 1)
        {
            if (error.empty())
            {
                std::ostringstream message;
                message << jointLabel(joint)
                        << " stream PID readback mismatch expected="
                        << desired.p << '/' << desired.d << '/' << desired.i;
                error = message.str();
            }
            return false;
        }
        joint.expected_position_p = desired.p;
        joint.expected_position_d = desired.d;
        joint.expected_position_i = desired.i;
        state = verified;
        return true;
    }

    bool prepareJoint(Joint &joint, std::string &error)
    {
        BusPort &bus = busFor(joint);
        std::uint16_t reported_id = 0;
        std::uint16_t configured_minimum = 0;
        std::uint16_t configured_maximum = 0;
        std::uint16_t configuration[4]{};
        std::uint16_t original_pid_lock = 0;

        if (!bus.readOne(joint.slave_id, kDeviceId, reported_id, error) ||
            !bus.readOne(joint.slave_id, kMinimumAngleLimit, configured_minimum, error) ||
            !bus.readOne(joint.slave_id, kMaximumAngleLimit, configured_maximum, error) ||
            !bus.readRegisters(joint.slave_id, kOperatingMode, 4, configuration, error) ||
            !bus.readOne(joint.slave_id, kPidLock, original_pid_lock, error))
            return false;

        if (reported_id != static_cast<std::uint16_t>(joint.slave_id) ||
            configured_minimum >= configured_maximum || configured_maximum > 4095 ||
            configuration[0] != 0 || configuration[1] > 254 ||
            configuration[2] > 254 || configuration[3] > 254 ||
            original_pid_lock > 1)
        {
            std::ostringstream message;
            message << "identity/config invalid: reported ID=" << reported_id
                    << " limits=[" << configured_minimum << ',' << configured_maximum
                    << "] mode=" << configuration[0]
                    << " PID=" << configuration[1] << '/' << configuration[2]
                    << '/' << configuration[3] << " lock=" << original_pid_lock;
            error = message.str();
            return false;
        }

        joint.configured_minimum = configured_minimum;
        joint.configured_maximum = configured_maximum;
        joint.soft_minimum_raw = configured_minimum;
        joint.soft_maximum_raw = configured_maximum;
        const std::size_t profile_index =
            static_cast<std::size_t>(joint.number - 1);
        const JointEngineeringProfile &profile =
            options_.engineering_profiles[profile_index];
        joint.original_position_p = configuration[1];
        joint.original_position_d = configuration[2];
        joint.original_position_i = configuration[3];
        joint.original_pid_lock = original_pid_lock;
        joint.original_pid_known = true;
        joint.expected_position_p = profile.position_p;
        joint.expected_position_d = profile.position_d;
        joint.expected_position_i = profile.position_i;
        if (joint.soft_limits_calibrated)
        {
            const int endpoint_a = jointDegreesToRaw(profile.soft_min_degrees, joint);
            const int endpoint_b = jointDegreesToRaw(profile.soft_max_degrees, joint);
            joint.soft_minimum_raw = std::max(
                static_cast<int>(configured_minimum), std::min(endpoint_a, endpoint_b));
            joint.soft_maximum_raw = std::min(
                static_cast<int>(configured_maximum), std::max(endpoint_a, endpoint_b));
            if (joint.soft_minimum_raw >= joint.soft_maximum_raw)
            {
                error = "calibrated soft limits do not overlap hardware limits";
                return false;
            }
        }

        ServoState state;
        if (!readState(joint, state, error) ||
            !validateState(joint, state, false, "preparation", error))
            return false;

        joint.enabled = state.torque_enabled;
        if (!bus.writeVerified(joint.slave_id, kTargetPosition,
                               static_cast<std::uint16_t>(state.position),
                               "target=current", error) ||
            !bus.writeVerified(joint.slave_id, kTorqueEnable, 0,
                               "torque off", error) ||
            !bus.writeVerified(joint.slave_id, kAcceleration,
                               options_.safe_acceleration[profile_index],
                               "safe acceleration", error) ||
            !bus.writeVerified(joint.slave_id, kSpeed,
                               options_.safe_speed[profile_index],
                               "safe speed", error) ||
            !bus.writeVerified(joint.slave_id, kTorqueLimit,
                               options_.safe_torque_limit[profile_index],
                               "safe torque limit", error))
            return false;
        joint.enabled = false;

        // PID registers live in the EPROM address range, so force lock=1
        // before every write. This makes the configured values runtime-only.
        // D and I are written before P to avoid briefly increasing proportional
        // action while stale damping is active.
        joint.runtime_pid_active = true;
        if (!bus.writeVerified(joint.slave_id, kPidLock, 1,
                               "runtime-only PID lock", error) ||
            !bus.writeVerified(joint.slave_id, kPositionD, profile.position_d,
                               "runtime position D", error) ||
            !bus.writeVerified(joint.slave_id, kPositionI, profile.position_i,
                               "runtime position I", error) ||
            !bus.writeVerified(joint.slave_id, kPositionP, profile.position_p,
                               "runtime position P", error))
            return false;

        if (!readState(joint, state, error) || state.torque_enabled ||
            std::abs(state.target_position - state.position) > kPositionToleranceSteps)
        {
            if (error.empty())
                error = "target/current or torque verification failed";
            return false;
        }

        joint.initial_position = state.position;
        joint.target_position = state.position;
        if (options_.position_limit_policy ==
            PositionLimitPolicy::CalibratedSoftOnly)
        {
            joint.workspace_minimum = joint.soft_minimum_raw;
            joint.workspace_maximum = joint.soft_maximum_raw;
        }
        else
        {
            const int workspace_steps = static_cast<int>(std::lround(
                profile.session_workspace_degrees * kStepsPerRevolution / 360.0));
            joint.workspace_minimum = std::max(joint.soft_minimum_raw,
                                               state.position - workspace_steps);
            joint.workspace_maximum = std::min(joint.soft_maximum_raw,
                                               state.position + workspace_steps);
        }
        if (joint.workspace_minimum > joint.workspace_maximum)
        {
            error = "current position is outside calibrated soft limits";
            return false;
        }
        joint.prepared = true;
        return true;
    }

    bool readState(Joint &joint, ServoState &state, std::string &error)
    {
        std::uint16_t feedback[kStatusRegisterCount]{};
        std::uint16_t runtime[6]{};
        std::uint16_t pid[3]{};
        BusPort &bus = busFor(joint);
        if (!bus.readRegisters(joint.slave_id, kStatusStart,
                               kStatusRegisterCount, feedback, error) ||
            !bus.readRegisters(joint.slave_id, kPositionP, 3, pid, error) ||
            !bus.readRegisters(joint.slave_id, kTargetPosition, 6, runtime, error))
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
        state.pid_lock = runtime[5];
        state.position_p = pid[0];
        state.position_d = pid[1];
        state.position_i = pid[2];
        return true;
    }

    bool validateState(const Joint &joint, const ServoState &state,
                       bool enforce_session_workspace,
                       const std::string &phase, std::string &error) const
    {
        const std::uint16_t fault_bits =
            static_cast<std::uint16_t>(state.status_word & ~kTorqueEnabledStatusBit);
        // When soft limits are not calibrated (calibrated=0), or when
        // enforce_session_workspace is false (e.g. during reset), skip the
        // position checks so that the controller can still start, reset, and
        // move joints back to a safe position.
        const bool skip_position_checks = !joint.soft_limits_calibrated ||
                                          !enforce_session_workspace;
        const bool outside_soft_limit = !skip_position_checks &&
                                        joint.soft_limits_calibrated &&
                                        (state.position < joint.soft_minimum_raw ||
                                         state.position > joint.soft_maximum_raw);
        const bool outside_session = !skip_position_checks &&
                                     enforce_session_workspace && joint.prepared &&
                                     (state.position < joint.workspace_minimum ||
                                      state.position > joint.workspace_maximum);
        if (fault_bits != 0 || outside_session ||
            outside_soft_limit ||
            (!skip_position_checks &&
             (!positionIsSingleTurn(state.position) ||
              state.position < joint.configured_minimum ||
              state.position > joint.configured_maximum)) ||
            state.temperature_c >= kMaximumTemperatureC ||
            state.voltage_v < kMinimumVoltageV || state.voltage_v > kMaximumVoltageV ||
            std::abs(state.current_a) >= kMaximumCurrentA)
        {
            std::ostringstream message;
            message << phase << ' ' << jointLabel(joint) << " unsafe: status=0x"
                    << std::hex << state.status_word << std::dec
                    << " pos=" << state.position << " temp=" << state.temperature_c
                    << "C voltage=" << state.voltage_v << "V current="
                    << state.current_a << "A reasons=";
            bool first_reason = true;
            const auto add_reason = [&](const char *reason)
            {
                if (!first_reason)
                    message << ',';
                message << reason;
                first_reason = false;
            };
            if (fault_bits != 0)
                add_reason("status_fault_bits");
            if (!positionIsSingleTurn(state.position))
                add_reason("outside_single_turn");
            if (outside_session)
                add_reason("outside_session_workspace");
            if (outside_soft_limit)
                add_reason("outside_calibrated_soft_limit");
            if (state.position < joint.configured_minimum ||
                state.position > joint.configured_maximum)
                add_reason("outside_device_limit");
            if (state.temperature_c >= kMaximumTemperatureC)
                add_reason("over_temperature");
            if (state.voltage_v < kMinimumVoltageV ||
                state.voltage_v > kMaximumVoltageV)
                add_reason("voltage_out_of_range");
            if (std::abs(state.current_a) >= kMaximumCurrentA)
                add_reason("over_current");
            error = message.str();
            return false;
        }
        if (joint.prepared)
        {
            const std::size_t index = static_cast<std::size_t>(joint.number - 1);
            if (state.acceleration != options_.safe_acceleration[index] ||
                state.speed_limit != options_.safe_speed[index] ||
                state.torque_limit != options_.safe_torque_limit[index] ||
                (joint.runtime_pid_active &&
                 (state.position_p != joint.expected_position_p ||
                  state.position_d != joint.expected_position_d ||
                  state.position_i != joint.expected_position_i ||
                  state.pid_lock != 1)))
            {
                std::ostringstream message;
                message << phase << ' ' << jointLabel(joint)
                        << " runtime parameter mismatch: acceleration="
                        << state.acceleration << '/' << options_.safe_acceleration[index]
                        << " speed=" << state.speed_limit << '/'
                        << options_.safe_speed[index] << " torque="
                        << state.torque_limit << '/'
                        << options_.safe_torque_limit[index]
                        << " PID=" << state.position_p << '/' << state.position_d
                        << '/' << state.position_i << " expected="
                        << joint.expected_position_p << '/'
                        << joint.expected_position_d << '/'
                        << joint.expected_position_i
                        << " lock=" << state.pid_lock << "/1";
                error = message.str();
                return false;
            }
        }
        return true;
    }

    ControllerOptions options_;
    std::array<BusPort, 4> buses_;
    std::array<Joint, kJointCount> joints_{};
};

enum class CommandKind
{
    Control,
    Trajectory,
    StreamTarget,
    LegacyZero,
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
    JointMask selection = 0;
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

enum class StreamPidPhase
{
    Hold,
    Move,
    Settling
};

struct StreamJointState
{
    bool target_valid = false;
    int target_raw = 0;
    StreamPidPhase phase = StreamPidPhase::Hold;
    Clock::time_point stable_since{};
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

const char *toString(PositionLimitPolicy policy) noexcept
{
    switch (policy)
    {
    case PositionLimitPolicy::SessionAndSoft: return "SESSION_AND_SOFT";
    case PositionLimitPolicy::CalibratedSoftOnly: return "CALIBRATED_SOFT_ONLY";
    }
    return "UNKNOWN";
}

const char *toString(TrajectoryExecutionPolicy policy) noexcept
{
    switch (policy)
    {
    case TrajectoryExecutionPolicy::HostInterpolated:
        return "HOST_INTERPOLATED";
    case TrajectoryExecutionPolicy::DeviceProfiled:
        return "DEVICE_PROFILED";
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
        for (std::size_t index = 0; index < options_.devices.size(); ++index)
        {
            if (options_.devices[index].empty())
            {
                error = "every Modbus device path must be non-empty";
                return false;
            }
            for (std::size_t other = index + 1; other < options_.devices.size(); ++other)
            {
                if (options_.devices[index] == options_.devices[other])
                {
                    error = "four Modbus device paths must be distinct";
                    return false;
                }
            }
        }
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            const JointEngineeringProfile &profile =
                options_.engineering_profiles[index];
            if (options_.safe_acceleration[index] > 254 ||
                options_.safe_speed[index] == 0 ||
                options_.safe_speed[index] > 254 ||
                options_.safe_torque_limit[index] == 0 ||
                options_.safe_torque_limit[index] > 1000 ||
                profile.position_p > 254 || profile.position_d > 254 ||
                profile.position_i > 254 ||
                profile.zero_raw < 0 || profile.zero_raw > 4095 ||
                !std::isfinite(profile.soft_min_degrees) ||
                !std::isfinite(profile.soft_max_degrees) ||
                profile.soft_min_degrees >= profile.soft_max_degrees ||
                !std::isfinite(profile.session_workspace_degrees) ||
                !std::isfinite(profile.maximum_command_delta_degrees) ||
                !std::isfinite(
                    profile.maximum_trajectory_speed_degrees_per_second) ||
                profile.maximum_trajectory_speed_degrees_per_second <= 0.0 ||
                profile.maximum_trajectory_speed_degrees_per_second > 450.0)
            {
                error = "invalid engineering profile for joint " +
                        std::to_string(index + 1);
                return false;
            }
            if (options_.position_limit_policy ==
                PositionLimitPolicy::CalibratedSoftOnly)
            {
                if (profile.session_workspace_degrees != 0.0 ||
                    profile.maximum_command_delta_degrees != 0.0)
                {
                    error = "CALIBRATED_SOFT_ONLY requires zero session/max-delta "
                            "fields for joint " +
                            std::to_string(index + 1);
                    return false;
                }
            }
            else if (profile.session_workspace_degrees <= 0.0 ||
                     profile.session_workspace_degrees > 30.0 ||
                     profile.maximum_command_delta_degrees <= 0.0 ||
                     profile.maximum_command_delta_degrees >
                         profile.session_workspace_degrees)
            {
                error = "invalid session workspace/max delta for joint " +
                        std::to_string(index + 1);
                return false;
            }
            if (options_.trajectory_execution_policy ==
                TrajectoryExecutionPolicy::DeviceProfiled)
            {
                const double configured_device_speed =
                    static_cast<double>(options_.safe_speed[index]) *
                    kSpeedRegisterStepsPerSecond * 360.0 / kStepsPerRevolution;
                if (configured_device_speed >
                    profile.maximum_trajectory_speed_degrees_per_second + 1e-9)
                {
                    std::ostringstream message;
                    message << "device speed register for joint " << index + 1
                            << " is approximately " << configured_device_speed
                            << " deg/s, above configured logical maximum "
                            << profile.maximum_trajectory_speed_degrees_per_second;
                    error = message.str();
                    return false;
                }
            }
            if (profile.soft_limits_calibrated)
            {
                Joint conversion_joint;
                conversion_joint.direction = options_.directions[index];
                conversion_joint.zero_raw = profile.zero_raw;
                const int endpoint_a =
                    jointDegreesToRaw(profile.soft_min_degrees, conversion_joint);
                const int endpoint_b =
                    jointDegreesToRaw(profile.soft_max_degrees, conversion_joint);
                if (!positionIsSingleTurn(endpoint_a) ||
                    !positionIsSingleTurn(endpoint_b))
                {
                    error = "calibrated soft limits convert outside raw range for joint " +
                            std::to_string(index + 1);
                    return false;
                }
            }
        }
        if (options_.trajectory_tick.count() <= 0 ||
            options_.observe_period.count() <= 0 ||
            options_.control_period.count() <= 0 ||
            options_.status_period.count() < 0 ||
            options_.maximum_queue_depth == 0)
        {
            error = "invalid controller options";
            return false;
        }

        worker_ = std::thread(&Impl::workerMain, this);
        startStatusThreads();
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
        status_cv_.notify_all();
        if (worker_.joinable() && worker_.get_id() != std::this_thread::get_id())
            worker_.join();
        stopStatusThreads();
    }

    SubmitResult submitControl(std::uint64_t sequence,
                               std::chrono::milliseconds ttl,
                               JointMask selection)
    {
        if (ttl < std::chrono::seconds(1) || ttl > kMaximumCommandTtl)
            return rejectedSubmission(sequence, "CONTROL TTL must be 1000..120000 ms");
        if (selection == 0 || (selection & ~kBothArmsMask) != 0)
            return rejectedSubmission(sequence, "CONTROL selection mask is invalid");
        Command command;
        command.kind = CommandKind::Control;
        command.sequence = sequence;
        command.selection = selection;
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
            duration > kMaximumTrajectoryDuration)
            return rejectedSubmission(sequence, "trajectory duration must be 200..60000 ms");
        if (ttl < duration + kTrajectoryArrivalAllowance ||
            ttl > kMaximumCommandTtl)
            return rejectedSubmission(sequence,
                                      "trajectory TTL must cover duration + 2500 ms and be <=120000 ms");
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

    SubmitResult submitStreamTarget(
        std::uint64_t sequence,
        std::chrono::milliseconds ttl,
        std::chrono::milliseconds duration,
        const std::array<double, kJointCount> &absolute_degrees)
    {
        if (duration < std::chrono::milliseconds(25) ||
            duration > std::chrono::milliseconds(1000))
            return rejectedSubmission(sequence,
                                      "stream target duration must be 25..1000 ms");
        if (ttl < std::chrono::seconds(1) || ttl > kMaximumCommandTtl)
            return rejectedSubmission(sequence,
                                      "stream target TTL must be 1000..120000 ms");
        if (!std::all_of(absolute_degrees.begin(), absolute_degrees.end(),
                         [](double value) { return std::isfinite(value); }))
            return rejectedSubmission(sequence, "all absolute targets must be finite");

        Command command;
        command.kind = CommandKind::StreamTarget;
        command.sequence = sequence;
        command.duration = duration;
        command.targets = absolute_degrees;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitHold(std::uint64_t sequence,
                            std::chrono::milliseconds ttl)
    {
        if (ttl < std::chrono::milliseconds(500) || ttl > kMaximumCommandTtl)
            return rejectedSubmission(sequence, "HOLD TTL must be 500..120000 ms");
        Command command;
        command.kind = CommandKind::Hold;
        command.sequence = sequence;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitHeartbeat(std::uint64_t sequence,
                                 std::chrono::milliseconds ttl)
    {
        if (ttl < std::chrono::seconds(1) || ttl > kMaximumCommandTtl)
            return rejectedSubmission(sequence, "heartbeat TTL must be 1000..120000 ms");
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
        if (ttl < std::chrono::milliseconds(500) || ttl > kMaximumCommandTtl)
            return rejectedSubmission(sequence, "RESET TTL must be 500..120000 ms");
        Command command;
        command.kind = CommandKind::Reset;
        command.sequence = sequence;
        return submit(std::move(command), ttl, false);
    }

    SubmitResult submitLegacyZero(std::uint64_t sequence,
                                  std::chrono::milliseconds ttl)
    {
        if (!options_.allow_legacy_zero_command)
            return rejectedSubmission(sequence, "legacy zero command is disabled");
        if (ttl < std::chrono::seconds(10) || ttl > std::chrono::seconds(120))
            return rejectedSubmission(sequence,
                                      "legacy zero TTL must be 10000..120000 ms");
        Command command;
        command.kind = CommandKind::LegacyZero;
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
            if (command.kind == CommandKind::StreamTarget)
            {
                for (auto it = command_queue_.begin(); it != command_queue_.end();)
                {
                    if (it->kind == CommandKind::StreamTarget)
                        it = command_queue_.erase(it);
                    else
                        ++it;
                }
            }
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
            !startupServiceDoesNotConflict(options_.startup_service))
        {
            finishInitialization(false,
                                 options_.startup_service +
                                     " is active and current process is not managed by it");
            return;
        }

        hardware_ = std::make_unique<Hardware>(options_);
        Hardware &hardware = *hardware_;
        std::array<ServoState, kJointCount> states{};
        if (!hardware.connectAndPrepare(states, error))
        {
            finishInitialization(false, error);
            return;
        }
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            hardware_ready_.store(true);
        }
        status_cv_.notify_all();

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
        const bool stopped = hardware.stopAll(&states, stop_error);
        std::string restore_error;
        const bool restored = stopped &&
                              hardware.restoreOriginalPids(&states, restore_error);
        if (!stopped)
            std::cerr << "Shutdown torque-off confirmation failed; runtime PID remains "
                         "lock=1: " << stop_error << '\n';
        else if (!restored)
            std::cerr << "Shutdown PID restoration failed; power-cycle before production "
                         "use: " << restore_error << '\n';
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
        case CommandKind::StreamTarget:
            return processStreamTarget(hardware, states, command);
        case CommandKind::LegacyZero:
            return processLegacyZero(hardware, states, command);
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

    void resetStreamPidState()
    {
        stream_pid_active_ = false;
        for (StreamJointState &state : stream_joint_states_)
            state = StreamJointState{};
    }

    bool updateStreamPidState(
        Hardware &hardware, std::array<ServoState, kJointCount> &states,
        const std::array<int, kJointCount> &commanded_raw,
        bool accept_new_targets, std::string &error)
    {
        const auto now = Clock::now();
        JointMask moving_selection = 0;
        std::array<double, kJointCount> target_degrees{};
        const auto &joints = hardware.joints();

        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            StreamJointState &stream = stream_joint_states_[index];
            if (!jointSelected(active_mask_, index))
            {
                stream = StreamJointState{};
                target_degrees[index] = rawToJointDegrees(
                    states[index].position, joints[index]);
                continue;
            }

            // CONTROL initializes the device target to the measured position.
            // Use that fixed register value as the first stream anchor instead
            // of recapturing the encoder position on every frame.
            if (!stream.target_valid)
            {
                stream.target_valid = true;
                stream.target_raw = states[index].target_position;
                stream.phase = StreamPidPhase::Hold;
            }

            const int next_target = accept_new_targets
                                        ? commanded_raw[index]
                                        : stream.target_raw;
            const bool target_changed = next_target != stream.target_raw;
            if (accept_new_targets)
                stream.target_raw = next_target;

            const bool tracking =
                std::abs(states[index].position - stream.target_raw) >
                    kPositionToleranceSteps ||
                states[index].moving;
            // Only a real command change may wake a HOLD joint into MOVE.
            // Encoder error/moving feedback is used to keep an already moving
            // joint in MOVE, but must never promote an unchanged joint.  That
            // would make idle joints chatter between MOVE and HOLD gains as
            // normal quantization or compliance crosses the tolerance.
            if (target_changed)
            {
                stream.phase = StreamPidPhase::Move;
                stream.stable_since = Clock::time_point{};
            }
            else if (stream.phase == StreamPidPhase::Move)
            {
                if (!tracking)
                {
                    stream.phase = StreamPidPhase::Settling;
                    stream.stable_since = now;
                }
            }
            else if (stream.phase == StreamPidPhase::Settling)
            {
                if (tracking)
                {
                    stream.phase = StreamPidPhase::Move;
                    stream.stable_since = Clock::time_point{};
                }
                else
                {
                    const auto settle_delay =
                        options_.engineering_profiles[index].pid_schedule.settle_delay;
                    if (now - stream.stable_since >= settle_delay)
                        stream.phase = StreamPidPhase::Hold;
                }
            }

            if (stream.phase != StreamPidPhase::Hold)
                moving_selection = static_cast<JointMask>(
                    moving_selection | static_cast<JointMask>(1U << index));
            target_degrees[index] = rawToJointDegrees(stream.target_raw,
                                                     joints[index]);
        }

        stream_pid_active_ = true;
        return hardware.applyStreamScheduledGains(
            active_mask_, moving_selection, states, target_degrees, error);
    }

    ProcessOutcome applySettledHoldSchedule(
        Hardware &hardware, std::array<ServoState, kJointCount> &states,
        const Command &command, std::string &error)
    {
        const std::chrono::milliseconds delay =
            hardware.pidScheduleSettleDelay(active_mask_);
        if (delay.count() != 0)
            waitUntilInterruptible(Clock::now() + delay);
        if (shutdown_requested_.load() || abort_requested_.load())
        {
            std::string stop_error;
            hardware.stopAll(&states, stop_error);
            mode_ = Mode::Observe;
            active_mask_ = 0;
            return {ResultCode::Cancelled,
                    "HOLD P/D scheduling interrupted by STOP", false};
        }
        if (Clock::now() > command.deadline)
            return {ResultCode::Expired,
                    "command expired before HOLD P/D scheduling", true};
        if (!hardware.readAll(states, error) ||
            !hardware.validateAll(states, active_mask_, true,
                                  "before HOLD P/D schedule", error))
            return {ResultCode::Faulted,
                    "HOLD P/D preflight failed: " + error, true};

        // Keep the commanded absolute target after arrival.  Replacing it
        // with the measured position would turn the arrival tolerance into a
        // permanent zero/endpoint offset.  The explicit HOLD command retains
        // its capture-current semantics; trajectory completion only changes
        // the runtime gains.
        std::array<double, kJointCount> held_degrees{};
        const auto &joints = hardware.joints();
        for (std::size_t index = 0; index < kJointCount; ++index)
            held_degrees[index] = rawToJointDegrees(states[index].position,
                                                    joints[index]);
        if (!hardware.applyScheduledGains(active_mask_, states, held_degrees,
                                          false, error) ||
            !hardware.readAll(states, error) ||
            !hardware.validateAll(states, active_mask_, true,
                                  "after HOLD P/D schedule", error))
            return {ResultCode::Faulted,
                    "HOLD P/D schedule failed: " + error, true};
        resetStreamPidState();
        publishSnapshot(hardware, states);
        return {ResultCode::Completed,
                "HOLD P/D schedule complete; absolute target retained", false};
    }

    ProcessOutcome processControl(Hardware &hardware,
                                  std::array<ServoState, kJointCount> &states,
                                  const Command &command)
    {
        if (mode_ != Mode::Observe)
            return {ResultCode::Rejected, "CONTROL requires OBSERVE mode", false};
        long span = 0;
        std::string error;
        if (!hardware.enterControl(command.selection, states, span, error))
            return {ResultCode::Faulted, "CONTROL failed: " + error, true};
        mode_ = Mode::Control;
        active_mask_ = command.selection;
        control_lease_deadline_ = command.deadline;
        resetStreamPidState();
        setLastCommandSpan(span);
        abort_requested_.store(false);
        return {ResultCode::Completed,
                "CONTROL enabled selected joints with targets=current", false};
    }

    ProcessOutcome processTrajectory(Hardware &hardware,
                                     std::array<ServoState, kJointCount> &states,
                                     const Command &command)
    {
        if (mode_ != Mode::Control)
            return {ResultCode::Rejected, "TARGET requires CONTROL mode", false};

        std::string error;
        if (!hardware.readAll(states, error) ||
            !hardware.validateAll(states, active_mask_, true,
                                  "trajectory preflight", error))
            return {ResultCode::Faulted, "trajectory preflight failed: " + error, true};

        std::array<int, kJointCount> start_raw{};
        std::array<int, kJointCount> final_raw{};
        const auto &joints = hardware.joints();
        const double duration_seconds =
            static_cast<double>(command.duration.count()) / 1000.0;
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            start_raw[index] = states[index].position;
            if (!jointSelected(active_mask_, index))
            {
                final_raw[index] = start_raw[index];
                continue;
            }
            final_raw[index] = jointDegreesToRaw(command.targets[index],
                                                 joints[index]);
            if (!hardware.targetInsideLimits(joints[index], final_raw[index], error))
                return {ResultCode::Rejected, error, false};

            const JointEngineeringProfile &profile =
                options_.engineering_profiles[index];
            const double current_degrees = rawToJointDegrees(
                start_raw[index], joints[index]);
            const double delta = std::abs(command.targets[index] - current_degrees);
            if (options_.position_limit_policy ==
                    PositionLimitPolicy::SessionAndSoft &&
                delta > profile.maximum_command_delta_degrees + 1e-9)
            {
                std::ostringstream message;
                message << jointLabel(joints[index]) << " absolute target delta " << delta
                        << " deg exceeds "
                        << profile.maximum_command_delta_degrees;
                return {ResultCode::Rejected, message.str(), false};
            }
            if (delta / duration_seconds >
                profile.maximum_trajectory_speed_degrees_per_second + 1e-9)
            {
                std::ostringstream message;
                message << jointLabel(joints[index]) << " trajectory speed "
                        << delta / duration_seconds << " deg/s exceeds "
                        << profile.maximum_trajectory_speed_degrees_per_second;
                return {ResultCode::Rejected, message.str(), false};
            }
        }

        control_lease_deadline_ = command.deadline;
        if (!hardware.applyScheduledGains(active_mask_, states, command.targets,
                                          true, error))
            return {ResultCode::Faulted,
                    "trajectory MOVE P/D schedule failed: " + error, true};
        const auto trajectory_start = Clock::now();

        if (options_.trajectory_execution_policy ==
            TrajectoryExecutionPolicy::DeviceProfiled)
        {
            if (shutdown_requested_.load() || abort_requested_.load())
            {
                std::string stop_error;
                hardware.stopAll(&states, stop_error);
                mode_ = Mode::Observe;
                active_mask_ = 0;
                return {ResultCode::Cancelled,
                        "device-profiled trajectory interrupted by STOP", false};
            }

            long span = 0;
            if (!hardware.writeTargets(final_raw, active_mask_, span, error))
                return {ResultCode::Faulted,
                        "device-profiled final target write failed: " + error, true};
            setLastCommandSpan(span);

            const auto arrival_deadline = std::min(
                command.deadline,
                trajectory_start + command.duration + kTrajectoryArrivalAllowance);
            while (Clock::now() < arrival_deadline)
            {
                if (shutdown_requested_.load() || abort_requested_.load())
                {
                    std::string stop_error;
                    hardware.stopAll(&states, stop_error);
                    mode_ = Mode::Observe;
                    active_mask_ = 0;
                    return {ResultCode::Cancelled,
                            "device-profiled trajectory interrupted by STOP", false};
                }
                if (Clock::now() > command.deadline ||
                    Clock::now() > control_lease_deadline_)
                    return {ResultCode::Expired,
                            "device-profiled trajectory or control lease expired", true};
                if (!hardware.readAll(states, error) ||
                    !hardware.validateAll(states, active_mask_, true,
                                          "device-profiled trajectory", error))
                    return {ResultCode::Faulted,
                            "device-profiled trajectory monitor failed: " + error,
                            true};
                publishSnapshot(hardware, states);
                if (hardware.targetsReached(active_mask_, states, final_raw))
                {
                    const ProcessOutcome hold_result = applySettledHoldSchedule(
                        hardware, states, command, error);
                    if (hold_result.code != ResultCode::Completed)
                        return hold_result;
                    return {ResultCode::Completed,
                            "device-profiled target reached; HOLD P/D scheduled",
                            false};
                }
                waitUntilInterruptible(Clock::now() +
                                       std::chrono::milliseconds(50));
            }
            return {ResultCode::Faulted,
                    "device-profiled trajectory arrival timeout", true};
        }

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
                active_mask_ = 0;
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
            if (!hardware.writeTargets(waypoint, active_mask_, span, error))
                return {ResultCode::Faulted, "trajectory write failed: " + error, true};
            setLastCommandSpan(span);
            if (!hardware.readAll(states, error) ||
                !hardware.validateAll(states, active_mask_, true,
                                      "during trajectory", error))
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
                active_mask_ = 0;
                return {ResultCode::Cancelled, "trajectory arrival interrupted by STOP", false};
            }
            if (!hardware.readAll(states, error) ||
                !hardware.validateAll(states, active_mask_, true,
                                      "trajectory arrival", error))
                return {ResultCode::Faulted, "arrival verification failed: " + error, true};
            publishSnapshot(hardware, states);
            if (hardware.targetsReached(active_mask_, states, final_raw))
            {
                const ProcessOutcome hold_result = applySettledHoldSchedule(
                    hardware, states, command, error);
                if (hold_result.code != ResultCode::Completed)
                    return hold_result;
                return {ResultCode::Completed,
                        "absolute target reached; HOLD P/D scheduled", false};
            }
            waitUntilInterruptible(Clock::now() + std::chrono::milliseconds(50));
        }
        return {ResultCode::Faulted, "absolute trajectory arrival timeout", true};
    }

    ProcessOutcome processStreamTarget(
        Hardware &hardware,
        std::array<ServoState, kJointCount> &states,
        const Command &command)
    {
        if (mode_ != Mode::Control)
            return {ResultCode::Rejected, "STREAM_TARGET requires CONTROL mode", false};

        std::string error;
        if (!hardware.readAll(states, error) ||
            !hardware.validateAll(states, active_mask_, true,
                                  "stream target preflight", error))
            return {ResultCode::Faulted, "stream target preflight failed: " + error, true};

        std::array<int, kJointCount> final_raw{};
        const auto &joints = hardware.joints();
        const double duration_seconds =
            static_cast<double>(command.duration.count()) / 1000.0;
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            final_raw[index] = states[index].position;
            if (!jointSelected(active_mask_, index))
                continue;
            final_raw[index] = jointDegreesToRaw(command.targets[index],
                                                 joints[index]);
            const StreamJointState &stream = stream_joint_states_[index];
            const int previous_target_raw = stream.target_valid
                                                ? stream.target_raw
                                                : states[index].target_position;
            if (stream.target_valid &&
                std::abs(final_raw[index] - previous_target_raw) <=
                    kStreamTargetDeadbandSteps)
            {
                // Keep the authoritative device target fixed.  This filtered
                // value is intentionally used by validation, PID scheduling,
                // and writeTargets below; filtering only the PID phase would
                // still make the motor chase encoder noise.
                final_raw[index] = previous_target_raw;
            }
            if (!hardware.targetInsideLimits(joints[index], final_raw[index], error))
                return {ResultCode::Rejected, error, false};

            const JointEngineeringProfile &profile =
                options_.engineering_profiles[index];
            // A stream rate limit is a command-to-command limit.  Basing it
            // on encoder feedback forces the sender to chase quantized,
            // delayed measurements and injects that noise back into the
            // target.  CONTROL already initializes target=current, so the
            // device target register is the authoritative first anchor.
            const double previous_target_degrees = rawToJointDegrees(
                previous_target_raw, joints[index]);
            const double effective_target_degrees = rawToJointDegrees(
                final_raw[index], joints[index]);
            const double delta = std::abs(
                effective_target_degrees - previous_target_degrees);
            if (options_.position_limit_policy ==
                    PositionLimitPolicy::SessionAndSoft &&
                delta > profile.maximum_command_delta_degrees + 1e-9)
            {
                std::ostringstream message;
                message << jointLabel(joints[index]) << " stream target delta " << delta
                        << " deg exceeds "
                        << profile.maximum_command_delta_degrees;
                return {ResultCode::Rejected, message.str(), false};
            }
            if (delta / duration_seconds >
                profile.maximum_trajectory_speed_degrees_per_second + 1e-9)
            {
                std::ostringstream message;
                message << jointLabel(joints[index]) << " stream target speed "
                        << delta / duration_seconds << " deg/s exceeds "
                        << profile.maximum_trajectory_speed_degrees_per_second;
                return {ResultCode::Rejected, message.str(), false};
            }
        }

        control_lease_deadline_ = command.deadline;
        long span = 0;
        if (!hardware.writeTargets(final_raw, active_mask_, span, error))
            return {ResultCode::Faulted, "stream target write failed: " + error, true};
        setLastCommandSpan(span);
        // The newest absolute command must reach the servo before any gain
        // transition work.  UDP ACK means queued/accepted, while this owner
        // thread provides the actual application ordering.
        if (!updateStreamPidState(hardware, states, final_raw, true, error))
            return {ResultCode::Faulted,
                    "stream MOVE/HOLD PID state update failed: " + error, true};
        return {ResultCode::Completed, "stream target applied", false};
    }

    ProcessOutcome processLegacyZero(
        Hardware &hardware,
        std::array<ServoState, kJointCount> &states,
        const Command &command)
    {
        if (!options_.allow_legacy_zero_command)
            return {ResultCode::Rejected, "legacy zero command is disabled", false};
        if (mode_ != Mode::Observe)
            return {ResultCode::Rejected, "legacy zero requires OBSERVE mode", false};

        long span = 0;
        std::string error;
        if (!hardware.enterControl(kBothArmsMask, states, span, error))
            return {ResultCode::Faulted,
                    "legacy zero enable failed: " + error, true};
        mode_ = Mode::Control;
        active_mask_ = kBothArmsMask;
        control_lease_deadline_ = command.deadline;
        abort_requested_.store(false);

        if (!hardware.writeLegacyZeroTargets(span, error))
            return {ResultCode::Faulted,
                    "legacy zero target write failed: " + error, true};
        setLastCommandSpan(span);

        while (Clock::now() < command.deadline)
        {
            if (shutdown_requested_.load() || abort_requested_.load())
            {
                std::string stop_error;
                hardware.stopAll(&states, stop_error);
                mode_ = Mode::Observe;
                active_mask_ = 0;
                return {ResultCode::Cancelled,
                        "legacy zero interrupted by STOP", false};
            }
            if (!hardware.readAll(states, error) ||
                !hardware.validateAll(states, active_mask_, false,
                                      "during legacy zero", error))
                return {ResultCode::Faulted,
                        "legacy zero monitor failed: " + error, true};
            publishSnapshot(hardware, states);
            if (hardware.legacyZeroReached(states, 8))
            {
                std::string stop_error;
                if (!hardware.stopAll(&states, stop_error))
                    return {ResultCode::Faulted,
                            "legacy zero reached but STOP ALL failed: " +
                                stop_error,
                            true};
                mode_ = Mode::Observe;
                active_mask_ = 0;
                return {ResultCode::Completed,
                        "all joints reached legacy raw=2048; torque OFF confirmed",
                        false};
            }
            waitUntilInterruptible(Clock::now() + std::chrono::milliseconds(100));
        }
        return {ResultCode::Faulted, "legacy zero timeout", true};
    }

    ProcessOutcome processHold(Hardware &hardware,
                               std::array<ServoState, kJointCount> &states,
                               const Command &command)
    {
        if (mode_ != Mode::Control)
            return {ResultCode::Rejected, "HOLD requires CONTROL mode", false};
        long span = 0;
        std::string error;
        if (!hardware.hold(active_mask_, states, span, error))
            return {ResultCode::Faulted, "HOLD failed: " + error, true};
        std::array<double, kJointCount> held_degrees{};
        const auto &joints = hardware.joints();
        for (std::size_t index = 0; index < kJointCount; ++index)
            held_degrees[index] = rawToJointDegrees(states[index].position,
                                                    joints[index]);
        if (!hardware.applyScheduledGains(active_mask_, states, held_degrees,
                                          false, error))
            return {ResultCode::Faulted,
                    "HOLD P/D schedule failed: " + error, true};
        resetStreamPidState();
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
            !hardware.validateAll(states, active_mask_, true,
                                  "heartbeat", error))
            return {ResultCode::Faulted, "heartbeat validation failed: " + error, true};
        control_lease_deadline_ = command.deadline;
        return {ResultCode::Completed, "control lease renewed", false};
    }

    ProcessOutcome processStop(Hardware &hardware,
                               std::array<ServoState, kJointCount> &states)
    {
        std::string error;
        const bool stopped = hardware.stopAll(&states, error);
        active_mask_ = 0;
        resetStreamPidState();
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
        active_mask_ = 0;
        resetStreamPidState();
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
            !hardware.validateAll(states, 0, false, "fault reset", error))
            return {ResultCode::Rejected, "RESET safety check failed: " + error, false};
        mode_ = Mode::Observe;
        active_mask_ = 0;
        resetStreamPidState();
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
        const JointMask expected_torque = mode_ == Mode::Control ? active_mask_ : 0;
        if (!hardware.validateAll(states, expected_torque,
                                  mode_ == Mode::Control,
                                  "periodic monitor", error))
        {
            latchFault(hardware, states, error, 0);
            return;
        }
        if (mode_ == Mode::Control && stream_pid_active_)
        {
            const std::array<int, kJointCount> unchanged_targets{};
            if (!updateStreamPidState(hardware, states, unchanged_targets,
                                      false, error))
            {
                latchFault(hardware, states,
                           "periodic STREAM PID update failed: " + error, 0);
                return;
            }
        }
    }

    void latchFault(Hardware &hardware,
                    std::array<ServoState, kJointCount> &states,
                    const std::string &reason, std::uint64_t source_sequence)
    {
        const bool first_fault = mode_ != Mode::Fault;
        std::string published_reason = reason +
                                       (source_sequence == 0
                                            ? std::string()
                                            : " [sequence " +
                                                  std::to_string(source_sequence) + ']');
        mode_ = Mode::Fault;
        abort_requested_.store(false);
        cancelQueuedCommands("cancelled by latched FAULT");
        std::string stop_error;
        const bool stopped = hardware.stopAll(&states, stop_error);
        active_mask_ = 0;
        resetStreamPidState();
        if (!stopped)
            published_reason += "; STOP ALL confirmation failed: " + stop_error;

        // Publish the terminal torque state before exposing fault_latched.
        // This prevents UDP observers from seeing a fault event paired with a
        // stale CONTROL/torque-on snapshot while STOP ALL is still executing.
        publishSnapshot(hardware, states);
        if (first_fault)
        {
            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_.fault_reason = published_reason;
            snapshot_.fault_latched = true;
        }
    }

    void publishSnapshot(const Hardware &hardware,
                         const std::array<ServoState, kJointCount> &states)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.mode = mode_;
        snapshot_.initialized = mode_ != Mode::Starting && mode_ != Mode::Stopped;
        snapshot_.active_mask = active_mask_;
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
        snapshot_.position_limit_policy = options_.position_limit_policy;
        snapshot_.trajectory_execution_policy =
            options_.trajectory_execution_policy;
        const auto &joints = hardware.joints();
        for (std::size_t index = 0; index < kJointCount; ++index)
        {
            JointSnapshot &destination = snapshot_.joints[index];
            const ServoState &source = states[index];
            destination.joint = joints[index].number;
            destination.arm = index < kArmJointCount ? 'L' : 'R';
            destination.arm_joint = static_cast<int>(index % kArmJointCount + 1);
            destination.bus_index = joints[index].bus_index;
            destination.slave_id = joints[index].slave_id;
            destination.direction = joints[index].direction;
            destination.raw_position = source.position;
            destination.raw_target = source.target_position;
            destination.tracking_error_steps =
                source.target_position - source.position;
            destination.zero_raw = joints[index].zero_raw;
            destination.absolute_degrees = rawToJointDegrees(
                source.position, joints[index]);
            destination.target_degrees = rawToJointDegrees(
                source.target_position, joints[index]);
            destination.tracking_error_degrees =
                destination.target_degrees - destination.absolute_degrees;
            destination.status_word = source.status_word;
            destination.speed_raw = source.speed_raw;
            destination.pwm_raw = source.pwm_raw;
            destination.acceleration_raw = source.acceleration;
            destination.speed_limit_raw = source.speed_limit;
            destination.torque_limit_raw = source.torque_limit;
            destination.position_p_raw = source.position_p;
            destination.position_d_raw = source.position_d;
            destination.position_i_raw = source.position_i;
            destination.pid_lock_raw = source.pid_lock;
            destination.voltage_v = source.voltage_v;
            destination.temperature_c = source.temperature_c;
            destination.current_a = source.current_a;
            const JointEngineeringProfile &profile =
                options_.engineering_profiles[index];
            destination.soft_limits_calibrated =
                profile.soft_limits_calibrated;
            destination.soft_min_degrees = profile.soft_min_degrees;
            destination.soft_max_degrees = profile.soft_max_degrees;
            destination.session_workspace_degrees =
                profile.session_workspace_degrees;
            destination.maximum_command_delta_degrees =
                profile.maximum_command_delta_degrees;
            destination.maximum_trajectory_speed_degrees_per_second =
                profile.maximum_trajectory_speed_degrees_per_second;
            destination.moving = source.moving;
            destination.torque_enabled = source.torque_enabled;
            destination.selected = jointSelected(active_mask_, index);
        }
    }

    void setLastCommandSpan(long span)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_.last_command_span_ms = span;
    }

    // Per-bus high-frequency status reader (yaocao-style): publishes the
    // freshest position/status for this bus's joints into the snapshot
    // while the control loop keeps command handling at its own rhythm.
    void statusLoop(int bus_index)
    {
        {
            std::unique_lock<std::mutex> lock(status_mutex_);
            status_cv_.wait(lock, [this]
                            { return hardware_ready_.load() || shutdown_requested_.load(); });
        }
        if (shutdown_requested_.load())
            return;
        const std::chrono::milliseconds period = options_.status_period;
        if (period.count() <= 0)
            return;
        BusPort &port = hardware_->bus(static_cast<std::size_t>(bus_index));
        auto next = Clock::now() + period;
        while (!shutdown_requested_.load())
        {
            // STREAM_TARGET already performs a complete readAll() before each
            // write and publishes the resulting snapshot.  Background scans
            // during CONTROL only contend for the same serial ports, causing
            // queued stream frames to be coalesced into visible position
            // steps.  Keep high-frequency readers for OBSERVE/telemetry only.
            bool control_mode = false;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                control_mode = snapshot_.mode == Mode::Control;
            }
            if (control_mode)
            {
                std::unique_lock<std::mutex> lock(status_mutex_);
                status_cv_.wait_for(lock, period, [this]
                                    { return shutdown_requested_.load(); });
                continue;
            }
            for (std::size_t index = 0; index < hardware_->joints().size(); ++index)
            {
                const Joint &joint = hardware_->joints()[index];
                if (joint.bus_index != bus_index)
                    continue;
                std::string ignored_error;
                std::uint16_t feedback[2]{};
                if (!port.readRegisters(joint.slave_id, kStatusStart, 2,
                                        feedback, ignored_error))
                    continue;
                const int position = static_cast<std::int16_t>(feedback[1]);
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    JointSnapshot &destination = snapshot_.joints[index];
                    destination.raw_position = position;
                    destination.status_word = feedback[0];
                    destination.absolute_degrees =
                        rawToJointDegrees(position, joint);
                    destination.tracking_error_steps =
                        destination.raw_target - position;
                    destination.tracking_error_degrees =
                        destination.target_degrees - destination.absolute_degrees;
                }
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                snapshot_.monotonic_timestamp_ms = monotonicMilliseconds();
            }
            // A complete bus scan takes longer than the nominal 4 ms on some
            // adapters.  Scheduling from the previous deadline would leave
            // `next` permanently in the past and turn this loop into a busy
            // poll that competes with target writes.  Always leave one full
            // period of bus-idle time after the completed scan.
            next = Clock::now() + period;
            const auto due = next;
            std::unique_lock<std::mutex> lock(status_mutex_);
            status_cv_.wait_until(lock, due, [this]
                                  { return shutdown_requested_.load(); });
        }
    }

    void startStatusThreads()
    {
        for (int bus = 0; bus < static_cast<int>(hardware_->allBuses().size()); ++bus)
            status_threads_[static_cast<std::size_t>(bus)] =
                std::thread(&Impl::statusLoop, this, bus);
        status_cv_.notify_all();
    }

    void stopStatusThreads()
    {
        status_cv_.notify_all();
        for (std::thread &reader : status_threads_)
        {
            if (reader.joinable())
                reader.join();
        }
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

    // Yaocao-style high-frequency state readers: one dedicated thread per
    // serial bus publishes the freshest joint position while the control
    // loop handles commands.  Period comes from options_.status_period.
    std::array<std::thread, 4> status_threads_;
    std::atomic<bool> hardware_ready_{false};
    std::mutex status_mutex_;
    std::condition_variable status_cv_;
    std::deque<Command> command_queue_;
    std::map<std::uint64_t, CommandResult> results_;
    std::unique_ptr<Hardware> hardware_;
    Snapshot snapshot_;
    Mode mode_ = Mode::Starting;
    JointMask active_mask_ = 0;
    // STREAM_TARGET PID state is tracked per joint.  Unchanged joints remain
    // in HOLD; moving joints transition through MOVE -> SETTLING -> HOLD.
    std::array<StreamJointState, kJointCount> stream_joint_states_{};
    bool stream_pid_active_ = false;
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
                                       std::chrono::milliseconds ttl,
                                       JointMask selection)
{
    return impl_->submitControl(sequence, ttl, selection);
}

SubmitResult Controller::submitTrajectory(
        std::uint64_t sequence,
        std::chrono::milliseconds ttl,
        std::chrono::milliseconds duration,
        const std::array<double, kJointCount> &absolute_degrees)
{
    return impl_->submitTrajectory(sequence, ttl, duration, absolute_degrees);
}

SubmitResult Controller::submitStreamTarget(
        std::uint64_t sequence,
        std::chrono::milliseconds ttl,
        std::chrono::milliseconds duration,
        const std::array<double, kJointCount> &absolute_degrees)
{
    return impl_->submitStreamTarget(sequence, ttl, duration, absolute_degrees);
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

SubmitResult Controller::submitLegacyZero(std::uint64_t sequence,
                                          std::chrono::milliseconds ttl)
{
    return impl_->submitLegacyZero(sequence, ttl);
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
} // namespace noetix::dual_arm_local
