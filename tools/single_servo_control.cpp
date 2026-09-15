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
constexpr std::uint16_t kPositionP = 0x11;
constexpr std::uint16_t kPositionD = 0x12;
constexpr std::uint16_t kPositionI = 0x13;

constexpr std::uint16_t kTargetPosition = 0x80;
constexpr std::uint16_t kTorqueEnable = 0x81;
constexpr std::uint16_t kAcceleration = 0x82;
constexpr std::uint16_t kSpeed = 0x83;
constexpr std::uint16_t kTorqueLimit = 0x84;
constexpr std::uint16_t kLockFlag = 0x85;

constexpr std::uint16_t kStatusStart = 0x100;
constexpr int kStatusRegisterCount = 8;
// Register 0x100 bit 4 reports normal torque-enable state; every other
// non-zero bit is treated conservatively as a fault/reserved condition.
constexpr std::uint16_t kTorqueEnabledStatusBit = 0x10;

constexpr std::uint16_t kSafeAcceleration = 2; // 200 steps/s^2
constexpr std::uint16_t kSafeSpeed = 2;        // 100 steps/s
constexpr std::uint16_t kSafeTorqueLimit = 100; // 10.0%

constexpr double kStepsPerRevolution = 4095.0;
constexpr double kMinimumMoveDegrees = 0.1;
constexpr double kMaxMoveDegrees = 2.0;
constexpr double kWorkspaceDegrees = 5.0;
constexpr int kCanonicalZeroRaw = 2048;
constexpr double kZeroWaypointDegrees = 1.0;
constexpr int kZeroStartConfirmationToleranceSteps = 3;
constexpr double kMaximumTemperatureC = 65.0;
constexpr double kMaximumCurrentA = 0.8;
constexpr double kMinimumVoltageV = 8.0;
constexpr double kMaximumVoltageV = 26.0;
constexpr int kPositionReadbackToleranceSteps = 3;
constexpr int kMaximumPidDeltaFromStartup = 8;
constexpr int kMaximumIntegralDeltaFromStartup = 2;
constexpr auto kPostArrivalObservation = std::chrono::milliseconds(500);

volatile std::sig_atomic_t g_stop_requested = 0;

void signalHandler(int)
{
    g_stop_requested = 1;
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

        bool found = false;
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
                found = true;
                break;
            }
        }
        closedir(fds);
        if (found)
            continue;
    }
    closedir(proc);
    return holders;
}

bool isAllowedDevice(const std::string &device)
{
    return device == "/dev/noetix_arm_l1" ||
           device == "/dev/noetix_arm_l2" ||
           device == "/dev/noetix_arm_r1" ||
           device == "/dev/noetix_arm_r2";
}

int degreesToSteps(double degrees)
{
    return static_cast<int>(std::lround(degrees * kStepsPerRevolution / 360.0));
}

double stepsToCenteredDegrees(int steps)
{
    return (static_cast<double>(steps) - 2048.0) * 360.0 / kStepsPerRevolution;
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

struct PidValues
{
    std::uint16_t p = 0;
    std::uint16_t d = 0;
    std::uint16_t i = 0;
};

bool operator==(const PidValues &left, const PidValues &right)
{
    return left.p == right.p && left.d == right.d && left.i == right.i;
}

bool operator!=(const PidValues &left, const PidValues &right)
{
    return !(left == right);
}

class SingleServoController
{
public:
    SingleServoController(std::string device, int slave_id)
        : device_(std::move(device)), slave_id_(slave_id)
    {
    }

    ~SingleServoController()
    {
        shutdownNoThrow();
        restorePidNoThrow();
        closePort();
    }

    bool connectExclusive()
    {
        const auto holders = findProcessesHoldingDevice(device_);
        if (!holders.empty())
        {
            std::cerr << "Refusing to open " << device_ << ": already held by PID";
            for (const int pid : holders)
                std::cerr << ' ' << pid;
            std::cerr << "\nStop startup.service before running this tool.\n";
            return false;
        }

        ctx_ = modbus_new_rtu(device_.c_str(), 115200, 'N', 8, 1);
        if (!ctx_)
        {
            std::cerr << "modbus_new_rtu failed\n";
            return false;
        }

        modbus_set_response_timeout(ctx_, 0, 250000);
        modbus_set_byte_timeout(ctx_, 0, 50000);

        if (modbus_set_slave(ctx_, slave_id_) == -1)
        {
            reportModbusError("modbus_set_slave");
            closePort();
            return false;
        }

        if (modbus_connect(ctx_) == -1)
        {
            reportModbusError("modbus_connect");
            closePort();
            return false;
        }
        connected_ = true;

        const int fd = modbus_get_socket(ctx_);
        if (fd < 0 || flock(fd, LOCK_EX | LOCK_NB) == -1)
        {
            std::cerr << "Could not acquire exclusive advisory lock on " << device_ << "\n";
            closePort();
            return false;
        }

        if (ioctl(fd, TIOCEXCL) == -1)
        {
            std::cerr << "Could not make serial device exclusive: " << std::strerror(errno) << "\n";
            closePort();
            return false;
        }

        // Close the small race between the first /proc check and TIOCEXCL.
        // Once TIOCEXCL succeeds, a non-root process cannot open a new handle.
        const auto holders_after_open = findProcessesHoldingDevice(device_);
        if (!holders_after_open.empty())
        {
            std::cerr << "Refusing control: another process opened " << device_
                      << " during startup.\n";
            closePort();
            return false;
        }

        healthy_ = true;
        return true;
    }

    bool prepare()
    {
        std::uint16_t reported_id = 0;
        std::uint16_t configured_minimum = 0;
        std::uint16_t configured_maximum = 0;
        if (!readOne(kDeviceId, reported_id) ||
            !readOne(kMinimumAngleLimit, configured_minimum) ||
            !readOne(kMaximumAngleLimit, configured_maximum))
            return false;

        if (reported_id != static_cast<std::uint16_t>(slave_id_))
        {
            std::cerr << "Refusing control: addressed slave " << slave_id_
                      << " reports device ID " << reported_id << ".\n";
            return false;
        }
        if (configured_minimum >= configured_maximum || configured_maximum > 4095)
        {
            std::cerr << "Refusing control: invalid device angle limits ["
                      << configured_minimum << ", " << configured_maximum << "].\n";
            return false;
        }

        std::uint16_t configuration[4]{};
        std::uint16_t lock_flag = 0;
        if (!readRegisters(kOperatingMode, 4, configuration) ||
            !readOne(kLockFlag, lock_flag))
            return false;

        std::cout << "Configuration: mode=" << configuration[0]
                  << " P=" << configuration[1]
                  << " D=" << configuration[2]
                  << " I=" << configuration[3]
                  << " lock=" << lock_flag
                  << (lock_flag == 1 ? " (EPROM writes are runtime-only)" :
                                       " (EPROM writes would be saved)")
                  << '\n';

        if (configuration[0] != 0)
        {
            std::cerr << "Refusing control: operating mode is not position-servo mode (0).\n";
            return false;
        }
        if (lock_flag > 1)
        {
            std::cerr << "Refusing control: invalid lock flag " << lock_flag << ".\n";
            return false;
        }

        original_pid_ = {configuration[1], configuration[2], configuration[3]};
        active_pid_ = original_pid_;
        original_lock_flag_ = lock_flag;
        original_pid_known_ = true;

        ServoState state;
        if (!readState(state))
            return false;
        printState(state);
        // Preserve the observed state conservatively so every later failure
        // path still attempts torque-off if the servo was already enabled.
        enabled_ = state.torque_enabled;

        if (!positionIsSingleTurn(state.position))
        {
            std::cerr << "Refusing control: present position is outside 0..4095.\n";
            return false;
        }
        if (state.position < configured_minimum || state.position > configured_maximum)
        {
            std::cerr << "Refusing control: present position is outside the device angle limits.\n";
            return false;
        }

        if (statusHasFault(state.status_word))
        {
            std::cerr << "Refusing control: servo status contains fault bits.\n";
            bestEffortTorqueOff();
            return false;
        }
        if (!validateSafeState(state, "preparation"))
        {
            bestEffortTorqueOff();
            return false;
        }

        // Align the target before disabling torque so an already-enabled servo
        // cannot continue chasing a stale target while the test takes ownership.
        if (!writeVerified(kTargetPosition, static_cast<std::uint16_t>(state.position), "target=current"))
            return false;
        if (!writeVerified(kTorqueEnable, 0, "torque off"))
            return false;
        enabled_ = false;

        if (!writeVerified(kAcceleration, kSafeAcceleration, "safe acceleration"))
            return false;
        if (!writeVerified(kSpeed, kSafeSpeed, "safe speed"))
            return false;
        if (!writeVerified(kTorqueLimit, kSafeTorqueLimit, "safe torque limit"))
            return false;

        if (!readState(state))
            return false;
        if (state.torque_enabled ||
            std::abs(state.target_position - state.position) > kPositionReadbackToleranceSteps)
        {
            std::cerr << "Preparation verification failed: torque or target state is unexpected.\n";
            bestEffortTorqueOff();
            return false;
        }

        initial_position_ = state.position;
        target_position_ = state.position;
        device_minimum_ = configured_minimum;
        device_maximum_ = configured_maximum;
        workspace_min_ = std::max(static_cast<int>(configured_minimum),
                                  initial_position_ - degreesToSteps(kWorkspaceDegrees));
        workspace_max_ = std::min(static_cast<int>(configured_maximum),
                                  initial_position_ + degreesToSteps(kWorkspaceDegrees));
        prepared_ = true;

        std::cout << "Prepared device ID " << reported_id
                  << " with torque OFF. Device limits=[" << configured_minimum
                  << ", " << configured_maximum << "], initial raw position=" << initial_position_
                  << ", allowed workspace=[" << workspace_min_ << ", " << workspace_max_ << "]\n";
        std::cout << "Type 'enable " << slave_id_ << "' to explicitly enable this servo.\n";
        return true;
    }

    bool printZeroPreview()
    {
        ServoState state;
        if (!readState(state) || !validateSafeState(state, "zero preview"))
            return false;

        const int delta_steps = kCanonicalZeroRaw - state.position;
        const double delta_degrees = static_cast<double>(delta_steps) *
                                     360.0 / kStepsPerRevolution;
        std::cout << std::fixed << std::setprecision(3)
                  << "ENCODER_ZERO_PREVIEW id=" << slave_id_
                  << " current_raw=" << state.position
                  << " zero_raw=" << kCanonicalZeroRaw
                  << " delta_steps=" << delta_steps
                  << " delta_deg=" << delta_degrees
                  << " raw_direction="
                  << (delta_steps > 0 ? "increase" :
                      delta_steps < 0 ? "decrease" : "none")
                  << " torque=" << (state.torque_enabled ? "ON" : "OFF")
                  << '\n';
        std::cout << "This is the legacy encoder zero, not a mechanical-limit search. "
                     "Keep the path clear and use Ctrl+C or physical power-off if needed.\n";
        return true;
    }

    bool zeroStep(int confirmed_id, int confirmed_start_raw)
    {
        return moveTowardCanonicalZero(confirmed_id, confirmed_start_raw, false);
    }

    bool zeroRun(int confirmed_id, int confirmed_start_raw)
    {
        return moveTowardCanonicalZero(confirmed_id, confirmed_start_raw, true);
    }

    bool printPid()
    {
        std::uint16_t values[3]{};
        std::uint16_t lock_flag = 0;
        if (!readRegisters(kPositionP, 3, values) || !readOne(kLockFlag, lock_flag))
            return false;

        active_pid_ = {values[0], values[1], values[2]};
        std::cout << "PID: P=" << active_pid_.p
                  << " D=" << active_pid_.d
                  << " I=" << active_pid_.i
                  << " lock=" << lock_flag
                  << (lock_flag == 1 ? " runtime-only" : " EPROM-save-enabled")
                  << " startup={P=" << original_pid_.p
                  << ",D=" << original_pid_.d
                  << ",I=" << original_pid_.i << "}\n";
        return true;
    }

    bool setTemporaryPid(int p, int d, int i)
    {
        if (!prepared_ || !original_pid_known_)
        {
            std::cerr << "PID change rejected: controller is not prepared.\n";
            return false;
        }
        if (enabled_)
        {
            std::cerr << "PID change rejected: first use 'stop' and confirm torque OFF.\n";
            return false;
        }
        if (p < 0 || p > 254 || d < 0 || d > 254 || i < 0 || i > 254)
        {
            std::cerr << "PID change rejected: official register range is 0..254.\n";
            return false;
        }

        const PidValues candidate{static_cast<std::uint16_t>(p),
                                  static_cast<std::uint16_t>(d),
                                  static_cast<std::uint16_t>(i)};
        const auto distance = [](std::uint16_t left, std::uint16_t right) {
            return std::abs(static_cast<int>(left) - static_cast<int>(right));
        };
        if (distance(candidate.p, original_pid_.p) > kMaximumPidDeltaFromStartup ||
            distance(candidate.d, original_pid_.d) > kMaximumPidDeltaFromStartup ||
            distance(candidate.i, original_pid_.i) > kMaximumIntegralDeltaFromStartup)
        {
            std::cerr << "PID change rejected: bench candidates must stay within startup "
                      << "+/-" << kMaximumPidDeltaFromStartup
                      << " for P/D and +/-" << kMaximumIntegralDeltaFromStartup
                      << " for I.\n";
            return false;
        }

        const int changed_fields = static_cast<int>(candidate.p != active_pid_.p) +
                                   static_cast<int>(candidate.d != active_pid_.d) +
                                   static_cast<int>(candidate.i != active_pid_.i);
        if (changed_fields > 1)
        {
            std::cerr << "PID change rejected: change only one coefficient per step.\n";
            return false;
        }

        ServoState state;
        if (!readState(state) || !validateSafeState(state, "before PID change"))
            return false;
        if (state.torque_enabled)
        {
            std::cerr << "PID change rejected: torque readback is ON.\n";
            return false;
        }

        const PidValues previous = active_pid_;
        if (!writeVerified(kLockFlag, 1, "runtime-only lock flag"))
            return false;
        if (!writePidVerified(candidate))
        {
            std::cerr << "Temporary PID write failed; attempting to restore the previous values.\n";
            (void)writePidVerified(previous);
            return false;
        }

        active_pid_ = candidate;
        pid_changed_ = active_pid_ != original_pid_;
        std::cout << "Temporary PID active: P=" << active_pid_.p
                  << " D=" << active_pid_.d
                  << " I=" << active_pid_.i
                  << ". It is NOT saved to EPROM and will be restored on exit.\n";
        return true;
    }

    bool restoreOriginalPid()
    {
        if (!pid_changed_)
            return true;
        if (enabled_)
        {
            std::cerr << "PID restore rejected: first stop the servo.\n";
            return false;
        }
        if (!writeVerified(kLockFlag, 1, "runtime-only lock flag") ||
            !writePidVerified(original_pid_))
            return false;
        if (original_lock_flag_ != 1 &&
            !writeVerified(kLockFlag, original_lock_flag_, "original lock flag"))
            return false;

        active_pid_ = original_pid_;
        pid_changed_ = false;
        std::cout << "Restored startup PID: P=" << active_pid_.p
                  << " D=" << active_pid_.d
                  << " I=" << active_pid_.i << ".\n";
        return true;
    }

    bool probe(double degrees)
    {
        if (!std::isfinite(degrees) || degrees < kMinimumMoveDegrees ||
            degrees > kMaxMoveDegrees)
        {
            std::cerr << "Probe rejected: DEG must be " << kMinimumMoveDegrees
                      << ".." << kMaxMoveDegrees << ".\n";
            return false;
        }
        std::cout << "PID probe: +" << degrees << " deg, then -" << degrees
                  << " deg. Keep hands clear.\n";
        if (!moveRelativeDegrees(degrees))
            return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        return moveRelativeDegrees(-degrees);
    }

    bool enableExplicitly(int confirmed_id)
    {
        if (!prepared_ || confirmed_id != slave_id_)
        {
            std::cerr << "Enable rejected: type the configured motor ID exactly.\n";
            return false;
        }

        ServoState before;
        if (!readState(before) || !validateSafeState(before, "before enable"))
            return false;
        if (!positionIsSingleTurn(before.position))
            return false;

        target_position_ = before.position;
        if (!writeVerified(kTargetPosition, static_cast<std::uint16_t>(target_position_), "target=current"))
            return false;
        // From the moment the write is attempted, the outcome can become
        // ambiguous if its readback is lost. Treat it as possibly enabled.
        enabled_ = true;
        if (!writeVerified(kTorqueEnable, 1, "torque on"))
        {
            bestEffortTorqueOff();
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        ServoState after;
        if (!readState(after))
            return false;

        if (!after.torque_enabled || std::abs(after.position - before.position) > 8 ||
            !validateSafeState(after, "after enable"))
        {
            std::cerr << "Enable verification failed; disabling torque.\n";
            disableImmediate();
            return false;
        }

        std::cout << "Servo " << slave_id_ << " enabled without a position jump.\n";
        return true;
    }

    bool moveRelativeDegrees(double degrees)
    {
        if (!enabled_)
        {
            std::cerr << "Move rejected: servo is not enabled.\n";
            return false;
        }
        if (!std::isfinite(degrees) || std::abs(degrees) < kMinimumMoveDegrees ||
            std::abs(degrees) > kMaxMoveDegrees)
        {
            std::cerr << "Move rejected: delta must be between " << kMinimumMoveDegrees
                      << " and " << kMaxMoveDegrees << " degrees in magnitude.\n";
            return false;
        }

        ServoState state;
        if (!readState(state) || !validateSafeState(state, "before move"))
            return false;

        const int requested = state.position + degreesToSteps(degrees);
        if (requested < workspace_min_ || requested > workspace_max_)
        {
            std::cerr << "Move rejected: target " << requested
                      << " is outside startup workspace [" << workspace_min_
                      << ", " << workspace_max_ << "].\n";
            return false;
        }

        target_position_ = requested;
        const auto movement_started = std::chrono::steady_clock::now();
        if (!writeVerified(kTargetPosition, static_cast<std::uint16_t>(requested), "small position target"))
            return false;

        std::cout << "Moving by " << degrees << " deg to raw target " << requested << "...\n";
        return waitForTarget(std::chrono::seconds(5), state.position, state.temperature_c,
                             movement_started);
    }

    bool hold()
    {
        ServoState state;
        if (!readState(state))
            return false;
        if (!positionIsSingleTurn(state.position))
            return false;

        target_position_ = state.position;
        if (!writeVerified(kTargetPosition, static_cast<std::uint16_t>(state.position), "HOLD target"))
            return false;

        std::cout << "HOLD at raw position " << state.position
                  << (enabled_ ? " with torque enabled.\n" : " with torque disabled.\n");
        return waitUntilStopped(std::chrono::seconds(2));
    }

    bool controlledStop()
    {
        bool hold_ok = true;
        if (connected_ && healthy_)
            hold_ok = hold();
        const bool disable_ok = disableImmediate();
        if (disable_ok)
            std::cout << "Controlled stop complete; torque OFF was confirmed by readback.\n";
        else
            std::cerr << "Controlled stop attempted, but torque OFF could not be confirmed. "
                         "Use the physical stop/power control.\n";
        return hold_ok && disable_ok;
    }

    bool disableImmediate()
    {
        if (!connected_)
            return false;

        const bool ok = writeVerified(kTorqueEnable, 0, "torque off");
        if (ok)
            enabled_ = false;
        return ok;
    }

    bool monitor()
    {
        ServoState state;
        if (!readState(state))
        {
            std::cerr << "COMMUNICATION LOST. No automatic reconnect or command replay will occur.\n"
                      << "Use the physical power/stop mechanism before reconnecting the adapter.\n";
            bestEffortTorqueOff();
            return false;
        }

        if (!validateSafeState(state, "periodic monitor"))
        {
            std::cerr << "Safety limit or servo fault detected; disabling torque.\n";
            disableImmediate();
            return false;
        }
        enabled_ = state.torque_enabled;
        return true;
    }

    bool printCurrentState()
    {
        ServoState state;
        if (!readState(state))
            return false;
        printState(state);
        return true;
    }

    bool isEnabled() const { return enabled_; }
    bool isHealthy() const { return healthy_; }
    int slaveId() const { return slave_id_; }

private:
    bool moveTowardCanonicalZero(int confirmed_id,
                                 int confirmed_start_raw,
                                 bool complete_path)
    {
        if (!prepared_ || confirmed_id != slave_id_)
        {
            std::cerr << "Encoder-zero command rejected: confirm the configured motor ID.\n";
            return false;
        }
        if (enabled_)
        {
            std::cerr << "Encoder-zero command rejected: first use 'stop' to confirm torque OFF.\n";
            return false;
        }
        if (kCanonicalZeroRaw < device_minimum_ || kCanonicalZeroRaw > device_maximum_)
        {
            std::cerr << "Encoder-zero command rejected: raw 2048 is outside the servo's "
                      << "configured limits [" << device_minimum_ << ", "
                      << device_maximum_ << "].\n";
            return false;
        }

        ServoState state;
        if (!readState(state) || !validateSafeState(state, "before encoder-zero move"))
            return false;
        if (state.torque_enabled)
        {
            std::cerr << "Encoder-zero command rejected: torque readback is ON.\n";
            enabled_ = true;
            return false;
        }
        if (std::abs(state.position - confirmed_start_raw) >
            kZeroStartConfirmationToleranceSteps)
        {
            std::cerr << "Encoder-zero command rejected: confirmed start raw="
                      << confirmed_start_raw << " but current raw=" << state.position
                      << ". Run 'zero-preview' again and type the new raw value.\n";
            return false;
        }

        const int initial_distance = std::abs(kCanonicalZeroRaw - state.position);
        if (initial_distance <= kPositionReadbackToleranceSteps)
        {
            std::cout << "Already at legacy encoder zero within "
                      << kPositionReadbackToleranceSteps << " steps; torque remains OFF.\n";
            return writeVerified(kTargetPosition,
                                 static_cast<std::uint16_t>(state.position),
                                 "target=current at encoder zero");
        }

        const int waypoint_steps = std::max(1, degreesToSteps(kZeroWaypointDegrees));
        const double initial_distance_degrees =
            static_cast<double>(initial_distance) * 360.0 / kStepsPerRevolution;
        std::cout << std::fixed << std::setprecision(3)
                  << (complete_path ? "ZERO_RUN" : "ZERO_STEP")
                  << " id=" << slave_id_
                  << " start_raw=" << state.position
                  << " destination_raw=" << kCanonicalZeroRaw
                  << " distance_deg=" << initial_distance_degrees
                  << " waypoint_deg<=" << kZeroWaypointDegrees << '\n';

        if (!enableExplicitly(confirmed_id))
            return false;

        bool success = true;
        int previous_distance = initial_distance;
        const int maximum_waypoints = initial_distance / waypoint_steps + 4;
        int completed_waypoints = 0;

        while (!g_stop_requested && completed_waypoints < maximum_waypoints)
        {
            if (!readState(state) || !validateSafeState(state, "during encoder-zero move"))
            {
                success = false;
                break;
            }
            if (!state.torque_enabled)
            {
                std::cerr << "Encoder-zero move stopped: torque became disabled.\n";
                enabled_ = false;
                success = false;
                break;
            }

            const int distance = std::abs(kCanonicalZeroRaw - state.position);
            if (distance <= kPositionReadbackToleranceSteps)
                break;
            if (completed_waypoints > 0 && distance >= previous_distance)
            {
                std::cerr << "Encoder-zero move stopped: no progress toward raw 2048 "
                          << "(previous distance=" << previous_distance
                          << ", current distance=" << distance << ").\n";
                success = false;
                break;
            }

            const int direction = kCanonicalZeroRaw > state.position ? 1 : -1;
            const int requested = state.position +
                                  direction * std::min(distance, waypoint_steps);
            if (requested < device_minimum_ || requested > device_maximum_)
            {
                std::cerr << "Encoder-zero waypoint rejected outside configured limits.\n";
                success = false;
                break;
            }

            previous_distance = distance;
            target_position_ = requested;
            const auto movement_started = std::chrono::steady_clock::now();
            if (!writeVerified(kTargetPosition,
                               static_cast<std::uint16_t>(requested),
                               "encoder-zero waypoint") ||
                !waitForTarget(std::chrono::seconds(4), state.position,
                               state.temperature_c, movement_started))
            {
                success = false;
                break;
            }
            ++completed_waypoints;
            if (!complete_path)
                break;
        }

        if (g_stop_requested)
        {
            std::cerr << "Encoder-zero move interrupted by signal.\n";
            success = false;
        }
        if (completed_waypoints >= maximum_waypoints)
        {
            std::cerr << "Encoder-zero move stopped: waypoint guard exceeded.\n";
            success = false;
        }

        ServoState final_state;
        const bool final_read_ok = readState(final_state);
        if (complete_path && success &&
            (!final_read_ok ||
             std::abs(final_state.position - kCanonicalZeroRaw) >
                 kPositionReadbackToleranceSteps))
        {
            std::cerr << "Encoder-zero move did not finish within position tolerance.\n";
            success = false;
        }

        const bool stop_ok = controlledStop();
        if (final_read_ok)
        {
            std::cout << "ENCODER_ZERO_RESULT id=" << slave_id_
                      << " final_raw=" << final_state.position
                      << " zero_error_steps="
                      << std::abs(final_state.position - kCanonicalZeroRaw)
                      << " scope=" << (complete_path ? "full" : "one-waypoint")
                      << " torque_off=" << (stop_ok ? "confirmed" : "unconfirmed")
                      << " result=" << (success && stop_ok ? "PASS" : "STOPPED")
                      << '\n';
        }
        return success && stop_ok;
    }

    bool readRegisters(std::uint16_t address, int count, std::uint16_t *destination)
    {
        if (!connected_ || !ctx_)
            return false;

        const int rc = modbus_read_registers(ctx_, address, count, destination);
        interTransactionDelay();
        if (rc != count)
        {
            reportModbusError("read 0x" + hexAddress(address));
            healthy_ = false;
            return false;
        }
        return true;
    }

    bool readOne(std::uint16_t address, std::uint16_t &value)
    {
        return readRegisters(address, 1, &value);
    }

    bool writeVerified(std::uint16_t address, std::uint16_t value, const char *description)
    {
        if (!connected_ || !ctx_)
            return false;

        const int rc = modbus_write_register(ctx_, address, value);
        interTransactionDelay();
        if (rc != 1)
        {
            reportModbusError(std::string("write ") + description);
            healthy_ = false;
            return false;
        }

        std::uint16_t readback = 0;
        if (!readOne(address, readback))
            return false;
        if (readback != value)
        {
            std::cerr << "Readback mismatch for " << description << " at 0x"
                      << hexAddress(address) << ": wrote " << value
                      << ", read " << readback << '\n';
            healthy_ = false;
            return false;
        }

        std::cout << "Verified " << description << ": register 0x"
                  << hexAddress(address) << " = " << value << '\n';
        return true;
    }

    bool writePidVerified(const PidValues &values)
    {
        // Torque is required to be OFF by the public caller. Write the
        // damping and integral terms before P so a partially written candidate
        // cannot briefly increase proportional action with stale damping.
        if (!writeVerified(kPositionD, values.d, "temporary position D") ||
            !writeVerified(kPositionI, values.i, "temporary position I") ||
            !writeVerified(kPositionP, values.p, "temporary position P"))
            return false;

        std::uint16_t readback[3]{};
        if (!readRegisters(kPositionP, 3, readback))
            return false;
        if (readback[0] != values.p || readback[1] != values.d ||
            readback[2] != values.i)
        {
            std::cerr << "PID block readback mismatch: expected P=" << values.p
                      << " D=" << values.d << " I=" << values.i
                      << ", got P=" << readback[0] << " D=" << readback[1]
                      << " I=" << readback[2] << ".\n";
            healthy_ = false;
            return false;
        }
        return true;
    }

    bool readState(ServoState &state)
    {
        std::uint16_t feedback[kStatusRegisterCount]{};
        std::uint16_t runtime[5]{};
        if (!readRegisters(kStatusStart, kStatusRegisterCount, feedback))
            return false;
        if (!readRegisters(kTargetPosition, 5, runtime))
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

    bool validateSafeState(const ServoState &state, const char *phase) const
    {
        const std::uint16_t fault_bits =
            static_cast<std::uint16_t>(state.status_word & ~kTorqueEnabledStatusBit);
        if (fault_bits != 0)
        {
            std::cerr << phase << ": fault bits are 0x" << std::hex
                      << fault_bits << " (full status 0x" << state.status_word
                      << ')' << std::dec << '\n';
            return false;
        }
        if (!positionIsSingleTurn(state.position))
        {
            std::cerr << phase << ": position is outside 0..4095\n";
            return false;
        }
        if (state.temperature_c >= kMaximumTemperatureC)
        {
            std::cerr << phase << ": temperature limit exceeded: "
                      << state.temperature_c << " C\n";
            return false;
        }
        if (state.voltage_v < kMinimumVoltageV || state.voltage_v > kMaximumVoltageV)
        {
            std::cerr << phase << ": voltage outside allowed range: "
                      << state.voltage_v << " V\n";
            return false;
        }
        if (std::abs(state.current_a) >= kMaximumCurrentA)
        {
            std::cerr << phase << ": current limit exceeded: "
                      << state.current_a << " A\n";
            return false;
        }
        return true;
    }

    bool waitForTarget(std::chrono::milliseconds timeout,
                       int start_position,
                       double start_temperature_c,
                       std::chrono::steady_clock::time_point movement_started)
    {
        const auto deadline = movement_started + timeout;
        const int direction = target_position_ >= start_position ? 1 : -1;
        int overshoot_steps = 0;
        int peak_speed_raw = 0;
        int peak_pwm_raw = 0;
        double peak_current_a = 0.0;
        bool arrived = false;
        auto arrival_time = movement_started;
        int observation_minimum = target_position_;
        int observation_maximum = target_position_;
        int maximum_post_arrival_error = 0;
        ServoState final_state;

        while (!g_stop_requested && std::chrono::steady_clock::now() < deadline)
        {
            ServoState state;
            if (!readState(state) || !validateSafeState(state, "during move"))
            {
                disableImmediate();
                return false;
            }
            enabled_ = state.torque_enabled;
            if (!enabled_)
            {
                std::cerr << "Torque became disabled during movement.\n";
                return false;
            }

            peak_speed_raw = std::max(peak_speed_raw, std::abs(state.speed_raw));
            peak_pwm_raw = std::max(peak_pwm_raw, std::abs(state.pwm_raw));
            peak_current_a = std::max(peak_current_a, std::abs(state.current_a));
            if (direction > 0)
                overshoot_steps = std::max(overshoot_steps, state.position - target_position_);
            else
                overshoot_steps = std::max(overshoot_steps, target_position_ - state.position);

            const auto now = std::chrono::steady_clock::now();
            const int error = std::abs(state.position - target_position_);
            if (!arrived && !state.moving && error <= kPositionReadbackToleranceSteps)
            {
                arrived = true;
                arrival_time = now;
                observation_minimum = state.position;
                observation_maximum = state.position;
                maximum_post_arrival_error = error;
            }

            if (arrived)
            {
                observation_minimum = std::min(observation_minimum, state.position);
                observation_maximum = std::max(observation_maximum, state.position);
                maximum_post_arrival_error = std::max(maximum_post_arrival_error, error);
                final_state = state;

                if (now - arrival_time >= kPostArrivalObservation)
                {
                    const auto arrival_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                                arrival_time - movement_started)
                                                .count();
                    const int final_error = std::abs(final_state.position - target_position_);
                    const int jitter_span = observation_maximum - observation_minimum;
                    const bool stable = !final_state.moving &&
                                        final_error <= kPositionReadbackToleranceSteps &&
                                        maximum_post_arrival_error <= 4 && jitter_span <= 4;

                    std::cout << "Target reached. raw=" << final_state.position
                              << ", error=" << final_error << " steps\n"
                              << std::fixed << std::setprecision(3)
                              << "STEP_METRICS start=" << start_position
                              << " target=" << target_position_
                              << " final=" << final_state.position
                              << " arrival_ms=" << arrival_ms
                              << " overshoot_steps=" << overshoot_steps
                              << " steady_error_steps=" << final_error
                              << " jitter_span_steps=" << jitter_span
                              << " max_post_error_steps=" << maximum_post_arrival_error
                              << " peak_speed_raw=" << peak_speed_raw
                              << " peak_pwm_raw=" << peak_pwm_raw
                              << " peak_current_A=" << peak_current_a
                              << " temperature_delta_C="
                              << (final_state.temperature_c - start_temperature_c)
                              << " stable=" << (stable ? "yes" : "no") << '\n';
                    return true;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }

        std::cerr << "Movement timed out; executing controlled stop.\n";
        controlledStop();
        return false;
    }

    bool waitUntilStopped(std::chrono::milliseconds timeout)
    {
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (!g_stop_requested && std::chrono::steady_clock::now() < deadline)
        {
            ServoState state;
            if (!readState(state))
                return false;
            if (!state.moving)
                return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return false;
    }

    void printState(const ServoState &state) const
    {
        std::cout << std::fixed << std::setprecision(3)
                  << "state: status=0x" << std::hex << std::setw(4)
                  << std::setfill('0') << state.status_word << std::dec
                  << std::setfill(' ')
                  << " pos_raw=" << state.position
                  << " centered_deg=" << stepsToCenteredDegrees(state.position)
                  << " target_raw=" << state.target_position
                  << " speed_raw=" << state.speed_raw
                  << " voltage=" << state.voltage_v << "V"
                  << " temperature=" << state.temperature_c << "C"
                  << " current=" << state.current_a << "A"
                  << " moving=" << (state.moving ? "yes" : "no")
                  << " torque=" << (state.torque_enabled ? "ON" : "OFF")
                  << " accel=" << state.acceleration
                  << " speed_limit=" << state.speed_limit
                  << " torque_limit=" << state.torque_limit
                  << '\n';
    }

    void bestEffortTorqueOff() noexcept
    {
        if (!ctx_ || !connected_)
            return;
        if (modbus_write_register(ctx_, kTorqueEnable, 0) == 1)
            enabled_ = false;
    }

    void shutdownNoThrow() noexcept
    {
        if (!ctx_ || !connected_ || !enabled_)
            return;

        if (healthy_)
        {
            ServoState state;
            if (readState(state) && positionIsSingleTurn(state.position))
            {
                (void)modbus_write_register(ctx_, kTargetPosition,
                                            static_cast<std::uint16_t>(state.position));
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        }
        bestEffortTorqueOff();
    }

    void restorePidNoThrow() noexcept
    {
        if (!ctx_ || !connected_ || !healthy_ || !pid_changed_ ||
            !original_pid_known_)
            return;

        // Best effort only: lock=1 makes these EPROM-address writes temporary.
        // A power cycle also restores the unchanged saved parameters.
        (void)modbus_write_register(ctx_, kLockFlag, 1);
        (void)modbus_write_register(ctx_, kPositionD, original_pid_.d);
        (void)modbus_write_register(ctx_, kPositionI, original_pid_.i);
        (void)modbus_write_register(ctx_, kPositionP, original_pid_.p);
        (void)modbus_write_register(ctx_, kLockFlag, original_lock_flag_);
        pid_changed_ = false;
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

    void reportModbusError(const std::string &operation) const
    {
        std::cerr << operation << " failed: " << modbus_strerror(errno) << '\n';
    }

    static void interTransactionDelay()
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    static bool positionIsSingleTurn(int position)
    {
        return position >= 0 && position <= 4095;
    }

    static bool statusHasFault(std::uint16_t status_word)
    {
        return (status_word & ~kTorqueEnabledStatusBit) != 0;
    }

    static std::string hexAddress(std::uint16_t address)
    {
        std::ostringstream output;
        output << std::uppercase << std::hex << std::setw(4)
               << std::setfill('0') << address;
        return output.str();
    }

    std::string device_;
    int slave_id_ = 0;
    modbus_t *ctx_ = nullptr;
    bool connected_ = false;
    bool healthy_ = false;
    bool prepared_ = false;
    bool enabled_ = false;
    int initial_position_ = 0;
    int target_position_ = 0;
    int workspace_min_ = 0;
    int workspace_max_ = 4095;
    int device_minimum_ = 0;
    int device_maximum_ = 4095;
    PidValues original_pid_{};
    PidValues active_pid_{};
    std::uint16_t original_lock_flag_ = 1;
    bool original_pid_known_ = false;
    bool pid_changed_ = false;
};

void printHelp(int slave_id)
{
    std::cout
        << "Commands:\n"
        << "  status              Read and print current state\n"
        << "  pid                 Read PID and EPROM lock state\n"
        << "  pid-temp P D I      Set one conservative PID change with torque OFF; not saved\n"
        << "  restore-pid         Restore PID values observed at program startup\n"
        << "  enable " << slave_id << "            Explicitly enable this one motor ID\n"
        << "  move DEG            Relative move; magnitude must be 0.1..2.0 deg\n"
        << "  probe DEG           Automated +DEG/-DEG pair with step metrics\n"
        << "  zero-preview        Show distance/direction to legacy encoder zero raw=2048\n"
        << "  zero-step ID RAW    Move <=1 deg toward raw=2048, then torque OFF\n"
        << "  zero-run ID RAW     Move in <=1 deg waypoints to raw=2048, then torque OFF\n"
        << "  hold                Set target to current position, keep torque state\n"
        << "  stop                HOLD, wait for stop, then turn torque off\n"
        << "  disable             Turn torque off immediately (bench-supported motor only)\n"
        << "  disconnect-test     HOLD, confirm torque OFF, then test adapter unplug\n"
        << "  help                Show commands\n"
        << "  quit                Controlled stop and exit\n";
}

void printUsage(const char *program)
{
    std::cerr << "Usage: " << program
              << " --device /dev/noetix_arm_l1 --id 1 --bench-confirmed\n";
}
} // namespace

int main(int argc, char **argv)
{
    std::string device;
    int slave_id = -1;
    bool bench_confirmed = false;

    for (int i = 1; i < argc; ++i)
    {
        const std::string argument = argv[i];
        if (argument == "--device" && i + 1 < argc)
            device = argv[++i];
        else if (argument == "--id" && i + 1 < argc)
            slave_id = std::atoi(argv[++i]);
        else if (argument == "--bench-confirmed")
            bench_confirmed = true;
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

    if (!bench_confirmed || !isAllowedDevice(device) || slave_id < 1 || slave_id > 7)
    {
        printUsage(argv[0]);
        std::cerr << "The test requires an allowed device, one motor ID (1..7), and explicit bench confirmation.\n";
        return 2;
    }

    // A manually stopped systemd service remains inactive even with Restart=always.
    // Require exactly "inactive" so activating/restarting states are rejected too.
    if (std::system("test \"$(systemctl is-active startup.service 2>/dev/null)\" = inactive") != 0)
    {
        std::cerr << "startup.service is not confirmed inactive. Stop it before running this tool.\n";
        return 2;
    }

    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    std::cout << "SINGLE-SERVO BENCH CONTROL\n"
              << "Device: " << device << ", motor ID: " << slave_id << '\n'
              << "Limits: speed=100 steps/s, acceleration=200 steps/s^2, torque=10%,\n"
              << "        one move <= 2 deg, total workspace = startup position +/-5 deg.\n"
              << "PID candidates are temporary, conservative, and restored on normal exit.\n"
              << "This program does not use UDP and never reconnects automatically.\n";

    SingleServoController controller(device, slave_id);
    if (!controller.connectExclusive() || !controller.prepare())
        return 1;

    printHelp(slave_id);
    bool prompt_visible = false;
    auto next_monitor = std::chrono::steady_clock::now() + std::chrono::milliseconds(200);

    while (!g_stop_requested)
    {
        if (!prompt_visible)
        {
            std::cout << "servo> " << std::flush;
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

            std::istringstream command_stream(line);
            std::string command;
            command_stream >> command;
            if (command.empty())
                continue;

            if (command == "status")
            {
                if (!controller.printCurrentState())
                    break;
            }
            else if (command == "pid")
            {
                if (!controller.printPid())
                    break;
            }
            else if (command == "pid-temp")
            {
                int p = -1;
                int d = -1;
                int i = -1;
                std::string extra;
                if (!(command_stream >> p >> d >> i) || command_stream >> extra)
                    std::cerr << "Usage: pid-temp P D I\n";
                else
                    controller.setTemporaryPid(p, d, i);
            }
            else if (command == "restore-pid")
            {
                controller.restoreOriginalPid();
            }
            else if (command == "enable")
            {
                int confirmation = -1;
                command_stream >> confirmation;
                controller.enableExplicitly(confirmation);
            }
            else if (command == "move")
            {
                double degrees = 0.0;
                if (!(command_stream >> degrees))
                    std::cerr << "Usage: move DEG\n";
                else
                    controller.moveRelativeDegrees(degrees);
            }
            else if (command == "probe")
            {
                double degrees = 0.0;
                std::string extra;
                if (!(command_stream >> degrees) || command_stream >> extra)
                    std::cerr << "Usage: probe DEG\n";
                else
                    controller.probe(degrees);
            }
            else if (command == "zero-preview")
            {
                std::string extra;
                if (command_stream >> extra)
                    std::cerr << "Usage: zero-preview\n";
                else if (!controller.printZeroPreview())
                    break;
            }
            else if (command == "zero-step" || command == "zero-run")
            {
                int confirmation = -1;
                int start_raw = -1;
                std::string extra;
                if (!(command_stream >> confirmation >> start_raw) ||
                    command_stream >> extra)
                {
                    std::cerr << "Usage: " << command << " ID CURRENT_RAW\n";
                }
                else if (command == "zero-step")
                {
                    controller.zeroStep(confirmation, start_raw);
                }
                else
                {
                    controller.zeroRun(confirmation, start_raw);
                }
            }
            else if (command == "hold")
            {
                controller.hold();
            }
            else if (command == "stop")
            {
                controller.controlledStop();
            }
            else if (command == "disable")
            {
                controller.disableImmediate();
            }
            else if (command == "disconnect-test")
            {
                if (controller.controlledStop())
                {
                    std::cout << "Safe disconnect test is armed with torque confirmed OFF.\n"
                              << "Unplug only the selected USB/RS-485 adapter now. "
                              << "The monitor must report COMMUNICATION LOST, exit, and never "
                                 "reconnect or replay a command.\n";
                }
                else
                {
                    std::cerr << "Disconnect test rejected because a safe stopped state was not confirmed.\n";
                }
            }
            else if (command == "help")
            {
                printHelp(controller.slaveId());
            }
            else if (command == "quit" || command == "exit")
            {
                controller.controlledStop();
                break;
            }
            else
            {
                std::cerr << "Unknown command. Type 'help'.\n";
            }

            if (!controller.isHealthy())
            {
                std::cerr << "A Modbus transaction failed; leaving control mode without retry.\n";
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
                break;
            next_monitor = now + std::chrono::milliseconds(200);
        }
    }

    if (controller.isEnabled())
        controller.controlledStop();
    if (!controller.restoreOriginalPid())
        std::cerr << "Could not restore startup PID. Keep startup.service stopped and "
                     "power-cycle the servo before production use.\n";
    std::cout << "Exiting single-servo bench control.\n";
    return g_stop_requested ? 130 : 0;
}
