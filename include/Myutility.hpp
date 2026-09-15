#ifndef YAOCAO_UTILITY_H
#define YAOCAO_UTILITY_H

#include <vector>
#include <string>
#include <stdexcept>
#include <cmath>
#include <cstdio>
#include <cstdlib>

class SineGenerator
{
public:
    // 在 [posMin, posMax] 作为输出上下限的前提下生成正弦
    // startValue: 第一个点的输出值（例如 0.0）
    // startRising: true 表示从 startValue 开始后先上升；false 表示先下降
    static std::vector<double> generateFromStartValue(std::size_t nPoints,
                                                      double posMin,
                                                      double posMax,
                                                      double startValue,
                                                      double cycles = 1.0,
                                                      bool startRising = true)
    {
        if (nPoints == 0)
            return {};
        if (posMax <= posMin)
            throw std::invalid_argument("posMax must be greater than posMin.");
        if (cycles <= 0.0)
            throw std::invalid_argument("cycles must be > 0.");

        // 由极值反推 offset / amplitude（不要求对称）
        const double offset = 0.5 * (posMax + posMin);
        const double amplitude = 0.5 * (posMax - posMin);

        // 起始值必须落在范围内（并且也必须落在可达的正弦输出内）
        if (startValue < posMin || startValue > posMax)
        {
            throw std::invalid_argument("startValue must be within [posMin, posMax].");
        }

        // 解 theta0: sin(theta0) = (startValue - offset)/amplitude
        double s = (startValue - offset) / amplitude;

        // 数值安全：由于浮点误差，稍微钳到 [-1, 1]
        if (s < -1.0)
            s = -1.0;
        if (s > 1.0)
            s = 1.0;

        // principal solution in [-pi/2, pi/2]
        const double thetaA = std::asin(s);
        // other solution: pi - thetaA
        constexpr double PI = 3.14159265358979323846;
        const double thetaB = PI - thetaA;

        // 选择“起步上升/下降”的那一个解：
        // y' = amplitude * cos(theta) * dtheta/dt
        // cos(theta) > 0 => 上升；cos(theta) < 0 => 下降（假设 cycles>0）
        auto isRising = [](double theta)
        { return std::cos(theta) >= 0.0; };
        double theta0 = 0.0;

        if (startRising)
        {
            theta0 = isRising(thetaA) ? thetaA : thetaB;
        }
        else
        {
            theta0 = !isRising(thetaA) ? thetaA : thetaB;
        }

        std::vector<double> result(nPoints);

        for (std::size_t i = 0; i < nPoints; ++i)
        {
            double t = (nPoints == 1) ? 0.0
                                      : static_cast<double>(i) / static_cast<double>(nPoints - 1);

            double theta = theta0 + 2.0 * PI * cycles * t;
            double y = offset + amplitude * std::sin(theta);

            // 防止浮点误差略微越界
            if (y < posMin)
                y = posMin;
            if (y > posMax)
                y = posMax;

            result[i] = y;
        }

        // 确保第一点严格等于 startValue（避免打印/对比时出现 1e-16 的误差）
        result[0] = startValue;

        return result;
    }
};

class CsvLogger
{
public:
    explicit CsvLogger(std::string baseName,
                       size_t flushEveryN = 0,
                       size_t bufferBytes = (1u << 16))
        : file_(nullptr),
          count_(0),
          flushEveryN_(flushEveryN)
    {
        filename_ = makeFilename_(std::move(baseName));

        file_ = std::fopen(filename_.c_str(), "ab"); // 二进制追加
        if (!file_)
        {
            throw std::runtime_error("Failed to open file: " + filename_);
        }

        // 全缓冲：减少写系统调用次数（更省 CPU/IO）
        if (bufferBytes < 1024)
            bufferBytes = 1024;
        std::setvbuf(file_, nullptr, _IOFBF, bufferBytes);

        // 可选：表头（想极致省磁盘就别写）
        // std::fwrite("timestamp_ms,value\n", 1,
        //             std::strlen("timestamp_ms,value\n"), file_);
    }

    CsvLogger(const CsvLogger &) = delete;
    CsvLogger &operator=(const CsvLogger &) = delete;

    CsvLogger(CsvLogger &&other) noexcept { moveFrom_(std::move(other)); }
    CsvLogger &operator=(CsvLogger &&other) noexcept
    {
        if (this != &other)
        {
            close_();
            moveFrom_(std::move(other));
        }
        return *this;
    }

    ~CsvLogger() { close_(); }

    // 追加一条数据：timestamp_ms,value
    inline void append(double value)
    {
        if (!file_)
            return;

        const long long ts = nowMs_();

        // snprintf + fwrite：简单、稳定、CPU开销低
        char buf[96];
        int n = std::snprintf(buf, sizeof(buf), "%lld,%.17g\n", ts, value);
        if (n > 0)
        {
            std::fwrite(buf, 1, static_cast<size_t>(n), file_);
        }

        ++count_;
        if (flushEveryN_ > 0 && (count_ % flushEveryN_ == 0))
        {
            std::fflush(file_);
        }
    }

    inline void flush()
    {
        if (file_)
            std::fflush(file_);
    }

    const std::string &filename() const { return filename_; }

private:
    std::FILE *file_;
    std::string filename_;
    size_t count_;
    size_t flushEveryN_;

    static inline long long nowMs_()
    {
        using namespace std::chrono;
        return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    }

    static std::string ensureCsvExt_(std::string s)
    {
        auto lower = [](char c)
        { return (c >= 'A' && c <= 'Z') ? char(c - 'A' + 'a') : c; };
        if (s.size() >= 4)
        {
            char a0 = lower(s[s.size() - 4]);
            char a1 = lower(s[s.size() - 3]);
            char a2 = lower(s[s.size() - 2]);
            char a3 = lower(s[s.size() - 1]);
            if (a0 == '.' && a1 == 'c' && a2 == 's' && a3 == 'v')
                return s;
        }
        s += ".csv";
        return s;
    }

    static std::string makeFilename_(std::string baseName)
    {
        using namespace std::chrono;
        auto now = system_clock::now();
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        std::time_t t = system_clock::to_time_t(now);
        std::tm tm{};
#if defined(_WIN32)
        localtime_s(&tm, &t);
#else
        localtime_r(&t, &tm);
#endif

        // 去掉用户输入末尾的 .csv，避免 data.csv_时间.csv
        if (baseName.size() >= 4)
        {
            std::string tail = baseName.substr(baseName.size() - 4);
            for (auto &c : tail)
                if (c >= 'A' && c <= 'Z')
                    c = char(c - 'A' + 'a');
            if (tail == ".csv")
                baseName = baseName.substr(0, baseName.size() - 4);
        }

        char suffix[64];
        std::snprintf(suffix, sizeof(suffix),
                      "_%04d%02d%02d_%02d%02d%02d_%03d",
                      tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday,
                      tm.tm_hour, tm.tm_min, tm.tm_sec,
                      static_cast<int>(ms.count()));

        baseName += suffix;
        return ensureCsvExt_(baseName);
    }

    inline void close_()
    {
        if (!file_)
            return;
        std::fflush(file_);
        std::fclose(file_);
        file_ = nullptr;
    }

    void moveFrom_(CsvLogger &&other) noexcept
    {
        file_ = other.file_;
        filename_ = std::move(other.filename_);
        count_ = other.count_;
        flushEveryN_ = other.flushEveryN_;
        other.file_ = nullptr;
        other.count_ = 0;
    }
};

class MovingAverageFilter
{
private:
    std::vector<double> buffer; // 环形缓冲区
    size_t head;                // 下一个写入位置
    size_t tail;                // 下一个读取位置
    size_t count;               // 当前元素数量
    size_t capacity;            // 缓冲区容量
    double sum;                 // 缓冲区元素总和
    bool full;                  // 缓冲区是否已满

public:
    // 构造函数，设置队列容量
    MovingAverageFilter(size_t cap)
        : buffer(cap, 0.0), head(0), tail(0), count(0),
          capacity(cap), sum(0.0), full(false)
    {
        if (cap == 0)
        {
            throw std::invalid_argument("Capacity must be greater than 0");
        }
    }

    // 添加新数据并返回当前平均值 - O(1) 时间复杂度
    double add(double value)
    {
        // 如果缓冲区已满，移除最早的数据
        if (full)
        {
            sum -= buffer[head];
        }
        else
        {
            count++;
        }

        // 添加新数据
        buffer[head] = value;
        sum += value;

        // 移动头指针
        head = (head + 1) % capacity;

        // 更新满状态
        if (head == tail)
        {
            full = true;
        }
        else
        {
            tail = full ? (tail + 1) % capacity : tail;
        }

        // 返回平均值
        return getAverage();
    }

    // 获取当前平均值 - O(1) 时间复杂度
    double getAverage() const
    {
        if (count == 0)
        {
            return 0.0;
        }
        return sum / count;
    }

    // 获取加权移动平均值（指数权重）
    double getWeightedAverage() const
    {
        if (count == 0)
        {
            return 0.0;
        }

        double weightedSum = 0.0;
        double weightSum = 0.0;
        double alpha = 2.0 / (count + 1); // 平滑因子

        for (size_t i = 0; i < count; ++i)
        {
            size_t idx = (tail + i) % capacity;
            double weight = std::pow(1 - alpha, count - i - 1);
            weightedSum += buffer[idx] * weight;
            weightSum += weight;
        }

        return weightedSum / weightSum;
    }

    // 清空队列 - O(1) 时间复杂度
    void clear()
    {
        head = 0;
        tail = 0;
        count = 0;
        sum = 0.0;
        full = false;
    }

    // 获取当前队列大小 - O(1) 时间复杂度
    size_t size() const
    {
        return count;
    }

    // 检查队列是否已满 - O(1) 时间复杂度
    bool isFull() const
    {
        return full;
    }

    // 获取队列容量 - O(1) 时间复杂度
    size_t getCapacity() const
    {
        return capacity;
    }

    // 设置新的队列容量 - O(n) 时间复杂度，仅在必要时调用
    void setCapacity(size_t newCapacity)
    {
        if (newCapacity == 0)
        {
            throw std::invalid_argument("Capacity must be greater than 0");
        }

        if (newCapacity == capacity)
        {
            return; // 容量未变，直接返回
        }

        // 创建新缓冲区并复制数据
        std::vector<double> newBuffer(newCapacity, 0.0);
        size_t newCount = std::min(count, newCapacity);

        // 复制数据到新缓冲区
        for (size_t i = 0; i < newCount; ++i)
        {
            size_t idx = (tail + i) % capacity;
            newBuffer[i] = buffer[idx];
        }

        // 更新状态
        buffer = std::move(newBuffer);
        capacity = newCapacity;
        head = newCount % capacity;
        tail = 0;
        count = newCount;
        full = (newCount == capacity);

        // 重新计算总和
        sum = 0.0;
        for (size_t i = 0; i < count; ++i)
        {
            sum += buffer[i];
        }
    }

    // 获取最近N个值的平均值（N <= 容量）
    double getRecentAverage(size_t n) const
    {
        if (n == 0 || count == 0)
        {
            return 0.0;
        }

        n = std::min(n, count);
        double recentSum = 0.0;

        // 从最新数据开始向前累加
        for (size_t i = 0; i < n; ++i)
        {
            // 计算索引：head-1 是最后一个元素，head-2 是倒数第二个，以此类推
            size_t idx = (head - 1 - i + capacity) % capacity;
            recentSum += buffer[idx];
        }

        return recentSum / n;
    }

    // 获取缓冲区中最小值和最大值
    void getMinMax(double &minVal, double &maxVal) const
    {
        if (count == 0)
        {
            minVal = maxVal = 0.0;
            return;
        }

        minVal = maxVal = buffer[tail];
        for (size_t i = 1; i < count; ++i)
        {
            size_t idx = (tail + i) % capacity;
            if (buffer[idx] < minVal)
                minVal = buffer[idx];
            if (buffer[idx] > maxVal)
                maxVal = buffer[idx];
        }
    }
};

class DirectionConfig
{
public:
    DirectionConfig(const std::string &filename)
    {
        load(filename);
    }

    std::array<int, 14> getVector() const { return vec_; }

private:
    std::array<int, 14> vec_{};

    void load(const std::string &filename)
    {
        std::ifstream file(filename);
        if (!file)
        {
            throw std::runtime_error("Cannot open file: " + filename);
        }

        std::string line;
        int count = 0;
        while (std::getline(file, line))
        {
            // 去掉注释和空行
            auto comment_pos = line.find('#');
            if (comment_pos != std::string::npos)
                line = line.substr(0, comment_pos);
            if (line.empty())
                continue;

            auto pos = line.find('=');
            if (pos == std::string::npos)
            {
                throw std::runtime_error("Invalid line (missing '='): " + line);
            }

            std::string key_str = line.substr(0, pos);
            std::string val_str = line.substr(pos + 1);

            // 转换 key
            int key = -1;
            try
            {
                key = std::stoi(key_str);
            }
            catch (...)
            {
                throw std::runtime_error("Invalid key: " + key_str);
            }

            if (key < 0 || key > 15)
            {
                throw std::runtime_error("Key out of range [0-15]: " + key_str);
            }

            // 转换值
            int val = 0;
            try
            {
                val = std::stoi(val_str);
            }
            catch (...)
            {
                throw std::runtime_error("Invalid value: " + val_str);
            }

            if (val != 1 && val != -1)
            {
                throw std::runtime_error("Value must be +1 or -1, got: " + val_str);
            }

            vec_[key] = val;
            count++;
        }

        if (count != 14)
        {
            std::cerr << "Warning: only " << count << " entries loaded, expected 14. Missing values default to +1.\n";
            for (int i = 0; i < 14; i++)
                if (vec_[i] != 1 && vec_[i] != -1)
                    vec_[i] = 1;
        }
    }
};

#endif