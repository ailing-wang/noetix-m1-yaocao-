#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>

#include "spdlog/sinks/rotating_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"
#include <spdlog/common.h>

template <typename... Args>
inline std::string format_string(const char *format, Args... args)
{
    constexpr size_t oldlen = BUFSIZ;
    char buffer[oldlen]; // 默认栈上的缓冲区

    size_t newlen = snprintf(&buffer[0], oldlen, format, args...);
    newlen++; // 算上终止符'\0'

    if (newlen > oldlen)
    { // 默认缓冲区不够大，从堆上分配
        std::vector<char> newbuffer(newlen);
        snprintf(newbuffer.data(), newlen, format, args...);
        return std::string(newbuffer.data());
    }

    return buffer;
}

class Logger
{
public:
    static Logger *intance()
    {
        static Logger logger;
        return &logger;
    }
    virtual ~Logger()
    {
        spdlog::drop_all(); // 关闭所有logger
    }

    template <typename... Args>
    void debug(spdlog::source_loc loc, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        // SPDLOG_WARN(fmt);
        std::shared_ptr<spdlog::logger> logger = spdlog::get(m_loggerName);
        logger->log(loc, spdlog::level::debug, spdlog::details::to_string_view(fmt), std::forward<Args>(args)...);
    }

    template <typename... Args>
    void info(spdlog::source_loc loc, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        // SPDLOG_WARN(fmt);
        std::shared_ptr<spdlog::logger> logger = spdlog::get(m_loggerName);
        logger->log(loc, spdlog::level::info, spdlog::details::to_string_view(fmt), std::forward<Args>(args)...);
    }

    // spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}
    template <typename... Args>
    void warn(spdlog::source_loc loc, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        // SPDLOG_WARN(fmt);
        std::shared_ptr<spdlog::logger> logger = spdlog::get(m_loggerName);
        logger->log(loc, spdlog::level::warn, spdlog::details::to_string_view(fmt), std::forward<Args>(args)...);
    }

    template <typename... Args>
    void error(spdlog::source_loc loc, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        // SPDLOG_WARN(fmt);
        std::shared_ptr<spdlog::logger> logger = spdlog::get(m_loggerName);
        logger->log(loc, spdlog::level::err, spdlog::details::to_string_view(fmt), std::forward<Args>(args)...);
    }

    template <typename... Args>
    void critical(spdlog::source_loc loc, spdlog::format_string_t<Args...> fmt, Args &&...args)
    {
        // SPDLOG_WARN(fmt);
        std::shared_ptr<spdlog::logger> logger = spdlog::get(m_loggerName);
        logger->log(loc, spdlog::level::critical, spdlog::details::to_string_view(fmt), std::forward<Args>(args)...);
    }

protected:
    Logger()
    {
        m_loggerName = "spdlog";
        try
        {
#if 1
            // 定义控制台
            auto consoleSink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
            // 设定日志最大，且最多保留10个
            auto rotatSink = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("./logs/log.txt", 10 * 1024 * 1024, 30);
            auto logger = std::make_shared<spdlog::logger>(m_loggerName, spdlog::sinks_init_list{consoleSink, rotatSink});
            spdlog::register_logger(logger);
#else
            // 创建rotaing_logger_mt日志记录器,设定日志最大，且最多保留10个
            std::shared_ptr<spdlog::logger> logger =
                spdlog::rotating_logger_mt(m_loggerName, "./logs/log.txt", 10 * 1024 * 1024, 30);
#endif

            spdlog::set_default_logger(logger);
            // 设置日志格式. 参数含义: [日志标识符] [日期] [日志级别] [线程号] [文件名 函数名:行号] [数据]
            // mylogger->set_pattern("[%n] [%Y-%m-%d %H:%M:%S.%e] [thread:%l] [%t] [%s %!:%#] %v");
            logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%l] [thread:%t] [%s %!:%#] %v");
            // 默认保存warn信息
            logger->set_level(spdlog::level::warn);
            // 定期刷新日志缓冲区
            spdlog::flush_every(std::chrono::seconds(2));
            // 遇到及以上级别会立马将缓存的buffer写到文件中，底层调用是std::fflush(_fd)
            spdlog::flush_on(spdlog::level::warn);
        }
        catch (const spdlog::spdlog_ex &ex)
        {
            std::cout << "Log initialization failed: " << ex.what() << std::endl;
        }
    }

private:
    std::string m_loggerName;
};

#define LogDebug(...) Logger::intance()->debug(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)
#define LogInfo(...) Logger::intance()->info(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)
#define LogWarn(...) Logger::intance()->warn(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)
#define LogError(...) Logger::intance()->error(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)
#define LogCritical(...) \
    Logger::intance()->critical(spdlog::source_loc{__FILE__, __LINE__, SPDLOG_FUNCTION}, __VA_ARGS__)

class ThrottledErrorReporter
{
public:
    // Remove delay parameter, only keep report_interval
    explicit ThrottledErrorReporter(int report_interval = 10)
        : report_interval_(report_interval), counter_(0) {}

    // report now only increments counter and executes action when threshold is met
    void report(std::function<void()> report_action)
    {
        counter_++;
        if (counter_ % report_interval_ == 0)
        {
            report_action();
        }
        // No delay
    }

    void reset() { counter_ = 0; }

private:
    int report_interval_;
    std::atomic<int> counter_;
};

#endif // LOGGER_H
