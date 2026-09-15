#ifndef VERSION_LIB_HPP
#define VERSION_LIB_HPP

#include <string>
#include <iostream>

namespace VersionSpace
{

    class VersionLib
    {
    public:
        /**
         * @brief 获取库版本号
         * @return 版本号字符串
         */
        static std::string getVersion();

        /**
         * @brief 打印库信息
         */
        static void printInfo();

        /**
         * @brief 获取库构建时间
         * @return 构建时间字符串
         */
        static std::string getBuildTime();

        /**
         * @brief 检查库功能是否可用
         * @return 如果库功能正常返回true
         */
        bool isAvailable() const;

    private:
        bool initialized = true;
    };

} // namespace VersionSpace

#endif // VERSION_LIB_HPP