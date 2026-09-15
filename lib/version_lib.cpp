#include "version_lib.hpp"

namespace VersionSpace
{

    std::string VersionLib::getVersion()
    {
        return "1.0.0";
    }

    void VersionLib::printInfo()
    {
        std::cout << "=====================================" << std::endl;
        std::cout << "Version Library v" << getVersion() << std::endl;
        std::cout << "Build Time: " << getBuildTime() << std::endl;
        std::cout << "Function: Provides version information" << std::endl;
        std::cout << "=====================================" << std::endl;
    }

    std::string VersionLib::getBuildTime()
    {
        return __DATE__ " " __TIME__;
    }

    bool VersionLib::isAvailable() const
    {
        return initialized;
    }

} // namespace VersionSpace