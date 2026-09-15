#ifndef NOETIX_SERIALIZER_H
#define NOETIX_SERIALIZER_H

#include <cstdint>
#include <cstddef>
#include <cmath>

#pragma pack(push, 1)

// 8字节联合体，用于 double <-> 原始bit( uint64 ) 转换
union Serializer_byte8
{
    double dfdata;
    uint8_t byte[8];
    uint64_t intdata;
};

// Xbox手柄映射结构体
typedef struct
{
    int32_t time;
    int32_t a;
    int32_t b;
    int32_t x;
    int32_t y;
    int32_t lb;
    int32_t rb;
    int32_t start;
    int32_t back;
    int32_t home;
    int32_t lo;
    int32_t ro;
    int32_t lx;
    int32_t ly;
    int32_t rx;
    int32_t ry;
    int32_t lt;
    int32_t rt;
    int32_t xx;
    int32_t yy;
} ser_xbox_map_t;

// 主数据结构体（payload内容）
typedef struct
{
    uint64_t sequence_number;
    uint64_t type;
    double left_arm_pos[8];
    double right_arm_pos[8];
    ser_xbox_map_t map;

} noetix_upd_in_t;

// 监控类型：仅包含我们关心的成员
struct Monitored
{
    double left_arm_pos[8];
    double right_arm_pos[8];

    bool operator!=(const Monitored &other) const
    {
        // 比较左臂数组
        for (int i = 0; i < 8; ++i)
        {
            if (std::abs(left_arm_pos[i] - other.left_arm_pos[i]) > 0.05)
                return true;
        }
        // 比较右臂数组
        for (int i = 0; i < 8; ++i)
        {
            if (std::abs(right_arm_pos[i] - other.right_arm_pos[i]) > 0.05)
                return true;
        }
        return false;
    }
};

// 协议头结构体
typedef struct
{
    uint32_t magic;     // 魔数，用于识别协议
    uint32_t version;   // 协议版本
    uint32_t data_size; // 数据部分大小（payload大小）
    uint32_t checksum;  // 简单校验和（对payload计算）
} noetix_header_t;

#pragma pack(pop)

// 协议常量定义
namespace noetix
{
    constexpr uint32_t MAGIC = 0x4E4F4554; // "NOET"
    constexpr uint32_t PROTOCOL_VERSION = 1;

    constexpr size_t HEADER_SIZE = sizeof(noetix_header_t);

    constexpr size_t DATA_SIZE = sizeof(noetix_upd_in_t);

    constexpr size_t TOTAL_SIZE = HEADER_SIZE + DATA_SIZE;

}

enum SerializationType
{
    SERIAL_HEARTBEAT = 0, // 心跳
    SERIAL_DATA = 1,      // 正常业务包
    SERIAL_ERROR = 2,     // 异常/告警包
};

// 错误码定义
enum class SerializationError
{
    SUCCESS = 0,
    INVALID_INPUT = -1,
    INVALID_MAGIC = -2,
    VERSION_MISMATCH = -3,
    SIZE_MISMATCH = -4,
    CHECKSUM_ERROR = -5,
    MEMORY_ALLOCATION_ERROR = -6
};

class NoetixSerializer
{
public:
    NoetixSerializer(const NoetixSerializer &) = delete;
    NoetixSerializer &operator=(const NoetixSerializer &) = delete;

    static uint8_t *serialize(const noetix_upd_in_t *data, size_t &out_size);
    static noetix_upd_in_t *deserialize(const uint8_t *data, size_t data_size, SerializationError &error_code);

    static constexpr size_t serializedSize()
    {
        return noetix::TOTAL_SIZE;
    }

    static SerializationError validate(const uint8_t *data, size_t data_size);
    static uint32_t calculateChecksum(const uint8_t *data, size_t size);
    static void printError(SerializationError error);

private:
    static uint64_t hostToNetwork64(uint64_t value);
    static uint64_t networkToHost64(uint64_t value);
    static uint32_t hostToNetwork32(uint32_t value);
    static uint32_t networkToHost32(uint32_t value);

    static uint8_t *serializeData(const noetix_upd_in_t *data, size_t &out_size);
    static noetix_upd_in_t *deserializeData(const uint8_t *data, size_t data_size);

    static bool isLittleEndian();
};

#endif // NOETIX_SERIALIZER_H
