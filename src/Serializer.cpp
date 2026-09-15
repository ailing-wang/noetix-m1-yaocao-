#include "Serializer.h"
#include <cstring>
#include <iostream>
#include <new>

// 使用标准C++11的字节序转换函数
#ifdef _WIN32
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

static inline void write_u64_be(uint8_t *&ptr, uint64_t v, uint64_t (*h2n64)(uint64_t))
{
    uint64_t net = h2n64(v);
    std::memcpy(ptr, &net, sizeof(net));
    ptr += sizeof(net);
}

static inline uint64_t read_u64_be(const uint8_t *&ptr, uint64_t (*n2h64)(uint64_t))
{
    uint64_t net;
    std::memcpy(&net, ptr, sizeof(net));
    ptr += sizeof(net);
    return n2h64(net);
}

static inline void write_double_be(uint8_t *&ptr, double d, uint64_t (*h2n64)(uint64_t))
{
    Serializer_byte8 cvt;
    cvt.dfdata = d;
    uint64_t net = h2n64(cvt.intdata);
    std::memcpy(ptr, &net, sizeof(net));
    ptr += sizeof(net);
}

static inline double read_double_be(const uint8_t *&ptr, uint64_t (*n2h64)(uint64_t))
{
    uint64_t net;
    std::memcpy(&net, ptr, sizeof(net));
    ptr += sizeof(net);

    Serializer_byte8 cvt;
    cvt.intdata = n2h64(net);
    return cvt.dfdata;
}

uint8_t *NoetixSerializer::serialize(const noetix_upd_in_t *data, size_t &out_size)
{
    out_size = 0;
    if (!data)
        return nullptr;

    try
    {
        // 序列化数据部分
        size_t payload_size = 0;
        uint8_t *payload = serializeData(data, payload_size);
        if (!payload)
            return nullptr;

        out_size = noetix::HEADER_SIZE + payload_size;
        uint8_t *buffer = new uint8_t[out_size];

        // 填充协议头
        noetix_header_t header;
        header.magic = hostToNetwork32(noetix::MAGIC);
        header.version = hostToNetwork32(noetix::PROTOCOL_VERSION);
        header.data_size = hostToNetwork32(static_cast<uint32_t>(payload_size));
        header.checksum = hostToNetwork32(calculateChecksum(payload, payload_size));

        // 组合头和数据
        std::memcpy(buffer, &header, noetix::HEADER_SIZE);
        std::memcpy(buffer + noetix::HEADER_SIZE, payload, payload_size);

        delete[] payload;
        return buffer;
    }
    catch (const std::bad_alloc &)
    {
        out_size = 0;
        return nullptr;
    }
}

noetix_upd_in_t *NoetixSerializer::deserialize(const uint8_t *data, size_t data_size, SerializationError &error_code)
{
    error_code = SerializationError::SUCCESS;

    // 验证数据
    error_code = validate(data, data_size);
    if (error_code != SerializationError::SUCCESS)
        return nullptr;

    // 解析协议头，得到真实payload长度
    noetix_header_t header;
    std::memcpy(&header, data, noetix::HEADER_SIZE);
    header.data_size = networkToHost32(header.data_size);

    // 解析数据部分
    const uint8_t *payload = data + noetix::HEADER_SIZE;
    noetix_upd_in_t *result = deserializeData(payload, header.data_size);
    if (!result)
        error_code = SerializationError::MEMORY_ALLOCATION_ERROR;

    return result;
}

SerializationError NoetixSerializer::validate(const uint8_t *data, size_t data_size)
{
    if (!data)
        return SerializationError::INVALID_INPUT;

    if (data_size < noetix::HEADER_SIZE)
        return SerializationError::SIZE_MISMATCH;

    // 解析协议头
    noetix_header_t header;
    std::memcpy(&header, data, noetix::HEADER_SIZE);

    header.magic = networkToHost32(header.magic);
    header.version = networkToHost32(header.version);
    header.data_size = networkToHost32(header.data_size);
    header.checksum = networkToHost32(header.checksum);

    if (header.magic != noetix::MAGIC)
        return SerializationError::INVALID_MAGIC;

    if (header.version != noetix::PROTOCOL_VERSION)
        return SerializationError::VERSION_MISMATCH;

    // 1) header.data_size 必须让整个包长度对得上
    if (data_size != noetix::HEADER_SIZE + static_cast<size_t>(header.data_size))
    {
        return SerializationError::SIZE_MISMATCH;
    }

    // 2) 当前版本仍然要求固定payload大小（如果你未来要支持心跳/错误包，可放开这句）
    if (static_cast<size_t>(header.data_size) != noetix::DATA_SIZE)
    {
        return SerializationError::SIZE_MISMATCH;
    }

    // 校验和
    const uint8_t *payload = data + noetix::HEADER_SIZE;
    uint32_t calc = calculateChecksum(payload, header.data_size);
    if (calc != header.checksum)
        return SerializationError::CHECKSUM_ERROR;

    return SerializationError::SUCCESS;
}

uint32_t NoetixSerializer::calculateChecksum(const uint8_t *data, size_t size)
{
    uint32_t checksum = 0;
    for (size_t i = 0; i < size; ++i)
        checksum += data[i];
    return checksum;
}

uint8_t *NoetixSerializer::serializeData(const noetix_upd_in_t *data, size_t &out_size)
{
    out_size = noetix::DATA_SIZE;
    uint8_t *buffer = new uint8_t[out_size];
    uint8_t *ptr = buffer;

    write_u64_be(ptr, data->sequence_number, &NoetixSerializer::hostToNetwork64);
    write_u64_be(ptr, data->type, &NoetixSerializer::hostToNetwork64);

    // left_arm_pos[8]
    for (int i = 0; i < 8; ++i)
        write_double_be(ptr, data->left_arm_pos[i], &NoetixSerializer::hostToNetwork64);

    // right_arm_pos[8]
    for (int i = 0; i < 8; ++i)
        write_double_be(ptr, data->right_arm_pos[i], &NoetixSerializer::hostToNetwork64);

    // ser_xbox_map_t（按 int32 字段做网络序）
    ser_xbox_map_t network_map = data->map;
    network_map.time = hostToNetwork32(data->map.time);
    network_map.a = hostToNetwork32(data->map.a);
    network_map.b = hostToNetwork32(data->map.b);
    network_map.x = hostToNetwork32(data->map.x);
    network_map.y = hostToNetwork32(data->map.y);
    network_map.lb = hostToNetwork32(data->map.lb);
    network_map.rb = hostToNetwork32(data->map.rb);
    network_map.start = hostToNetwork32(data->map.start);
    network_map.back = hostToNetwork32(data->map.back);
    network_map.home = hostToNetwork32(data->map.home);
    network_map.lo = hostToNetwork32(data->map.lo);
    network_map.ro = hostToNetwork32(data->map.ro);
    network_map.lx = hostToNetwork32(data->map.lx);
    network_map.ly = hostToNetwork32(data->map.ly);
    network_map.rx = hostToNetwork32(data->map.rx);
    network_map.ry = hostToNetwork32(data->map.ry);
    network_map.lt = hostToNetwork32(data->map.lt);
    network_map.rt = hostToNetwork32(data->map.rt);
    network_map.xx = hostToNetwork32(data->map.xx);
    network_map.yy = hostToNetwork32(data->map.yy);

    std::memcpy(ptr, &network_map, sizeof(ser_xbox_map_t));
    ptr += sizeof(ser_xbox_map_t);

    // 防呆：确保写入字节数完全等于 out_size
    // （如果触发，说明你未来改了字段但没同步序列化）
    if (static_cast<size_t>(ptr - buffer) != out_size)
    {
        delete[] buffer;
        out_size = 0;
        return nullptr;
    }

    return buffer;
}

noetix_upd_in_t *NoetixSerializer::deserializeData(const uint8_t *data, size_t data_size)
{
    if (!data)
        return nullptr;

    if (data_size != noetix::DATA_SIZE)
        return nullptr;

    noetix_upd_in_t *result = new noetix_upd_in_t;
    const uint8_t *ptr = data;

    result->sequence_number = read_u64_be(ptr, &NoetixSerializer::networkToHost64);
    result->type = read_u64_be(ptr, &NoetixSerializer::networkToHost64);

    for (int i = 0; i < 8; ++i)
        result->left_arm_pos[i] = read_double_be(ptr, &NoetixSerializer::networkToHost64);

    for (int i = 0; i < 8; ++i)
        result->right_arm_pos[i] = read_double_be(ptr, &NoetixSerializer::networkToHost64);

    ser_xbox_map_t network_map;
    std::memcpy(&network_map, ptr, sizeof(ser_xbox_map_t));
    ptr += sizeof(ser_xbox_map_t);

    result->map.time = networkToHost32(network_map.time);
    result->map.a = networkToHost32(network_map.a);
    result->map.b = networkToHost32(network_map.b);
    result->map.x = networkToHost32(network_map.x);
    result->map.y = networkToHost32(network_map.y);
    result->map.lb = networkToHost32(network_map.lb);
    result->map.rb = networkToHost32(network_map.rb);
    result->map.start = networkToHost32(network_map.start);
    result->map.back = networkToHost32(network_map.back);
    result->map.home = networkToHost32(network_map.home);
    result->map.lo = networkToHost32(network_map.lo);
    result->map.ro = networkToHost32(network_map.ro);
    result->map.lx = networkToHost32(network_map.lx);
    result->map.ly = networkToHost32(network_map.ly);
    result->map.rx = networkToHost32(network_map.rx);
    result->map.ry = networkToHost32(network_map.ry);
    result->map.lt = networkToHost32(network_map.lt);
    result->map.rt = networkToHost32(network_map.rt);
    result->map.xx = networkToHost32(network_map.xx);
    result->map.yy = networkToHost32(network_map.yy);

    // 防呆：确保正好读完
    if (static_cast<size_t>(ptr - data) != data_size)
    {
        delete result;
        return nullptr;
    }

    return result;
}

// 字节序转换实现
uint64_t NoetixSerializer::hostToNetwork64(uint64_t value)
{
    if (isLittleEndian())
    {
        return ((value & 0x00000000000000FFULL) << 56) |
               ((value & 0x000000000000FF00ULL) << 40) |
               ((value & 0x0000000000FF0000ULL) << 24) |
               ((value & 0x00000000FF000000ULL) << 8) |
               ((value & 0x000000FF00000000ULL) >> 8) |
               ((value & 0x0000FF0000000000ULL) >> 24) |
               ((value & 0x00FF000000000000ULL) >> 40) |
               ((value & 0xFF00000000000000ULL) >> 56);
    }
    return value;
}

uint64_t NoetixSerializer::networkToHost64(uint64_t value)
{
    return hostToNetwork64(value); // 对称
}

uint32_t NoetixSerializer::hostToNetwork32(uint32_t value)
{
    return htonl(value);
}

uint32_t NoetixSerializer::networkToHost32(uint32_t value)
{
    return ntohl(value);
}

bool NoetixSerializer::isLittleEndian()
{
    static const union
    {
        uint32_t i;
        uint8_t c[4];
    } test = {0x01020304};

    return test.c[0] == 0x04;
}

void NoetixSerializer::printError(SerializationError error)
{
    switch (error)
    {
    case SerializationError::SUCCESS:
        std::cout << "成功" << std::endl;
        break;
    case SerializationError::INVALID_INPUT:
        std::cout << "无效输入" << std::endl;
        break;
    case SerializationError::INVALID_MAGIC:
        std::cout << "魔数错误" << std::endl;
        break;
    case SerializationError::VERSION_MISMATCH:
        std::cout << "版本不匹配" << std::endl;
        break;
    case SerializationError::SIZE_MISMATCH:
        std::cout << "大小不匹配" << std::endl;
        break;
    case SerializationError::CHECKSUM_ERROR:
        std::cout << "校验和错误" << std::endl;
        break;
    case SerializationError::MEMORY_ALLOCATION_ERROR:
        std::cout << "内存分配错误" << std::endl;
        break;
    default:
        std::cout << "未知错误" << std::endl;
    }
}
