#pragma once

#include <cstdint>

#ifdef __cplusplus
extern "C"
{
#endif
typedef struct _modbus modbus_t;

modbus_t *modbus_new_rtu(const char *device, int baud, char parity,
                         int data_bit, int stop_bit);
int modbus_set_response_timeout(modbus_t *context, std::uint32_t seconds,
                                std::uint32_t microseconds);
int modbus_set_byte_timeout(modbus_t *context, std::uint32_t seconds,
                            std::uint32_t microseconds);
int modbus_set_slave(modbus_t *context, int slave);
int modbus_connect(modbus_t *context);
int modbus_get_socket(modbus_t *context);
int modbus_read_registers(modbus_t *context, int address, int count,
                          std::uint16_t *values);
int modbus_write_register(modbus_t *context, int address, int value);
const char *modbus_strerror(int error_number);
void modbus_close(modbus_t *context);
void modbus_free(modbus_t *context);
#ifdef __cplusplus
}
#endif
