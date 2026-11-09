/**
  ******************************************************************************
  * @file    modbus_tcp_server.h
  * @brief   Multi-client (up to 4) Modbus TCP server task built on the LwIP
  *          netconn API and nanoMODBUS as protocol implementation.
  ******************************************************************************
  */
#ifndef APPLICATION_MODBUS_TCP_SERVER_H
#define APPLICATION_MODBUS_TCP_SERVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Returns true while at least one TCP client is connected. */
bool modbus_tcp_server_has_client(void);

/** Number of currently connected TCP clients (0..4). */
uint8_t modbus_tcp_server_client_count(void);

/** Start the Modbus TCP server task. Must be called after MX_LWIP_Init(). */
void modbus_tcp_server_start(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_MODBUS_TCP_SERVER_H */
