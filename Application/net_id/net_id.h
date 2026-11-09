/**
  ******************************************************************************
  * @file    net_id.h
  * @brief   Per-device network identity derived from the STM32 96-bit unique
  *          device ID. Produces a stable, locally-administered unicast MAC that
  *          is unique per chip and identical between the application and the
  *          bootloader (both run on the same silicon and use this same code).
  ******************************************************************************
  */
#ifndef APPLICATION_NET_ID_H
#define APPLICATION_NET_ID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Fill @p mac (6 bytes) with a MAC address derived from the chip unique ID.
 *
 * mac[0] = 0x02 (locally administered, unicast: bit0 = 0, bit1 = 1). The
 * remaining octets are a hash of the 96-bit UID. The result is deterministic:
 * calling this on the same chip always yields the same MAC, so the application
 * and the bootloader present an identical address without any stored value.
 */
void net_id_get_mac(uint8_t mac[6]);

/**
 * Fill @p ip (4 bytes) with a deterministic IPv4 link-local address
 * (RFC 3927, 169.254.0.0/16) derived from the same UID hash as the MAC:
 * 169.254.<mac[4]>.<mac[5]>, with each host octet clamped to 1..254 to avoid
 * the reserved 169.254.0.x / 169.254.255.x ranges.
 *
 * Deterministic (no AutoIP re-selection) so the address can be printed on the
 * device label and reached by a plain Modbus client. Used as the factory /
 * "unconfigured" network state.
 */
void net_id_get_linklocal(uint8_t ip[4]);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_NET_ID_H */
