/**
  ******************************************************************************
  * @file    discovery.h
  * @brief   PLCJS Discovery Protocol (PDP) responder — UDP/20556 broadcast.
  *
  * Lets a commissioning tool (or a factory-fresh, link-local device) be found
  * and addressed by MAC without a preassigned IP — similar in spirit to
  * PROFINET DCP, but implemented over UDP broadcast so it works on our stack.
  *
  * Requests and responses are UDP broadcast to 255.255.255.255:20556 so they
  * traverse subnet mismatches within a single L2 segment. A device acts on a
  * request only if the frame's target MAC is all-zero (broadcast) or equals
  * its own MAC.
  ******************************************************************************
  */
#ifndef APPLICATION_DISCOVERY_H
#define APPLICATION_DISCOVERY_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Open the UDP discovery responder. Call once after the netif is up. */
void discovery_init(void);

/* Deferred actions requested over discovery, consumed by the application's
 * housekeeping loop so the Flash write / reset runs outside the tcpip thread. */
bool discovery_take_pending_save(void);
bool discovery_take_pending_reboot(void);
bool discovery_take_pending_factory(void);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_DISCOVERY_H */
