/* ---------------------------------------------------------------------------
 * Driver for Microchip/Micrel KSZ8863MLLI 3-port Ethernet switch.
 *
 * In this project the chip is used as an unmanaged switch — RMII REF_CLK is
 * generated internally by the PHY (REFCLKO_3 -> REFCLKI_3), and the STM32 ETH
 * MAC sits on the third port. Management is done over SMI/MIIM (the same
 * MDC/MDIO bus the STM32 ETH peripheral uses), so no extra wires are needed.
 *
 * Over SMI the chip exposes the standard MII PHY register set on two PHY
 * addresses, one per external port:
 *     PHY 0x01 = port 1
 *     PHY 0x02 = port 2
 *
 * The third port (the one that connects to the STM32 MAC) is not visible
 * over SMI — it is configured by strap pins / internal defaults.
 *
 * All register access goes through HAL_ETH_ReadPHYRegister() /
 * HAL_ETH_WritePHYRegister(), so HAL_ETH_Init() must have been called
 * before any of the read/write APIs below are used. The only exception is
 * ksz8863_hw_reset(), which only toggles the ETHRST GPIO and must run
 * BEFORE HAL_ETH_Init() (i.e. before MX_LWIP_Init()).
 * ------------------------------------------------------------------------- */
#ifndef KSZ8863_H
#define KSZ8863_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    KSZ8863_PORT1 = 1u,
    KSZ8863_PORT2 = 2u,
} ksz8863_port_t;

typedef enum {
    KSZ8863_SPEED_10  = 0u,
    KSZ8863_SPEED_100 = 1u,
} ksz8863_speed_t;

typedef enum {
    KSZ8863_DUPLEX_HALF = 0u,
    KSZ8863_DUPLEX_FULL = 1u,
} ksz8863_duplex_t;

typedef struct {
    bool             link_up;        /* current link state                */
    bool             autoneg_done;   /* auto-negotiation completed        */
    ksz8863_speed_t  speed;          /* operating speed                   */
    ksz8863_duplex_t duplex;         /* operating duplex                  */
} ksz8863_link_status_t;

/* Pulse ETHRST low briefly, then release and wait for the chip to be ready
 * for SMI access (~20 ms total). Interrupts pass-through traffic between the
 * external ports for the duration of the reset + auto-negotiation, so it is
 * reserved for cold boot and explicit recovery — see ksz8863_boot_init() and
 * ksz8863_service(). */
void ksz8863_hw_reset(void);

/* Boot-time reset policy. Call once early at boot (before MX_LWIP_Init()).
 *
 * The switch forwards traffic between ports 1 and 2 autonomously, so a warm
 * MCU reboot (software reset, IWDG, NRST pin) must NOT reset it — the module
 * acts as a pass-through switch and a reset would drop the link on both
 * external ports for seconds (auto-negotiation). ETHRST stays high through
 * the reboot: the board pull-up covers the MCU reset window and
 * MX_GPIO_Init() initialises the pin to high.
 *
 * Only a cold boot (power-on / brown-out) performs a defensive hardware
 * reset: the chip has just powered up and no traffic exists yet.
 *
 * Consumes (clears) the RCC reset-cause flags. */
void ksz8863_boot_init(void);

/* Make sure switch port 3 (STM32 MAC) is in RMII mode: Global Control 4,
 * bit 6. Idempotent — reads first and writes only when the bit is clear.
 * Requires SMI (HAL_ETH_Init() done). */
void ksz8863_ensure_rmii_port3(void);

/* Request an operator-driven recovery reset of the switch. Safe to call from
 * any task (only sets a flag); the reset itself is executed by
 * ksz8863_service() in the link-polling thread so that all SMI access stays
 * on one thread. */
void ksz8863_request_recovery(void);

/* Periodic health service — call from the link-polling thread once per poll
 * (~100 ms), passing whether SMI answered this cycle. Performs a hardware
 * recovery reset when either:
 *   - an operator requested one via ksz8863_request_recovery(), or
 *   - SMI has been dead for KSZ8863_SMI_FAIL_POLLS_LIMIT consecutive polls
 *     (rate-limited to one automatic reset per
 *     KSZ8863_RECOVERY_MIN_INTERVAL_MS).
 * Returns true when a reset was executed this call. */
bool ksz8863_service(bool smi_ok);

/* Consecutive failed polls (~100 ms each) before an automatic recovery. */
#define KSZ8863_SMI_FAIL_POLLS_LIMIT      50u      /* ~5 s of dead SMI */
/* Minimum spacing between automatic recovery resets. */
#define KSZ8863_RECOVERY_MIN_INTERVAL_MS  30000u   /* 30 s */

/* Read the standard PHY ID registers (MII reg 2 / 3) from port 1 and verify
 * that the OUI MSB matches the Micrel/Microchip OUI (0x0022). Returns true
 * only when the chip answered and the ID looks valid. The raw IDs are
 * written through *id1_out / *id2_out (NULLs are tolerated). */
bool ksz8863_self_test(uint16_t *id1_out, uint16_t *id2_out);

/* Read live link status of an external port (1 or 2). */
bool ksz8863_get_link(ksz8863_port_t port, ksz8863_link_status_t *out);

/* Force a port into a fixed speed/duplex mode (auto-negotiation OFF). */
bool ksz8863_set_force_mode(ksz8863_port_t port,
                            ksz8863_speed_t speed,
                            ksz8863_duplex_t duplex);

/* Re-enable auto-negotiation on a port and trigger a restart. */
bool ksz8863_restart_autoneg(ksz8863_port_t port);

/* Power the port up (enable=true) or down (enable=false) via BMCR.
 * A powered-down port stops responding on the wire. */
bool ksz8863_port_enable(ksz8863_port_t port, bool enable);

#ifdef __cplusplus
}
#endif

#endif /* KSZ8863_H */
