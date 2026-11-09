/* ---------------------------------------------------------------------------
 * Driver for Microchip/Micrel KSZ8863MLLI Ethernet switch (SMI/MIIM only).
 * See ksz8863.h for the public API and integration constraints.
 * ------------------------------------------------------------------------- */
#include "ksz8863.h"

#include "main.h"
#include "stm32f4xx_hal.h"

/* heth is defined in LWIP/Target/ethernetif.c. We share it so that PHY
 * register access goes through the same MAC peripheral that LwIP drives. */
extern ETH_HandleTypeDef heth;

/* OUI MSB used by all Micrel / Microchip KSZ-series PHYs. The exact value of
 * PHYID2 (model + revision nibble) varies between silicon revisions of the
 * KSZ8863, so we don't enforce it — checking PHYID1 is enough for the
 * "is the chip there and answering" self-test. */
#define KSZ8863_PHYID1_MICREL       0x0022u

/* Standard MII (clause 22) register addresses — same for any 802.3 PHY. */
#define MII_REG_BMCR                0x00u
#define MII_REG_BMSR                0x01u
#define MII_REG_PHYID1              0x02u
#define MII_REG_PHYID2              0x03u
#define MII_REG_ANLPAR              0x05u

/* BMCR bit masks. */
#define BMCR_RESET                  0x8000u
#define BMCR_SPEED_SELECT_100       0x2000u
#define BMCR_AUTONEG_ENABLE         0x1000u
#define BMCR_POWER_DOWN             0x0800u
#define BMCR_ISOLATE                0x0400u
#define BMCR_RESTART_AUTONEG        0x0200u
#define BMCR_DUPLEX_FULL            0x0100u

/* BMSR bit masks. */
#define BMSR_AUTONEG_COMPLETE       0x0020u
#define BMSR_LINK_UP                0x0004u

/* ANLPAR bit masks (link partner advertisement). */
#define ANLPAR_100BASE_TX_FULL      0x0100u
#define ANLPAR_100BASE_TX_HALF      0x0080u
#define ANLPAR_10BASE_T_FULL        0x0040u
#define ANLPAR_10BASE_T_HALF        0x0020u

static bool ksz_phy_read(uint8_t phy_addr, uint8_t reg, uint16_t *out)
{
    uint32_t v = 0u;
    if (HAL_ETH_ReadPHYRegister(&heth, phy_addr, reg, &v) != HAL_OK) {
        return false;
    }
    *out = (uint16_t)(v & 0xFFFFu);
    return true;
}

static bool ksz_phy_write(uint8_t phy_addr, uint8_t reg, uint16_t val)
{
    return HAL_ETH_WritePHYRegister(&heth, phy_addr, reg, (uint32_t)val) == HAL_OK;
}

static bool port_is_valid(ksz8863_port_t port)
{
    return port == KSZ8863_PORT1 || port == KSZ8863_PORT2;
}

void ksz8863_hw_reset(void)
{
    /* Drive ETHRST low. */
    HAL_GPIO_WritePin(ETHRST_GPIO_Port, ETHRST_Pin, GPIO_PIN_RESET);
    /* KSZ8863 reset pulse min is 10 us; pad to be safe. */
    HAL_Delay(10);

    /* Release reset and wait for the chip to come up. The datasheet quotes
     * ~10 ms from RESETn rising edge to SMI-ready. We give it 20 ms. */
    HAL_GPIO_WritePin(ETHRST_GPIO_Port, ETHRST_Pin, GPIO_PIN_SET);
    HAL_Delay(20);
}

void ksz8863_boot_init(void)
{
    /* Cold boot = power-on / brown-out: the chip has just been powered, no
     * pass-through traffic exists yet, so a defensive hardware reset costs
     * nothing and guarantees a clean start even with a slow supply ramp.
     *
     * Warm reboot (software reset, IWDG, NRST pin): the switch is alive and
     * forwarding between ports 1 and 2 — do NOT touch it, the pass-through
     * traffic must survive the MCU restart. ETHRST is already high (board
     * pull-up during the MCU reset window, then MX_GPIO_Init()). */
    const uint32_t csr = RCC->CSR;
    const bool cold_boot = (csr & (RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)) != 0u;

    if (cold_boot) {
        ksz8863_hw_reset();
    }

    /* Consume the flags so the next reboot sees only its own cause. */
    __HAL_RCC_CLEAR_RESET_FLAGS();
}

void ksz8863_ensure_rmii_port3(void)
{
    /* Global Control 4 (SMI reg 6 at PHY addr 0), bit 6 = port 3 RMII mode.
     * Normally latched from the strap pins at chip reset; this is a
     * defensive, idempotent re-check. */
    uint32_t tmp = 0;
    if (HAL_ETH_ReadPHYRegister(&heth, 0, 6, &tmp) != HAL_OK) {
        return;
    }
    if ((tmp & 0x0040u) == 0u) {
        tmp |= 0x0040u;
        HAL_ETH_WritePHYRegister(&heth, 0, 6, tmp);
    }
}

/* ---------------------------------------------------------------------------
 * Health monitoring / recovery
 * ------------------------------------------------------------------------- */
static volatile uint8_t s_recovery_request  = 0u;
static uint16_t         s_smi_fail_polls    = 0u;
static uint32_t         s_last_recovery_tick = 0u;

void ksz8863_request_recovery(void)
{
    s_recovery_request = 1u;
}

bool ksz8863_service(bool smi_ok)
{
    if (smi_ok) {
        s_smi_fail_polls = 0u;
    } else if (s_smi_fail_polls < 0xFFFFu) {
        s_smi_fail_polls++;
    }

    /* Operator-requested reset executes unconditionally; the automatic
     * SMI-dead recovery is rate-limited so a persistently broken chip does
     * not turn into a reset storm on the pass-through ports. */
    bool reset_due = (s_recovery_request != 0u);
    if (!reset_due && s_smi_fail_polls >= KSZ8863_SMI_FAIL_POLLS_LIMIT) {
        const uint32_t now = HAL_GetTick();
        if (s_last_recovery_tick == 0u ||
            (now - s_last_recovery_tick) >= KSZ8863_RECOVERY_MIN_INTERVAL_MS) {
            reset_due = true;
        }
    }
    if (!reset_due) {
        return false;
    }

    s_recovery_request   = 0u;
    s_smi_fail_polls     = 0u;
    s_last_recovery_tick = HAL_GetTick();
    if (s_last_recovery_tick == 0u) {
        s_last_recovery_tick = 1u;
    }

    ksz8863_hw_reset();
    ksz8863_ensure_rmii_port3();
    return true;
}

bool ksz8863_self_test(uint16_t *id1_out, uint16_t *id2_out)
{
    uint16_t id1 = 0u;
    uint16_t id2 = 0u;

    if (!ksz_phy_read((uint8_t)KSZ8863_PORT1, MII_REG_PHYID1, &id1)) {
        return false;
    }
    if (!ksz_phy_read((uint8_t)KSZ8863_PORT1, MII_REG_PHYID2, &id2)) {
        return false;
    }
    if (id1_out != NULL) { *id1_out = id1; }
    if (id2_out != NULL) { *id2_out = id2; }

    /* All 1s = chip not responding (MDIO bus pulled high by the resistors).
     * All 0s = chip held in reset / not powered. Both cases must fail. */
    if (id1 == 0x0000u || id1 == 0xFFFFu) {
        return false;
    }
    return id1 == KSZ8863_PHYID1_MICREL;
}

bool ksz8863_get_link(ksz8863_port_t port, ksz8863_link_status_t *out)
{
    if (!port_is_valid(port) || out == NULL) {
        return false;
    }

    uint16_t bmcr   = 0u;
    uint16_t bmsr   = 0u;
    uint16_t anlpar = 0u;
    if (!ksz_phy_read((uint8_t)port, MII_REG_BMCR,   &bmcr))   { return false; }
    if (!ksz_phy_read((uint8_t)port, MII_REG_BMSR,   &bmsr))   { return false; }
    if (!ksz_phy_read((uint8_t)port, MII_REG_ANLPAR, &anlpar)) { return false; }

    out->link_up      = (bmsr & BMSR_LINK_UP) != 0u;
    out->autoneg_done = (bmsr & BMSR_AUTONEG_COMPLETE) != 0u;

    if ((bmcr & BMCR_AUTONEG_ENABLE) != 0u) {
        /* Auto-neg mode: highest common ability advertised by the partner
         * wins. Walk in priority order 100/full > 100/half > 10/full > 10/half. */
        if ((anlpar & ANLPAR_100BASE_TX_FULL) != 0u) {
            out->speed  = KSZ8863_SPEED_100;
            out->duplex = KSZ8863_DUPLEX_FULL;
        } else if ((anlpar & ANLPAR_100BASE_TX_HALF) != 0u) {
            out->speed  = KSZ8863_SPEED_100;
            out->duplex = KSZ8863_DUPLEX_HALF;
        } else if ((anlpar & ANLPAR_10BASE_T_FULL) != 0u) {
            out->speed  = KSZ8863_SPEED_10;
            out->duplex = KSZ8863_DUPLEX_FULL;
        } else {
            out->speed  = KSZ8863_SPEED_10;
            out->duplex = KSZ8863_DUPLEX_HALF;
        }
    } else {
        /* Forced mode: use what BMCR was programmed to. */
        out->speed  = ((bmcr & BMCR_SPEED_SELECT_100) != 0u)
                          ? KSZ8863_SPEED_100 : KSZ8863_SPEED_10;
        out->duplex = ((bmcr & BMCR_DUPLEX_FULL) != 0u)
                          ? KSZ8863_DUPLEX_FULL : KSZ8863_DUPLEX_HALF;
    }
    return true;
}

bool ksz8863_set_force_mode(ksz8863_port_t port,
                            ksz8863_speed_t speed,
                            ksz8863_duplex_t duplex)
{
    if (!port_is_valid(port)) {
        return false;
    }
    uint16_t bmcr = 0u;
    if (speed  == KSZ8863_SPEED_100)   { bmcr |= BMCR_SPEED_SELECT_100; }
    if (duplex == KSZ8863_DUPLEX_FULL) { bmcr |= BMCR_DUPLEX_FULL; }
    /* AN disabled, no power-down, no isolate. */
    return ksz_phy_write((uint8_t)port, MII_REG_BMCR, bmcr);
}

bool ksz8863_restart_autoneg(ksz8863_port_t port)
{
    if (!port_is_valid(port)) {
        return false;
    }
    uint16_t bmcr = 0u;
    if (!ksz_phy_read((uint8_t)port, MII_REG_BMCR, &bmcr)) {
        return false;
    }
    bmcr |= (uint16_t)(BMCR_AUTONEG_ENABLE | BMCR_RESTART_AUTONEG);
    bmcr &= (uint16_t)~(BMCR_POWER_DOWN | BMCR_ISOLATE);
    return ksz_phy_write((uint8_t)port, MII_REG_BMCR, bmcr);
}

bool ksz8863_port_enable(ksz8863_port_t port, bool enable)
{
    if (!port_is_valid(port)) {
        return false;
    }
    uint16_t bmcr = 0u;
    if (!ksz_phy_read((uint8_t)port, MII_REG_BMCR, &bmcr)) {
        return false;
    }
    if (enable) {
        bmcr &= (uint16_t)~BMCR_POWER_DOWN;
    } else {
        bmcr |= (uint16_t)BMCR_POWER_DOWN;
    }
    return ksz_phy_write((uint8_t)port, MII_REG_BMCR, bmcr);
}
