/**
  ******************************************************************************
  * @file    ads8688.c
  * @brief   ADS8688 blocking start-up driver (see ads8688.h).
  ******************************************************************************
  */
#include "ads8688.h"

#include "board_io.h"
#include "cmsis_os.h"

bool ads8688_set_range(uint8_t n, uint8_t range_code)
{
    (void)board_adc_frame(ADS_REG_WRITE(ADS_REG_RANGE(n), range_code));
    const uint32_t f = board_adc_frame(ADS_REG_READ(ADS_REG_RANGE(n)));
    return ADS_REG_DATA(f) == range_code;
}

bool ads8688_init(uint8_t range_code)
{
    for (uint8_t attempt = 0u; attempt < 3u; attempt++) {
        (void)board_adc_frame(ADS_CMD_RST);
        osDelay(ADS_RST_WAIT_MS + 1u);   /* +1: osDelay may return a tick early */

        bool ok = true;
        for (uint8_t n = 0u; n < 8u; n++) {
            ok = ads8688_set_range(n, range_code) && ok;
        }
        if (ok) { return true; }
    }
    return false;
}
