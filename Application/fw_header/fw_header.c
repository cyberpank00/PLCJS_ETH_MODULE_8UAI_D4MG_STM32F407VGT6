/**
 * @file  fw_header.c
 * @brief Firmware image header instance placed at APP_FLASH_BASE + 0x200.
 *
 * The linker script pads .isr_vector to 0x200 bytes and then keeps .fw_header
 * immediately after, so g_fw_header lands at exactly the expected offset.
 *
 * All fields are set at compile time by CMake -D flags.
 * CRC32 and image size are NOT stored here — they travel in OTA metadata.
 */

#include "fw_header.h"

const fw_header_t g_fw_header
    __attribute__((section(".fw_header")))
    __attribute__((used)) =
{
    .magic               = FW_IMAGE_MAGIC,
    .product_id          = FW_PRODUCT_ID,
    .hw_revision         = (uint16_t)FW_HW_REVISION,
    .reserved0           = 0u,
    .fw_version          = FW_VERSION_VALUE,
    .vector_table_offset = 0u,
    .reserved1           = {0u, 0u},
};
