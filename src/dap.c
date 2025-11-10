/**
 * @file dap.c
 * @brief Debug Access Port (DAP) implementation with caching
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#include "pico2-swd-riscv/dap.h"
#include "pico2-swd-riscv/swd.h"
#include "internal.h"
#include "pico/stdlib.h"
#include <stdio.h>

//==============================================================================
// RP2350-Specific DP_SELECT Encoding
//==============================================================================

uint32_t make_dp_select_rp2350(uint8_t apsel, uint8_t bank, bool ctrlsel) {
    // RP2350 uses non-standard encoding:
    // [15:12] = APSEL, [11:8] = 0xD, [7:4] = bank, [0] = ctrlsel
    return ((apsel & 0xF) << 12) | (0xD << 8) | ((bank & 0xF) << 4) | (ctrlsel ? 1 : 0);
}

//==============================================================================
// AP Bank Selection with Caching
//==============================================================================

static swd_error_t select_ap_bank(swd_target_t *target, uint8_t apsel, uint8_t bank) {
    // Check if already selected
    if (target->dap.current_apsel == apsel &&
        target->dap.current_bank == bank &&
        target->dap.ctrlsel == true) {
        SWD_DEBUG("AP bank already selected (APSEL=%u, bank=%u)\n", apsel, bank);
        return SWD_OK;
    }

    // Build DP_SELECT value for RP2350
    uint32_t select_val = make_dp_select_rp2350(apsel, bank, true);

    // Write to DP_SELECT
    swd_error_t err = swd_write_dp_raw(target, DP_SELECT, select_val);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to select AP bank (APSEL=%u, bank=%u)", apsel, bank);
        return err;
    }

    // Update cache
    target->dap.current_apsel = apsel;
    target->dap.current_bank = bank;
    target->dap.ctrlsel = true;
    target->dap.select_cache = select_val;

    SWD_DEBUG("Selected AP bank: APSEL=%u, bank=%u\n", apsel, bank);
    return SWD_OK;
}

//==============================================================================
// Power Management
//==============================================================================

swd_error_t dap_power_up(swd_target_t *target) {
    if (!target) {
        return SWD_ERROR_INVALID_PARAM;
    }

    if (target->dap.powered) {
        return SWD_OK;  // Already powered
    }

    SWD_INFO("Powering up debug domains...\n");

    // Clear errors first
    swd_error_t err = swd_write_dp_raw(target, DP_CTRL_STAT, 0);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to clear DP_CTRL_STAT");
        return err;
    }

    // Request power-up: CDBGPWRUPREQ | CSYSPWRUPREQ
    uint32_t ctrl_stat = (1 << 28) | (1 << 30);
    err = swd_write_dp_raw(target, DP_CTRL_STAT, ctrl_stat);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to request power-up");
        return err;
    }

    // Poll for ACK (bits 29 and 31)
    for (int i = 0; i < 10; i++) {
        uint32_t status;
        err = swd_read_dp_raw(target, DP_CTRL_STAT, &status);
        if (err != SWD_OK) {
            swd_set_error(target, err, "Failed to read power status");
            return err;
        }

        bool cdbgpwrupack = (status >> 29) & 1;
        bool csyspwrupack = (status >> 31) & 1;

        if (cdbgpwrupack && csyspwrupack) {
            SWD_INFO("Debug domains powered up\n");
            target->dap.powered = true;
            return SWD_OK;
        }

        sleep_ms(20);
    }

    swd_set_error(target, SWD_ERROR_TIMEOUT, "Power-up timeout");
    return SWD_ERROR_TIMEOUT;
}

swd_error_t dap_power_down(swd_target_t *target) {
    if (!target) {
        return SWD_ERROR_INVALID_PARAM;
    }

    if (!target->dap.powered) {
        return SWD_OK;
    }

    SWD_INFO("Powering down debug domains...\n");

    swd_error_t err = swd_write_dp_raw(target, DP_CTRL_STAT, 0);
    if (err == SWD_OK) {
        target->dap.powered = false;
    }

    return err;
}

bool dap_is_powered(const swd_target_t *target) {
    return target && target->dap.powered;
}

//==============================================================================
// Debug Port Register Access
//==============================================================================

swd_result_t dap_read_dp(swd_target_t *target, uint8_t reg) {
    swd_result_t result = {.error = SWD_ERROR_INVALID_PARAM, .value = 0};

    if (!target) {
        return result;
    }

    result.error = swd_read_dp_raw(target, reg, &result.value);
    if (result.error != SWD_OK) {
        swd_set_error(target, result.error, "DP read failed (reg=0x%02x)", reg);
    } else {
        SWD_DEBUG("DP read: reg=0x%02x, value=0x%08lx\n", reg, (unsigned long)result.value);
    }

    return result;
}

swd_error_t dap_write_dp(swd_target_t *target, uint8_t reg, uint32_t value) {
    if (!target) {
        return SWD_ERROR_INVALID_PARAM;
    }

    SWD_DEBUG("DP write: reg=0x%02x, value=0x%08lx\n", reg, (unsigned long)value);

    swd_error_t err = swd_write_dp_raw(target, reg, value);
    if (err != SWD_OK) {
        swd_set_error(target, err, "DP write failed (reg=0x%02x, value=0x%08x)", reg, value);
    }

    return err;
}

//==============================================================================
// Access Port Register Access
//==============================================================================

swd_result_t dap_read_ap(swd_target_t *target, uint8_t apsel, uint8_t reg) {
    swd_result_t result = {.error = SWD_ERROR_NOT_CONNECTED, .value = 0};

    if (!target || !target->connected) {
        return result;
    }

    // Select appropriate bank
    uint8_t bank = (reg >> 4) & 0xF;
    result.error = select_ap_bank(target, apsel, bank);
    if (result.error != SWD_OK) {
        return result;
    }

    // Read from AP
    result.error = swd_read_ap_raw(target, reg, &result.value);
    if (result.error != SWD_OK) {
        swd_set_error(target, result.error,
                     "AP read failed (apsel=%u, reg=0x%02x)", apsel, reg);
        return result;
    }

    // Read RDBUFF to get actual value
    uint32_t final_value;
    result.error = swd_read_dp_raw(target, DP_RDBUFF, &final_value);
    if (result.error == SWD_OK) {
        result.value = final_value;
        SWD_DEBUG("AP read: apsel=%u, reg=0x%02x, value=0x%08lx\n",
                  apsel, reg, (unsigned long)result.value);
    } else {
        swd_set_error(target, result.error, "RDBUFF read failed");
    }

    return result;
}

swd_error_t dap_write_ap(swd_target_t *target, uint8_t apsel, uint8_t reg, uint32_t value) {
    if (!target || !target->connected) {
        return SWD_ERROR_NOT_CONNECTED;
    }

    // Select appropriate bank
    uint8_t bank = (reg >> 4) & 0xF;
    swd_error_t err = select_ap_bank(target, apsel, bank);
    if (err != SWD_OK) {
        return err;
    }

    SWD_DEBUG("AP write: apsel=%u, reg=0x%02x, value=0x%08lx\n", apsel, reg, (unsigned long)value);

    // Write to AP
    err = swd_write_ap_raw(target, reg, value);
    if (err != SWD_OK) {
        swd_set_error(target, err,
                     "AP write failed (apsel=%u, reg=0x%02x, value=0x%08x)",
                     apsel, reg, value);
    }

    return err;
}

//==============================================================================
// Memory Access Through AP
//==============================================================================

swd_result_t dap_read_mem32(swd_target_t *target, uint32_t addr) {
    swd_result_t result = {.error = SWD_ERROR_NOT_CONNECTED, .value = 0};

    if (!target || !target->connected) {
        return result;
    }

    if (addr & 0x3) {
        result.error = SWD_ERROR_ALIGNMENT;
        swd_set_error(target, result.error, "Address 0x%08x not 4-byte aligned", addr);
        return result;
    }

    SWD_DEBUG("MEM read: addr=0x%08lx\n", (unsigned long)addr);

    // Write address to TAR
    result.error = dap_write_ap(target, AP_RISCV, AP_TAR, addr);
    if (result.error != SWD_OK) {
        return result;
    }

    // Read from DRW (triggers memory read)
    result.error = swd_read_ap_raw(target, AP_DRW, &result.value);
    if (result.error != SWD_OK) {
        swd_set_error(target, result.error, "DRW read failed");
        return result;
    }

    // Get value from RDBUFF
    uint32_t value;
    result.error = swd_read_dp_raw(target, DP_RDBUFF, &value);
    if (result.error == SWD_OK) {
        result.value = value;
        SWD_DEBUG("MEM read: addr=0x%08lx -> 0x%08lx\n", (unsigned long)addr, (unsigned long)result.value);
    }

    return result;
}

swd_error_t dap_write_mem32(swd_target_t *target, uint32_t addr, uint32_t value) {
    if (!target || !target->connected) {
        return SWD_ERROR_NOT_CONNECTED;
    }

    if (addr & 0x3) {
        swd_set_error(target, SWD_ERROR_ALIGNMENT,
                     "Address 0x%08x not 4-byte aligned", addr);
        return SWD_ERROR_ALIGNMENT;
    }

    SWD_DEBUG("MEM write: addr=0x%08lx <- 0x%08lx\n", (unsigned long)addr, (unsigned long)value);

    // Write address to TAR
    swd_error_t err = dap_write_ap(target, AP_RISCV, AP_TAR, addr);
    if (err != SWD_OK) {
        return err;
    }

    // Write value to DRW (triggers memory write)
    err = dap_write_ap(target, AP_RISCV, AP_DRW, value);
    if (err != SWD_OK) {
        swd_set_error(target, err, "DRW write failed");
        return err;
    }

    // Read RDBUFF to ensure write completes (AP writes are posted/pipelined)
    swd_result_t dummy = dap_read_dp(target, DP_RDBUFF);
    if (dummy.error != SWD_OK) {
        swd_set_error(target, dummy.error, "Failed to complete write");
        return dummy.error;
    }

    return SWD_OK;
}


//==============================================================================
// Error Management
//==============================================================================

swd_error_t dap_clear_errors(swd_target_t *target) {
    if (!target) {
        return SWD_ERROR_INVALID_PARAM;
    }

    SWD_INFO("Clearing sticky error flags\n");

    // Clear STICKYERR, WDATAERR, STICKYORUN, STICKYCMP
    uint32_t clear_bits = (1 << 5) | (1 << 7) | (1 << 1) | (1 << 4);

    swd_error_t err = swd_write_dp_raw(target, DP_CTRL_STAT, clear_bits);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to clear error flags");
    }

    return err;
}
