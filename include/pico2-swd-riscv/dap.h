/**
 * @file dap.h
 * @brief Debug Access Port (DAP) operations
 *
 * Lower-level interface for accessing Debug Port (DP) and Access Port (AP)
 * registers. Most users should use the rp2350.h interface instead.
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef PICO2_SWD_RISCV_DAP_H
#define PICO2_SWD_RISCV_DAP_H

#include "pico2-swd-riscv/types.h"

#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// DP Register Definitions
//==============================================================================

#define DP_IDCODE     0x0   ///< Identification Code Register
#define DP_CTRL_STAT  0x4   ///< Control/Status Register
#define DP_SELECT     0x8   ///< AP Select Register
#define DP_RDBUFF     0xC   ///< Read Buffer

//==============================================================================
// AP Register Definitions
//==============================================================================

#define AP_CSW  0x00   ///< Control/Status Word
#define AP_TAR  0x04   ///< Transfer Address Register
#define AP_DRW  0x0C   ///< Data Read/Write Register
#define AP_IDR  0xFC   ///< Identification Register

//==============================================================================
// RP2350 AP Selection
//==============================================================================

#define AP_ROM_TABLE    0x0   ///< ROM Table
#define AP_ARM_CORE0    0x2   ///< ARM Core 0 AHB-AP
#define AP_ARM_CORE1    0x4   ///< ARM Core 1 AHB-AP
#define AP_RISCV        0xA   ///< RISC-V APB-AP
#define AP_RP_SPECIFIC  0x8   ///< RP-AP (Raspberry Pi specific)

//==============================================================================
// Power Management
//==============================================================================

/**
 * @brief Power up debug domains
 *
 * Powers up the debug system and system power domains, waits for
 * acknowledgment. Must be called after connection.
 *
 * @param target Target to power up
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t dap_power_up(swd_target_t *target);

/**
 * @brief Power down debug domains
 *
 * Powers down debug domains. Called automatically on disconnect.
 *
 * @param target Target to power down
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t dap_power_down(swd_target_t *target);

/**
 * @brief Check if debug domains are powered
 *
 * @param target Target to check
 * @return true if powered up, false otherwise
 */
bool dap_is_powered(const swd_target_t *target);

//==============================================================================
// Register Access
//==============================================================================

/**
 * @brief Read Debug Port register
 *
 * @param target Target to read from
 * @param reg Register address (DP_IDCODE, DP_CTRL_STAT, etc.)
 * @return Result containing register value or error
 */
swd_result_t dap_read_dp(swd_target_t *target, uint8_t reg);

/**
 * @brief Write Debug Port register
 *
 * @param target Target to write to
 * @param reg Register address
 * @param value Value to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t dap_write_dp(swd_target_t *target, uint8_t reg, uint32_t value);

/**
 * @brief Read Access Port register
 *
 * Automatically handles AP bank selection for RP2350.
 *
 * @param target Target to read from
 * @param apsel AP selection (AP_RISCV, etc.)
 * @param reg Register address
 * @return Result containing register value or error
 */
swd_result_t dap_read_ap(swd_target_t *target, uint8_t apsel, uint8_t reg);

/**
 * @brief Write Access Port register
 *
 * Automatically handles AP bank selection for RP2350.
 *
 * @param target Target to write to
 * @param apsel AP selection
 * @param reg Register address
 * @param value Value to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t dap_write_ap(swd_target_t *target, uint8_t apsel, uint8_t reg, uint32_t value);

//==============================================================================
// Memory Access Through AP
//==============================================================================

/**
 * @brief Read 32-bit value using MEM-AP protocol (TAR/DRW/RDBUFF)
 *
 * Low-level function that uses the MEM-AP TAR (Transfer Address Register)
 * and DRW (Data Read/Write) mechanism with proper RDBUFF handling for
 * pipelined reads. Primarily used for accessing Debug Module registers.
 *
 * For general target memory access, use rp2350_read_mem32() instead,
 * which uses System Bus Access (SBA).
 *
 * @param target Target to read from
 * @param addr Address (must be 4-byte aligned)
 * @return Result containing value or error
 */
swd_result_t dap_read_mem32(swd_target_t *target, uint32_t addr);

/**
 * @brief Write 32-bit value using MEM-AP protocol (TAR/DRW/RDBUFF)
 *
 * Low-level function that uses the MEM-AP TAR (Transfer Address Register)
 * and DRW (Data Read/Write) mechanism with proper RDBUFF handling to ensure
 * write completion. Primarily used for accessing Debug Module registers.
 *
 * For general target memory access, use rp2350_write_mem32() instead,
 * which uses System Bus Access (SBA).
 *
 * @param target Target to write to
 * @param addr Address (must be 4-byte aligned)
 * @param value Value to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t dap_write_mem32(swd_target_t *target, uint32_t addr, uint32_t value);

//==============================================================================
// Error Management
//==============================================================================

/**
 * @brief Clear sticky error flags
 *
 * Clears error flags in DP_CTRL_STAT register after a fault.
 *
 * @param target Target to clear errors on
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t dap_clear_errors(swd_target_t *target);

#ifdef __cplusplus
}
#endif

#endif // PICO2_SWD_RISCV_DAP_H
