/**
 * @file rp2350.c
 * @brief RP2350 RISC-V Debug Module implementation
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#include "pico2-swd-riscv/rp2350.h"
#include "pico2-swd-riscv/dap.h"
#include "pico2-swd-riscv/swd.h"
#include "internal.h"
#include "pico/stdlib.h"
#include <stdio.h>

//==============================================================================
// Debug Module Register Addresses
//==============================================================================

#define DM_DMCONTROL   (0x10 * 4)
#define DM_DMSTATUS    (0x11 * 4)
#define DM_ABSTRACTCS  (0x16 * 4)
#define DM_COMMAND     (0x17 * 4)
#define DM_DATA0       (0x04 * 4)
#define DM_PROGBUF0    (0x20 * 4)
#define DM_PROGBUF1    (0x21 * 4)
#define DM_SBCS        (0x38 * 4)
#define DM_SBADDRESS0  (0x39 * 4)
#define DM_SBDATA0     (0x3C * 4)

//==============================================================================
// Forward Declarations
//==============================================================================

static swd_error_t rp2350_init_sba(swd_target_t *target);

//==============================================================================
// Helper: Hart Validation and Selection
//==============================================================================

/**
 * @brief Validate hart ID
 *
 * @param hart_id Hart ID to validate
 * @return true if valid (0 or 1 for RP2350), false otherwise
 */
static inline bool validate_hart_id(uint8_t hart_id) {
    return hart_id < RP2350_NUM_HARTS;
}

/**
 * @brief Build DMCONTROL value with hart selection
 *
 * @param hart_id Hart ID to select (0 or 1)
 * @param haltreq Set haltreq bit
 * @param resumereq Set resumereq bit
 * @param ndmreset Set ndmreset bit
 * @return DMCONTROL register value
 */
static inline uint32_t make_dmcontrol(uint8_t hart_id, bool haltreq,
                                       bool resumereq, bool ndmreset) {
    uint32_t dmcontrol = (1 << 0);  // dmactive = 1
    dmcontrol |= ((uint32_t)hart_id << 16);  // hartsello[9:0] at bits 25:16
    if (haltreq) dmcontrol |= (1 << 31);
    if (resumereq) dmcontrol |= (1 << 30);
    if (ndmreset) dmcontrol |= (1 << 1);
    return dmcontrol;
}

//==============================================================================
// Helper: Wait for Abstract Command Completion
//==============================================================================

static swd_error_t wait_abstract_command(swd_target_t *target) {
    for (int i = 0; i < 100; i++) {
        swd_result_t result = dap_read_mem32(target, DM_ABSTRACTCS);
        if (result.error != SWD_OK) {
            return result.error;
        }

        uint32_t abstractcs = result.value;
        bool busy = (abstractcs >> 12) & 1;
        uint8_t cmderr = (abstractcs >> 8) & 0x7;

        if (!busy) {
            if (cmderr != 0) {
                // Clear error
                dap_write_mem32(target, DM_ABSTRACTCS, 0x00000700);
                swd_set_error(target, SWD_ERROR_ABSTRACT_CMD,
                             "Abstract command error: %u", cmderr);
                return SWD_ERROR_ABSTRACT_CMD;
            }
            return SWD_OK;
        }

        sleep_us(100);
    }

    swd_set_error(target, SWD_ERROR_TIMEOUT, "Abstract command timeout");
    return SWD_ERROR_TIMEOUT;
}

//==============================================================================
// Helper: Poll DMSTATUS
//==============================================================================

/**
 * @brief Poll DMSTATUS for halt/running state
 *
 * DMSTATUS.allhalted/allrunning reflects the state of the currently selected hart.
 *
 * @param target Target to poll
 * @param hart_id Hart ID being polled
 * @param wait_for_halted true to wait for halt, false to wait for running
 * @return SWD_OK on success, error otherwise
 */
static swd_error_t poll_dmstatus_halted(swd_target_t *target, uint8_t hart_id,
                                         bool wait_for_halted) {
    hart_state_t *hart = &target->rp2350.harts[hart_id];

    for (int i = 0; i < 10; i++) {
        swd_result_t result = dap_read_mem32(target, DM_DMSTATUS);
        if (result.error != SWD_OK) {
            return result.error;
        }

        bool allhalted = (result.value >> 9) & 1;
        bool allrunning = (result.value >> 11) & 1;

        if (wait_for_halted && allhalted) {
            hart->halted = true;
            hart->halt_state_known = true;
            return SWD_OK;
        }

        if (!wait_for_halted && allrunning) {
            hart->halted = false;
            hart->halt_state_known = true;
            return SWD_OK;
        }

        sleep_ms(10);
    }

    return SWD_ERROR_TIMEOUT;
}

//==============================================================================
// Debug Module Initialization
//==============================================================================

swd_error_t rp2350_init(swd_target_t *target) {
    if (!target || !target->connected) {
        return SWD_ERROR_NOT_CONNECTED;
    }

    if (target->rp2350.initialized) {
        return SWD_OK;  // Already initialized
    }

    SWD_INFO("Initializing RP2350 Debug Module...\n");

    // Select RISC-V APB-AP, Bank 0
    uint32_t sel_bank0 = make_dp_select_rp2350(AP_RISCV, 0, true);
    swd_error_t err = dap_write_dp(target, DP_SELECT, sel_bank0);
    if (err != SWD_OK) {
        return err;
    }

    // Configure CSW for 32-bit word access
    uint32_t csw = 0xA2000002;  // Standard CSW value
    err = dap_write_ap(target, AP_RISCV, AP_CSW, csw);
    if (err != SWD_OK) {
        return err;
    }

    // Point TAR at DMCONTROL
    err = dap_write_ap(target, AP_RISCV, AP_TAR, DM_DMCONTROL);
    if (err != SWD_OK) {
        return err;
    }

    // Switch to Bank 1 for DM control
    uint32_t sel_bank1 = make_dp_select_rp2350(AP_RISCV, 1, true);
    err = dap_write_dp(target, DP_SELECT, sel_bank1);
    if (err != SWD_OK) {
        return err;
    }

    // DM Activation Sequence
    SWD_DEBUG("Performing DM activation handshake...\n");

    // Reset
    err = dap_write_ap(target, AP_RISCV, AP_CSW, 0x00000000);
    if (err != SWD_OK) return err;
    dap_read_dp(target, DP_RDBUFF);
    sleep_ms(50);

    // Activate
    err = dap_write_ap(target, AP_RISCV, AP_CSW, 0x00000001);
    if (err != SWD_OK) return err;
    dap_read_dp(target, DP_RDBUFF);
    sleep_ms(50);

    // Configure
    err = dap_write_ap(target, AP_RISCV, AP_CSW, 0x07FFFFC1);
    if (err != SWD_OK) return err;
    dap_read_dp(target, DP_RDBUFF);
    sleep_ms(50);

    // Verify DM is responding
    swd_read_ap_raw(target, AP_CSW, NULL);
    swd_result_t status_result = dap_read_dp(target, DP_RDBUFF);
    if (status_result.error != SWD_OK) {
        swd_set_error(target, status_result.error, "Failed to read DM status");
        return status_result.error;
    }

    if (status_result.value != 0x04010001) {
        swd_set_error(target, SWD_ERROR_INVALID_STATE,
                     "Unexpected DM status: 0x%08x (expected 0x04010001)",
                     status_result.value);
        return SWD_ERROR_INVALID_STATE;
    }

    // Switch back to Bank 0
    err = dap_write_dp(target, DP_SELECT, sel_bank0);
    if (err != SWD_OK) {
        return err;
    }

    SWD_INFO("Debug Module initialized successfully\n");

    target->rp2350.initialized = true;

    // Initialize per-hart state
    for (uint8_t i = 0; i < RP2350_NUM_HARTS; i++) {
        target->rp2350.harts[i].halt_state_known = false;
        target->rp2350.harts[i].halted = false;
        target->rp2350.harts[i].cache_valid = false;
    }

    // Initialize SBA
    rp2350_init_sba(target);

    return SWD_OK;
}

bool rp2350_is_initialized(const swd_target_t *target) {
    return target && target->rp2350.initialized;
}

//==============================================================================
// Program Buffer Execution
//==============================================================================

/**
 * @brief Execute instructions in the program buffer
 *
 * Low-level function to execute RISC-V instructions in the program buffer.
 * This function does NOT use abstract commands - it directly writes to the
 * program buffer and triggers execution.
 *
 * @param target Target to operate on
 * @param hart_id Hart ID to execute on
 * @param instructions Array of RISC-V instructions (up to 16)
 * @param count Number of instructions
 * @return SWD_OK on success, error otherwise
 */
static swd_error_t execute_progbuf_simple(swd_target_t *target, uint8_t hart_id,
                                           const uint32_t *instructions, uint8_t count) {
    if (!instructions || count == 0 || count > 16) {
        return SWD_ERROR_INVALID_PARAM;
    }

    // Select hart
    uint32_t dmcontrol = make_dmcontrol(hart_id, false, false, false);
    swd_error_t err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Write instructions to program buffer
    for (uint8_t i = 0; i < count; i++) {
        err = dap_write_mem32(target, DM_PROGBUF0 + (i * 4), instructions[i]);
        if (err != SWD_OK) {
            return err;
        }
    }

    // Execute progbuf without abstract command (just postexec with no transfer)
    uint32_t command = (1 << 18);  // postexec only, no register transfer
    err = dap_write_mem32(target, DM_COMMAND, command);
    if (err != SWD_OK) {
        return err;
    }

    return wait_abstract_command(target);
}

/**
 * @brief Public API for program buffer execution
 */
swd_error_t rp2350_execute_progbuf(swd_target_t *target, uint8_t hart_id,
                                    const uint32_t *instructions, uint8_t count) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    return execute_progbuf_simple(target, hart_id, instructions, count);
}

//==============================================================================
// Hart Control
//==============================================================================

swd_error_t rp2350_halt(swd_target_t *target, uint8_t hart_id) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    // Check if already halted
    if (hart->halt_state_known && hart->halted) {
        SWD_DEBUG("Hart %u already halted\n", hart_id);
        return SWD_ERROR_ALREADY_HALTED;
    }

    SWD_INFO("Halting hart %u...\n", hart_id);

    // Write DMCONTROL with hartsel, haltreq=1, dmactive=1
    uint32_t dmcontrol = make_dmcontrol(hart_id, true, false, false);
    swd_error_t err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Wait for halt
    err = poll_dmstatus_halted(target, hart_id, true);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to halt hart %u", hart_id);
        return err;
    }

    hart->halted = true;
    hart->halt_state_known = true;

    // Invalidate register cache
    hart->cache_valid = false;

    SWD_INFO("Hart %u halted\n", hart_id);
    return SWD_OK;
}

swd_error_t rp2350_resume(swd_target_t *target, uint8_t hart_id) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    // No-op if already running
    if (hart->halt_state_known && !hart->halted) {
        SWD_DEBUG("Hart %u already running\n", hart_id);
        return SWD_OK;
    }

    SWD_INFO("Resuming hart %u...\n", hart_id);

    // Write DMCONTROL with hartsel, resumereq=1, dmactive=1
    uint32_t dmcontrol = make_dmcontrol(hart_id, false, true, false);
    swd_error_t err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Wait for resume
    err = poll_dmstatus_halted(target, hart_id, false);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to resume hart %u", hart_id);
        return err;
    }

    hart->halted = false;
    hart->halt_state_known = true;

    // Invalidate cache (hart is now running)
    hart->cache_valid = false;

    SWD_INFO("Hart %u resumed\n", hart_id);
    return SWD_OK;
}

// Helper: Read DCSR via generic CSR function (which uses progbuf on RP2350)
static swd_result_t read_dcsr(swd_target_t *target, uint8_t hart_id) {
    return rp2350_read_csr(target, hart_id, 0x7b0);  // DCSR = 0x7b0
}

// Helper: Write DCSR via generic CSR function (which uses progbuf on RP2350)
static swd_error_t write_dcsr(swd_target_t *target, uint8_t hart_id, uint32_t value) {
    return rp2350_write_csr(target, hart_id, 0x7b0, value);  // DCSR = 0x7b0
}

swd_error_t rp2350_step(swd_target_t *target, uint8_t hart_id) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    if (!hart->halted) {
        return SWD_ERROR_NOT_HALTED;
    }

    SWD_INFO("Single-stepping hart %u...\n", hart_id);

    // Read current DCSR value via program buffer
    swd_result_t dcsr_result = read_dcsr(target, hart_id);
    if (dcsr_result.error != SWD_OK) {
        swd_set_error(target, dcsr_result.error, "Failed to read DCSR");
        return dcsr_result.error;
    }

    // Set step bit (bit 2), preserve other fields
    uint32_t dcsr_stepped = dcsr_result.value | (1 << 2);

    // Write modified DCSR via program buffer
    swd_error_t err = write_dcsr(target, hart_id, dcsr_stepped);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to write DCSR");
        return err;
    }

    // Resume hart directly (will execute ONE instruction, then halt due to step bit)
    // Write DMCONTROL with hartsel and dmactive
    uint32_t dmcontrol = make_dmcontrol(hart_id, false, false, false);
    err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Set resumereq
    dmcontrol = make_dmcontrol(hart_id, false, true, false);
    err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Update state
    hart->halted = false;
    hart->halt_state_known = true;

    // Wait for automatic halt after single instruction
    err = poll_dmstatus_halted(target, hart_id, true);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Step did not halt");
        return err;
    }

    hart->halted = true;
    hart->halt_state_known = true;
    hart->cache_valid = false;

    // Clear step bit for normal halted behavior
    err = write_dcsr(target, hart_id, dcsr_result.value);
    if (err != SWD_OK) {
        swd_set_error(target, err, "Failed to clear step bit");
    }

    SWD_INFO("Step completed\n");
    return err;
}

swd_error_t rp2350_reset(swd_target_t *target, uint8_t hart_id, bool halt_on_reset) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    SWD_INFO("Resetting hart %u (halt=%d)...\n", hart_id, halt_on_reset);

    // Step 1: Assert ndmreset with dmactive and hartsel
    uint32_t dmcontrol = make_dmcontrol(hart_id, halt_on_reset, false, true);

    swd_error_t err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Step 2: Hold reset for a few milliseconds
    sleep_ms(10);

    // Step 3: Deassert ndmreset
    dmcontrol = make_dmcontrol(hart_id, halt_on_reset, false, false);

    err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Step 4: Wait for hart to come out of reset
    sleep_ms(50);

    // Step 5: If halt_on_reset, wait for halt
    if (halt_on_reset) {
        err = poll_dmstatus_halted(target, hart_id, true);
        if (err != SWD_OK) {
            swd_set_error(target, err, "Failed to halt after reset");
            return err;
        }
        hart->halted = true;
        hart->halt_state_known = true;
        SWD_INFO("Hart %u reset and halted\n", hart_id);
    } else {
        hart->halted = false;
        hart->halt_state_known = true;
        SWD_INFO("Hart %u reset and running\n", hart_id);
    }

    // Invalidate caches
    hart->cache_valid = false;

    return SWD_OK;
}

bool rp2350_is_halted(const swd_target_t *target, uint8_t hart_id) {
    if (!target || !target->rp2350.initialized) {
        return false;
    }

    if (!validate_hart_id(hart_id)) {
        return false;
    }

    hart_state_t *hart = &((swd_target_t*)target)->rp2350.harts[hart_id];

    if (hart->halt_state_known) {
        return hart->halted;
    }

    // Query from hardware - need to select the hart first
    uint32_t dmcontrol = make_dmcontrol(hart_id, false, false, false);
    swd_error_t err = dap_write_mem32((swd_target_t*)target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return false;
    }

    swd_result_t result = dap_read_mem32((swd_target_t*)target, DM_DMSTATUS);
    if (result.error == SWD_OK) {
        bool halted = (result.value >> 9) & 1;
        hart->halted = halted;
        hart->halt_state_known = true;
        return halted;
    }

    return false;
}

//==============================================================================
// Register Access
//==============================================================================

swd_result_t rp2350_read_reg(swd_target_t *target, uint8_t hart_id, uint8_t reg_num) {
    swd_result_t result = {.error = SWD_ERROR_NOT_INITIALIZED, .value = 0};

    if (!target || !target->rp2350.initialized) {
        return result;
    }

    if (!validate_hart_id(hart_id)) {
        result.error = SWD_ERROR_INVALID_PARAM;
        swd_set_error(target, result.error, "Invalid hart_id: %u", hart_id);
        return result;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    if (!hart->halted) {
        result.error = SWD_ERROR_NOT_HALTED;
        swd_set_error(target, result.error, "Hart %u must be halted to read registers", hart_id);
        return result;
    }

    if (reg_num >= 32) {
        result.error = SWD_ERROR_INVALID_PARAM;
        swd_set_error(target, result.error, "Invalid register number: %u", reg_num);
        return result;
    }

    // Check cache
    if (target->rp2350.cache_enabled && hart->cache_valid) {
        result.value = hart->cached_gprs[reg_num];
        result.error = SWD_OK;
        SWD_DEBUG("Read cached hart%u x%u = 0x%08lx\n", hart_id, (unsigned)reg_num, (unsigned long)result.value);
        return result;
    }

    SWD_DEBUG("Reading hart%u x%u...\n", hart_id, reg_num);

    // Select hart
    uint32_t dmcontrol = make_dmcontrol(hart_id, false, false, false);
    result.error = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (result.error != SWD_OK) {
        return result;
    }

    // Build abstract command: read GPR
    uint32_t command = 0;
    command |= (0x1000 + reg_num) << 0;   // regno
    command |= (1 << 17);                  // transfer
    command |= (2 << 20);                  // aarsize=32-bit

    result.error = dap_write_mem32(target, DM_COMMAND, command);
    if (result.error != SWD_OK) {
        return result;
    }

    result.error = wait_abstract_command(target);
    if (result.error != SWD_OK) {
        return result;
    }

    // Read value from DATA0
    result = dap_read_mem32(target, DM_DATA0);
    if (result.error == SWD_OK) {
        // Update cache
        if (target->rp2350.cache_enabled) {
            hart->cached_gprs[reg_num] = result.value;
        }
        SWD_INFO("hart%u x%u = 0x%08lx\n", hart_id, (unsigned)reg_num, (unsigned long)result.value);
    }

    return result;
}

swd_error_t rp2350_write_reg(swd_target_t *target, uint8_t hart_id, uint8_t reg_num, uint32_t value) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    if (!hart->halted) {
        swd_set_error(target, SWD_ERROR_NOT_HALTED,
                     "Hart %u must be halted to write registers", hart_id);
        return SWD_ERROR_NOT_HALTED;
    }

    if (reg_num >= 32) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM,
                     "Invalid register number: %u", reg_num);
        return SWD_ERROR_INVALID_PARAM;
    }

    SWD_INFO("Writing hart%u x%u = 0x%08lx\n", hart_id, (unsigned)reg_num, (unsigned long)value);

    // Select hart
    uint32_t dmcontrol = make_dmcontrol(hart_id, false, false, false);
    swd_error_t err = dap_write_mem32(target, DM_DMCONTROL, dmcontrol);
    if (err != SWD_OK) {
        return err;
    }

    // Write value to DATA0
    err = dap_write_mem32(target, DM_DATA0, value);
    if (err != SWD_OK) {
        return err;
    }

    // Build abstract command: write GPR
    uint32_t command = 0;
    command |= (0x1000 + reg_num) << 0;   // regno
    command |= (1 << 16);                  // write
    command |= (1 << 17);                  // transfer
    command |= (2 << 20);                  // aarsize=32-bit

    err = dap_write_mem32(target, DM_COMMAND, command);
    if (err != SWD_OK) {
        return err;
    }

    err = wait_abstract_command(target);
    if (err == SWD_OK) {
        // Update cache
        if (target->rp2350.cache_enabled) {
            hart->cached_gprs[reg_num] = value;
        }
    }

    return err;
}

swd_error_t rp2350_read_all_regs(swd_target_t *target, uint8_t hart_id, uint32_t regs[32]) {
    if (!target || !regs) {
        return SWD_ERROR_INVALID_PARAM;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    SWD_INFO("Reading all 32 registers from hart%u...\n", hart_id);

    for (uint8_t i = 0; i < 32; i++) {
        swd_result_t result = rp2350_read_reg(target, hart_id, i);
        if (result.error != SWD_OK) {
            return result.error;
        }
        regs[i] = result.value;
    }

    // Mark cache as valid
    if (target->rp2350.cache_enabled) {
        target->rp2350.harts[hart_id].cache_valid = true;
    }

    return SWD_OK;
}

//==============================================================================
// PC and CSR Access
//==============================================================================

swd_result_t rp2350_read_pc(swd_target_t *target, uint8_t hart_id) {
    return rp2350_read_csr(target, hart_id, 0x7b1);  // DPC = 0x7b1
}

swd_error_t rp2350_write_pc(swd_target_t *target, uint8_t hart_id, uint32_t pc) {
    return rp2350_write_csr(target, hart_id, 0x7b1, pc);  // DPC = 0x7b1
}

swd_result_t rp2350_read_csr(swd_target_t *target, uint8_t hart_id, uint16_t csr_addr) {
    swd_result_t result = {.error = SWD_ERROR_NOT_INITIALIZED, .value = 0};

    if (!target || !target->rp2350.initialized) {
        return result;
    }

    if (!validate_hart_id(hart_id)) {
        result.error = SWD_ERROR_INVALID_PARAM;
        swd_set_error(target, result.error, "Invalid hart_id: %u", hart_id);
        return result;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    if (!hart->halted) {
        result.error = SWD_ERROR_NOT_HALTED;
        return result;
    }

    // RP2350 doesn't support abstract CSR access, use program buffer
    // Save s0 (x8) first
    swd_result_t saved_s0 = rp2350_read_reg(target, hart_id, 8);
    if (saved_s0.error != SWD_OK) {
        return saved_s0;
    }

    // Build CSR read instruction: csrr s0, csr_addr
    uint32_t csr_inst = 0x00002473 | (csr_addr << 20);  // csrr s0, csr

    uint32_t progbuf[] = {
        csr_inst,       // csrr s0, <csr>
        0x00100073      // ebreak
    };

    result.error = execute_progbuf_simple(target, hart_id, progbuf, 2);
    if (result.error != SWD_OK) {
        rp2350_write_reg(target, hart_id, 8, saved_s0.value);
        return result;
    }

    // Read s0 to get CSR value
    result = rp2350_read_reg(target, hart_id, 8);

    // Restore s0
    rp2350_write_reg(target, hart_id, 8, saved_s0.value);

    return result;
}

swd_error_t rp2350_write_csr(swd_target_t *target, uint8_t hart_id, uint16_t csr_addr, uint32_t value) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    hart_state_t *hart = &target->rp2350.harts[hart_id];

    if (!hart->halted) {
        return SWD_ERROR_NOT_HALTED;
    }

    // RP2350 doesn't support abstract CSR access, use program buffer
    // Save s0 (x8) first
    swd_result_t saved_s0 = rp2350_read_reg(target, hart_id, 8);
    if (saved_s0.error != SWD_OK) {
        return saved_s0.error;
    }

    swd_error_t err = SWD_OK;

    // Write value to s0
    err = rp2350_write_reg(target, hart_id, 8, value);
    if (err != SWD_OK) {
        goto err_handler;
    }

    // Build CSR write instruction: csrw csr_addr, s0
    uint32_t csr_inst = 0x00041073 | (csr_addr << 20);  // csrw csr, s0

    uint32_t progbuf[] = {
        csr_inst,       // csrw <csr>, s0
        0x00100073      // ebreak
    };

    err = execute_progbuf_simple(target, hart_id, progbuf, 2);

err_handler:
    // Restore s0 regardless of result
    rp2350_write_reg(target, hart_id, 8, saved_s0.value);

    return err;
}

//==============================================================================
// Cache Management
//==============================================================================

void rp2350_invalidate_cache(swd_target_t *target, uint8_t hart_id) {
    if (target && validate_hart_id(hart_id)) {
        target->rp2350.harts[hart_id].cache_valid = false;
    }
}

void rp2350_enable_cache(swd_target_t *target, bool enable) {
    if (target) {
        target->rp2350.cache_enabled = enable;
        if (!enable) {
            // Invalidate all hart caches
            for (uint8_t i = 0; i < RP2350_NUM_HARTS; i++) {
                target->rp2350.harts[i].cache_valid = false;
            }
        }
    }
}

//==============================================================================
// System Bus Access (SBA) Initialization
//==============================================================================

static swd_error_t rp2350_init_sba(swd_target_t *target) {
    SWD_INFO("Initializing System Bus Access...\n");

    // Read SBCS
    swd_result_t result = dap_read_mem32(target, DM_SBCS);
    if (result.error != SWD_OK) {
        return result.error;
    }

    // Check 32-bit support via sbasize field (bits [11:5])
    // sbasize indicates address width; we check sbaccess support by attempting to set it
    uint32_t sbasize = (result.value >> 5) & 0x7F;
    if (sbasize == 0) {
        SWD_WARN("SBA: No address width reported (sbasize=0)\n");
        return SWD_ERROR_INVALID_STATE;
    }

    // Clear errors
    uint8_t sberror = (result.value >> 12) & 0x7;
    if (sberror != 0) {
        uint32_t sbcs = result.value | (0x7 << 12);
        dap_write_mem32(target, DM_SBCS, sbcs);
    }

    // Configure: 32-bit, auto-read on address
    uint32_t sbcs = 0;
    sbcs |= (2 << 17);  // sbaccess = 32-bit
    sbcs |= (1 << 20);  // sbreadonaddr

    swd_error_t err = dap_write_mem32(target, DM_SBCS, sbcs);
    if (err == SWD_OK) {
        target->rp2350.sba_initialized = true;
        SWD_INFO("SBA initialized\n");
    }

    return err;
}

//==============================================================================
// Memory Access
//==============================================================================

swd_result_t rp2350_read_mem32(swd_target_t *target, uint32_t addr) {
    swd_result_t result = {.error = SWD_ERROR_NOT_INITIALIZED, .value = 0};

    if (!target || !target->rp2350.initialized) {
        return result;
    }

    if (addr & 0x3) {
        result.error = SWD_ERROR_ALIGNMENT;
        return result;
    }

    // Use SBA if available
    if (target->rp2350.sba_initialized) {
        // Write address
        result.error = dap_write_mem32(target, DM_SBADDRESS0, addr);
        if (result.error != SWD_OK) {
            return result;
        }

        // Read data
        result = dap_read_mem32(target, DM_SBDATA0);
    } else {
        // Fallback to DAP
        result = dap_read_mem32(target, addr);
    }

    return result;
}

swd_error_t rp2350_write_mem32(swd_target_t *target, uint32_t addr, uint32_t value) {
    if (!target || !target->rp2350.initialized) {
        return SWD_ERROR_NOT_INITIALIZED;
    }

    if (addr & 0x3) {
        return SWD_ERROR_ALIGNMENT;
    }

    // Use SBA if available
    if (target->rp2350.sba_initialized) {
        swd_error_t err = dap_write_mem32(target, DM_SBADDRESS0, addr);
        if (err != SWD_OK) {
            return err;
        }

        return dap_write_mem32(target, DM_SBDATA0, value);
    } else {
        return dap_write_mem32(target, addr, value);
    }
}

swd_result_t rp2350_read_mem16(swd_target_t *target, uint32_t addr) {
    swd_result_t result = {.error = SWD_ERROR_NOT_INITIALIZED, .value = 0};

    if (!target) {
        return result;
    }

    if (addr & 0x1) {
        result.error = SWD_ERROR_ALIGNMENT;
        return result;
    }

    // Read aligned word
    uint32_t aligned_addr = addr & ~3;
    result = rp2350_read_mem32(target, aligned_addr);
    if (result.error != SWD_OK) {
        return result;
    }

    // Extract halfword
    uint32_t offset = addr & 3;
    if (offset == 0) {
        result.value = result.value & 0xFFFF;
    } else {
        result.value = (result.value >> 16) & 0xFFFF;
    }

    return result;
}

swd_error_t rp2350_write_mem16(swd_target_t *target, uint32_t addr, uint16_t value) {
    if (!target) {
        return SWD_ERROR_INVALID_PARAM;
    }

    if (addr & 0x1) {
        return SWD_ERROR_ALIGNMENT;
    }

    // Read-modify-write
    uint32_t aligned_addr = addr & ~3;
    swd_result_t current = rp2350_read_mem32(target, aligned_addr);
    if (current.error != SWD_OK) {
        return current.error;
    }

    uint32_t offset = addr & 3;
    uint32_t new_value;
    if (offset == 0) {
        new_value = (current.value & 0xFFFF0000) | value;
    } else {
        new_value = (current.value & 0x0000FFFF) | ((uint32_t)value << 16);
    }

    return rp2350_write_mem32(target, aligned_addr, new_value);
}

swd_result_t rp2350_read_mem8(swd_target_t *target, uint32_t addr) {
    swd_result_t result = rp2350_read_mem32(target, addr & ~3);
    if (result.error == SWD_OK) {
        uint32_t offset = addr & 3;
        result.value = (result.value >> (offset * 8)) & 0xFF;
    }
    return result;
}

swd_error_t rp2350_write_mem8(swd_target_t *target, uint32_t addr, uint8_t value) {
    uint32_t aligned_addr = addr & ~3;
    swd_result_t current = rp2350_read_mem32(target, aligned_addr);
    if (current.error != SWD_OK) {
        return current.error;
    }

    uint32_t offset = addr & 3;
    uint32_t mask = ~(0xFF << (offset * 8));
    uint32_t new_value = (current.value & mask) | ((uint32_t)value << (offset * 8));

    return rp2350_write_mem32(target, aligned_addr, new_value);
}

swd_error_t rp2350_read_mem_block(swd_target_t *target, uint32_t addr,
                                   uint32_t *buffer, uint32_t count) {
    if (!target || !buffer) {
        return SWD_ERROR_INVALID_PARAM;
    }

    for (uint32_t i = 0; i < count; i++) {
        swd_result_t result = rp2350_read_mem32(target, addr + (i * 4));
        if (result.error != SWD_OK) {
            return result.error;
        }
        buffer[i] = result.value;
    }

    return SWD_OK;
}

swd_error_t rp2350_write_mem_block(swd_target_t *target, uint32_t addr,
                                    const uint32_t *buffer, uint32_t count) {
    if (!target || !buffer) {
        return SWD_ERROR_INVALID_PARAM;
    }

    for (uint32_t i = 0; i < count; i++) {
        swd_error_t err = rp2350_write_mem32(target, addr + (i * 4), buffer[i]);
        if (err != SWD_OK) {
            return err;
        }
    }

    return SWD_OK;
}

//==============================================================================
// Code Execution
//==============================================================================

swd_error_t rp2350_upload_code(swd_target_t *target, uint32_t addr,
                                const uint32_t *code, uint32_t word_count) {
    if (!target || !code || word_count == 0) {
        return SWD_ERROR_INVALID_PARAM;
    }

    if (addr & 0x3) {
        return SWD_ERROR_ALIGNMENT;
    }

    SWD_INFO("Uploading %lu words to 0x%08lx...\n", (unsigned long)word_count, (unsigned long)addr);

    // Write code
    for (uint32_t i = 0; i < word_count; i++) {
        swd_error_t err = rp2350_write_mem32(target, addr + (i * 4), code[i]);
        if (err != SWD_OK) {
            return err;
        }

        // Verify
        swd_result_t readback = rp2350_read_mem32(target, addr + (i * 4));
        if (readback.error != SWD_OK) {
            return readback.error;
        }

        if (readback.value != code[i]) {
            swd_set_error(target, SWD_ERROR_VERIFY,
                         "Verification failed at word %u: wrote 0x%08x, read 0x%08x",
                         i, code[i], readback.value);
            return SWD_ERROR_VERIFY;
        }
    }

    SWD_INFO("Code upload complete\n");
    return SWD_OK;
}

swd_error_t rp2350_execute_code(swd_target_t *target, uint8_t hart_id, uint32_t entry_point,
                                 const uint32_t *code, uint32_t word_count) {
    if (!target || !code) {
        return SWD_ERROR_INVALID_PARAM;
    }

    if (!validate_hart_id(hart_id)) {
        swd_set_error(target, SWD_ERROR_INVALID_PARAM, "Invalid hart_id: %u", hart_id);
        return SWD_ERROR_INVALID_PARAM;
    }

    SWD_INFO("Executing code on hart%u at 0x%08lx (%lu words)...\n", hart_id, (unsigned long)entry_point, (unsigned long)word_count);

    // Upload code
    swd_error_t err = rp2350_upload_code(target, entry_point, code, word_count);
    if (err != SWD_OK) {
        return err;
    }

    // Halt if needed
    bool was_halted = target->rp2350.harts[hart_id].halted;
    if (!was_halted) {
        err = rp2350_halt(target, hart_id);
        if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
            return err;
        }
    }

    // Set PC
    err = rp2350_write_pc(target, hart_id, entry_point);
    if (err != SWD_OK) {
        return err;
    }

    // Verify PC
    swd_result_t pc = rp2350_read_pc(target, hart_id);
    if (pc.error != SWD_OK || pc.value != entry_point) {
        swd_set_error(target, SWD_ERROR_VERIFY,
                     "PC verification failed: expected 0x%08x, got 0x%08x",
                     entry_point, pc.value);
        return SWD_ERROR_VERIFY;
    }

    // Resume
    err = rp2350_resume(target, hart_id);
    if (err != SWD_OK) {
        return err;
    }

    SWD_INFO("Code execution started on hart%u\n", hart_id);
    return SWD_OK;
}

//==============================================================================
// Instruction Tracing
//==============================================================================

int rp2350_trace(swd_target_t *target, uint8_t hart_id, uint32_t max_instructions,
                 trace_callback_t callback, void *user_data,
                 bool capture_regs) {
    if (!target || !callback) {
        return -SWD_ERROR_INVALID_PARAM;
    }

    if (!target->rp2350.initialized) {
        return -SWD_ERROR_NOT_INITIALIZED;
    }

    if (!validate_hart_id(hart_id)) {
        return -SWD_ERROR_INVALID_PARAM;
    }

    // Ensure hart is halted before we start
    if (!target->rp2350.harts[hart_id].halted) {
        swd_error_t err = rp2350_halt(target, hart_id);
        if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
            return -err;
        }
    }

    uint32_t count = 0;
    bool unlimited = (max_instructions == 0);

    SWD_INFO("Starting instruction trace on hart%u (max=%lu, capture_regs=%d)...\n",
             hart_id, (unsigned long)max_instructions, capture_regs);

    while (unlimited || count < max_instructions) {
        trace_record_t record = {0};

        // Read PC
        swd_result_t pc_result = rp2350_read_pc(target, hart_id);
        if (pc_result.error != SWD_OK) {
            SWD_INFO("Trace stopped: failed to read PC\n");
            return count > 0 ? (int)count : -pc_result.error;
        }
        record.pc = pc_result.value;

        // Read instruction at PC
        swd_result_t inst_result = rp2350_read_mem32(target, record.pc);
        if (inst_result.error != SWD_OK) {
            SWD_INFO("Trace stopped: failed to read instruction at 0x%08lx\n", (unsigned long)record.pc);
            return count > 0 ? (int)count : -inst_result.error;
        }
        record.instruction = inst_result.value;

        // Optional: Capture all registers
        if (capture_regs) {
            swd_error_t err = rp2350_read_all_regs(target, hart_id, record.regs);
            if (err != SWD_OK) {
                SWD_INFO("Trace stopped: failed to read registers\n");
                return count > 0 ? (int)count : -err;
            }
        }

        // Count this instruction as traced
        count++;

        // Call user callback
        if (!callback(&record, user_data)) {
            SWD_INFO("Trace stopped by callback after %lu instructions\n", (unsigned long)count);
            break;
        }

        // Execute one instruction
        swd_error_t err = rp2350_step(target, hart_id);
        if (err != SWD_OK) {
            SWD_INFO("Trace stopped: step failed\n");
            return count > 0 ? (int)count : -err;
        }
    }

    SWD_INFO("Trace completed: %lu instructions\n", (unsigned long)count);
    return (int)count;
}
