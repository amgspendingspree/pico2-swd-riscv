/**
 * @file test_api_coverage.c
 * @brief Tests for API functions not covered by other test files
 *
 * Covers: DAP layer utilities, connection state queries, resource management
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#include "test_framework.h"
#include "pico2-swd-riscv/rp2350.h"
#include "pico2-swd-riscv/dap.h"
#include <stdio.h>

//==============================================================================
// Test 1: DAP Power State Query
//==============================================================================

static bool test_dap_is_powered(swd_target_t *target) {
    printf("# Testing dap_is_powered()...\n");

    // Should be powered since we're connected
    bool powered = dap_is_powered(target);
    if (!powered) {
        printf("# DAP should be powered but reports not powered\n");
        test_send_response(RESP_FAIL, "DAP not powered");
        return false;
    }

    printf("# DAP is powered: %s\n", powered ? "yes" : "no");
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 2: Connection State Query
//==============================================================================

static bool test_swd_is_connected(swd_target_t *target) {
    printf("# Testing swd_is_connected()...\n");

    // Should be connected
    bool connected = swd_is_connected(target);
    if (!connected) {
        printf("# Should be connected but reports not connected\n");
        test_send_response(RESP_FAIL, "Not connected");
        return false;
    }

    printf("# SWD is connected: %s\n", connected ? "yes" : "no");
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 3: SWD Frequency Query
//==============================================================================

static bool test_swd_get_frequency(swd_target_t *target) {
    printf("# Testing swd_get_frequency()...\n");

    uint32_t freq_khz = swd_get_frequency(target);
    if (freq_khz == 0) {
        printf("# Frequency should not be zero\n");
        test_send_response(RESP_FAIL, "Zero frequency");
        return false;
    }

    printf("# SWCLK frequency: %lu kHz\n", (unsigned long)freq_khz);
    test_send_value(freq_khz);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 4: Resource Usage Query
//==============================================================================

static bool test_swd_get_resource_usage(swd_target_t *target) {
    printf("# Testing swd_get_resource_usage()...\n");

    swd_resource_info_t info = swd_get_resource_usage();

    printf("# Active targets: %lu\n", (unsigned long)info.active_targets);
    printf("# PIO0 SMs used: %d%d%d%d\n",
           info.pio0_sm_used[0], info.pio0_sm_used[1],
           info.pio0_sm_used[2], info.pio0_sm_used[3]);
    printf("# PIO1 SMs used: %d%d%d%d\n",
           info.pio1_sm_used[0], info.pio1_sm_used[1],
           info.pio1_sm_used[2], info.pio1_sm_used[3]);

    // Should have at least one active target
    if (info.active_targets == 0) {
        printf("# Should have at least 1 active target\n");
        test_send_response(RESP_FAIL, "No active targets");
        return false;
    }

    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 5: RP2350 Initialization State Query
//==============================================================================

static bool test_rp2350_is_initialized(swd_target_t *target) {
    printf("# Testing rp2350_is_initialized()...\n");

    bool initialized = rp2350_is_initialized(target);
    if (!initialized) {
        printf("# RP2350 should be initialized but reports not initialized\n");
        test_send_response(RESP_FAIL, "Not initialized");
        return false;
    }

    printf("# RP2350 is initialized: %s\n", initialized ? "yes" : "no");
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 6: Hart Halted State Query
//==============================================================================

static bool test_rp2350_is_halted(swd_target_t *target) {
    printf("# Testing rp2350_is_halted()...\n");

    // Halt hart 0
    swd_error_t err = rp2350_halt(target, 0);
    if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
        printf("# Failed to halt hart 0: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Failed to halt");
        return false;
    }

    // Check if halted
    bool halted = rp2350_is_halted(target, 0);
    if (!halted) {
        printf("# Hart 0 should be halted but reports not halted\n");
        test_send_response(RESP_FAIL, "Hart not halted");
        return false;
    }

    printf("# Hart 0 is halted: %s\n", halted ? "yes" : "no");

    // Resume
    err = rp2350_resume(target, 0);
    if (err != SWD_OK) {
        printf("# Failed to resume hart 0: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Failed to resume");
        return false;
    }

    // Check if running
    sleep_ms(10);
    halted = rp2350_is_halted(target, 0);
    if (halted) {
        printf("# Hart 0 should be running but reports halted\n");
        test_send_response(RESP_FAIL, "Hart still halted");
        return false;
    }

    printf("# Hart 0 is running: %s\n", halted ? "no" : "yes");
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 7: DAP Clear Errors
//==============================================================================

static bool test_dap_clear_errors(swd_target_t *target) {
    printf("# Testing dap_clear_errors()...\n");

    // Clear any sticky error flags
    swd_error_t err = dap_clear_errors(target);
    if (err != SWD_OK) {
        printf("# Failed to clear errors: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Failed to clear errors");
        return false;
    }

    printf("# DAP errors cleared\n");
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 8: DAP Read AP Register
//==============================================================================

static bool test_dap_read_ap(swd_target_t *target) {
    printf("# Testing dap_read_ap()...\n");

    // Read AP IDR register (offset 0xFC) from RISC-V AP
    swd_result_t result = dap_read_ap(target, AP_RISCV, 0xFC);
    if (result.error != SWD_OK) {
        printf("# Failed to read AP IDR: %s\n", swd_error_string(result.error));
        test_send_response(RESP_FAIL, "Failed to read AP");
        return false;
    }

    printf("# AP IDR: 0x%08lx\n", (unsigned long)result.value);
    test_send_value(result.value);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test Array
//==============================================================================

test_case_t api_coverage_tests[] = {
    {"DAP Power State Query", test_dap_is_powered, false, false},
    {"SWD Connection State Query", test_swd_is_connected, false, false},
    {"SWD Frequency Query", test_swd_get_frequency, false, false},
    {"Resource Usage Query", test_swd_get_resource_usage, false, false},
    {"RP2350 Initialization State", test_rp2350_is_initialized, false, false},
    {"Hart Halted State Query", test_rp2350_is_halted, false, false},
    {"DAP Clear Errors", test_dap_clear_errors, false, false},
    {"DAP Read AP Register", test_dap_read_ap, false, false},
};

const uint32_t api_coverage_test_count = sizeof(api_coverage_tests) / sizeof(api_coverage_tests[0]);
