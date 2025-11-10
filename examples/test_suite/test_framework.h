/**
 * @file test_framework.h
 * @brief Test framework for pico2-swd-riscv library
 *
 * Provides common definitions, test infrastructure, and cleanup mechanisms
 * to ensure tests are isolated from each other.
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef TEST_FRAMEWORK_H
#define TEST_FRAMEWORK_H

#include <stdint.h>
#include <stdbool.h>
#include "pico2-swd-riscv/swd.h"

//==============================================================================
// Test Protocol Definitions
//==============================================================================

// Commands
#define CMD_READY "READY"
#define CMD_CONNECT "CONNECT"
#define CMD_INIT "INIT"
#define CMD_HALT "HALT"
#define CMD_RESUME "RESUME"
#define CMD_READ_PC "READ_PC"
#define CMD_WRITE_PC "WRITE_PC"
#define CMD_READ_REG "READ_REG"
#define CMD_WRITE_REG "WRITE_REG"
#define CMD_READ_MEM "READ_MEM"
#define CMD_WRITE_MEM "WRITE_MEM"
#define CMD_TRACE "TRACE"
#define CMD_RESET "RESET"
#define CMD_SET_BP "SET_BP"
#define CMD_CLEAR_BP "CLEAR_BP"
#define CMD_CLEAR_ALL_BP "CLEAR_ALL_BP"
#define CMD_TEST_ALL "TEST_ALL"
#define CMD_DISCONNECT "DISCONNECT"

// Responses
#define RESP_PASS "PASS"
#define RESP_FAIL "FAIL"
#define RESP_VALUE "VALUE"

//==============================================================================
// Test Result Tracking
//==============================================================================

typedef struct {
    const char *name;
    bool (*test_func)(swd_target_t *target);
    bool passed;
    bool ran;
} test_case_t;

typedef struct {
    uint32_t total;
    uint32_t passed;
    uint32_t failed;
    uint32_t skipped;
} test_stats_t;

//==============================================================================
// Test Framework Functions
//==============================================================================

/**
 * @brief Initialize the test framework
 * @param target The SWD target to use for all tests
 */
void test_framework_init(swd_target_t *target);

/**
 * @brief Get the current target
 * @return Pointer to the current SWD target
 */
swd_target_t* test_get_target(void);

/**
 * @brief Send a response to the test runner
 * @param status Status string (PASS, FAIL, VALUE)
 * @param message Optional message
 */
void test_send_response(const char *status, const char *message);

/**
 * @brief Send a value response
 * @param value The value to send
 */
void test_send_value(uint32_t value);

/**
 * @brief Setup before each test (connect and initialize)
 * @return SWD_OK on success
 */
swd_error_t test_setup(void);

/**
 * @brief Cleanup after each test (minimal - just resume harts)
 */
void test_cleanup(void);

/**
 * @brief Final cleanup at end of test suite (disconnect everything)
 */
void test_final_cleanup(void);

/**
 * @brief Run a single test with setup/cleanup
 * @param test_case The test case to run
 * @return true if test passed
 */
bool test_run_single(test_case_t *test_case);

/**
 * @brief Run all tests in a test suite
 * @param tests Array of test cases
 * @param count Number of tests
 * @return Test statistics
 */
test_stats_t test_run_suite(test_case_t *tests, uint32_t count);

/**
 * @brief Print test statistics
 * @param stats Test statistics to print
 */
void test_print_stats(const test_stats_t *stats);

//==============================================================================
// Test Declarations (implemented in separate files)
//==============================================================================

// Basic tests (test_basic.c)
extern test_case_t basic_tests[];
extern const uint32_t basic_test_count;

// Hart 0 tests (test_hart0.c)
extern test_case_t hart0_tests[];
extern const uint32_t hart0_test_count;

// Hart 1 tests (test_hart1.c)
extern test_case_t hart1_tests[];
extern const uint32_t hart1_test_count;

// Dual-hart tests (test_dual_hart.c)
extern test_case_t dual_hart_tests[];
extern const uint32_t dual_hart_test_count;

// Memory tests (test_mem.c)
extern test_case_t memory_tests[];
extern const uint32_t memory_test_count;

// Trace tests (test_trace.c)
extern test_case_t trace_tests[];
extern const uint32_t trace_test_count;

// API coverage tests (test_api_coverage.c)
extern test_case_t api_coverage_tests[];
extern const uint32_t api_coverage_test_count;

// Memory operations tests (test_memory_ops.c)
extern test_case_t memory_ops_tests[];
extern const uint32_t memory_ops_test_count;

// Cache tests (test_cache.c)
extern test_case_t cache_tests[];
extern const uint32_t cache_test_count;

// Code execution tests (test_code_exec.c)
extern test_case_t code_exec_tests[];
extern const uint32_t code_exec_test_count;

#endif // TEST_FRAMEWORK_H
