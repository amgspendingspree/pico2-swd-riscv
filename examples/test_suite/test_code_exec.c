/**
 * @file test_code_exec.c
 * @brief Tests for code execution functions
 *
 * Covers: rp2350_execute_code, rp2350_execute_progbuf
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#include "test_framework.h"
#include "pico2-swd-riscv/rp2350.h"
#include <stdio.h>

// Test address in SRAM
#define CODE_BASE 0x20077000
#define DATA_BASE 0x20078000

//==============================================================================
// Test 1: Execute Simple Addition Code
//==============================================================================

static bool test_execute_addition_code(swd_target_t *target) {
    printf("# Testing code execution (addition)...\n");

    // Program: x5 = x6 + x7, then infinite loop
    const uint32_t program[] = {
        0x007302B3,  // add x5, x6, x7
        0x0000006F,  // j 0 (infinite loop)
    };

    // Set up inputs
    uint32_t a = 42;
    uint32_t b = 58;

    swd_error_t err = rp2350_write_reg(target, 0, 6, a);
    if (err != SWD_OK) {
        printf("# Failed to write x6: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Setup failed");
        return false;
    }

    err = rp2350_write_reg(target, 0, 7, b);
    if (err != SWD_OK) {
        printf("# Failed to write x7: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Setup failed");
        return false;
    }

    // Execute code
    err = rp2350_execute_code(target, 0, CODE_BASE, program, 2);
    if (err != SWD_OK) {
        printf("# Failed to execute code: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Execution failed");
        return false;
    }

    printf("# Code started, waiting for execution...\n");
    sleep_ms(10);

    // Halt and read result
    err = rp2350_halt(target, 0);
    if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
        printf("# Failed to halt hart: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Halt failed");
        return false;
    }

    swd_result_t result = rp2350_read_reg(target, 0, 5);
    if (result.error != SWD_OK) {
        printf("# Failed to read result: %s\n", swd_error_string(result.error));
        test_send_response(RESP_FAIL, "Read failed");
        return false;
    }

    uint32_t expected = a + b;
    if (result.value != expected) {
        printf("# Incorrect result: got %lu, expected %lu\n",
               (unsigned long)result.value, (unsigned long)expected);
        test_send_response(RESP_FAIL, "Incorrect result");
        return false;
    }

    printf("# Code executed successfully: %lu + %lu = %lu\n",
           (unsigned long)a, (unsigned long)b, (unsigned long)result.value);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 2: Execute Memory Store Code
//==============================================================================

static bool test_execute_memory_store_code(swd_target_t *target) {
    printf("# Testing code execution (memory store)...\n");

    uint32_t store_addr = DATA_BASE;
    uint32_t store_value = 0xCAFEBABE;

    // Program: Store x10 to memory, then loop
    // lui x11, (store_addr >> 12)
    // addi x11, x11, (store_addr & 0xFFF)
    // sw x10, 0(x11)
    // j 0
    const uint32_t program[] = {
        0x200785B7,  // lui x11, 0x20078 (upper 20 bits of address)
        0x00058593,  // addi x11, x11, 0  (lower 12 bits)
        0x00A5A023,  // sw x10, 0(x11)
        0x0000006F,  // j 0
    };

    // Set x10 to the value we want to store
    swd_error_t err = rp2350_write_reg(target, 0, 10, store_value);
    if (err != SWD_OK) {
        printf("# Failed to write x10: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Setup failed");
        return false;
    }

    // Execute code
    err = rp2350_execute_code(target, 0, CODE_BASE, program, 4);
    if (err != SWD_OK) {
        printf("# Failed to execute code: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Execution failed");
        return false;
    }

    sleep_ms(10);

    // Halt
    err = rp2350_halt(target, 0);
    if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
        printf("# Failed to halt hart: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Halt failed");
        return false;
    }

    // Read memory to verify store
    swd_result_t mem_result = rp2350_read_mem32(target, store_addr);
    if (mem_result.error != SWD_OK) {
        printf("# Failed to read memory: %s\n", swd_error_string(mem_result.error));
        test_send_response(RESP_FAIL, "Memory read failed");
        return false;
    }

    if (mem_result.value != store_value) {
        printf("# Memory mismatch: got 0x%08lx, expected 0x%08lx\n",
               (unsigned long)mem_result.value, (unsigned long)store_value);
        test_send_response(RESP_FAIL, "Memory mismatch");
        return false;
    }

    printf("# Memory store successful: stored 0x%08lx at 0x%08lx\n",
           (unsigned long)store_value, (unsigned long)store_addr);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 3: Execute Code on Hart 1
//==============================================================================

static bool test_execute_code_on_hart1(swd_target_t *target) {
    printf("# Testing code execution on hart 1...\n");

    // Simple program: x12 = x13 * 2 (shift left by 1), then loop
    const uint32_t program[] = {
        0x00169693,  // slli x13, x13, 1  (multiply by 2)
        0x00068633,  // add x12, x13, x0  (move to x12)
        0x0000006F,  // j 0
    };

    uint32_t input = 25;
    uint32_t expected = input * 2;

    // Setup input on hart 1
    swd_error_t err = rp2350_write_reg(target, 1, 13, input);
    if (err != SWD_OK) {
        printf("# Failed to write x13 on hart 1: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Setup failed");
        return false;
    }

    // Execute on hart 1
    err = rp2350_execute_code(target, 1, CODE_BASE, program, 3);
    if (err != SWD_OK) {
        printf("# Failed to execute code on hart 1: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Execution failed");
        return false;
    }

    sleep_ms(10);

    // Halt hart 1
    err = rp2350_halt(target, 1);
    if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
        printf("# Failed to halt hart 1: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Halt failed");
        return false;
    }

    // Read result from hart 1
    swd_result_t result = rp2350_read_reg(target, 1, 12);
    if (result.error != SWD_OK) {
        printf("# Failed to read result from hart 1: %s\n", swd_error_string(result.error));
        test_send_response(RESP_FAIL, "Read failed");
        return false;
    }

    if (result.value != expected) {
        printf("# Incorrect result: got %lu, expected %lu\n",
               (unsigned long)result.value, (unsigned long)expected);
        test_send_response(RESP_FAIL, "Incorrect result");
        return false;
    }

    printf("# Hart 1 code executed successfully: %lu * 2 = %lu\n",
           (unsigned long)input, (unsigned long)result.value);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 4: Execute Program Buffer Instructions
//==============================================================================

static bool test_execute_progbuf(swd_target_t *target) {
    printf("# Testing program buffer execution...\n");

    rp2350_halt(target, 0);

    // Set x14 to a test value
    uint32_t input = 0x12345678;
    swd_error_t err = rp2350_write_reg(target, 0, 14, input);
    if (err != SWD_OK) {
        printf("# Failed to write x14: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Setup failed");
        return false;
    }

    // Program buffer: NOT x14 (xori with -1), store in x15
    // xori x15, x14, -1
    // ebreak
    const uint32_t progbuf[] = {
        0xFFF74793,  // xori x15, x14, -1 (bitwise NOT)
        0x00100073,  // ebreak
    };

    // Execute program buffer
    err = rp2350_execute_progbuf(target, 0, progbuf, 2);
    if (err != SWD_OK) {
        printf("# Failed to execute progbuf: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Progbuf execution failed");
        return false;
    }

    // Read result from x15
    swd_result_t result = rp2350_read_reg(target, 0, 15);
    if (result.error != SWD_OK) {
        printf("# Failed to read result: %s\n", swd_error_string(result.error));
        test_send_response(RESP_FAIL, "Read failed");
        return false;
    }

    uint32_t expected = ~input;
    if (result.value != expected) {
        printf("# Incorrect result: got 0x%08lx, expected 0x%08lx\n",
               (unsigned long)result.value, (unsigned long)expected);
        test_send_response(RESP_FAIL, "Incorrect result");
        return false;
    }

    printf("# Progbuf executed successfully: NOT(0x%08lx) = 0x%08lx\n",
           (unsigned long)input, (unsigned long)result.value);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test 5: Execute Code with Loop
//==============================================================================

static bool test_execute_code_with_loop(swd_target_t *target) {
    printf("# Testing code execution (loop)...\n");

    // Program: Count from 0 to 10 in x16
    // li x16, 0
    // li x17, 10
    // loop:
    //   addi x16, x16, 1
    //   bne x16, x17, loop
    // j 0
    const uint32_t program[] = {
        0x00000813,  // addi x16, x0, 0  (li x16, 0)
        0x00A00893,  // addi x17, x0, 10 (li x17, 10)
        0x00180813,  // addi x16, x16, 1
        0xFF181EE3,  // bne x16, x17, -4 (loop back)
        0x0000006F,  // j 0 (infinite loop)
    };

    // Execute code
    swd_error_t err = rp2350_execute_code(target, 0, CODE_BASE, program, 5);
    if (err != SWD_OK) {
        printf("# Failed to execute code: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Execution failed");
        return false;
    }

    sleep_ms(10);

    // Halt and read result
    err = rp2350_halt(target, 0);
    if (err != SWD_OK && err != SWD_ERROR_ALREADY_HALTED) {
        printf("# Failed to halt hart: %s\n", swd_error_string(err));
        test_send_response(RESP_FAIL, "Halt failed");
        return false;
    }

    swd_result_t result = rp2350_read_reg(target, 0, 16);
    if (result.error != SWD_OK) {
        printf("# Failed to read result: %s\n", swd_error_string(result.error));
        test_send_response(RESP_FAIL, "Read failed");
        return false;
    }

    uint32_t expected = 10;
    if (result.value != expected) {
        printf("# Incorrect result: got %lu, expected %lu\n",
               (unsigned long)result.value, (unsigned long)expected);
        test_send_response(RESP_FAIL, "Incorrect result");
        return false;
    }

    printf("# Loop executed successfully: counted to %lu\n", (unsigned long)result.value);
    test_send_response(RESP_PASS, NULL);
    return true;
}

//==============================================================================
// Test Array
//==============================================================================

test_case_t code_exec_tests[] = {
    {"Execute Addition Code", test_execute_addition_code, false, false},
    {"Execute Memory Store Code", test_execute_memory_store_code, false, false},
    {"Execute Code on Hart 1", test_execute_code_on_hart1, false, false},
    {"Execute Program Buffer", test_execute_progbuf, false, false},
    {"Execute Code with Loop", test_execute_code_with_loop, false, false},
};

const uint32_t code_exec_test_count = sizeof(code_exec_tests) / sizeof(code_exec_tests[0]);
