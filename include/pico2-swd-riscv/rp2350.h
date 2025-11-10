/**
 * @file rp2350.h
 * @brief RP2350 RISC-V Debug Module interface
 *
 * High-level interface for debugging RP2350 RISC-V cores (Hazard3).
 * This is the main API most users will interact with.
 *
 * @section usage Basic Usage
 * @code
 * swd_target_t *target = swd_target_create(&config);
 * swd_connect(target);
 *
 * rp2350_init(target);
 * rp2350_halt(target);
 * swd_result_t pc = rp2350_read_pc(target);
 * rp2350_resume(target);
 * @endcode
 *
 * Copyright (c) 2025
 * SPDX-License-Identifier: MIT
 */

#ifndef PICO2_SWD_RISCV_RP2350_H
#define PICO2_SWD_RISCV_RP2350_H

#include "pico2-swd-riscv/types.h"

#ifdef __cplusplus
extern "C" {
#endif

//==============================================================================
// Debug Module Initialization
//==============================================================================

/**
 * @brief Initialize RP2350 Debug Module
 *
 * Performs the RP2350-specific Debug Module activation sequence:
 * 1. Select RISC-V APB-AP
 * 2. Configure CSW for 32-bit access
 * 3. Switch to bank 1 for DM control
 * 4. Activate Debug Module
 * 5. Verify DM is responding
 *
 * Must be called after swd_connect() and dap_power_up().
 *
 * @param target Target to initialize
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_init(swd_target_t *target);

/**
 * @brief Check if Debug Module is initialized
 *
 * @param target Target to check
 * @return true if initialized, false otherwise
 */
bool rp2350_is_initialized(const swd_target_t *target);

//==============================================================================
// Hart Control
//==============================================================================

/**
 * @brief Halt the RISC-V hart
 *
 * Stops execution of the specified hart. Returns SWD_ERROR_ALREADY_HALTED if
 * hart is already halted (not an error condition).
 *
 * @param target Target to halt
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @return SWD_OK on success, SWD_ERROR_ALREADY_HALTED if already halted,
 *         error code otherwise
 */
swd_error_t rp2350_halt(swd_target_t *target, uint8_t hart_id);

/**
 * @brief Resume the RISC-V hart
 *
 * Resumes execution from current PC. No-op if hart is already running.
 *
 * @param target Target to resume
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_resume(swd_target_t *target, uint8_t hart_id);

/**
 * @brief Single-step the RISC-V hart
 *
 * Executes one instruction and halts. Hart must be halted before calling.
 *
 * @param target Target to step
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_step(swd_target_t *target, uint8_t hart_id);

/**
 * @brief Reset the RISC-V hart
 *
 * Performs a hart reset. Optionally halts immediately after reset.
 *
 * @param target Target to reset
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param halt_on_reset If true, halt immediately after reset
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_reset(swd_target_t *target, uint8_t hart_id, bool halt_on_reset);

/**
 * @brief Check if hart is halted
 *
 * @param target Target to check
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @return true if halted, false if running or error
 */
bool rp2350_is_halted(const swd_target_t *target, uint8_t hart_id);

//==============================================================================
// Register Access
//==============================================================================

/**
 * @brief Read Program Counter
 *
 * Reads the Debug PC (DPC) CSR. Hart must be halted.
 *
 * @param target Target to read from
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @return Result containing PC value or error
 */
swd_result_t rp2350_read_pc(swd_target_t *target, uint8_t hart_id);

/**
 * @brief Write Program Counter
 *
 * Sets the PC to a new value. Hart must be halted. PC will be used
 * when hart is resumed.
 *
 * @param target Target to write to
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param pc New PC value
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_pc(swd_target_t *target, uint8_t hart_id, uint32_t pc);

/**
 * @brief Read General Purpose Register
 *
 * Reads one of the 32 RISC-V integer registers (x0-x31).
 * Hart must be halted. Note: x0 always reads as 0.
 *
 * @param target Target to read from
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param reg_num Register number (0-31)
 * @return Result containing register value or error
 */
swd_result_t rp2350_read_reg(swd_target_t *target, uint8_t hart_id, uint8_t reg_num);

/**
 * @brief Write General Purpose Register
 *
 * Writes one of the 32 RISC-V integer registers (x1-x31).
 * Hart must be halted. Writing x0 has no effect.
 *
 * @param target Target to write to
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param reg_num Register number (0-31)
 * @param value Value to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_reg(swd_target_t *target, uint8_t hart_id, uint8_t reg_num, uint32_t value);

/**
 * @brief Read all general purpose registers at once
 *
 * Efficiently reads all 32 integer registers in one operation.
 * Much faster than calling rp2350_read_reg() 32 times.
 * Hart must be halted.
 *
 * @param target Target to read from
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param regs Array to store 32 register values (x0-x31)
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_read_all_regs(swd_target_t *target, uint8_t hart_id, uint32_t regs[32]);

/**
 * @brief Read Control and Status Register
 *
 * Reads a RISC-V CSR by address. Hart must be halted.
 *
 * @param target Target to read from
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param csr_addr CSR address (e.g., 0x300 for mstatus)
 * @return Result containing CSR value or error
 */
swd_result_t rp2350_read_csr(swd_target_t *target, uint8_t hart_id, uint16_t csr_addr);

/**
 * @brief Write Control and Status Register
 *
 * Writes a RISC-V CSR by address. Hart must be halted.
 *
 * @param target Target to write to
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param csr_addr CSR address
 * @param value Value to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_csr(swd_target_t *target, uint8_t hart_id, uint16_t csr_addr, uint32_t value);

//==============================================================================
// Cache Management
//==============================================================================

/**
 * @brief Invalidate register cache for a specific hart
 *
 * Forces next register read to fetch from target instead of cache.
 * Useful after target has been running.
 *
 * @param target Target to invalidate cache for
 * @param hart_id Hart ID (0 or 1 for RP2350)
 */
void rp2350_invalidate_cache(swd_target_t *target, uint8_t hart_id);

/**
 * @brief Enable or disable register caching (shared across all harts)
 *
 * When enabled, register values are cached to avoid redundant reads.
 * Cache is automatically invalidated when hart resumes.
 *
 * @param target Target to configure
 * @param enable true to enable caching, false to disable
 */
void rp2350_enable_cache(swd_target_t *target, bool enable);

//==============================================================================
// Memory Access
//==============================================================================

/**
 * @brief Read 32-bit word from memory
 *
 * Uses System Bus Access (SBA) for non-intrusive memory reads.
 * Works even while hart is running. Address must be 4-byte aligned.
 *
 * @param target Target to read from
 * @param addr Memory address (must be 4-byte aligned)
 * @return Result containing memory value or error
 */
swd_result_t rp2350_read_mem32(swd_target_t *target, uint32_t addr);

/**
 * @brief Write 32-bit word to memory
 *
 * Uses System Bus Access (SBA) for non-intrusive memory writes.
 * Works even while hart is running. Address must be 4-byte aligned.
 *
 * @param target Target to write to
 * @param addr Memory address (must be 4-byte aligned)
 * @param value Value to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_mem32(swd_target_t *target, uint32_t addr, uint32_t value);

/**
 * @brief Read 16-bit halfword from memory
 *
 * Address must be 2-byte aligned. Implemented using read-modify-write
 * on aligned 32-bit word.
 *
 * @param target Target to read from
 * @param addr Memory address (must be 2-byte aligned)
 * @return Result containing memory value or error
 */
swd_result_t rp2350_read_mem16(swd_target_t *target, uint32_t addr);

/**
 * @brief Write 16-bit halfword to memory
 *
 * Address must be 2-byte aligned. Implemented using read-modify-write
 * on aligned 32-bit word.
 *
 * @param target Target to write to
 * @param addr Memory address (must be 2-byte aligned)
 * @param value Value to write (lower 16 bits)
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_mem16(swd_target_t *target, uint32_t addr, uint16_t value);

/**
 * @brief Read 8-bit byte from memory
 *
 * Implemented using read-modify-write on aligned 32-bit word.
 *
 * @param target Target to read from
 * @param addr Memory address
 * @return Result containing memory value or error
 */
swd_result_t rp2350_read_mem8(swd_target_t *target, uint32_t addr);

/**
 * @brief Write 8-bit byte to memory
 *
 * Implemented using read-modify-write on aligned 32-bit word.
 *
 * @param target Target to write to
 * @param addr Memory address
 * @param value Value to write (lower 8 bits)
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_mem8(swd_target_t *target, uint32_t addr, uint8_t value);

/**
 * @brief Read block of memory
 *
 * Efficiently reads multiple 32-bit words. Address must be 4-byte aligned.
 *
 * @param target Target to read from
 * @param addr Starting address (must be 4-byte aligned)
 * @param buffer Buffer to store results
 * @param count Number of 32-bit words to read
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_read_mem_block(swd_target_t *target, uint32_t addr,
                                   uint32_t *buffer, uint32_t count);

/**
 * @brief Write block of memory
 *
 * Efficiently writes multiple 32-bit words. Address must be 4-byte aligned.
 *
 * @param target Target to write to
 * @param addr Starting address (must be 4-byte aligned)
 * @param buffer Data to write
 * @param count Number of 32-bit words to write
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_write_mem_block(swd_target_t *target, uint32_t addr,
                                    const uint32_t *buffer, uint32_t count);

//==============================================================================
// Program Buffer Execution
//==============================================================================

/**
 * @brief Execute custom instructions in the program buffer
 *
 * Provides low-level access to execute RISC-V instructions directly on the
 * target hart via the Debug Module's program buffer. Useful for operations
 * that cannot be performed via abstract commands.
 *
 * The program buffer can hold up to 16 instructions. The last instruction
 * should typically be `ebreak` (0x00100073) to return to Debug Mode.
 *
 * WARNING: Using postexec=true can corrupt debug CSRs like DPC. Only use
 * postexec when you understand the side effects. Per RISC-V Debug Spec,
 * DPC must be accessible via abstract commands without postexec.
 *
 * @param target Target to execute on
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param instructions Array of RISC-V instruction words (up to 16)
 * @param count Number of instructions (1-16)
 * @param postexec Execute progbuf after abstract command (dangerous for DPC!)
 * @return SWD_OK on success, error code otherwise
 *
 * @example
 * @code
 * // Read a custom CSR using progbuf
 * uint32_t progbuf[] = {
 *     0x34202473,  // csrr s0, mcause (read mcause to s0)
 *     0x00100073   // ebreak (return to Debug Mode)
 * };
 * rp2350_execute_progbuf(target, 0, progbuf, 2, false);
 * swd_result_t result = rp2350_read_reg(target, 0, 8); // Read s0
 * @endcode
 */
swd_error_t rp2350_execute_progbuf(swd_target_t *target, uint8_t hart_id,
                                    const uint32_t *instructions, uint8_t count);

//==============================================================================
// Code Execution
//==============================================================================

/**
 * @brief Upload code to target memory
 *
 * Uploads RISC-V machine code to target memory using SBA.
 * Verifies each word after writing. Hart can remain running.
 *
 * @param target Target to upload to
 * @param addr Destination address (must be 4-byte aligned)
 * @param code Array of 32-bit instruction words
 * @param word_count Number of words to upload
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_upload_code(swd_target_t *target, uint32_t addr,
                                const uint32_t *code, uint32_t word_count);

/**
 * @brief Upload and execute code
 *
 * Complete workflow to execute custom code:
 * 1. Upload code to memory
 * 2. Halt hart if running
 * 3. Set PC to entry point
 * 4. Resume execution
 *
 * @param target Target to execute on
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param entry_point Address to start execution (must be 4-byte aligned)
 * @param code Array of 32-bit instruction words
 * @param word_count Number of words in code
 * @return SWD_OK on success, error code otherwise
 */
swd_error_t rp2350_execute_code(swd_target_t *target, uint8_t hart_id, uint32_t entry_point,
                                 const uint32_t *code, uint32_t word_count);

//==============================================================================
// Instruction Tracing
//==============================================================================

// Note: Hardware breakpoint support has been removed and will be
// reimplemented in a future version with proper per-hart support.

/**
 * @brief Trace record for single instruction
 *
 * Contains execution state captured after executing one instruction.
 */
typedef struct {
    uint32_t pc;          // Program counter
    uint32_t instruction; // Instruction word at PC
    uint32_t regs[32];    // Register snapshot (only valid if capture_regs enabled)
} trace_record_t;

/**
 * @brief Trace callback function
 *
 * Called after each instruction execution. Return false to stop tracing.
 *
 * @param record Trace data for the instruction just executed
 * @param user_data User-provided context pointer
 * @return true to continue tracing, false to stop
 */
typedef bool (*trace_callback_t)(const trace_record_t *record, void *user_data);

/**
 * @brief Trace execution for N instructions
 *
 * Single-steps through instructions, calling callback for each.
 * Uses DCSR.step mechanism for precise instruction-level control.
 *
 * Performance: ~15-20ms per instruction without register capture,
 * ~80ms per instruction with full register capture.
 *
 * @param target Target to trace
 * @param hart_id Hart ID (0 or 1 for RP2350)
 * @param max_instructions Maximum instructions to trace (0 = unlimited)
 * @param callback Function called for each instruction (must not be NULL)
 * @param user_data Passed to callback (may be NULL)
 * @param capture_regs If true, read all 32 registers each step (slower)
 * @return Number of instructions traced (>=0), or negative error code on failure
 */
int rp2350_trace(swd_target_t *target, uint8_t hart_id, uint32_t max_instructions,
                 trace_callback_t callback, void *user_data,
                 bool capture_regs);

#ifdef __cplusplus
}
#endif

#endif // PICO2_SWD_RISCV_RP2350_H
