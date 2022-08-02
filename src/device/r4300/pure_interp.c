/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Mupen64plus - pure_interp.c                                           *
 *   Mupen64Plus homepage: https://mupen64plus.org/                        *
 *   Copyright (C) 2015 Nebuleon <nebuleon.fumika@gmail.com>               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#include "pure_interp.h"

#include <stdint.h>
#include <stdbool.h>

#define __STDC_FORMAT_MACROS
#include <inttypes.h>

#include "api/callbacks.h"
#include "api/m64p_types.h"
#include "device/r4300/r4300_core.h"
#include "osal/preproc.h"

void InterpretOpcode(struct r4300_core* r4300);

#define DECLARE_R4300
#define PCADDR r4300->interp_PC.addr
#define ADD_TO_PC(x) r4300->interp_PC.addr += x*4;
#define DECLARE_INSTRUCTION(name) void name(struct r4300_core* r4300, uint32_t op)
#define DECLARE_JUMP(name, destination, condition, link, likely, cop1) \
   void name(struct r4300_core* r4300, uint32_t op) \
   { \
      const int take_jump = (condition); \
      const uint32_t jump_target = (destination); \
      int64_t *link_register = (link); \
      if (cop1 && check_cop1_unusable(r4300)) return; \
      if (link_register != &r4300_regs(r4300)[0]) \
      { \
          *link_register = SE32(r4300->interp_PC.addr + 8); \
      } \
      if (!likely || take_jump) \
      { \
        r4300->interp_PC.addr += 4; \
        r4300->delay_slot=1; \
        InterpretOpcode(r4300); \
        cp0_update_count(r4300); \
        r4300->delay_slot=0; \
        if (take_jump && !r4300->skip_jump) \
        { \
          r4300->interp_PC.addr = jump_target; \
        } \
      } \
      else \
      { \
         r4300->interp_PC.addr += 8; \
         cp0_update_count(r4300); \
      } \
      r4300->cp0.last_addr = r4300->interp_PC.addr; \
      if (*r4300_cp0_cycle_count(&r4300->cp0) >= 0) gen_interrupt(r4300); \
   } \
   void name##_IDLE(struct r4300_core* r4300, uint32_t op) \
   { \
      uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0); \
      int* cp0_cycle_count = r4300_cp0_cycle_count(&r4300->cp0); \
      const int take_jump = (condition); \
      if (cop1 && check_cop1_unusable(r4300)) return; \
      if (take_jump) \
      { \
         cp0_update_count(r4300); \
         if(*cp0_cycle_count < 0) \
         { \
             cp0_regs[CP0_COUNT_REG] -= *cp0_cycle_count; \
             *cp0_cycle_count = 0; \
         } \
      } \
      name(r4300, op); \
   }

#define RD_OF(op)      (((op) >> 11) & 0x1F)
#define RS_OF(op)      (((op) >> 21) & 0x1F)
#define RT_OF(op)      (((op) >> 16) & 0x1F)
#define SA_OF(op)      (((op) >>  6) & 0x1F)
#define IMM16S_OF(op)  ((int16_t) (op))
#define IMM16U_OF(op)  ((uint16_t) (op))
#define FD_OF(op)      (((op) >>  6) & 0x1F)
#define FS_OF(op)      (((op) >> 11) & 0x1F)
#define FT_OF(op)      (((op) >> 16) & 0x1F)
#define JUMP_OF(op)    ((op) & UINT32_C(0x3FFFFFF))

/* Determines whether a relative jump in a 16-bit immediate goes back to the
 * same instruction without doing any work in its delay slot. The jump is
 * relative to the instruction in the delay slot, so 1 instruction backwards
 * (-1) goes back to the jump. */
#define IS_RELATIVE_IDLE_LOOP(r4300, op, addr) \
	(IMM16S_OF(op) == -1 && *fast_mem_access((r4300), (addr) + 4) == 0)

/* Determines whether an absolute jump in a 26-bit immediate goes back to the
 * same instruction without doing any work in its delay slot. The jump is
 * in the same 256 MiB segment as the delay slot, so if the jump instruction
 * is at the last address in its segment, it does not jump back to itself. */
#define IS_ABSOLUTE_IDLE_LOOP(r4300, op, addr) \
	(JUMP_OF(op) == ((addr) & UINT32_C(0x0FFFFFFF)) >> 2 \
	 && ((addr) & UINT32_C(0x0FFFFFFF)) != UINT32_C(0x0FFFFFFC) \
	 && *fast_mem_access((r4300), (addr) + 4) == 0)

/* These macros parse opcode fields. */
#define rrt r4300_regs(r4300)[RT_OF(op)]
#define rrd r4300_regs(r4300)[RD_OF(op)]
#define rfs FS_OF(op)
#define rrs r4300_regs(r4300)[RS_OF(op)]
#define rsa SA_OF(op)
#define irt r4300_regs(r4300)[RT_OF(op)]
#define ioffset IMM16S_OF(op)
#define iimmediate IMM16S_OF(op)
#define irs r4300_regs(r4300)[RS_OF(op)]
#define ibase r4300_regs(r4300)[RS_OF(op)]
#define jinst_index JUMP_OF(op)
#define lfbase RS_OF(op)
#define lfft FT_OF(op)
#define lfoffset IMM16S_OF(op)
#define cfft FT_OF(op)
#define cffs FS_OF(op)
#define cffd FD_OF(op)

// 32 bits macros
#ifndef M64P_BIG_ENDIAN
#define rrt32 *((int32_t*) &r4300_regs(r4300)[RT_OF(op)])
#define rrd32 *((int32_t*) &r4300_regs(r4300)[RD_OF(op)])
#define rrs32 *((int32_t*) &r4300_regs(r4300)[RS_OF(op)])
#define irs32 *((int32_t*) &r4300_regs(r4300)[RS_OF(op)])
#define irt32 *((int32_t*) &r4300_regs(r4300)[RT_OF(op)])
#else
#define rrt32 *((int32_t*) &r4300_regs(r4300)[RT_OF(op)] + 1)
#define rrd32 *((int32_t*) &r4300_regs(r4300)[RD_OF(op)] + 1)
#define rrs32 *((int32_t*) &r4300_regs(r4300)[RS_OF(op)] + 1)
#define irs32 *((int32_t*) &r4300_regs(r4300)[RS_OF(op)] + 1)
#define irt32 *((int32_t*) &r4300_regs(r4300)[RT_OF(op)] + 1)
#endif

// two functions are defined from the macros above but never used
// these prototype declarations will prevent a warning
#if defined(__GNUC__)
  void JR_IDLE(struct r4300_core*, uint32_t) __attribute__((used));
  void JALR_IDLE(struct r4300_core*, uint32_t) __attribute__((used));
#endif

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Mupen64plus - mips_instructions.def                                   *
 *   Mupen64Plus homepage: https://mupen64plus.org/                        *
 *   Copyright (C) 2002 Hacktarux                                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

/* Before #including this file the following macros must be defined:
 *
 * instruction operands accessor macros:
 * rrt, rrd, rfs, rrs, rsa,
 * rrt32, rrd32, rrs32, irs32, irt32,
 * irt, irs, ibase, ioffset, iimmediate,
 * jinst_index,
 * lfbase, lfft, lfoffset,
 * cfft, cffs, cffd
 *
 * DECLARE_R4300: An optionnal way of declaring teh r4300 pointer used
 * in instructions definitions
 * PCADDR: Program counter (memory address of the current instruction).
 *
 * ADD_TO_PC(x): Increment the program counter in 'x' instructions.
 * This is only used for small changes to PC, so the new program counter
 * is guaranteed to fall in the current cached interpreter or dynarec block.
 *
 * DECLARE_INSTRUCTION(name)
 * Declares an instruction function which is not a jump.
 * Followed by a block of code.
 *
 * DECLARE_JUMP(name, destination, condition, link, likely, cop1)
 * name is the name of the jump or branch instruction.
 * destination is the destination memory address of the jump.
 * If condition is nonzero, the jump is taken.
 * link is a pointer to a variable where (PC+8) is written unconditionally.
 *     To avoid linking, pass &reg[0]
 * If likely is nonzero, the delay slot is only executed if the jump is taken.
 * If cop1 is nonzero, a COP1 unusable check will be done.
 */

#include "fpu.h"
#include "r4300_core.h"
#include "device/memory.h"
#include "device/rcp/mi_controller.h"
#include "device/rdram.h"
#include "osal/preproc.h"

#include <inttypes.h>
#include <stdint.h>

/* Assists unaligned memory accessors with making masks to preserve or apply
 * bits in registers and memory.
 *
 * BITS_BELOW_MASK32 and BITS_BELOW_MASK64 make masks where bits 0 to (x - 1)
 * are set.
 *
 * BITS_ABOVE_MASK32 makes masks where bits x to 31 are set.
 * BITS_ABOVE_MASK64 makes masks where bits x to 63 are set.
 *
 * e.g. x = 8
 * 0000 0000 0000 0000 0000 0000 1111 1111 <- BITS_BELOW_MASK32(8)
 * 1111 1111 1111 1111 1111 1111 0000 0000 <- BITS_ABOVE_MASK32(8)
 *
 * Giving a negative value or one that is >= the bit count of the mask results
 * in undefined behavior.
 */

#define BITS_BELOW_MASK32(x) ((UINT32_C(1) << (x)) - 1)
#define BITS_ABOVE_MASK32(x) (~(BITS_BELOW_MASK32((x))))

#define BITS_BELOW_MASK64(x) ((UINT64_C(1) << (x)) - 1)
#define BITS_ABOVE_MASK64(x) (~(BITS_BELOW_MASK64((x))))


static unsigned int bshift(uint32_t address)
{
    return ((address & 3) ^ 3) << 3;
}

static unsigned int hshift(uint32_t address)
{
    return ((address & 2) ^ 2) << 3;
}


/* M64P Pseudo instructions */

DECLARE_INSTRUCTION(NI)
{
    DECLARE_R4300
    DebugMessage(M64MSG_ERROR, "NI() @ 0x%" PRIX32, PCADDR);
    DebugMessage(M64MSG_ERROR, "opcode not implemented: %" PRIX32 ":%" PRIX32, PCADDR, *fast_mem_access(r4300, PCADDR));
    *r4300_stop(r4300) = 1;
}

/* Reserved */

DECLARE_INSTRUCTION(RESERVED)
{
    DECLARE_R4300
    DebugMessage(M64MSG_ERROR, "reserved opcode: %" PRIX32 ":%" PRIX32, PCADDR, *fast_mem_access(r4300, PCADDR));
    *r4300_stop(r4300) = 1;
}

/* Load instructions */

DECLARE_INSTRUCTION(LB)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    uint32_t value;
    unsigned int shift = bshift(lsaddr);

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = SE8((value >> shift) & 0xff);
    }
}

DECLARE_INSTRUCTION(LBU)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    uint32_t value;
    unsigned int shift = bshift(lsaddr);

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = (value >> shift) & 0xff;
    }
}

DECLARE_INSTRUCTION(LH)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    uint32_t value;
    unsigned int shift = hshift(lsaddr);

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = SE16((value >> shift) & 0xffff);
    }
}

DECLARE_INSTRUCTION(LHU)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    uint32_t value;
    unsigned int shift = hshift(lsaddr);

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = (value >> shift) & 0xffff;
    }
}

DECLARE_INSTRUCTION(LL)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    uint32_t value;

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = SE32(value);
        r4300->llbit = 1;
    }
}

DECLARE_INSTRUCTION(LW)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    uint32_t value;

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = SE32(value);
    }
}

DECLARE_INSTRUCTION(LWU)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    uint32_t value;

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = value;
    }
}

DECLARE_INSTRUCTION(LWL)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 3);
    unsigned int shift = 8 * n;
    uint32_t mask = BITS_BELOW_MASK32(8 * n);
    uint32_t value;

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = SE32(((uint32_t)*lsrtp & mask) | ((uint32_t)value << shift));
    }
}

DECLARE_INSTRUCTION(LWR)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 3);
    unsigned int shift = 8 * (3 - n);
    uint32_t mask = (n == 3)
        ? UINT32_C(0)
        : BITS_ABOVE_MASK32(8 * (n + 1));
    uint32_t value;

    if (r4300_read_aligned_word(r4300, lsaddr, &value)) {
        *lsrtp = SE32(((uint32_t)*lsrtp & mask) | ((uint32_t)value >> shift));
    }
}

DECLARE_INSTRUCTION(LD)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    r4300_read_aligned_dword(r4300, lsaddr, (uint64_t*)lsrtp);
}

DECLARE_INSTRUCTION(LDL)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 7);
    unsigned int shift = 8 * n;
    uint64_t mask = BITS_BELOW_MASK64(8 * n);
    uint64_t value;

    if (r4300_read_aligned_dword(r4300, lsaddr & ~UINT32_C(7), &value)) {
        *lsrtp = ((uint64_t)*lsrtp & mask) | (value << shift);
    }
}

DECLARE_INSTRUCTION(LDR)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 7);
    unsigned int shift = 8 * (7 - n);
    uint64_t mask = (n == 7)
        ? UINT64_C(0)
        : BITS_ABOVE_MASK64(8 * (n + 1));
    uint64_t value;

    if (r4300_read_aligned_dword(r4300, lsaddr & ~UINT32_C(7), &value)) {
        *lsrtp = ((uint64_t)*lsrtp & mask) | (value >> shift);
    }
}

/* Store instructions */

DECLARE_INSTRUCTION(SB)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    unsigned int shift = bshift(lsaddr);

    r4300_write_aligned_word(r4300, lsaddr, (uint32_t)*lsrtp << shift, UINT32_C(0xff) << shift);
}

DECLARE_INSTRUCTION(SH)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);
    unsigned int shift = hshift(lsaddr);

    r4300_write_aligned_word(r4300, lsaddr, (uint32_t)*lsrtp << shift, UINT32_C(0xffff) << shift);
}

DECLARE_INSTRUCTION(SC)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    if (r4300->llbit)
    {
        if (r4300_write_aligned_word(r4300, lsaddr, (uint32_t)*lsrtp, ~UINT32_C(0))) {
            r4300->llbit = 0;
            *lsrtp = 1;
        }
    }
    else
    {
        *lsrtp = 0;
    }
}

DECLARE_INSTRUCTION(SW)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    r4300_write_aligned_word(r4300, lsaddr, (uint32_t)*lsrtp, ~UINT32_C(0));
}

DECLARE_INSTRUCTION(SWL)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 3);
    unsigned int shift = 8 * n;
    uint32_t mask = (n == 0)
        ? ~UINT32_C(0)
        : BITS_BELOW_MASK32(8 * (4 - n));
    uint32_t value = (uint32_t)*lsrtp;

    r4300_write_aligned_word(r4300, lsaddr & ~UINT32_C(0x3), value >> shift, mask);
}

DECLARE_INSTRUCTION(SWR)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 3);
    unsigned int shift = 8 * (3 - n);
    uint32_t mask = BITS_ABOVE_MASK32(8 * (3 - n));
    uint32_t value = (uint32_t)*lsrtp;

    r4300_write_aligned_word(r4300, lsaddr & ~UINT32_C(0x3), value << shift, mask);
}

DECLARE_INSTRUCTION(SD)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    r4300_write_aligned_dword(r4300, lsaddr, (uint64_t)*lsrtp, ~UINT64_C(0));
}

DECLARE_INSTRUCTION(SDL)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 7);
    unsigned int shift = 8 * n;
    uint64_t mask = (n == 0)
        ? ~UINT64_C(0)
        : BITS_BELOW_MASK64(8 * (8 - n));
    uint64_t value = (uint64_t)*lsrtp;

    r4300_write_aligned_dword(r4300, lsaddr & ~UINT32_C(0x7), value >> shift, mask);
}

DECLARE_INSTRUCTION(SDR)
{
    DECLARE_R4300
    const uint32_t lsaddr = (uint32_t) irs32 + (uint32_t) iimmediate;
    int64_t *lsrtp = &irt;
    ADD_TO_PC(1);

    unsigned int n = (lsaddr & 7);
    unsigned int shift = 8 * (7 - n);
    uint64_t mask = BITS_ABOVE_MASK64(8 * (7 - n));
    uint64_t value = (uint64_t)*lsrtp;

    r4300_write_aligned_dword(r4300, lsaddr & ~UINT32_C(0x7), value << shift, mask);
}

/* Computational instructions */

DECLARE_INSTRUCTION(ADD)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrs32 + (uint32_t) rrt32);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ADDU)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrs32 + (uint32_t) rrt32);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ADDI)
{
    DECLARE_R4300
    irt = SE32((uint32_t) irs32 + (uint32_t) iimmediate);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ADDIU)
{
    DECLARE_R4300
    irt = SE32((uint32_t) irs32 + (uint32_t) iimmediate);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DADD)
{
    DECLARE_R4300
    rrd = (uint64_t) rrs + (uint64_t) rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DADDU)
{
    DECLARE_R4300
    rrd = (uint64_t) rrs + (uint64_t) rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DADDI)
{
    DECLARE_R4300
    irt = (uint64_t) irs + (uint64_t) iimmediate;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DADDIU)
{
    DECLARE_R4300
    irt = (uint64_t) irs + (uint64_t) iimmediate;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SUB)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrs32 - (uint32_t) rrt32);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SUBU)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrs32 - (uint32_t) rrt32);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSUB)
{
    DECLARE_R4300
    rrd = (uint64_t) rrs - (uint64_t) rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSUBU)
{
    DECLARE_R4300
    rrd = (uint64_t) rrs - (uint64_t) rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SLT)
{
    DECLARE_R4300
    if (rrs < rrt) { rrd = 1; }
    else { rrd = 0; }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SLTU)
{
    DECLARE_R4300
    if ((uint64_t) rrs < (uint64_t) rrt) { rrd = 1; }
    else { rrd = 0; }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SLTI)
{
    DECLARE_R4300
    if (irs < iimmediate) { irt = 1; }
    else { irt = 0; }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SLTIU)
{
    DECLARE_R4300
    if ((uint64_t) irs < (uint64_t) ((int64_t) iimmediate)) { irt = 1; }
    else { irt = 0; }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(AND)
{
    DECLARE_R4300
    rrd = rrs & rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ANDI)
{
    DECLARE_R4300
    irt = irs & (uint16_t) iimmediate;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(OR)
{
    DECLARE_R4300
    rrd = rrs | rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ORI)
{
    DECLARE_R4300
    irt = irs | (uint16_t) iimmediate;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(XOR)
{
    DECLARE_R4300
    rrd = rrs ^ rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(XORI)
{
    DECLARE_R4300
    irt = irs ^ (uint16_t) iimmediate;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(NOR)
{
    DECLARE_R4300
    rrd = ~(rrs | rrt);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(LUI)
{
    DECLARE_R4300
    irt = SE32((uint32_t) iimmediate << 16);
    ADD_TO_PC(1);
}

/* Shift instructions */

DECLARE_INSTRUCTION(NOP)
{
    DECLARE_R4300
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SLL)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrt32 << rsa);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SLLV)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrt32 << (rrs32 & 0x1F));
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSLL)
{
    DECLARE_R4300
    rrd = (uint64_t) rrt << rsa;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSLLV)
{
    DECLARE_R4300
    rrd = (uint64_t) rrt << (rrs32 & 0x3F);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSLL32)
{
    DECLARE_R4300
    rrd = (uint64_t) rrt << (32 + rsa);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SRL)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrt32 >> rsa);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SRLV)
{
    DECLARE_R4300
    rrd = SE32((uint32_t) rrt32 >> (rrs32 & 0x1F));
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSRL)
{
    DECLARE_R4300
    rrd = (uint64_t) rrt >> rsa;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSRLV)
{
    DECLARE_R4300
    rrd = (uint64_t) rrt >> (rrs32 & 0x3F);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSRL32)
{
    DECLARE_R4300
    rrd = (uint64_t) rrt >> (32 + rsa);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SRA)
{
    DECLARE_R4300
    rrd = SE32((int32_t) rrt32 >> rsa);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SRAV)
{
    DECLARE_R4300
    rrd = SE32((int32_t) rrt32 >> (rrs32 & 0x1F));
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSRA)
{
    DECLARE_R4300
    rrd = rrt >> rsa;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSRAV)
{
    DECLARE_R4300
    rrd = (int64_t) rrt >> (rrs32 & 0x3F);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DSRA32)
{
    DECLARE_R4300
    rrd = (int64_t) rrt >> (32 + rsa);
    ADD_TO_PC(1);
}

/* Multiply / Divide instructions */

DECLARE_INSTRUCTION(MULT)
{
    DECLARE_R4300
    int64_t temp;
    temp = rrs32 * (int64_t)rrt32;
    *r4300_mult_hi(r4300) = temp >> 32;
    *r4300_mult_lo(r4300) = SE32(temp);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MULTU)
{
    DECLARE_R4300
    uint64_t temp;
    temp = (uint32_t) rrs * (uint64_t) ((uint32_t) rrt);
    *r4300_mult_hi(r4300) = (int64_t) temp >> 32;
    *r4300_mult_lo(r4300) = SE32(temp);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DMULT)
{
    DECLARE_R4300
    uint64_t op1, op2, op3, op4;
    uint64_t result1, result2, result3, result4;
    uint64_t temp1, temp2, temp3, temp4;
    int sign = 0;

    if (rrs < 0)
    {
        op2 = -rrs;
        sign = 1 - sign;
    }
    else { op2 = rrs; }
    if (rrt < 0)
    {
        op4 = -rrt;
        sign = 1 - sign;
    }
    else { op4 = rrt; }

    op1 = op2 & UINT64_C(0xFFFFFFFF);
    op2 = (op2 >> 32) & UINT64_C(0xFFFFFFFF);
    op3 = op4 & UINT64_C(0xFFFFFFFF);
    op4 = (op4 >> 32) & UINT64_C(0xFFFFFFFF);

    temp1 = op1 * op3;
    temp2 = (temp1 >> 32) + op1 * op4;
    temp3 = op2 * op3;
    temp4 = (temp3 >> 32) + op2 * op4;

    result1 = temp1 & UINT64_C(0xFFFFFFFF);
    result2 = temp2 + (temp3 & UINT64_C(0xFFFFFFFF));
    result3 = (result2 >> 32) + temp4;
    result4 = (result3 >> 32);

    *r4300_mult_lo(r4300) = result1 | (result2 << 32);
    *r4300_mult_hi(r4300) = (result3 & UINT64_C(0xFFFFFFFF)) | (result4 << 32);
    if (sign)
    {
        *r4300_mult_hi(r4300) = ~*r4300_mult_hi(r4300);
        if (!*r4300_mult_lo(r4300)) { (*r4300_mult_hi(r4300))++; }
        else { *r4300_mult_lo(r4300) = ~*r4300_mult_lo(r4300) + 1; }
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DMULTU)
{
    DECLARE_R4300
    uint64_t op1, op2, op3, op4;
    uint64_t result1, result2, result3, result4;
    uint64_t temp1, temp2, temp3, temp4;

    op1 = rrs & UINT64_C(0xFFFFFFFF);
    op2 = (rrs >> 32) & UINT64_C(0xFFFFFFFF);
    op3 = rrt & UINT64_C(0xFFFFFFFF);
    op4 = (rrt >> 32) & UINT64_C(0xFFFFFFFF);

    temp1 = op1 * op3;
    temp2 = (temp1 >> 32) + op1 * op4;
    temp3 = op2 * op3;
    temp4 = (temp3 >> 32) + op2 * op4;

    result1 = temp1 & UINT64_C(0xFFFFFFFF);
    result2 = temp2 + (temp3 & UINT64_C(0xFFFFFFFF));
    result3 = (result2 >> 32) + temp4;
    result4 = (result3 >> 32);

    *r4300_mult_lo(r4300) = result1 | (result2 << 32);
    *r4300_mult_hi(r4300) = (result3 & UINT64_C(0xFFFFFFFF)) | (result4 << 32);

    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DIV)
{
    DECLARE_R4300
    if (rrt32)
    {
        if (rrs32 == INT32_MIN && rrt32 == -1)
        {
            *r4300_mult_lo(r4300) = SE32(rrs32);
            *r4300_mult_hi(r4300) = 0;
        }
        else
        {
            *r4300_mult_lo(r4300) = SE32(rrs32 / rrt32);
            *r4300_mult_hi(r4300) = SE32(rrs32 % rrt32);
        }
    }
    else
    {
        *r4300_mult_lo(r4300) = rrs32 < 0 ? 1 : -1;
        *r4300_mult_hi(r4300) = SE32(rrs32);
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DIVU)
{
    DECLARE_R4300
    if (rrt32)
    {
        *r4300_mult_lo(r4300) = SE32((uint32_t) rrs32 / (uint32_t) rrt32);
        *r4300_mult_hi(r4300) = SE32((uint32_t) rrs32 % (uint32_t) rrt32);
    }
    else
    {
        *r4300_mult_lo(r4300) = -1;
        *r4300_mult_hi(r4300) = SE32(rrs32);
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DDIV)
{
    DECLARE_R4300
    if (rrt)
    {
        if (rrs == INT64_MIN && rrt == -1)
        {
            *r4300_mult_lo(r4300) = rrs;
            *r4300_mult_hi(r4300) = 0;
        }
        else
        {
            *r4300_mult_lo(r4300) = rrs / rrt;
            *r4300_mult_hi(r4300) = rrs % rrt;
        }
    }
    else
    {
        *r4300_mult_lo(r4300) = rrs < 0 ? 1 : -1;
        *r4300_mult_hi(r4300) = rrs;
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DDIVU)
{
    DECLARE_R4300
    if (rrt)
    {
        *r4300_mult_lo(r4300) = (uint64_t) rrs / (uint64_t) rrt;
        *r4300_mult_hi(r4300) = (uint64_t) rrs % (uint64_t) rrt;
    }
    else
    {
        *r4300_mult_lo(r4300) = -1;
        *r4300_mult_hi(r4300) = rrs;
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MFHI)
{
    DECLARE_R4300
    rrd = *r4300_mult_hi(r4300);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MTHI)
{
    DECLARE_R4300
    *r4300_mult_hi(r4300) = rrs;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MFLO)
{
    DECLARE_R4300
    rrd = *r4300_mult_lo(r4300);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MTLO)
{
    DECLARE_R4300
    *r4300_mult_lo(r4300) = rrs;
    ADD_TO_PC(1);
}

/* Jump & Branch instructions */

DECLARE_JUMP(J,   (jinst_index<<2) | ((PCADDR+4) & UINT32_C(0xF0000000)), 1, &r4300_regs(r4300)[0],  0, 0)
DECLARE_JUMP(JAL, (jinst_index<<2) | ((PCADDR+4) & UINT32_C(0xF0000000)), 1, &r4300_regs(r4300)[31], 0, 0)

DECLARE_JUMP(JR,   irs32, 1, &r4300_regs(r4300)[0], 0, 0)
DECLARE_JUMP(JALR, irs32, 1, &rrd,    0, 0)

DECLARE_JUMP(BEQ,     PCADDR + (iimmediate+1)*4, irs == irt, &r4300_regs(r4300)[0], 0, 0)
DECLARE_JUMP(BEQL,    PCADDR + (iimmediate+1)*4, irs == irt, &r4300_regs(r4300)[0], 1, 0)

DECLARE_JUMP(BNE,     PCADDR + (iimmediate+1)*4, irs != irt, &r4300_regs(r4300)[0], 0, 0)
DECLARE_JUMP(BNEL,    PCADDR + (iimmediate+1)*4, irs != irt, &r4300_regs(r4300)[0], 1, 0)

DECLARE_JUMP(BLEZ,    PCADDR + (iimmediate+1)*4, irs <= 0,   &r4300_regs(r4300)[0], 0, 0)
DECLARE_JUMP(BLEZL,   PCADDR + (iimmediate+1)*4, irs <= 0,   &r4300_regs(r4300)[0], 1, 0)

DECLARE_JUMP(BGTZ,    PCADDR + (iimmediate+1)*4, irs > 0,    &r4300_regs(r4300)[0], 0, 0)
DECLARE_JUMP(BGTZL,   PCADDR + (iimmediate+1)*4, irs > 0,    &r4300_regs(r4300)[0], 1, 0)

DECLARE_JUMP(BLTZ,    PCADDR + (iimmediate+1)*4, irs < 0,    &r4300_regs(r4300)[0],  0, 0)
DECLARE_JUMP(BLTZAL,  PCADDR + (iimmediate+1)*4, irs < 0,    &r4300_regs(r4300)[31], 0, 0)
DECLARE_JUMP(BLTZL,   PCADDR + (iimmediate+1)*4, irs < 0,    &r4300_regs(r4300)[0],  1, 0)
DECLARE_JUMP(BLTZALL, PCADDR + (iimmediate+1)*4, irs < 0,    &r4300_regs(r4300)[31], 1, 0)

DECLARE_JUMP(BGEZ,    PCADDR + (iimmediate+1)*4, irs >= 0,   &r4300_regs(r4300)[0],  0, 0)
DECLARE_JUMP(BGEZAL,  PCADDR + (iimmediate+1)*4, irs >= 0,   &r4300_regs(r4300)[31], 0, 0)
DECLARE_JUMP(BGEZL,   PCADDR + (iimmediate+1)*4, irs >= 0,   &r4300_regs(r4300)[0],  1, 0)
DECLARE_JUMP(BGEZALL, PCADDR + (iimmediate+1)*4, irs >= 0,   &r4300_regs(r4300)[31], 1, 0)

DECLARE_JUMP(BC1F,  PCADDR + (iimmediate+1)*4, ((*r4300_cp1_fcr31(&r4300->cp1)) & FCR31_CMP_BIT)==0, &r4300_regs(r4300)[0], 0, 1)
DECLARE_JUMP(BC1FL, PCADDR + (iimmediate+1)*4, ((*r4300_cp1_fcr31(&r4300->cp1)) & FCR31_CMP_BIT)==0, &r4300_regs(r4300)[0], 1, 1)
DECLARE_JUMP(BC1T,  PCADDR + (iimmediate+1)*4, ((*r4300_cp1_fcr31(&r4300->cp1)) & FCR31_CMP_BIT)!=0, &r4300_regs(r4300)[0], 0, 1)
DECLARE_JUMP(BC1TL, PCADDR + (iimmediate+1)*4, ((*r4300_cp1_fcr31(&r4300->cp1)) & FCR31_CMP_BIT)!=0, &r4300_regs(r4300)[0], 1, 1)

/* Special instructions */

DECLARE_INSTRUCTION(CACHE)
{
    DECLARE_R4300
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ERET)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);
    int* cp0_cycle_count = r4300_cp0_cycle_count(&r4300->cp0);

    cp0_update_count(r4300);
    if (cp0_regs[CP0_STATUS_REG] & CP0_STATUS_ERL)
    {
        DebugMessage(M64MSG_ERROR, "error in ERET");
        *r4300_stop(r4300)=1;
    }
    else
    {
        cp0_regs[CP0_STATUS_REG] &= ~CP0_STATUS_EXL;
        generic_jump_to(r4300, cp0_regs[CP0_EPC_REG]);
    }
    r4300->llbit = 0;
    r4300_check_interrupt(r4300, CP0_CAUSE_IP2, r4300->mi->regs[MI_INTR_REG] & r4300->mi->regs[MI_INTR_MASK_REG]); // ???
    r4300->cp0.last_addr = PCADDR;
    if (*cp0_cycle_count >= 0) { gen_interrupt(r4300); }
}

DECLARE_INSTRUCTION(SYNC)
{
    DECLARE_R4300
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SYSCALL)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);

    cp0_regs[CP0_CAUSE_REG] = CP0_CAUSE_EXCCODE_SYS;
    exception_general(r4300);
}

/* Trap instructions */

#define DECLARE_TRAP(name, cond) \
DECLARE_INSTRUCTION(name) \
{ \
    DECLARE_R4300 \
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0); \
    if (cond) \
    { \
        cp0_regs[CP0_CAUSE_REG] = CP0_CAUSE_EXCCODE_TR; \
        exception_general(r4300); \
    } \
    else \
    { \
        ADD_TO_PC(1); \
    } \
}

DECLARE_TRAP(TGE, rrs >= rrt)
DECLARE_TRAP(TGEU, (uint64_t) rrs >= (uint64_t) rrt)
DECLARE_TRAP(TGEI, rrs >= (int64_t)iimmediate)
DECLARE_TRAP(TGEIU, (uint64_t)rrs >= (uint64_t)(int64_t)iimmediate)
DECLARE_TRAP(TLT, rrs < rrt)
DECLARE_TRAP(TLTU, (uint64_t) rrs < (uint64_t) rrt)
DECLARE_TRAP(TLTI, rrs < (int64_t)iimmediate)
DECLARE_TRAP(TLTIU, (uint64_t)rrs < (uint64_t)(int64_t)iimmediate)
DECLARE_TRAP(TEQ, rrs == rrt)
DECLARE_TRAP(TEQI, rrs == (int64_t)iimmediate)
DECLARE_TRAP(TNE, rrs != rrt)
DECLARE_TRAP(TNEI, rrs != (int64_t)iimmediate)

#undef DECLARE_TRAP

/* TLB instructions */

DECLARE_INSTRUCTION(TLBP)
{
    DECLARE_R4300
    int i;
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);

    cp0_regs[CP0_INDEX_REG] |= UINT32_C(0x80000000);
    for (i = 0; i < 32; ++i)
    {
        if (((r4300->cp0.tlb.entries[i].vpn2 & (~r4300->cp0.tlb.entries[i].mask)) ==
                    (((cp0_regs[CP0_ENTRYHI_REG] & UINT32_C(0xFFFFE000)) >> 13) & (~r4300->cp0.tlb.entries[i].mask))) &&
                ((r4300->cp0.tlb.entries[i].g) ||
                 (r4300->cp0.tlb.entries[i].asid == (cp0_regs[CP0_ENTRYHI_REG] & UINT32_C(0xFF)))))
        {
            cp0_regs[CP0_INDEX_REG] = i;
            break;
        }
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(TLBR)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);

    int index;
    index = cp0_regs[CP0_INDEX_REG] & UINT32_C(0x1F);
    cp0_regs[CP0_PAGEMASK_REG] = r4300->cp0.tlb.entries[index].mask << 13;
    cp0_regs[CP0_ENTRYHI_REG] = ((r4300->cp0.tlb.entries[index].vpn2 << 13) | r4300->cp0.tlb.entries[index].asid);
    cp0_regs[CP0_ENTRYLO0_REG] = (r4300->cp0.tlb.entries[index].pfn_even << 6) | (r4300->cp0.tlb.entries[index].c_even << 3)
        | (r4300->cp0.tlb.entries[index].d_even << 2) | (r4300->cp0.tlb.entries[index].v_even << 1)
        | r4300->cp0.tlb.entries[index].g;
    cp0_regs[CP0_ENTRYLO1_REG] = (r4300->cp0.tlb.entries[index].pfn_odd << 6) | (r4300->cp0.tlb.entries[index].c_odd << 3)
        | (r4300->cp0.tlb.entries[index].d_odd << 2) | (r4300->cp0.tlb.entries[index].v_odd << 1)
        | r4300->cp0.tlb.entries[index].g;
    ADD_TO_PC(1);
}

static void TLBWrite(struct r4300_core* r4300, unsigned int idx)
{
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);
    uint32_t pc_addr = *r4300_pc(r4300);

    if (pc_addr >= r4300->cp0.tlb.entries[idx].start_even && pc_addr < r4300->cp0.tlb.entries[idx].end_even && r4300->cp0.tlb.entries[idx].v_even)
        return;
    if (pc_addr >= r4300->cp0.tlb.entries[idx].start_odd && pc_addr < r4300->cp0.tlb.entries[idx].end_odd && r4300->cp0.tlb.entries[idx].v_odd)
        return;

    tlb_unmap(&r4300->cp0.tlb, idx);

    r4300->cp0.tlb.entries[idx].g = (cp0_regs[CP0_ENTRYLO0_REG] & cp0_regs[CP0_ENTRYLO1_REG] & 1);
    r4300->cp0.tlb.entries[idx].pfn_even = (cp0_regs[CP0_ENTRYLO0_REG] & UINT32_C(0x3FFFFFC0)) >> 6;
    r4300->cp0.tlb.entries[idx].pfn_odd = (cp0_regs[CP0_ENTRYLO1_REG] & UINT32_C(0x3FFFFFC0)) >> 6;
    r4300->cp0.tlb.entries[idx].c_even = (cp0_regs[CP0_ENTRYLO0_REG] & UINT32_C(0x38)) >> 3;
    r4300->cp0.tlb.entries[idx].c_odd = (cp0_regs[CP0_ENTRYLO1_REG] & UINT32_C(0x38)) >> 3;
    r4300->cp0.tlb.entries[idx].d_even = (cp0_regs[CP0_ENTRYLO0_REG] & UINT32_C(0x4)) >> 2;
    r4300->cp0.tlb.entries[idx].d_odd = (cp0_regs[CP0_ENTRYLO1_REG] & UINT32_C(0x4)) >> 2;
    r4300->cp0.tlb.entries[idx].v_even = (cp0_regs[CP0_ENTRYLO0_REG] & UINT32_C(0x2)) >> 1;
    r4300->cp0.tlb.entries[idx].v_odd = (cp0_regs[CP0_ENTRYLO1_REG] & UINT32_C(0x2)) >> 1;
    r4300->cp0.tlb.entries[idx].asid = (cp0_regs[CP0_ENTRYHI_REG] & UINT32_C(0xFF));
    r4300->cp0.tlb.entries[idx].vpn2 = (cp0_regs[CP0_ENTRYHI_REG] & UINT32_C(0xFFFFE000)) >> 13;
    //r4300->cp0.tlb.entries[idx].r = (cp0_regs[CP0_ENTRYHI_REG] & 0xC000000000000000LL) >> 62;
    r4300->cp0.tlb.entries[idx].mask = (cp0_regs[CP0_PAGEMASK_REG] & UINT32_C(0x1FFE000)) >> 13;

    r4300->cp0.tlb.entries[idx].start_even = r4300->cp0.tlb.entries[idx].vpn2 << 13;
    r4300->cp0.tlb.entries[idx].end_even = r4300->cp0.tlb.entries[idx].start_even+
        (r4300->cp0.tlb.entries[idx].mask << 12) + UINT32_C(0xFFF);
    r4300->cp0.tlb.entries[idx].phys_even = r4300->cp0.tlb.entries[idx].pfn_even << 12;


    r4300->cp0.tlb.entries[idx].start_odd = r4300->cp0.tlb.entries[idx].end_even+1;
    r4300->cp0.tlb.entries[idx].end_odd = r4300->cp0.tlb.entries[idx].start_odd+
        (r4300->cp0.tlb.entries[idx].mask << 12) + UINT32_C(0xFFF);
    r4300->cp0.tlb.entries[idx].phys_odd = r4300->cp0.tlb.entries[idx].pfn_odd << 12;

    tlb_map(&r4300->cp0.tlb, idx);
}

DECLARE_INSTRUCTION(TLBWR)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);
    cp0_update_count(r4300);
    cp0_regs[CP0_RANDOM_REG] = (cp0_regs[CP0_COUNT_REG]/r4300->cp0.count_per_op % (32 - cp0_regs[CP0_WIRED_REG]))
        + cp0_regs[CP0_WIRED_REG];
    TLBWrite(r4300, cp0_regs[CP0_RANDOM_REG]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(TLBWI)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);

    TLBWrite(r4300, cp0_regs[CP0_INDEX_REG] & UINT32_C(0x3F));
    ADD_TO_PC(1);
}

/* CP0 load/store instructions */

DECLARE_INSTRUCTION(MFC0)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);

    switch(rfs)
    {
    case CP0_RANDOM_REG:
        cp0_update_count(r4300);
        cp0_regs[CP0_RANDOM_REG] = (cp0_regs[CP0_COUNT_REG]/r4300->cp0.count_per_op % (32 - cp0_regs[CP0_WIRED_REG]))
            + cp0_regs[CP0_WIRED_REG];
        break;
    case CP0_COUNT_REG:
        cp0_update_count(r4300);
        break;
    }
    rrt = SE32(cp0_regs[rfs]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MTC0)
{
    DECLARE_R4300
    uint32_t* cp0_regs = r4300_cp0_regs(&r4300->cp0);
    int* cp0_cycle_count = r4300_cp0_cycle_count(&r4300->cp0);

    switch(rfs)
    {
    case CP0_INDEX_REG:
        cp0_regs[CP0_INDEX_REG] = rrt32 & UINT32_C(0x8000003F);
        if ((cp0_regs[CP0_INDEX_REG] & UINT32_C(0x3F)) > UINT32_C(31))
        {
            DebugMessage(M64MSG_ERROR, "MTC0 instruction writing Index register with TLB index > 31");
            *r4300_stop(r4300)=1;
        }
        break;
    case CP0_RANDOM_REG:
        break;
    case CP0_ENTRYLO0_REG:
        cp0_regs[CP0_ENTRYLO0_REG] = rrt32 & UINT32_C(0x3FFFFFFF);
        break;
    case CP0_ENTRYLO1_REG:
        cp0_regs[CP0_ENTRYLO1_REG] = rrt32 & UINT32_C(0x3FFFFFFF);
        break;
    case CP0_CONTEXT_REG:
        cp0_regs[CP0_CONTEXT_REG] = (rrt32 & UINT32_C(0xFF800000))
            | (cp0_regs[CP0_CONTEXT_REG] & UINT32_C(0x007FFFF0));
        break;
    case CP0_PAGEMASK_REG:
        cp0_regs[CP0_PAGEMASK_REG] = rrt32 & UINT32_C(0x01FFE000);
        break;
    case CP0_WIRED_REG:
        cp0_regs[CP0_WIRED_REG] = rrt32;
        cp0_regs[CP0_RANDOM_REG] = UINT32_C(31);
        break;
    case CP0_BADVADDR_REG:
        break;
    case CP0_COUNT_REG:
        cp0_update_count(r4300);
        r4300->cp0.interrupt_unsafe_state |= INTR_UNSAFE_R4300;
        if (*cp0_cycle_count >= 0) { gen_interrupt(r4300); }
        r4300->cp0.interrupt_unsafe_state &= ~INTR_UNSAFE_R4300;
        translate_event_queue(&r4300->cp0, rrt32);
        break;
    case CP0_ENTRYHI_REG:
        cp0_regs[CP0_ENTRYHI_REG] = rrt32 & UINT32_C(0xFFFFE0FF);
        break;
    case CP0_COMPARE_REG:
        cp0_update_count(r4300);
        remove_event(&r4300->cp0.q, COMPARE_INT);

        /* Add count_per_op to avoid wrong event order in case CP0_COUNT_REG == CP0_COMPARE_REG */
        cp0_regs[CP0_COUNT_REG] += r4300->cp0.count_per_op;
        *cp0_cycle_count += r4300->cp0.count_per_op;
        add_interrupt_event_count(&r4300->cp0, COMPARE_INT, rrt32);
        cp0_regs[CP0_COUNT_REG] -= r4300->cp0.count_per_op;

        /* Update next interrupt in case first event is COMPARE_INT */
        *cp0_cycle_count = cp0_regs[CP0_COUNT_REG] - r4300->cp0.q.first->data.count;
        cp0_regs[CP0_COMPARE_REG] = rrt32;
        cp0_regs[CP0_CAUSE_REG] &= ~CP0_CAUSE_IP7;
        break;
    case CP0_STATUS_REG:
        if((rrt32 & CP0_STATUS_FR) != (cp0_regs[CP0_STATUS_REG] & CP0_STATUS_FR))
            set_fpr_pointers(&r4300->cp1, rrt32);

        cp0_regs[CP0_STATUS_REG] = rrt32;
        ADD_TO_PC(1);
        cp0_update_count(r4300);
        r4300_check_interrupt(r4300, CP0_CAUSE_IP2, r4300->mi->regs[MI_INTR_REG] & r4300->mi->regs[MI_INTR_MASK_REG]); // ???
        r4300->cp0.interrupt_unsafe_state |= INTR_UNSAFE_R4300;
        if (*cp0_cycle_count >= 0) { gen_interrupt(r4300); }
        r4300->cp0.interrupt_unsafe_state &= ~INTR_UNSAFE_R4300;
        return;
    case CP0_CAUSE_REG:
        cp0_regs[CP0_CAUSE_REG] &= ~(CP0_CAUSE_IP0 | CP0_CAUSE_IP1);
        cp0_regs[CP0_CAUSE_REG] |= rrt32 & (CP0_CAUSE_IP0 | CP0_CAUSE_IP1);
        break;
    case CP0_EPC_REG:
        cp0_regs[CP0_EPC_REG] = rrt32;
        break;
    case CP0_PREVID_REG:
        break;
    case CP0_CONFIG_REG:
        cp0_regs[CP0_CONFIG_REG] = rrt32;
        break;
    case CP0_WATCHLO_REG:
        cp0_regs[CP0_WATCHLO_REG] = rrt32;
        break;
    case CP0_WATCHHI_REG:
        cp0_regs[CP0_WATCHHI_REG] = rrt32;
        break;
    case CP0_TAGLO_REG:
        cp0_regs[CP0_TAGLO_REG] = rrt32 & UINT32_C(0x0FFFFFC0);
        break;
    case CP0_TAGHI_REG:
        cp0_regs[CP0_TAGHI_REG] = 0;
        break;
    case CP0_ERROREPC_REG:
        cp0_regs[CP0_ERROREPC_REG] = rrt32;
        break;
    default:
        DebugMessage(M64MSG_ERROR, "Unknown MTC0 write: %d", rfs);
        *r4300_stop(r4300)=1;
    }
    ADD_TO_PC(1);
}

/* CP1 load/store instructions */

DECLARE_INSTRUCTION(LWC1)
{
    DECLARE_R4300
    const unsigned char lslfft = lfft;
    const uint32_t lslfaddr = (uint32_t) r4300_regs(r4300)[lfbase] + lfoffset;
    if (check_cop1_unusable(r4300)) { return; }
    ADD_TO_PC(1);

    r4300_read_aligned_word(r4300, lslfaddr, (uint32_t*)r4300_cp1_regs_simple(&r4300->cp1)[lslfft]);
}

DECLARE_INSTRUCTION(LDC1)
{
    DECLARE_R4300
    const unsigned char lslfft = lfft;
    const uint32_t lslfaddr = (uint32_t) r4300_regs(r4300)[lfbase] + lfoffset;
    if (check_cop1_unusable(r4300)) { return; }
    ADD_TO_PC(1);

    r4300_read_aligned_dword(r4300, lslfaddr, (uint64_t*)r4300_cp1_regs_double(&r4300->cp1)[lslfft]);
}

DECLARE_INSTRUCTION(SWC1)
{
    DECLARE_R4300
    const unsigned char lslfft = lfft;
    const uint32_t lslfaddr = (uint32_t) r4300_regs(r4300)[lfbase] + lfoffset;
    if (check_cop1_unusable(r4300)) { return; }
    ADD_TO_PC(1);

    r4300_write_aligned_word(r4300, lslfaddr, *((uint32_t*)(r4300_cp1_regs_simple(&r4300->cp1))[lslfft]), ~UINT32_C(0));
}

DECLARE_INSTRUCTION(SDC1)
{
    DECLARE_R4300
    const unsigned char lslfft = lfft;
    const uint32_t lslfaddr = (uint32_t) r4300_regs(r4300)[lfbase] + lfoffset;
    if (check_cop1_unusable(r4300)) { return; }
    ADD_TO_PC(1);

    r4300_write_aligned_dword(r4300, lslfaddr, *((uint64_t*)(r4300_cp1_regs_double(&r4300->cp1))[lslfft]), ~UINT64_C(0));
}

DECLARE_INSTRUCTION(MFC1)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    rrt = SE32(*((int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[rfs]));
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DMFC1)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    rrt = *((int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[rfs]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CFC1)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (rfs==31)
    {
        rrt = SE32((*r4300_cp1_fcr31(&r4300->cp1)));
    }
    if (rfs==0)
    {
        rrt = SE32((*r4300_cp1_fcr0(&r4300->cp1)));
    }
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MTC1)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    *((int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[rfs]) = rrt32;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DMTC1)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    *((int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[rfs]) = rrt;
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CTC1)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (rfs==31)
    {
        (*r4300_cp1_fcr31(&r4300->cp1)) = rrt32;
        update_x86_rounding_mode(&r4300->cp1);
    }
    //if (((*r4300_cp1_fcr31(&r4300->cp1)) >> 7) & 0x1F) printf("FPU Exception enabled : %x\n",
    //                 (int)(((*r4300_cp1_fcr31(&r4300->cp1)) >> 7) & 0x1F));
    ADD_TO_PC(1);
}

/* CP1 computational instructions */

DECLARE_INSTRUCTION(ABS_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    abs_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ABS_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    abs_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ADD_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    add_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ADD_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    add_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DIV_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if(((*r4300_cp1_fcr31(&r4300->cp1)) & UINT32_C(0x400)) && *(r4300_cp1_regs_simple(&r4300->cp1))[cfft] == 0)
    {
        DebugMessage(M64MSG_ERROR, "DIV_S by 0");
    }
    div_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(DIV_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if(((*r4300_cp1_fcr31(&r4300->cp1)) & UINT32_C(0x400)) && *(r4300_cp1_regs_double(&r4300->cp1))[cfft] == 0)
    {
        //(*r4300_cp1_fcr31(&r4300->cp1)) |= 0x8020;
        /*(*r4300_cp1_fcr31(&r4300->cp1)) |= 0x8000;
          Cause = 15 << 2;
          exception_general(r4300);*/
        DebugMessage(M64MSG_ERROR, "DIV_D by 0");
        //return;
    }
    div_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MOV_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    mov_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MOV_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    mov_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MUL_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    mul_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(MUL_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    mul_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(NEG_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    neg_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(NEG_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    neg_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SQRT_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    sqrt_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SQRT_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    sqrt_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SUB_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    sub_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(SUB_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    sub_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(TRUNC_W_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    trunc_w_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(TRUNC_W_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    trunc_w_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(TRUNC_L_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    trunc_l_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(TRUNC_L_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    trunc_l_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ROUND_W_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    round_w_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ROUND_W_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    round_w_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ROUND_L_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    round_l_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(ROUND_L_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    round_l_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CEIL_W_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    ceil_w_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CEIL_W_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    ceil_w_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CEIL_L_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    ceil_l_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CEIL_L_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    ceil_l_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(FLOOR_W_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    floor_w_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(FLOOR_W_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    floor_w_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(FLOOR_L_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    floor_l_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(FLOOR_L_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    floor_l_d((r4300_cp1_regs_double(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_S_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_s_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_S_W)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_s_w(*r4300_cp1_fcr31(&r4300->cp1), (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_S_L)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_s_l(*r4300_cp1_fcr31(&r4300->cp1), (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_D_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_d_s((r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_D_W)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_d_w((int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_D_L)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_d_l(*r4300_cp1_fcr31(&r4300->cp1), (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_W_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_w_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_W_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_w_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (int32_t*) (r4300_cp1_regs_simple(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_L_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_l_s(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(CVT_L_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    cvt_l_d(*r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (int64_t*) (r4300_cp1_regs_double(&r4300->cp1))[cffd]);
    ADD_TO_PC(1);
}

/* CP1 relational instructions */

DECLARE_INSTRUCTION(C_F_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_f_s(r4300_cp1_fcr31(&r4300->cp1));
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_F_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_f_d(r4300_cp1_fcr31(&r4300->cp1));
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_UN_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_un_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_UN_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_un_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_EQ_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_eq_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_EQ_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_eq_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_UEQ_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ueq_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_UEQ_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ueq_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_OLT_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_olt_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_OLT_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_olt_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_ULT_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ult_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_ULT_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ult_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_OLE_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ole_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_OLE_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ole_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_ULE_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ule_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_ULE_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    c_ule_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_SF_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_sf_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_SF_D)
{
    DECLARE_R4300
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_sf_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGLE_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_ngle_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGLE_D)
{
    DECLARE_R4300
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_ngle_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_SEQ_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_seq_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_SEQ_D)
{
    DECLARE_R4300
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_seq_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGL_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_ngl_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGL_D)
{
    DECLARE_R4300
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_ngl_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_LT_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_lt_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_LT_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_lt_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGE_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_nge_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGE_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_nge_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_LE_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_le_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_LE_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_le_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGT_S)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_simple(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_ngt_s(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_simple(&r4300->cp1))[cffs], (r4300_cp1_regs_simple(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}

DECLARE_INSTRUCTION(C_NGT_D)
{
    DECLARE_R4300
    if (check_cop1_unusable(r4300)) { return; }
    if (isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cffs]) || isnan(*(r4300_cp1_regs_double(&r4300->cp1))[cfft]))
    {
        DebugMessage(M64MSG_ERROR, "Invalid operation exception in C opcode");
        *r4300_stop(r4300)=1;
    }
    c_ngt_d(r4300_cp1_fcr31(&r4300->cp1), (r4300_cp1_regs_double(&r4300->cp1))[cffs], (r4300_cp1_regs_double(&r4300->cp1))[cfft]);
    ADD_TO_PC(1);
}


bool breakloop;
extern void main_check_inputs(void);
void new_vi(void)
{
    // apply_speed_limiter();
    main_check_inputs();
    breakloop=true;
}

void InterpretOpcode(struct r4300_core* r4300)
{
    while(!breakloop)
	{

	
    loop1:

	uint32_t* op_address = fast_mem_access(r4300, *r4300_pc(r4300));
	if (op_address == NULL)
		return;
	uint32_t op = *op_address;

	switch ((op >> 26) & 0x3F) {
	case 0: /* SPECIAL prefix */
		switch (op & 0x3F) {
		case 0: /* SPECIAL opcode 0: SLL */
			if (RD_OF(op) != 0) SLL(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 2: /* SPECIAL opcode 2: SRL */
			if (RD_OF(op) != 0) SRL(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 3: /* SPECIAL opcode 3: SRA */
			if (RD_OF(op) != 0) SRA(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 4: /* SPECIAL opcode 4: SLLV */
			if (RD_OF(op) != 0) SLLV(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 6: /* SPECIAL opcode 6: SRLV */
			if (RD_OF(op) != 0) SRLV(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 7: /* SPECIAL opcode 7: SRAV */
			if (RD_OF(op) != 0) SRAV(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 8: JR(r4300, op); break;
		case 9: /* SPECIAL opcode 9: JALR */
			/* Note: This can omit the check for Rd == 0 because the JALR
			 * function checks for link_register != &r4300_regs(4300)[0]. If you're
			 * using this as a reference for a JIT, do check Rd == 0 in it. */
			JALR(r4300, op);
			break;
		case 12: SYSCALL(r4300, op); break;
		case 13: /* SPECIAL opcode 13: BREAK (Not implemented) */
			NI(r4300, op);
			break;
		case 15: SYNC(r4300, op); break;
		case 16: /* SPECIAL opcode 16: MFHI */
			if (RD_OF(op) != 0) MFHI(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 17: MTHI(r4300, op); break;
		case 18: /* SPECIAL opcode 18: MFLO */
			if (RD_OF(op) != 0) MFLO(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 19: MTLO(r4300, op); break;
		case 20: /* SPECIAL opcode 20: DSLLV */
			if (RD_OF(op) != 0) DSLLV(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 22: /* SPECIAL opcode 22: DSRLV */
			if (RD_OF(op) != 0) DSRLV(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 23: /* SPECIAL opcode 23: DSRAV */
			if (RD_OF(op) != 0) DSRAV(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 24: MULT(r4300, op); break;
		case 25: MULTU(r4300, op); break;
		case 26: DIV(r4300, op); break;
		case 27: DIVU(r4300, op); break;
		case 28: DMULT(r4300, op); break;
		case 29: DMULTU(r4300, op); break;
		case 30: DDIV(r4300, op); break;
		case 31: DDIVU(r4300, op); break;
		case 32: /* SPECIAL opcode 32: ADD */
			if (RD_OF(op) != 0) ADD(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 33: /* SPECIAL opcode 33: ADDU */
			if (RD_OF(op) != 0) ADDU(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 34: /* SPECIAL opcode 34: SUB */
			if (RD_OF(op) != 0) SUB(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 35: /* SPECIAL opcode 35: SUBU */
			if (RD_OF(op) != 0) SUBU(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 36: /* SPECIAL opcode 36: AND */
			if (RD_OF(op) != 0) AND(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 37: /* SPECIAL opcode 37: OR */
			if (RD_OF(op) != 0) OR(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 38: /* SPECIAL opcode 38: XOR */
			if (RD_OF(op) != 0) XOR(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 39: /* SPECIAL opcode 39: NOR */
			if (RD_OF(op) != 0) NOR(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 42: /* SPECIAL opcode 42: SLT */
			if (RD_OF(op) != 0) SLT(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 43: /* SPECIAL opcode 43: SLTU */
			if (RD_OF(op) != 0) SLTU(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 44: /* SPECIAL opcode 44: DADD */
			if (RD_OF(op) != 0) DADD(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 45: /* SPECIAL opcode 45: DADDU */
			if (RD_OF(op) != 0) DADDU(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 46: /* SPECIAL opcode 46: DSUB */
			if (RD_OF(op) != 0) DSUB(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 47: /* SPECIAL opcode 47: DSUBU */
			if (RD_OF(op) != 0) DSUBU(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 48: TGE(r4300, op); break;
		case 49: TGEU(r4300, op); break;
		case 50: TLT(r4300, op); break;
		case 51: TLTU(r4300, op); break;
		case 52: TEQ(r4300, op); break;
		case 54: TNE(r4300, op); break;
		case 56: /* SPECIAL opcode 56: DSLL */
			if (RD_OF(op) != 0) DSLL(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 58: /* SPECIAL opcode 58: DSRL */
			if (RD_OF(op) != 0) DSRL(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 59: /* SPECIAL opcode 59: DSRA */
			if (RD_OF(op) != 0) DSRA(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 60: /* SPECIAL opcode 60: DSLL32 */
			if (RD_OF(op) != 0) DSLL32(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 62: /* SPECIAL opcode 62: DSRL32 */
			if (RD_OF(op) != 0) DSRL32(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 63: /* SPECIAL opcode 63: DSRA32 */
			if (RD_OF(op) != 0) DSRA32(r4300, op);
			else                NOP(r4300, 0);
			break;
		default: /* SPECIAL opcodes 1, 5, 10, 11, 14, 21, 40, 41, 53, 55, 57,
		            61: Reserved Instructions */
			RESERVED(r4300, op);
			break;
		} /* switch (op & 0x3F) for the SPECIAL prefix */
		break;
	case 1: /* REGIMM prefix */
		switch ((op >> 16) & 0x1F) {
		case 0: /* REGIMM opcode 0: BLTZ */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BLTZ_IDLE(r4300, op);
			else                                             BLTZ(r4300, op);
			break;
		case 1: /* REGIMM opcode 1: BGEZ */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BGEZ_IDLE(r4300, op);
			else                                             BGEZ(r4300, op);
			break;
		case 2: /* REGIMM opcode 2: BLTZL */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BLTZL_IDLE(r4300, op);
			else                                             BLTZL(r4300, op);
			break;
		case 3: /* REGIMM opcode 3: BGEZL */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BGEZL_IDLE(r4300, op);
			else                                             BGEZL(r4300, op);
			break;
		case 8: TGEI(r4300, op); break;
		case 9: TGEIU(r4300, op); break;
		case 10: TLTI(r4300, op); break;
		case 11: TLTIU(r4300, op); break;
		case 12: TEQI(r4300, op); break;
		case 14: TNEI(r4300, op); break;
		case 16: /* REGIMM opcode 16: BLTZAL */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BLTZAL_IDLE(r4300, op);
			else                                             BLTZAL(r4300, op);
			break;
		case 17: /* REGIMM opcode 17: BGEZAL */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BGEZAL_IDLE(r4300, op);
			else                                             BGEZAL(r4300, op);
			break;
		case 18: /* REGIMM opcode 18: BLTZALL */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BLTZALL_IDLE(r4300, op);
			else                                             BLTZALL(r4300, op);
			break;
		case 19: /* REGIMM opcode 19: BGEZALL */
			if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BGEZALL_IDLE(r4300, op);
			else                                             BGEZALL(r4300, op);
			break;
		default: /* REGIMM opcodes 4..7, 13, 15, 20..31:
		            Reserved Instructions */
			RESERVED(r4300, op);
			break;
		} /* switch ((op >> 16) & 0x1F) for the REGIMM prefix */
		break;
	case 2: /* Major opcode 2: J */
		if (IS_ABSOLUTE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) J_IDLE(r4300, op);
		else                                             J(r4300, op);
		break;
	case 3: /* Major opcode 3: JAL */
		if (IS_ABSOLUTE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) JAL_IDLE(r4300, op);
		else                                             JAL(r4300, op);
		break;
	case 4: /* Major opcode 4: BEQ */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BEQ_IDLE(r4300, op);
		else                                             BEQ(r4300, op);
		break;
	case 5: /* Major opcode 5: BNE */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BNE_IDLE(r4300, op);
		else                                             BNE(r4300, op);
		break;
	case 6: /* Major opcode 6: BLEZ */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BLEZ_IDLE(r4300, op);
		else                                             BLEZ(r4300, op);
		break;
	case 7: /* Major opcode 7: BGTZ */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BGTZ_IDLE(r4300, op);
		else                                             BGTZ(r4300, op);
		break;
	case 8: /* Major opcode 8: ADDI */
		if (RT_OF(op) != 0) ADDI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 9: /* Major opcode 9: ADDIU */
		if (RT_OF(op) != 0) ADDIU(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 10: /* Major opcode 10: SLTI */
		if (RT_OF(op) != 0) SLTI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 11: /* Major opcode 11: SLTIU */
		if (RT_OF(op) != 0) SLTIU(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 12: /* Major opcode 12: ANDI */
		if (RT_OF(op) != 0) ANDI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 13: /* Major opcode 13: ORI */
		if (RT_OF(op) != 0) ORI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 14: /* Major opcode 14: XORI */
		if (RT_OF(op) != 0) XORI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 15: /* Major opcode 15: LUI */
		if (RT_OF(op) != 0) LUI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 16: /* Coprocessor 0 prefix */
		switch ((op >> 21) & 0x1F) {
		case 0: /* Coprocessor 0 opcode 0: MFC0 */
			if (RT_OF(op) != 0) MFC0(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 4: MTC0(r4300, op); break;
		case 16: /* Coprocessor 0 opcode 16: TLB */
			switch (op & 0x3F) {
			case 1: TLBR(r4300, op); break;
			case 2: TLBWI(r4300, op); break;
			case 6: TLBWR(r4300, op); break;
			case 8: TLBP(r4300, op); break;
			case 24: ERET(r4300, op); break;
			default: /* TLB sub-opcodes 0, 3..5, 7, 9..23, 25..63:
			            Reserved Instructions */
				RESERVED(r4300, op);
				break;
			} /* switch (op & 0x3F) for Coprocessor 0 TLB opcodes */
			break;
		default: /* Coprocessor 0 opcodes 1..3, 4..15, 17..31:
		            Reserved Instructions */
			RESERVED(r4300, op);
			break;
		} /* switch ((op >> 21) & 0x1F) for the Coprocessor 0 prefix */
		break;
	case 17: /* Coprocessor 1 prefix */
		switch ((op >> 21) & 0x1F) {
		case 0: /* Coprocessor 1 opcode 0: MFC1 */
			if (RT_OF(op) != 0) MFC1(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 1: /* Coprocessor 1 opcode 1: DMFC1 */
			if (RT_OF(op) != 0) DMFC1(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 2: /* Coprocessor 1 opcode 2: CFC1 */
			if (RT_OF(op) != 0) CFC1(r4300, op);
			else                NOP(r4300, 0);
			break;
		case 4: MTC1(r4300, op); break;
		case 5: DMTC1(r4300, op); break;
		case 6: CTC1(r4300, op); break;
		case 8: /* Coprocessor 1 opcode 8: Branch on C1 condition... */
			switch ((op >> 16) & 0x3) {
			case 0: /* opcode 0: BC1F */
				if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BC1F_IDLE(r4300, op);
				else                                             BC1F(r4300, op);
				break;
			case 1: /* opcode 1: BC1T */
				if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BC1T_IDLE(r4300, op);
				else                                             BC1T(r4300, op);
				break;
			case 2: /* opcode 2: BC1FL */
				if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BC1FL_IDLE(r4300, op);
				else                                             BC1FL(r4300, op);
				break;
			case 3: /* opcode 3: BC1TL */
				if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BC1TL_IDLE(r4300, op);
				else                                             BC1TL(r4300, op);
				break;
			} /* switch ((op >> 16) & 0x3) for branches on C1 condition */
			break;
		case 16: /* Coprocessor 1 S-format opcodes */
			switch (op & 0x3F) {
			case 0: ADD_S(r4300, op); break;
			case 1: SUB_S(r4300, op); break;
			case 2: MUL_S(r4300, op); break;
			case 3: DIV_S(r4300, op); break;
			case 4: SQRT_S(r4300, op); break;
			case 5: ABS_S(r4300, op); break;
			case 6: MOV_S(r4300, op); break;
			case 7: NEG_S(r4300, op); break;
			case 8: ROUND_L_S(r4300, op); break;
			case 9: TRUNC_L_S(r4300, op); break;
			case 10: CEIL_L_S(r4300, op); break;
			case 11: FLOOR_L_S(r4300, op); break;
			case 12: ROUND_W_S(r4300, op); break;
			case 13: TRUNC_W_S(r4300, op); break;
			case 14: CEIL_W_S(r4300, op); break;
			case 15: FLOOR_W_S(r4300, op); break;
			case 33: CVT_D_S(r4300, op); break;
			case 36: CVT_W_S(r4300, op); break;
			case 37: CVT_L_S(r4300, op); break;
			case 48: C_F_S(r4300, op); break;
			case 49: C_UN_S(r4300, op); break;
			case 50: C_EQ_S(r4300, op); break;
			case 51: C_UEQ_S(r4300, op); break;
			case 52: C_OLT_S(r4300, op); break;
			case 53: C_ULT_S(r4300, op); break;
			case 54: C_OLE_S(r4300, op); break;
			case 55: C_ULE_S(r4300, op); break;
			case 56: C_SF_S(r4300, op); break;
			case 57: C_NGLE_S(r4300, op); break;
			case 58: C_SEQ_S(r4300, op); break;
			case 59: C_NGL_S(r4300, op); break;
			case 60: C_LT_S(r4300, op); break;
			case 61: C_NGE_S(r4300, op); break;
			case 62: C_LE_S(r4300, op); break;
			case 63: C_NGT_S(r4300, op); break;
			default: /* Coprocessor 1 S-format opcodes 16..32, 34..35, 38..47:
			            Reserved Instructions */
				RESERVED(r4300, op);
				break;
			} /* switch (op & 0x3F) for Coprocessor 1 S-format opcodes */
			break;
		case 17: /* Coprocessor 1 D-format opcodes */
			switch (op & 0x3F) {
			case 0: ADD_D(r4300, op); break;
			case 1: SUB_D(r4300, op); break;
			case 2: MUL_D(r4300, op); break;
			case 3: DIV_D(r4300, op); break;
			case 4: SQRT_D(r4300, op); break;
			case 5: ABS_D(r4300, op); break;
			case 6: MOV_D(r4300, op); break;
			case 7: NEG_D(r4300, op); break;
			case 8: ROUND_L_D(r4300, op); break;
			case 9: TRUNC_L_D(r4300, op); break;
			case 10: CEIL_L_D(r4300, op); break;
			case 11: FLOOR_L_D(r4300, op); break;
			case 12: ROUND_W_D(r4300, op); break;
			case 13: TRUNC_W_D(r4300, op); break;
			case 14: CEIL_W_D(r4300, op); break;
			case 15: FLOOR_W_D(r4300, op); break;
			case 32: CVT_S_D(r4300, op); break;
			case 36: CVT_W_D(r4300, op); break;
			case 37: CVT_L_D(r4300, op); break;
			case 48: C_F_D(r4300, op); break;
			case 49: C_UN_D(r4300, op); break;
			case 50: C_EQ_D(r4300, op); break;
			case 51: C_UEQ_D(r4300, op); break;
			case 52: C_OLT_D(r4300, op); break;
			case 53: C_ULT_D(r4300, op); break;
			case 54: C_OLE_D(r4300, op); break;
			case 55: C_ULE_D(r4300, op); break;
			case 56: C_SF_D(r4300, op); break;
			case 57: C_NGLE_D(r4300, op); break;
			case 58: C_SEQ_D(r4300, op); break;
			case 59: C_NGL_D(r4300, op); break;
			case 60: C_LT_D(r4300, op); break;
			case 61: C_NGE_D(r4300, op); break;
			case 62: C_LE_D(r4300, op); break;
			case 63: C_NGT_D(r4300, op); break;
			default: /* Coprocessor 1 D-format opcodes 16..31, 33..35, 38..47:
			            Reserved Instructions */
				RESERVED(r4300, op);
				break;
			} /* switch (op & 0x3F) for Coprocessor 1 D-format opcodes */
			break;
		case 20: /* Coprocessor 1 W-format opcodes */
			switch (op & 0x3F) {
			case 32: CVT_S_W(r4300, op); break;
			case 33: CVT_D_W(r4300, op); break;
			default: /* Coprocessor 1 W-format opcodes 0..31, 34..63:
			            Reserved Instructions */
				RESERVED(r4300, op);
				break;
			}
			break;
		case 21: /* Coprocessor 1 L-format opcodes */
			switch (op & 0x3F) {
			case 32: CVT_S_L(r4300, op); break;
			case 33: CVT_D_L(r4300, op); break;
			default: /* Coprocessor 1 L-format opcodes 0..31, 34..63:
			            Reserved Instructions */
				RESERVED(r4300, op);
				break;
			}
			break;
		default: /* Coprocessor 1 opcodes 3, 7, 9..15, 18..19, 22..31:
		            Reserved Instructions */
			RESERVED(r4300, op);
			break;
		} /* switch ((op >> 21) & 0x1F) for the Coprocessor 1 prefix */
		break;
	case 20: /* Major opcode 20: BEQL */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BEQL_IDLE(r4300, op);
		else                                             BEQL(r4300, op);
		break;
	case 21: /* Major opcode 21: BNEL */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BNEL_IDLE(r4300, op);
		else                                             BNEL(r4300, op);
		break;
	case 22: /* Major opcode 22: BLEZL */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BLEZL_IDLE(r4300, op);
		else                                             BLEZL(r4300, op);
		break;
	case 23: /* Major opcode 23: BGTZL */
		if (IS_RELATIVE_IDLE_LOOP(r4300, op, *r4300_pc(r4300))) BGTZL_IDLE(r4300, op);
		else                                             BGTZL(r4300, op);
		break;
	case 24: /* Major opcode 24: DADDI */
		if (RT_OF(op) != 0) DADDI(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 25: /* Major opcode 25: DADDIU */
		if (RT_OF(op) != 0) DADDIU(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 26: /* Major opcode 26: LDL */
		if (RT_OF(op) != 0) LDL(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 27: /* Major opcode 27: LDR */
		if (RT_OF(op) != 0) LDR(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 32: /* Major opcode 32: LB */
		if (RT_OF(op) != 0) LB(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 33: /* Major opcode 33: LH */
		if (RT_OF(op) != 0) LH(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 34: /* Major opcode 34: LWL */
		if (RT_OF(op) != 0) LWL(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 35: /* Major opcode 35: LW */
		if (RT_OF(op) != 0) LW(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 36: /* Major opcode 36: LBU */
		if (RT_OF(op) != 0) LBU(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 37: /* Major opcode 37: LHU */
		if (RT_OF(op) != 0) LHU(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 38: /* Major opcode 38: LWR */
		if (RT_OF(op) != 0) LWR(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 39: /* Major opcode 39: LWU */
		if (RT_OF(op) != 0) LWU(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 40: SB(r4300, op); break;
	case 41: SH(r4300, op); break;
	case 42: SWL(r4300, op); break;
	case 43: SW(r4300, op); break;
	case 44: SDL(r4300, op); break;
	case 45: SDR(r4300, op); break;
	case 46: SWR(r4300, op); break;
	case 47: CACHE(r4300, op); break;
	case 48: /* Major opcode 48: LL */
		if (RT_OF(op) != 0) LL(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 49: LWC1(r4300, op); break;
	case 52: /* Major opcode 52: LLD (Not implemented) */
		NI(r4300, op);
		break;
	case 53: LDC1(r4300, op); break;
	case 55: /* Major opcode 55: LD */
		if (RT_OF(op) != 0) LD(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 56: /* Major opcode 56: SC */
		if (RT_OF(op) != 0) SC(r4300, op);
		else                NOP(r4300, 0);
		break;
	case 57: SWC1(r4300, op); break;
	case 60: /* Major opcode 60: SCD (Not implemented) */
		NI(r4300, op);
		break;
	case 61: SDC1(r4300, op); break;
	case 63: SD(r4300, op); break;
	default: /* Major opcodes 18..19, 28..31, 50..51, 54, 58..59, 62:
	            Reserved Instructions */
		RESERVED(r4300, op);
		break;
	} /* switch ((op >> 26) & 0x3F) */

	}
}

void run_r4300(struct r4300_core* r4300)
{

#ifdef OSAL_SSE
    //Save FTZ/DAZ mode
    unsigned int daz = _MM_GET_DENORMALS_ZERO_MODE();
    unsigned int ftz = _MM_GET_FLUSH_ZERO_MODE();
    _MM_SET_DENORMALS_ZERO_MODE(_MM_DENORMALS_ZERO_OFF);
#endif
	breakloop=false;
    if(r4300->startup)
	{
    *r4300_stop(r4300) = 0;
    r4300->emumode = EMUMODE_PURE_INTERPRETER;
   *r4300_pc_struct(r4300) = &r4300->interp_PC;
   *r4300_pc(r4300) = r4300->cp0.last_addr = r4300->start_address;
   r4300->startup=0;
	}

    while(!breakloop)
     InterpretOpcode(r4300);

	 #ifdef OSAL_SSE
    //Restore FTZ/DAZ mode
    _MM_SET_DENORMALS_ZERO_MODE(daz);
    _MM_SET_FLUSH_ZERO_MODE(ftz);
#endif
}
