#!/usr/bin/env python3
"""
HP OfficeJet "Eridani" ARM926EJ-S Emulator
==========================================
Emulates the boot sequence of the HP OfficeJet printer firmware
from the SPI flash dump.

Features:
- ARM32 instruction set emulation (ARM + Thumb)
- Memory-mapped I/O peripherals (UART, USB, GPIO)
- CP15 coprocessor for cache/MMU configuration
- UART output capture (boot messages)
- Configurable trace/debug output

Usage: python emulator.py [options]
  --trace     Enable instruction trace
  --max N     Max instructions to execute (default: 500000)
  --dump      Dump registers every N instructions
"""

import struct
import sys
import os
import argparse
import time

# ============================================================
# Constants
# ============================================================
FLASH_FILE = 'readResult_GenericSPI_2026-28-2-14-13-42.bin'
FLASH_SIZE = 0x100000   # Use first 1MB copy
RAM_BASE = 0x20000000
RAM_SIZE = 0x00100000   # 1MB SRAM
PERIPH_BASE = 0x40000000

# ARM Registers
R0, R1, R2, R3, R4, R5, R6, R7 = range(8)
R8, R9, R10, R11, R12, SP, LR, PC = range(8, 16)
CPSR = 16
REG_NAMES = ['R0','R1','R2','R3','R4','R5','R6','R7',
             'R8','R9','R10','R11','R12','SP','LR','PC','CPSR']

# ARM Condition codes
COND_EQ = 0
COND_NE = 1
COND_CS = 2
COND_CC = 3
COND_MI = 4
COND_PL = 5
COND_VS = 6
COND_VC = 7
COND_HI = 8
COND_LS = 9
COND_GE = 10
COND_LT = 11
COND_GT = 12
COND_LE = 13
COND_AL = 14

# CPSR flags
N_FLAG = 1 << 31
Z_FLAG = 1 << 30
C_FLAG = 1 << 29
V_FLAG = 1 << 28
T_FLAG = 1 << 5  # Thumb mode

# ============================================================
# Clock/PLL Controller Peripheral
# ============================================================
class ClockPLL:
    """Emulates the HP Eridani clock/PLL controller.
    
    Register map (16-bit halfword registers):
      +0x00: Clock source select
      +0x02: PLL multiplier
      +0x04: PLL divider
      +0x06: Clock divider
      +0x08: PLL enable / config
      +0x0A: Control / trigger (write triggers reconfiguration)
      +0x0C: Status register 1 (bit 0: PLL busy, 0=ready)
      +0x0E: Status register 2 (bits 0-1: PLL lock, both set=locked)
    """
    def __init__(self, name, base_addr):
        self.name = name
        self.base = base_addr
        self.regs = [0] * 8  # 8 x 16-bit registers
    
    def read(self, offset, size):
        idx = (offset & 0xF) >> 1
        if idx >= len(self.regs):
            return 0
        
        if offset & 0xF == 0x0C:
            # Status 1: bit 0 = busy. Always 0 (PLL ready instantly)
            return 0x0000
        elif offset & 0xF == 0x0E:
            # Status 2: bits 0-1 = lock. Both set = locked
            return 0x0003
        
        return self.regs[idx]
    
    def write(self, offset, value, size):
        idx = (offset & 0xF) >> 1
        if idx < len(self.regs):
            self.regs[idx] = value & 0xFFFF


# ============================================================
# UART Peripheral (HP Eridani)
# ============================================================
class EridaniUART:
    """UART peripheral - intercepts writes to capture boot messages.
    
    Reverse-engineered register map (16-bit halfword registers):
      +0x00: Status register (bit 8: TX complete, must be 1 for TX done)
      +0x02: TX data register (write a char here)
      +0x04: RX data register
      +0x06: Control register
      +0x08: Baud rate config
      +0x0A: Extended status
    """
    def __init__(self, name, base_addr):
        self.name = name
        self.base = base_addr
        self.output_lines = []
        self.current_line = ''
        self.tx_count = 0
        self.regs = {}
    
    def read(self, offset, size):
        off = offset & 0xFF
        if off == 0x0E:
            # Status register: bit 9 = TX busy (0 = ready, i.e. always ready)
            return 0x0000
        return self.regs.get(off, 0)
    
    def write(self, offset, value, size):
        off = offset & 0xFF
        self.regs[off] = value
        
        # TX data register at offset +0x06
        if off == 0x06:
            self.tx_count += 1
            # Try extracting char from low byte
            ch = value & 0xFF
            if ch == 0x0A or ch == 0x0D:
                if self.current_line:
                    self.output_lines.append(self.current_line)
                    print(f"[{self.name}] {self.current_line}")
                    self.current_line = ''
            elif 32 <= ch < 127:
                self.current_line += chr(ch)
            else:
                # Try high byte (in case of byte-swap)
                ch2 = (value >> 8) & 0xFF
                if ch2 == 0x0A or ch2 == 0x0D:
                    if self.current_line:
                        self.output_lines.append(self.current_line)
                        print(f"[{self.name}] {self.current_line}")
                        self.current_line = ''
                elif 32 <= ch2 < 127:
                    self.current_line += chr(ch2)
    
    def flush(self):
        if self.current_line:
            self.output_lines.append(self.current_line)
            print(f"[{self.name}] {self.current_line}")
            self.current_line = ''


# ============================================================
# Generic MMIO Region
# ============================================================
class GenericMMIO:
    """Generic memory-mapped I/O region. Stores writes, returns stored values."""
    def __init__(self, name, base_addr, size):
        self.name = name
        self.base = base_addr
        self.size = size
        self.regs = {}
    
    def read(self, offset, size):
        # I/O command register at base+0x400: bit 8 = command complete
        if offset == 0x400:
            return 0x0100
        return self.regs.get(offset, 0)
    
    def write(self, offset, value, size):
        self.regs[offset] = value


# ============================================================
# Memory System
# ============================================================
class Memory:
    """Memory system with regions and MMIO peripherals."""
    def __init__(self, big_endian_flash=True):
        self.regions = {}
        self.big_endian_flash = big_endian_flash
        self.io_log = []
        
        # Peripheral device map: (base_addr, size) -> device
        self.peripherals = {}
        
        # Clock/PLL controllers (reverse-engineered from boot code)
        self.clk0 = ClockPLL("CLK0", 0x20813500)
        self.clk1 = ClockPLL("CLK1", 0x20813A00)
        
        # UART - at 0x20812900 (reverse-engineered from putchar literal pool)
        #   +0x06: TX data (STRH char here)
        #   +0x0E: Status (bit 9 = TX busy, 0=ready)
        self.uart = EridaniUART("UART", 0x20812900)
        
        # Generic MMIO for other register blocks we've seen
        self.gpio = GenericMMIO("GPIO", 0x20813200, 0x100)
        self.iomux = GenericMMIO("IOMUX", 0x20812000, 0x900)  # Below UART
        self.misc = GenericMMIO("MISC", 0x2081B000, 0x1000)
        
        # Register all peripherals (order matters: more specific first)
        self._register_periph(0x20813500, 0x10, self.clk0)
        self._register_periph(0x20813A00, 0x10, self.clk1)
        self._register_periph(0x20812900, 0x100, self.uart)  # UART at 0x20812900
        self._register_periph(0x20813200, 0x100, self.gpio)
        self._register_periph(0x20812000, 0x900, self.iomux)  # Don't overlap UART
        self._register_periph(0x2081B000, 0x1000, self.misc)
    
    def _register_periph(self, base, size, device):
        self.peripherals[(base, size)] = device
    
    def _find_periph(self, addr):
        for (base, size), dev in self.peripherals.items():
            if base <= addr < base + size:
                return dev, addr - base
        return None, 0
    
    def add_region(self, base, size, data=None, name="unnamed", writable=True):
        buf = bytearray(size)
        if data:
            buf[:len(data)] = data[:size]
        self.regions[name] = {
            'base': base,
            'size': size,
            'data': buf,
            'writable': writable
        }
    
    def _find_region(self, addr):
        for name, reg in self.regions.items():
            if reg['base'] <= addr < reg['base'] + reg['size']:
                return name, reg
        return None, None
    
    def read32(self, addr):
        name, reg = self._find_region(addr)
        if reg:
            offset = addr - reg['base']
            if self.big_endian_flash and name in ('flash', 'flash_mirror'):
                return struct.unpack_from('>I', reg['data'], offset)[0]
            return struct.unpack_from('<I', reg['data'], offset)[0]
        return self._periph_read(addr, 4)
    
    def read16(self, addr):
        name, reg = self._find_region(addr)
        if reg:
            offset = addr - reg['base']
            if self.big_endian_flash and name in ('flash', 'flash_mirror'):
                return struct.unpack_from('>H', reg['data'], offset)[0]
            return struct.unpack_from('<H', reg['data'], offset)[0]
        return self._periph_read(addr, 2) & 0xFFFF
    
    def read8(self, addr):
        name, reg = self._find_region(addr)
        if reg:
            offset = addr - reg['base']
            return reg['data'][offset]
        return self._periph_read(addr, 1) & 0xFF
    
    def write32(self, addr, value):
        name, reg = self._find_region(addr)
        if reg and reg['writable']:
            offset = addr - reg['base']
            struct.pack_into('<I', reg['data'], offset, value & 0xFFFFFFFF)
            return
        self._periph_write(addr, value, 4)
    
    def write16(self, addr, value):
        name, reg = self._find_region(addr)
        if reg and reg['writable']:
            offset = addr - reg['base']
            struct.pack_into('<H', reg['data'], offset, value & 0xFFFF)
            return
        self._periph_write(addr, value, 2)
    
    def write8(self, addr, value):
        name, reg = self._find_region(addr)
        if reg and reg['writable']:
            offset = addr - reg['base']
            reg['data'][offset] = value & 0xFF
            return
        self._periph_write(addr, value, 1)
    
    def _periph_read(self, addr, size):
        """Dispatch peripheral reads to registered devices."""
        dev, offset = self._find_periph(addr)
        if dev:
            return dev.read(offset, size)
        
        # Fallback: return 0 for unknown peripherals
        return 0
    
    def _periph_write(self, addr, value, size):
        """Dispatch peripheral writes to registered devices."""
        dev, offset = self._find_periph(addr)
        if dev:
            dev.write(offset, value, size)
            return
        
        # Log writes to unknown peripherals
        self.io_log.append((addr, value, size, 'W'))

# ============================================================
# ARM CPU
# ============================================================
class ARM926:
    """ARM926EJ-S CPU emulator."""
    
    def __init__(self, memory):
        self.mem = memory
        self.regs = [0] * 17  # R0-R15 + CPSR
        self.regs[CPSR] = 0xD3  # Supervisor mode, IRQ/FIQ disabled
        self.cp15 = [0] * 16  # CP15 registers (simplified)
        self.cp15[0] = 0x41069265  # ARM926EJ-S Main ID
        self.halted = False
        self.trace = False
        self.instr_count = 0
        self.max_instr = 500000
        self.breakpoints = set()
        
        # Banked registers for different modes
        self.mode_regs = {}
    
    def reset(self):
        for i in range(16):
            self.regs[i] = 0
        self.regs[CPSR] = 0xD3  # SVC mode
        self.regs[PC] = 0  # Reset vector
        self.halted = False
        self.instr_count = 0
    
    @property
    def pc(self):
        return self.regs[PC]
    
    @pc.setter
    def pc(self, val):
        self.regs[PC] = val & 0xFFFFFFFF
    
    def get_flag(self, flag):
        return bool(self.regs[CPSR] & flag)
    
    def set_flag(self, flag, value):
        if value:
            self.regs[CPSR] |= flag
        else:
            self.regs[CPSR] &= ~flag
    
    def check_condition(self, cond):
        n = self.get_flag(N_FLAG)
        z = self.get_flag(Z_FLAG)
        c = self.get_flag(C_FLAG)
        v = self.get_flag(V_FLAG)
        
        conditions = {
            0: z,               # EQ
            1: not z,           # NE
            2: c,               # CS/HS
            3: not c,           # CC/LO
            4: n,               # MI
            5: not n,           # PL
            6: v,               # VS
            7: not v,           # VC
            8: c and not z,     # HI
            9: not c or z,      # LS
            10: n == v,         # GE
            11: n != v,         # LT
            12: not z and (n == v),  # GT
            13: z or (n != v),  # LE
            14: True,           # AL
            15: False,          # NV (never)
        }
        return conditions.get(cond, False)
    
    def alu_add(self, a, b, carry_in=0):
        result = a + b + carry_in
        result32 = result & 0xFFFFFFFF
        
        self.set_flag(N_FLAG, result32 & 0x80000000)
        self.set_flag(Z_FLAG, result32 == 0)
        self.set_flag(C_FLAG, result > 0xFFFFFFFF)
        # Overflow: positive + positive = negative, or negative + negative = positive
        a_sign = (a >> 31) & 1
        b_sign = (b >> 31) & 1
        r_sign = (result32 >> 31) & 1
        self.set_flag(V_FLAG, (a_sign == b_sign) and (a_sign != r_sign))
        
        return result32
    
    def alu_sub(self, a, b, carry_in=1):
        return self.alu_add(a, (~b) & 0xFFFFFFFF, carry_in)
    
    def barrel_shift(self, value, shift_type, amount, carry_in):
        """Barrel shifter: LSL, LSR, ASR, ROR"""
        if amount == 0:
            return value, carry_in
        
        if shift_type == 0:  # LSL
            if amount >= 32:
                carry = (value >> (32 - amount)) & 1 if amount == 32 else 0
                return 0, carry
            carry = (value >> (32 - amount)) & 1
            return (value << amount) & 0xFFFFFFFF, carry
        elif shift_type == 1:  # LSR
            if amount >= 32:
                carry = (value >> 31) & 1 if amount == 32 else 0
                return 0, carry
            carry = (value >> (amount - 1)) & 1
            return (value >> amount) & 0xFFFFFFFF, carry
        elif shift_type == 2:  # ASR
            if amount >= 32:
                if value & 0x80000000:
                    return 0xFFFFFFFF, 1
                else:
                    return 0, 0
            carry = (value >> (amount - 1)) & 1
            # Sign-extend
            if value & 0x80000000:
                result = (value >> amount) | (0xFFFFFFFF << (32 - amount))
            else:
                result = value >> amount
            return result & 0xFFFFFFFF, carry
        elif shift_type == 3:  # ROR
            amount = amount % 32
            if amount == 0:
                return value, carry_in
            result = ((value >> amount) | (value << (32 - amount))) & 0xFFFFFFFF
            carry = (result >> 31) & 1
            return result, carry
        
        return value, carry_in
    
    def decode_operand2(self, word, is_imm):
        """Decode operand 2 (immediate or register with shift)."""
        carry = self.get_flag(C_FLAG)
        
        if is_imm:
            imm8 = word & 0xFF
            rot = ((word >> 8) & 0xF) * 2
            if rot == 0:
                return imm8, carry
            result = ((imm8 >> rot) | (imm8 << (32 - rot))) & 0xFFFFFFFF
            carry = (result >> 31) & 1
            return result, carry
        else:
            rm = word & 0xF
            rm_val = self.regs[rm]
            if rm == PC:
                rm_val += 8  # Pipeline
            
            shift_type = (word >> 5) & 3
            
            if (word >> 4) & 1:  # Register shift
                rs = (word >> 8) & 0xF
                amount = self.regs[rs] & 0xFF
            else:  # Immediate shift
                amount = (word >> 7) & 0x1F
                # Special cases for LSR #0 and ASR #0
                if amount == 0:
                    if shift_type == 1:  # LSR #32
                        amount = 32
                    elif shift_type == 2:  # ASR #32
                        amount = 32
                    elif shift_type == 3:  # RRX
                        c_in = 1 if carry else 0
                        result = (c_in << 31) | (rm_val >> 1)
                        carry = rm_val & 1
                        return result & 0xFFFFFFFF, carry
            
            return self.barrel_shift(rm_val, shift_type, amount, carry)
    
    def execute_arm(self, word):
        """Execute one ARM instruction."""
        cond = (word >> 28) & 0xF
        if not self.check_condition(cond):
            self.pc += 4
            return
        
        bits27_25 = (word >> 25) & 7
        bit24 = (word >> 24) & 1
        bit23 = (word >> 23) & 1
        bit22 = (word >> 22) & 1
        bit21 = (word >> 21) & 1
        bit20 = (word >> 20) & 1
        bit4 = (word >> 4) & 1
        bit7 = (word >> 7) & 1
        
        rd = (word >> 12) & 0xF
        rn = (word >> 16) & 0xF
        rm = word & 0xF
        
        old_pc = self.pc
        
        # ---- Branch ----
        if bits27_25 == 0b101:
            link = bit24
            offset = word & 0x00FFFFFF
            if offset & 0x800000:
                offset = offset | 0xFF000000
                offset = offset - 0x100000000  # sign extend
            target = (self.pc + 8 + (offset << 2)) & 0xFFFFFFFF
            if link:
                self.regs[LR] = self.pc + 4
            self.pc = target
            return
        
        # ---- BX / BLX (register) ----
        if (word & 0x0FFFFFF0) == 0x012FFF10:  # BX
            target = self.regs[rm]
            if target & 1:
                self.regs[CPSR] |= T_FLAG
                self.pc = target & ~1
            else:
                self.regs[CPSR] &= ~T_FLAG
                self.pc = target & ~3
            return
        
        if (word & 0x0FFFFFF0) == 0x012FFF30:  # BLX
            self.regs[LR] = self.pc + 4
            target = self.regs[rm]
            if target & 1:
                self.regs[CPSR] |= T_FLAG
                self.pc = target & ~1
            else:
                self.regs[CPSR] &= ~T_FLAG
                self.pc = target & ~3
            return
        
        # ---- Multiply ----
        if (word & 0x0FC000F0) == 0x00000090:  # MUL
            rs = (word >> 8) & 0xF
            result = (self.regs[rm] * self.regs[rs]) & 0xFFFFFFFF
            self.regs[rd] = result
            if bit20:
                self.set_flag(N_FLAG, result & 0x80000000)
                self.set_flag(Z_FLAG, result == 0)
            self.pc += 4
            return
        
        if (word & 0x0FC000F0) == 0x00200090:  # MLA
            rs = (word >> 8) & 0xF
            result = (self.regs[rm] * self.regs[rs] + self.regs[rn]) & 0xFFFFFFFF
            self.regs[rd] = result
            if bit20:
                self.set_flag(N_FLAG, result & 0x80000000)
                self.set_flag(Z_FLAG, result == 0)
            self.pc += 4
            return
        
        # ---- Halfword transfers ----
        if (word & 0x0E000090) == 0x00000090 and bit7 and bit4:
            sh = (word >> 5) & 3
            if sh == 0:
                # Could be MUL/MLA (handled above) or SWP
                self.pc += 4
                return
            
            l = bit20
            u = bit23
            p = bit24
            w = bit21
            
            # Calculate offset
            if bit22:  # Immediate
                imm_hi = (word >> 8) & 0xF
                imm_lo = word & 0xF
                off = (imm_hi << 4) | imm_lo
            else:
                off = self.regs[rm]
            
            rn_val = self.regs[rn]
            if rn == PC:
                rn_val += 8
            
            addr = rn_val + (off if u else -off) if p else rn_val
            
            if l:  # Load
                if sh == 1:  # LDRH
                    val = self.mem.read16(addr)
                elif sh == 2:  # LDRSB
                    val = self.mem.read8(addr)
                    if val & 0x80:
                        val |= 0xFFFFFF00
                elif sh == 3:  # LDRSH
                    val = self.mem.read16(addr)
                    if val & 0x8000:
                        val |= 0xFFFF0000
                else:
                    val = 0
                self.regs[rd] = val & 0xFFFFFFFF
            else:  # Store
                if sh == 1:  # STRH
                    self.mem.write16(addr, self.regs[rd] & 0xFFFF)
                else:
                    self.mem.write32(addr, self.regs[rd])
            
            if not p:
                addr = rn_val + (off if u else -off)
            if (p and w) or not p:
                self.regs[rn] = addr & 0xFFFFFFFF
            
            self.pc += 4
            return
        
        # ---- Data Processing ----
        if bits27_25 in (0b000, 0b001):
            is_imm = bool(bits27_25 & 1)
            opcode = (word >> 21) & 0xF
            s = bit20
            
            rn_val = self.regs[rn]
            if rn == PC:
                rn_val += 8
            
            op2, shift_carry = self.decode_operand2(word, is_imm)
            carry_in = 1 if self.get_flag(C_FLAG) else 0
            
            result = 0
            write_rd = True
            
            if opcode == 0:  # AND
                result = rn_val & op2
            elif opcode == 1:  # EOR
                result = rn_val ^ op2
            elif opcode == 2:  # SUB
                result = self.alu_sub(rn_val, op2)
                s = False  # alu_sub already sets flags
                if bit20:
                    pass  # flags already set
            elif opcode == 3:  # RSB
                result = self.alu_sub(op2, rn_val)
                s = False
            elif opcode == 4:  # ADD
                result = self.alu_add(rn_val, op2)
                s = False
            elif opcode == 5:  # ADC
                result = self.alu_add(rn_val, op2, carry_in)
                s = False
            elif opcode == 6:  # SBC
                result = self.alu_add(rn_val, (~op2) & 0xFFFFFFFF, carry_in)
                s = False
            elif opcode == 7:  # RSC
                result = self.alu_add(op2, (~rn_val) & 0xFFFFFFFF, carry_in)
                s = False
            elif opcode == 8:  # TST
                result = rn_val & op2
                write_rd = False
            elif opcode == 9:  # TEQ
                result = rn_val ^ op2
                write_rd = False
            elif opcode == 10:  # CMP
                self.alu_sub(rn_val, op2)
                write_rd = False
                s = False
            elif opcode == 11:  # CMN
                self.alu_add(rn_val, op2)
                write_rd = False
                s = False
            elif opcode == 12:  # ORR
                result = rn_val | op2
            elif opcode == 13:  # MOV
                result = op2
            elif opcode == 14:  # BIC
                result = rn_val & (~op2 & 0xFFFFFFFF)
            elif opcode == 15:  # MVN
                result = (~op2) & 0xFFFFFFFF
            
            result = result & 0xFFFFFFFF
            
            if s and bit20:
                if opcode in (0, 1, 8, 9, 12, 13, 14, 15):
                    self.set_flag(N_FLAG, result & 0x80000000)
                    self.set_flag(Z_FLAG, result == 0)
                    self.set_flag(C_FLAG, shift_carry)
            
            if write_rd:
                if rd == PC:
                    if s and bit20:
                        # MOVS PC, ... restores CPSR from SPSR
                        pass
                    self.pc = result & ~3
                    return
                else:
                    self.regs[rd] = result
            
            self.pc += 4
            return
        
        # ---- Single data transfer (LDR/STR) ----
        if bits27_25 in (0b010, 0b011):
            l = bit20
            b = bit22
            u = bit23
            p = bit24
            w = bit21
            
            rn_val = self.regs[rn]
            if rn == PC:
                rn_val += 8
            
            if bits27_25 == 0b010:  # Immediate offset
                offset = word & 0xFFF
            else:  # Register offset
                shift_type = (word >> 5) & 3
                shift_amount = (word >> 7) & 0x1F
                rm_val = self.regs[rm]
                offset, _ = self.barrel_shift(rm_val, shift_type, shift_amount, False)
            
            # Pre-index
            if p:
                addr = rn_val + (offset if u else -offset)
            else:
                addr = rn_val
            
            addr = addr & 0xFFFFFFFF
            
            if l:  # Load
                if b:
                    val = self.mem.read8(addr)
                else:
                    val = self.mem.read32(addr & ~3)
                    # Rotate for unaligned reads
                    rot = (addr & 3) * 8
                    if rot:
                        val = ((val >> rot) | (val << (32 - rot))) & 0xFFFFFFFF
                
                if rd == PC:
                    # ARM interworking: bit 0 selects Thumb mode
                    if val & 1:
                        self.regs[CPSR] |= T_FLAG
                        self.pc = val & ~1
                    else:
                        self.regs[CPSR] &= ~T_FLAG
                        self.pc = val & ~3
                    # Check for post-index writeback
                    if not p:
                        wb_addr = rn_val + (offset if u else -offset)
                        self.regs[rn] = wb_addr & 0xFFFFFFFF
                    elif w:
                        self.regs[rn] = addr & 0xFFFFFFFF
                    return
                else:
                    self.regs[rd] = val & 0xFFFFFFFF
            else:  # Store
                val = self.regs[rd]
                if rd == PC:
                    val += 8  # store PC + 12 for STR
                if b:
                    self.mem.write8(addr, val & 0xFF)
                else:
                    self.mem.write32(addr & ~3, val)
            
            # Post-index or pre-index writeback
            if not p:
                addr = rn_val + (offset if u else -offset)
                self.regs[rn] = addr & 0xFFFFFFFF
            elif w:
                self.regs[rn] = addr & 0xFFFFFFFF
            
            self.pc += 4
            return
        
        # ---- Block data transfer (LDM/STM) ----
        if bits27_25 == 0b100:
            l = bit20
            u = bit23
            p = bit24
            w = bit21
            s = bit22
            
            base = self.regs[rn]
            reg_list = word & 0xFFFF
            count = bin(reg_list).count('1')
            
            if not u:
                base -= count * 4
                if not p:
                    base += 4
            else:
                if p:
                    base += 4
            
            addr = base & 0xFFFFFFFF
            
            for i in range(16):
                if reg_list & (1 << i):
                    if l:
                        val = self.mem.read32(addr)
                        self.regs[i] = val
                    else:
                        val = self.regs[i]
                        if i == PC:
                            val += 8
                        self.mem.write32(addr, val)
                    addr += 4
            
            if w:
                if u:
                    new_base = self.regs[rn] + count * 4
                else:
                    new_base = self.regs[rn] - count * 4
                self.regs[rn] = new_base & 0xFFFFFFFF
            
            if l and (reg_list & (1 << PC)):
                val = self.regs[PC]
                # ARM interworking: bit 0 selects Thumb mode
                if val & 1:
                    self.regs[CPSR] |= T_FLAG
                    self.pc = val & ~1
                else:
                    self.regs[CPSR] &= ~T_FLAG
                    self.pc = val & ~3
                return
            
            self.pc += 4
            return
        
        # ---- Coprocessor (MCR/MRC - CP15) ----
        if (word & 0x0F000010) == 0x0E000010:
            cp = (word >> 8) & 0xF
            op1 = (word >> 21) & 7
            crn = (word >> 16) & 0xF
            crm = word & 0xF
            op2 = (word >> 5) & 7
            l = bit20
            
            if cp == 15:
                if l:  # MRC - read from coprocessor
                    val = self.cp15[crn] if crn < len(self.cp15) else 0
                    if rd == PC:
                        # Copy N,Z,C,V to CPSR
                        self.regs[CPSR] = (self.regs[CPSR] & 0x0FFFFFFF) | (val & 0xF0000000)
                    else:
                        self.regs[rd] = val
                else:  # MCR - write to coprocessor
                    val = self.regs[rd]
                    if crn < len(self.cp15):
                        self.cp15[crn] = val
                    if self.trace:
                        print(f"  CP15 write: c{crn},c{crm},{op1},{op2} = 0x{val:08X}")
            
            self.pc += 4
            return
        
        # ---- CDP (coprocessor data processing) ----
        if (word & 0x0F000010) == 0x0E000000:
            self.pc += 4
            return
        
        # ---- SWP (swap) ----
        if (word & 0x0FB00FF0) == 0x01000090:
            addr = self.regs[rn]
            if bit22:  # SWPB
                old = self.mem.read8(addr)
                self.mem.write8(addr, self.regs[rm] & 0xFF)
                self.regs[rd] = old
            else:  # SWP
                old = self.mem.read32(addr)
                self.mem.write32(addr, self.regs[rm])
                self.regs[rd] = old
            self.pc += 4
            return
        
        # ---- MSR/MRS ----
        if (word & 0x0FBF0FFF) == 0x010F0000:  # MRS
            self.regs[rd] = self.regs[CPSR]
            self.pc += 4
            return
        
        if (word & 0x0DB0F000) == 0x0120F000:  # MSR
            if (word >> 25) & 1:  # Immediate
                imm8 = word & 0xFF
                rot = ((word >> 8) & 0xF) * 2
                val = ((imm8 >> rot) | (imm8 << (32 - rot))) & 0xFFFFFFFF if rot else imm8
            else:
                val = self.regs[rm]
            
            mask = 0
            if word & (1 << 19): mask |= 0xF0000000  # Flags
            if word & (1 << 16): mask |= 0x000000FF  # Control
            
            self.regs[CPSR] = (self.regs[CPSR] & ~mask) | (val & mask)
            self.pc += 4
            return
        
        # ---- SWI (Software Interrupt) ----
        if (word & 0x0F000000) == 0x0F000000:
            swi_num = word & 0x00FFFFFF
            if self.trace:
                print(f"  SWI #{swi_num}")
            self.pc += 4
            return
        
        # ---- Unhandled ----
        if self.trace:
            print(f"  UNHANDLED: 0x{word:08X}")
        self.pc += 4
    
    def execute_thumb(self, hw):
        """Execute one Thumb instruction."""
        old_pc = self.pc
        
        # Format 1: Move shifted register (LSL/LSR/ASR only, NOT add/sub)
        # Must exclude 0x1800-0x1FFF range which is Format 2
        if (hw & 0xE000) == 0x0000 and (hw & 0xF800) != 0x1800:
            op = (hw >> 11) & 3
            offset = (hw >> 6) & 0x1F
            rs = (hw >> 3) & 7
            rd = hw & 7
            val = self.regs[rs]
            
            if op == 0:  # LSL
                if offset:
                    self.set_flag(C_FLAG, (val >> (32 - offset)) & 1)
                    result = (val << offset) & 0xFFFFFFFF
                else:
                    result = val
            elif op == 1:  # LSR
                if offset:
                    self.set_flag(C_FLAG, (val >> (offset - 1)) & 1)
                    result = val >> offset
                else:
                    self.set_flag(C_FLAG, (val >> 31) & 1)
                    result = 0
            elif op == 2:  # ASR
                if offset:
                    self.set_flag(C_FLAG, (val >> (offset - 1)) & 1)
                    if val & 0x80000000:
                        result = (val >> offset) | (0xFFFFFFFF << (32 - offset))
                    else:
                        result = val >> offset
                    result &= 0xFFFFFFFF
                else:
                    self.set_flag(C_FLAG, (val >> 31) & 1)
                    result = 0xFFFFFFFF if val & 0x80000000 else 0
            else:
                result = val
            
            self.regs[rd] = result
            self.set_flag(N_FLAG, result & 0x80000000)
            self.set_flag(Z_FLAG, result == 0)
            self.pc += 2
            return
        
        # Format 2: Add/subtract
        if (hw & 0xF800) == 0x1800:
            i = (hw >> 10) & 1
            op = (hw >> 9) & 1
            rn_or_imm = (hw >> 6) & 7
            rs = (hw >> 3) & 7
            rd = hw & 7
            
            val = rn_or_imm if i else self.regs[rn_or_imm]
            
            if op:  # SUB
                result = self.alu_sub(self.regs[rs], val)
            else:  # ADD
                result = self.alu_add(self.regs[rs], val)
            
            self.regs[rd] = result
            self.pc += 2
            return
        
        # Format 3: Move/compare/add/subtract immediate
        if (hw & 0xE000) == 0x2000:
            op = (hw >> 11) & 3
            rd = (hw >> 8) & 7
            imm = hw & 0xFF
            
            if op == 0:  # MOV
                self.regs[rd] = imm
                self.set_flag(N_FLAG, False)
                self.set_flag(Z_FLAG, imm == 0)
            elif op == 1:  # CMP
                self.alu_sub(self.regs[rd], imm)
            elif op == 2:  # ADD
                self.regs[rd] = self.alu_add(self.regs[rd], imm)
            elif op == 3:  # SUB
                self.regs[rd] = self.alu_sub(self.regs[rd], imm)
            
            self.pc += 2
            return
        
        # Format 5: Hi register operations / BX
        if (hw & 0xFC00) == 0x4400:
            op = (hw >> 8) & 3
            h1 = (hw >> 7) & 1
            h2 = (hw >> 6) & 1
            rs = ((hw >> 3) & 7) + (8 if h2 else 0)
            rd = (hw & 7) + (8 if h1 else 0)
            
            if op == 0:  # ADD
                self.regs[rd] = (self.regs[rd] + self.regs[rs]) & 0xFFFFFFFF
                if rd == PC:
                    self.pc = self.regs[PC] & ~1
                    return
            elif op == 1:  # CMP
                self.alu_sub(self.regs[rd], self.regs[rs])
            elif op == 2:  # MOV
                self.regs[rd] = self.regs[rs]
                if rd == PC:
                    self.pc = self.regs[PC] & ~1
                    return
            elif op == 3:  # BX / BLX
                target = self.regs[rs]
                if target & 1:
                    self.pc = target & ~1
                else:
                    self.regs[CPSR] &= ~T_FLAG
                    self.pc = target & ~3
                return
            
            self.pc += 2
            return
        
        # Format 4: ALU operations
        if (hw & 0xFC00) == 0x4000:
            op = (hw >> 6) & 0xF
            rs = (hw >> 3) & 7
            rd = hw & 7
            
            a = self.regs[rd]
            b = self.regs[rs]
            
            if op == 0:  # AND
                result = a & b
            elif op == 1:  # EOR
                result = a ^ b
            elif op == 2:  # LSL
                shift = b & 0xFF
                if shift:
                    self.set_flag(C_FLAG, (a >> (32 - shift)) & 1 if shift <= 32 else 0)
                    result = (a << shift) & 0xFFFFFFFF if shift < 32 else 0
                else:
                    result = a
            elif op == 3:  # LSR
                shift = b & 0xFF
                if shift:
                    self.set_flag(C_FLAG, (a >> (shift - 1)) & 1 if shift <= 32 else 0)
                    result = (a >> shift) if shift < 32 else 0
                else:
                    result = a
            elif op == 4:  # ASR
                shift = b & 0xFF
                if shift:
                    if shift >= 32:
                        result = 0xFFFFFFFF if a & 0x80000000 else 0
                        self.set_flag(C_FLAG, result & 1)
                    else:
                        self.set_flag(C_FLAG, (a >> (shift - 1)) & 1)
                        if a & 0x80000000:
                            result = ((a >> shift) | (0xFFFFFFFF << (32 - shift))) & 0xFFFFFFFF
                        else:
                            result = a >> shift
                else:
                    result = a
            elif op == 5:  # ADC
                result = self.alu_add(a, b, 1 if self.get_flag(C_FLAG) else 0)
            elif op == 6:  # SBC
                result = self.alu_add(a, ~b & 0xFFFFFFFF, 1 if self.get_flag(C_FLAG) else 0)
            elif op == 7:  # ROR
                shift = b & 0x1F
                if shift:
                    result = ((a >> shift) | (a << (32 - shift))) & 0xFFFFFFFF
                    self.set_flag(C_FLAG, (result >> 31) & 1)
                else:
                    result = a
            elif op == 8:  # TST
                result = a & b
                self.set_flag(N_FLAG, result & 0x80000000)
                self.set_flag(Z_FLAG, result == 0)
                self.pc += 2
                return
            elif op == 9:  # NEG
                result = self.alu_sub(0, b)
            elif op == 10:  # CMP
                self.alu_sub(a, b)
                self.pc += 2
                return
            elif op == 11:  # CMN
                self.alu_add(a, b)
                self.pc += 2
                return
            elif op == 12:  # ORR
                result = a | b
            elif op == 13:  # MUL
                result = (a * b) & 0xFFFFFFFF
            elif op == 14:  # BIC
                result = a & (~b & 0xFFFFFFFF)
            elif op == 15:  # MVN
                result = (~b) & 0xFFFFFFFF
            else:
                result = 0
            
            self.regs[rd] = result & 0xFFFFFFFF
            self.set_flag(N_FLAG, result & 0x80000000)
            self.set_flag(Z_FLAG, (result & 0xFFFFFFFF) == 0)
            self.pc += 2
            return
        
        # Format 6: PC-relative load
        if (hw & 0xF800) == 0x4800:
            rd = (hw >> 8) & 7
            offset = (hw & 0xFF) << 2
            addr = ((self.pc + 4) & ~3) + offset
            self.regs[rd] = self.mem.read32(addr)
            self.pc += 2
            return
        
        # Format 7/8: Load/store with register offset
        if (hw & 0xF200) == 0x5000:
            l = (hw >> 11) & 1
            b = (hw >> 10) & 1
            ro = (hw >> 6) & 7
            rb = (hw >> 3) & 7
            rd = hw & 7
            addr = (self.regs[rb] + self.regs[ro]) & 0xFFFFFFFF
            
            if l:
                if b:
                    self.regs[rd] = self.mem.read8(addr)
                else:
                    self.regs[rd] = self.mem.read32(addr)
            else:
                if b:
                    self.mem.write8(addr, self.regs[rd] & 0xFF)
                else:
                    self.mem.write32(addr, self.regs[rd])
            self.pc += 2
            return
        
        # Format 9: Load/store with immediate offset
        if (hw & 0xE000) == 0x6000:
            b = (hw >> 12) & 1
            l = (hw >> 11) & 1
            offset = (hw >> 6) & 0x1F
            rb = (hw >> 3) & 7
            rd = hw & 7
            
            if b:
                addr = (self.regs[rb] + offset) & 0xFFFFFFFF
            else:
                addr = (self.regs[rb] + (offset << 2)) & 0xFFFFFFFF
            
            if l:
                if b:
                    self.regs[rd] = self.mem.read8(addr)
                else:
                    self.regs[rd] = self.mem.read32(addr)
            else:
                if b:
                    self.mem.write8(addr, self.regs[rd] & 0xFF)
                else:
                    self.mem.write32(addr, self.regs[rd])
            self.pc += 2
            return
        
        # Format 10: Load/store halfword
        if (hw & 0xF000) == 0x8000:
            l = (hw >> 11) & 1
            offset = ((hw >> 6) & 0x1F) << 1
            rb = (hw >> 3) & 7
            rd = hw & 7
            addr = (self.regs[rb] + offset) & 0xFFFFFFFF
            
            if l:
                self.regs[rd] = self.mem.read16(addr)
            else:
                self.mem.write16(addr, self.regs[rd] & 0xFFFF)
            self.pc += 2
            return
        
        # Format 11: SP-relative load/store
        if (hw & 0xF000) == 0x9000:
            l = (hw >> 11) & 1
            rd = (hw >> 8) & 7
            offset = (hw & 0xFF) << 2
            addr = (self.regs[SP] + offset) & 0xFFFFFFFF
            
            if l:
                self.regs[rd] = self.mem.read32(addr)
            else:
                self.mem.write32(addr, self.regs[rd])
            self.pc += 2
            return
        
        # Format 12: Load address (ADD Rd, PC/SP, #imm)
        if (hw & 0xF000) == 0xA000:
            sp_flag = (hw >> 11) & 1
            rd = (hw >> 8) & 7
            offset = (hw & 0xFF) << 2
            
            if sp_flag:
                self.regs[rd] = (self.regs[SP] + offset) & 0xFFFFFFFF
            else:
                self.regs[rd] = ((self.pc + 4) & ~3) + offset
            self.pc += 2
            return
        
        # Format 13: Add offset to SP
        if (hw & 0xFF00) == 0xB000:
            s = (hw >> 7) & 1
            offset = (hw & 0x7F) << 2
            if s:
                self.regs[SP] = (self.regs[SP] - offset) & 0xFFFFFFFF
            else:
                self.regs[SP] = (self.regs[SP] + offset) & 0xFFFFFFFF
            self.pc += 2
            return
        
        # Format 14: PUSH/POP
        if (hw & 0xF600) == 0xB400:
            l = (hw >> 11) & 1
            r = (hw >> 8) & 1  # PC/LR bit
            reg_list = hw & 0xFF
            
            if l:  # POP
                addr = self.regs[SP]
                for i in range(8):
                    if reg_list & (1 << i):
                        self.regs[i] = self.mem.read32(addr)
                        addr += 4
                if r:
                    val = self.mem.read32(addr)
                    addr += 4
                    if val & 1:
                        self.pc = val & ~1
                    else:
                        self.regs[CPSR] &= ~T_FLAG
                        self.pc = val & ~3
                    self.regs[SP] = addr & 0xFFFFFFFF
                    return
                self.regs[SP] = addr & 0xFFFFFFFF
            else:  # PUSH
                count = bin(reg_list).count('1') + (1 if r else 0)
                addr = (self.regs[SP] - count * 4) & 0xFFFFFFFF
                self.regs[SP] = addr
                for i in range(8):
                    if reg_list & (1 << i):
                        self.mem.write32(addr, self.regs[i])
                        addr += 4
                if r:
                    self.mem.write32(addr, self.regs[LR])
            
            self.pc += 2
            return
        
        # Format 15: Multiple load/store
        if (hw & 0xF000) == 0xC000:
            l = (hw >> 11) & 1
            rb = (hw >> 8) & 7
            reg_list = hw & 0xFF
            addr = self.regs[rb]
            
            for i in range(8):
                if reg_list & (1 << i):
                    if l:
                        self.regs[i] = self.mem.read32(addr)
                    else:
                        self.mem.write32(addr, self.regs[i])
                    addr += 4
            
            self.regs[rb] = addr & 0xFFFFFFFF
            self.pc += 2
            return
        
        # Format 16: Conditional branch
        if (hw & 0xF000) == 0xD000:
            cond = (hw >> 8) & 0xF
            if cond == 0xF:  # SWI
                self.pc += 2
                return
            offset = hw & 0xFF
            if offset & 0x80:
                offset = offset - 0x100
            if self.check_condition(cond):
                self.pc = (self.pc + 4 + (offset << 1)) & 0xFFFFFFFF
                return
            self.pc += 2
            return
        
        # Format 18: Unconditional branch
        if (hw & 0xF800) == 0xE000:
            offset = hw & 0x7FF
            if offset & 0x400:
                offset = offset - 0x800
            self.pc = (self.pc + 4 + (offset << 1)) & 0xFFFFFFFF
            return
        
        # Format 19: Long branch with link (BL - 2 halfwords)
        if (hw & 0xF800) == 0xF000:
            offset_hi = hw & 0x7FF
            if offset_hi & 0x400:
                offset_hi |= 0xFFFFF800
            self.regs[LR] = (self.pc + 4 + (offset_hi << 12)) & 0xFFFFFFFF
            self.pc += 2
            return
        
        if (hw & 0xF800) == 0xF800:
            offset_lo = hw & 0x7FF
            target = (self.regs[LR] + (offset_lo << 1)) & 0xFFFFFFFF
            self.regs[LR] = (self.pc + 2) | 1
            self.pc = target
            return
        
        # Unhandled
        if self.trace:
            print(f"  UNHANDLED THUMB: 0x{hw:04X}")
        self.pc += 2
    
    def step(self):
        """Execute one instruction. No hacks - proper peripheral emulation."""
        if self.halted:
            return False
        
        pc = self.pc
        thumb = bool(self.regs[CPSR] & T_FLAG)
        
        # Execute the instruction
        try:
            if thumb:
                hw = self.mem.read16(pc)
                if self.trace:
                    print(f"[{self.instr_count:6d}] 0x{pc:08X}: {hw:04X} (Thumb)")
                self.execute_thumb(hw)
            else:
                word = self.mem.read32(pc)
                if self.trace:
                    print(f"[{self.instr_count:6d}] 0x{pc:08X}: {word:08X}")
                self.execute_arm(word)
        except Exception as e:
            print(f"\n!!! EXCEPTION at PC=0x{pc:08X}: {e}")
            self.dump_regs()
            self.halted = True
            return False
        
        self.instr_count += 1
        
        # Detect infinite loops
        if self.pc == pc and not thumb:
            word = self.mem.read32(pc)
            if word == 0xEAFFFFFE:  # B .
                print(f"\n[CPU] Infinite loop detected at 0x{pc:08X}")
                self.halted = True
                return False
        
        if self.instr_count >= self.max_instr:
            print(f"\n[CPU] Max instructions ({self.max_instr}) reached")
            self.halted = True
            return False
        
        if self.pc in self.breakpoints:
            print(f"\n[CPU] Breakpoint at 0x{self.pc:08X}")
            self.dump_regs()
            return False
        
        return True
    
    def dump_regs(self):
        """Dump all registers."""
        print("=" * 60)
        for i in range(0, 16, 4):
            line = ''
            for j in range(4):
                if i + j < 16:
                    line += f'{REG_NAMES[i+j]:4s}=0x{self.regs[i+j]:08X}  '
            print(f"  {line}")
        
        cpsr = self.regs[CPSR]
        flags = ''
        flags += 'N' if cpsr & N_FLAG else 'n'
        flags += 'Z' if cpsr & Z_FLAG else 'z'
        flags += 'C' if cpsr & C_FLAG else 'c'
        flags += 'V' if cpsr & V_FLAG else 'v'
        thumb = 'THUMB' if cpsr & T_FLAG else 'ARM'
        mode_bits = cpsr & 0x1F
        modes = {0x10:'USR',0x11:'FIQ',0x12:'IRQ',0x13:'SVC',0x17:'ABT',0x1B:'UND',0x1F:'SYS'}
        mode = modes.get(mode_bits, f'0x{mode_bits:02X}')
        print(f"  CPSR=0x{cpsr:08X}  [{flags}]  {thumb}  {mode}")
        print(f"  Instructions executed: {self.instr_count}")
        print("=" * 60)
    
    def run(self):
        """Run until halted."""
        print(f"\n{'='*60}")
        print(f"  HP OfficeJet 'Eridani' ARM926EJ-S Emulator")
        print(f"  Starting execution at PC=0x{self.pc:08X}")
        print(f"{'='*60}\n")
        
        start_time = time.time()
        
        while self.step():
            pass
        
        elapsed = time.time() - start_time
        
        # Flush UART
        self.mem.uart.flush()
        
        print(f"\n{'='*60}")
        print(f"  Emulation stopped after {self.instr_count:,} instructions")
        print(f"  Elapsed time: {elapsed:.3f}s")
        if elapsed > 0:
            print(f"  Speed: {self.instr_count/elapsed:,.0f} instructions/sec")
        print(f"{'='*60}")
        
        # Print final state
        self.dump_regs()
        
        # Print UART output summary
        uart = self.mem.uart
        if uart.output_lines:
            print(f"\n{'='*60}")
            print(f"  UART Output ({uart.tx_count} chars transmitted):")
            print(f"{'='*60}")
            for line in uart.output_lines:
                print(f"  {line}")
        else:
            print(f"\n  UART: {uart.tx_count} chars transmitted, no complete lines")
            if uart.current_line:
                print(f"  UART partial: \"{uart.current_line}\"")
        
        # Print unknown I/O log summary
        if self.mem.io_log:
            print(f"\n  Unknown I/O operations: {len(self.mem.io_log)}")
            pages = {}
            for addr, val, sz, rw in self.mem.io_log:
                p = addr & 0xFFFFF000
                if p not in pages:
                    pages[p] = 0
                pages[p] += 1
            for p in sorted(pages.keys()):
                print(f"    0x{p:08X}: {pages[p]} writes")

# ============================================================
# Main
# ============================================================
def main():
    parser = argparse.ArgumentParser(description='HP OfficeJet ARM926 Emulator')
    parser.add_argument('--trace', action='store_true', help='Enable instruction trace')
    parser.add_argument('--max', type=int, default=500000, help='Max instructions')
    parser.add_argument('--dump', type=int, default=0, help='Dump registers every N instructions')
    parser.add_argument('--file', default=FLASH_FILE, help='Flash dump file')
    args = parser.parse_args()
    
    # Load firmware
    print(f"Loading firmware from {args.file}...")
    if not os.path.exists(args.file):
        print(f"Error: {args.file} not found!")
        sys.exit(1)
    
    with open(args.file, 'rb') as f:
        flash_data = f.read(FLASH_SIZE)
    
    print(f"  Loaded {len(flash_data):,} bytes ({len(flash_data)/1024:.0f} KB)")
    
    # Verify "otto" magic
    if flash_data[0x0C:0x10] == b'otto':
        print("  [OK] 'otto' magic found at offset 0x0C")
    else:
        print("  [WARN] 'otto' magic not found!")
    
    # Setup memory
    mem = Memory()
    mem.add_region(0x00000000, FLASH_SIZE, flash_data, 'flash', writable=False)
    mem.add_region(RAM_BASE, RAM_SIZE, name='ram', writable=True)
    # Mirror flash at high address range (boot code jumps to 0xE0001C)
    mem.add_region(0x00E00000, FLASH_SIZE, flash_data, 'flash_mirror', writable=False)
    # SRAM at 0xFC000000 (used by boot code for memory test, needs 8MB)
    mem.add_region(0xFC000000, 0x00800000, name='sram_hi', writable=True)
    # SRAM at 0xFD000000 (boot code uses for stack + ROMOBJ copy dest)
    mem.add_region(0xFD000000, 0x00400000, name='sram_fd', writable=True)
    # SRAM at 0xFE000000 (firmware data area)
    mem.add_region(0xFE000000, 0x00200000, name='sram_fe', writable=True)
    
    # Create CPU
    cpu = ARM926(mem)
    cpu.trace = args.trace
    cpu.max_instr = args.max
    cpu.reset()
    
    # Set initial SP to top of RAM
    cpu.regs[SP] = RAM_BASE + RAM_SIZE - 4
    
    print(f"  Memory map:")
    print(f"    Flash:  0x00000000 - 0x{FLASH_SIZE-1:08X} ({FLASH_SIZE//1024} KB)")
    print(f"    Mirror: 0x00E00000 - 0x00EFFFFF ({FLASH_SIZE//1024} KB)")
    print(f"    RAM:    0x{RAM_BASE:08X} - 0x{RAM_BASE+RAM_SIZE-1:08X} ({RAM_SIZE//1024} KB)")
    print(f"    SRAM:   0xFC000000 - 0xFD0FFFFF (2 MB)")
    print(f"  Peripherals:")
    print(f"    CLK0:   0x20813500 (Clock/PLL controller 0)")
    print(f"    CLK1:   0x20813A00 (Clock/PLL controller 1)")
    print(f"    UART:   0x20812900 (Debug UART)")
    print(f"    GPIO:   0x20813200 (GPIO/misc)")
    
    # Run
    cpu.run()
    
    print("\nEmulation complete.")

if __name__ == '__main__':
    main()
