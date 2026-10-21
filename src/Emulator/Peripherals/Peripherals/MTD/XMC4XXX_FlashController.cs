//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Memory;

namespace Antmicro.Renode.Peripherals.MTD
{
    // XMC4500 Flash Memory Controller (FLASH0 module).
    //
    // Register base: 0x58002000. In the SVD the FLASH0 peripheral baseAddress is
    // 0x58001000 but its register offsets are 0x10xx (e.g. ID 0x1008, FCON 0x1014),
    // so the effective hardware addresses are 0x5800_20xx. The peripheral-local
    // offsets below (ID 0x08, FCON 0x14, ...) therefore map to 0x58002008 /
    // 0x58002014 / ... only when the controller is registered at 0x58002000.
    //
    // The XMC4500 uses a command-sequence-based flash interface: programming
    // and erase operations are triggered by writes to specific "magic" addresses
    // within the flash address space (0x0C00_5554, 0x0C00_AAA8), not by register
    // writes.  This peripheral handles:
    //   1. Status registers (FSR) — so firmware sees "ready, no errors"
    //   2. Configuration registers (FCON, MARP) — read/write for wait states etc.
    //   3. Protection registers (PROCON0-2) — read-only, report no protection
    //   4. Command-sequence interception via a second connection region on the
    //      uncached flash address space
    //
    // The command interface intercepts writes to the uncached flash region and
    // implements the enter-page-mode / load-page / write-page / erase-sector
    // state machine, applying changes to the underlying cached flash MappedMemory.
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord | AllowedTranslation.WordToDoubleWord)]
    public class XMC4XXX_FlashController : IDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_FlashController(IMachine machine, MappedMemory flash)
        {
            this.machine = machine;
            this.flash = flash;
            pageBuffer = new byte[PageBufferSize];

            registers = new DoubleWordRegisterCollection(this);
            DefineRegisters();
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers.Write(offset, value);
        }

        // ── Command interface (uncached flash region) ──────────────────

        [ConnectionRegion("cmdInterface")]
        public uint ReadDoubleWordFromCmdInterface(long offset)
        {
            // Reads from the uncached flash go through to the underlying
            // cached flash memory (they share the same physical flash).
            return flash.ReadDoubleWord(offset);
        }

        [ConnectionRegion("cmdInterface")]
        public void WriteDoubleWordToCmdInterface(long offset, uint value)
        {
            HandleCommandWrite(offset, value);
        }

        [ConnectionRegion("cmdInterface")]
        public byte ReadByteFromCmdInterface(long offset)
        {
            return flash.ReadByte(offset);
        }

        [ConnectionRegion("cmdInterface")]
        public void WriteByteToCmdInterface(long offset, byte value)
        {
            // Byte writes during page load — buffer them
            if(cmdState == CommandState.PageMode || cmdState == CommandState.Loading)
            {
                if(pageBufferIndex < PageBufferSize)
                {
                    pageBuffer[pageBufferIndex++] = value;
                    cmdState = CommandState.Loading;
                }
            }
            else
            {
                this.Log(LogLevel.Debug, "Byte write 0x{0:X2} to uncached flash offset 0x{1:X} outside page mode", value, offset);
            }
        }

        [ConnectionRegion("cmdInterface")]
        public ushort ReadWordFromCmdInterface(long offset)
        {
            return flash.ReadWord(offset);
        }

        [ConnectionRegion("cmdInterface")]
        public void WriteWordToCmdInterface(long offset, ushort value)
        {
            this.Log(LogLevel.Debug, "Word write 0x{0:X4} to uncached flash offset 0x{1:X}", value, offset);
        }

        public void Reset()
        {
            registers.Reset();
            cmdState = CommandState.Idle;
            pageBufferIndex = 0;
            targetPageAddress = 0;
            Array.Clear(pageBuffer, 0, pageBuffer.Length);
        }

        public long Size => 0x30;  // Register space: offsets 0x00-0x2C

        // ── Register definitions ───────────────────────────────────────

        private void DefineRegisters()
        {
            // ID register: Module identification
            // SVD offset 0x1008, local offset 0x08
            Registers.Id.Define(registers, ModuleIdValue)
                .WithValueField(0, 8, FieldMode.Read, name: "MOD_REV")
                .WithValueField(8, 8, FieldMode.Read, name: "MOD_TYPE")
                .WithValueField(16, 16, FieldMode.Read, name: "MOD_NUMBER");

            // FSR: Flash Status Register
            // SVD offset 0x1010, local offset 0x10
            Registers.FlashStatus.Define(registers, 0x0)
                .WithFlag(0, FieldMode.Read, name: "PBUSY",
                    valueProviderCallback: _ => false)  // never busy
                .WithFlag(1, FieldMode.Read, name: "FABUSY",
                    valueProviderCallback: _ => false)
                .WithReservedBits(2, 2)
                .WithFlag(4, FieldMode.Read, name: "PROG",
                    valueProviderCallback: _ => programming)
                .WithFlag(5, FieldMode.Read, name: "ERASE",
                    valueProviderCallback: _ => erasing)
                .WithFlag(6, FieldMode.Read, name: "PFPAGE",
                    valueProviderCallback: _ => cmdState == CommandState.PageMode || cmdState == CommandState.Loading)
                .WithReservedBits(7, 1)
                .WithFlag(8, FieldMode.Read, name: "PFOPER",
                    valueProviderCallback: _ => false)
                .WithReservedBits(9, 1)
                .WithFlag(10, FieldMode.Read, name: "SQER",
                    valueProviderCallback: _ => false)
                .WithFlag(11, FieldMode.Read, name: "PROER",
                    valueProviderCallback: _ => false)
                .WithFlag(12, FieldMode.Read, name: "PFSBER",
                    valueProviderCallback: _ => false)
                .WithReservedBits(13, 1)
                .WithFlag(14, FieldMode.Read, name: "PFDBER",
                    valueProviderCallback: _ => false)
                .WithReservedBits(15, 1)
                .WithFlag(16, FieldMode.Read, name: "PROIN",
                    valueProviderCallback: _ => false)
                .WithReservedBits(17, 1)
                .WithFlag(18, FieldMode.Read, name: "RPROIN",
                    valueProviderCallback: _ => false)
                .WithFlag(19, FieldMode.Read, name: "RPRODIS",
                    valueProviderCallback: _ => false)
                .WithReservedBits(20, 1)
                .WithFlag(21, FieldMode.Read, name: "WPROIN0",
                    valueProviderCallback: _ => false)
                .WithFlag(22, FieldMode.Read, name: "WPROIN1",
                    valueProviderCallback: _ => false)
                .WithFlag(23, FieldMode.Read, name: "WPROIN2",
                    valueProviderCallback: _ => false)
                .WithFlag(24, FieldMode.Read, name: "WPRODIS0",
                    valueProviderCallback: _ => false)
                .WithFlag(25, FieldMode.Read, name: "WPRODIS1",
                    valueProviderCallback: _ => false)
                .WithReservedBits(26, 1)
                .WithFlag(27, FieldMode.Read, name: "SLM",
                    valueProviderCallback: _ => false)
                .WithFlag(28, FieldMode.Read, name: "VER",
                    valueProviderCallback: _ => true)  // verify OK
                .WithReservedBits(29, 3);

            // FCON: Flash Configuration Register
            // SVD offset 0x1014, local offset 0x14
            Registers.FlashConfig.Define(registers, 0x6)
                .WithValueField(0, 4, name: "WSPFLASH")   // wait states
                .WithFlag(4, name: "WSECPF")               // ECC wait state
                .WithReservedBits(5, 8)
                .WithFlag(13, name: "IDLE")
                .WithFlag(14, name: "ESLDIS")
                .WithFlag(15, name: "SLEEP")
                .WithFlag(16, FieldMode.Read, name: "RPA",
                    valueProviderCallback: _ => false)
                .WithFlag(17, FieldMode.Read, name: "DCF",
                    valueProviderCallback: _ => false)
                .WithFlag(18, FieldMode.Read, name: "DDF",
                    valueProviderCallback: _ => false)
                .WithReservedBits(19, 5)
                .WithValueField(24, 4, name: "VOPERM")
                .WithValueField(28, 4, name: "SQERM");

            // MARP: Margin Control Register
            // SVD offset 0x1018, local offset 0x18
            Registers.MarginControl.Define(registers, 0x0)
                .WithFlag(0, name: "MARGIN")
                .WithReservedBits(1, 3)
                .WithFlag(4, name: "TRAPDIS")
                .WithReservedBits(5, 27);

            // PROCON0: Protection Configuration User 0
            // SVD offset 0x1020, local offset 0x20
            Registers.ProtectionConfig0.Define(registers, 0x0)
                .WithValueField(0, 12, FieldMode.Read, name: "S0L_S11L")  // sector lock bits
                .WithReservedBits(12, 3)
                .WithFlag(15, FieldMode.Read, name: "RPRO",
                    valueProviderCallback: _ => false)
                .WithReservedBits(16, 16);

            // PROCON1: Protection Configuration User 1
            Registers.ProtectionConfig1.Define(registers, 0x0)
                .WithValueField(0, 12, FieldMode.Read, name: "S0L_S11L")
                .WithReservedBits(12, 4)
                .WithFlag(16, FieldMode.Read, name: "PSR",
                    valueProviderCallback: _ => false)
                .WithReservedBits(17, 15);

            // PROCON2: Protection Configuration User 2
            Registers.ProtectionConfig2.Define(registers, 0x0)
                .WithValueField(0, 12, FieldMode.Read, name: "S0ROM_S11ROM")
                .WithReservedBits(12, 20);
        }

        // ── Command sequence state machine ─────────────────────────────

        private void HandleCommandWrite(long offset, uint value)
        {
            var relOffset = offset & 0xFFFFF;  // offset within flash bank

            switch(cmdState)
            {
                case CommandState.Idle:
                    if(relOffset == CommandAddress1 && value == EnterPageModeCmd)
                    {
                        this.Log(LogLevel.Debug, "Flash: Enter page mode");
                        cmdState = CommandState.PageMode;
                        pageBufferIndex = 0;
                        Array.Clear(pageBuffer, 0, pageBuffer.Length);
                    }
                    else if(relOffset == CommandAddress1 && value == EraseSectorCmd)
                    {
                        this.Log(LogLevel.Debug, "Flash: Erase sector sequence started");
                        cmdState = CommandState.EraseStep1;
                    }
                    else if(relOffset == CommandAddress1 && value == ResetToReadCmd)
                    {
                        this.Log(LogLevel.Debug, "Flash: Reset to read mode");
                        cmdState = CommandState.Idle;
                    }
                    else
                    {
                        // Not a command — let write fall through to flash
                        this.Log(LogLevel.Noisy, "Flash: Non-command write 0x{0:X8} to offset 0x{1:X}", value, offset);
                    }
                    break;

                case CommandState.PageMode:
                case CommandState.Loading:
                    if(relOffset == CommandAddress1 && value == WritePageCmd1)
                    {
                        this.Log(LogLevel.Debug, "Flash: Write page step 1 (loaded {0} bytes)", pageBufferIndex);
                        cmdState = CommandState.WriteStep1;
                    }
                    else
                    {
                        // Loading page buffer data
                        if(pageBufferIndex + 3 < PageBufferSize)
                        {
                            pageBuffer[pageBufferIndex++] = (byte)(value & 0xFF);
                            pageBuffer[pageBufferIndex++] = (byte)((value >> 8) & 0xFF);
                            pageBuffer[pageBufferIndex++] = (byte)((value >> 16) & 0xFF);
                            pageBuffer[pageBufferIndex++] = (byte)((value >> 24) & 0xFF);
                        }
                        cmdState = CommandState.Loading;
                    }
                    break;

                case CommandState.WriteStep1:
                    if(relOffset == CommandAddress2 && value == WritePageCmd2)
                    {
                        this.Log(LogLevel.Debug, "Flash: Write page step 2");
                        cmdState = CommandState.WriteConfirm;
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Flash: Bad write page sequence, expected 0x55 at 0xAAA8, got 0x{0:X} at 0x{1:X}", value, relOffset);
                        cmdState = CommandState.Idle;
                    }
                    break;

                case CommandState.WriteConfirm:
                    // value is the target page address (relative to flash base)
                    targetPageAddress = (long)(value & 0xFFFFF);
                    ExecutePageWrite();
                    cmdState = CommandState.Idle;
                    break;

                case CommandState.EraseStep1:
                    if(relOffset == CommandAddress1 && value == WritePageCmd1)
                    {
                        cmdState = CommandState.EraseStep2;
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Flash: Bad erase sequence step 1");
                        cmdState = CommandState.Idle;
                    }
                    break;

                case CommandState.EraseStep2:
                    if(relOffset == CommandAddress2 && value == WritePageCmd2)
                    {
                        cmdState = CommandState.EraseConfirm;
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Flash: Bad erase sequence step 2");
                        cmdState = CommandState.Idle;
                    }
                    break;

                case CommandState.EraseConfirm:
                    // value is the sector base address
                    ExecuteSectorErase((long)(value & 0xFFFFF));
                    cmdState = CommandState.Idle;
                    break;

                default:
                    this.Log(LogLevel.Warning, "Flash: Unexpected write in state {0}", cmdState);
                    cmdState = CommandState.Idle;
                    break;
            }
        }

        private void ExecutePageWrite()
        {
            var writeLength = Math.Min(pageBufferIndex, PageBufferSize);
            if(writeLength == 0)
            {
                this.Log(LogLevel.Warning, "Flash: Page write with empty buffer");
                return;
            }

            this.Log(LogLevel.Debug, "Flash: Writing {0} bytes to offset 0x{1:X}", writeLength, targetPageAddress);
            programming = true;

            // Write page buffer to the main (cached) flash memory
            for(int i = 0; i < writeLength; i++)
            {
                // Flash write: can only clear bits (AND with existing data)
                var existing = flash.ReadByte(targetPageAddress + i);
                flash.WriteByte(targetPageAddress + i, (byte)(existing & pageBuffer[i]));
            }

            programming = false;
            pageBufferIndex = 0;
        }

        private void ExecuteSectorErase(long sectorAddress)
        {
            // Determine sector size from address
            int sectorSize = GetSectorSize(sectorAddress);
            long sectorBase = GetSectorBase(sectorAddress);

            this.Log(LogLevel.Debug, "Flash: Erasing sector at 0x{0:X}, size 0x{1:X}", sectorBase, sectorSize);
            erasing = true;

            // Erase = fill with 0xFF
            for(int i = 0; i < sectorSize; i++)
            {
                flash.WriteByte(sectorBase + i, 0xFF);
            }

            erasing = false;
        }

        private int GetSectorSize(long address)
        {
            // XMC4500 flash sector layout (1MB total):
            // Sectors 0-3:  16KB each (0x00000 - 0x0FFFF)
            // Sectors 4-7:  16KB each (0x10000 - 0x1FFFF)
            // Sector 8:    128KB      (0x20000 - 0x3FFFF)
            // Sector 9:    256KB      (0x40000 - 0x7FFFF)
            // Sector 10:   256KB      (0x80000 - 0xBFFFF) [if present]
            // Sector 11:   256KB      (0xC0000 - 0xFFFFF) [if present]
            if(address < 0x10000)
                return 0x4000;   // 16KB
            if(address < 0x20000)
                return 0x4000;   // 16KB
            if(address < 0x40000)
                return 0x20000;  // 128KB
            return 0x40000;      // 256KB
        }

        private long GetSectorBase(long address)
        {
            int size = GetSectorSize(address);
            return (address / size) * size;
        }

        // ── State ──────────────────────────────────────────────────────

        private bool programming;
        private bool erasing;
        private CommandState cmdState;
        private int pageBufferIndex;
        private long targetPageAddress;

        private readonly IMachine machine;
        private readonly MappedMemory flash;
        private readonly byte[] pageBuffer;
        private readonly DoubleWordRegisterCollection registers;

        // ── Constants ──────────────────────────────────────────────────

        // Module ID: MOD_NUMBER=0x00A2, MOD_TYPE=0xC0, MOD_REV=0x00
        private const uint ModuleIdValue = 0x00A2C000;

        // Page buffer = 256 bytes (64 doublewords)
        private const int PageBufferSize = 256;

        // Command addresses (relative to flash base, within 1MB range)
        private const long CommandAddress1 = 0x5554;
        private const long CommandAddress2 = 0xAAA8;

        // Command codes
        private const uint EnterPageModeCmd = 0x50;
        private const uint WritePageCmd1 = 0xAA;
        private const uint WritePageCmd2 = 0x55;
        private const uint EraseSectorCmd = 0x20;
        private const uint ResetToReadCmd = 0xF0;

        private enum CommandState
        {
            Idle,
            PageMode,       // Page mode entered, accepting data loads
            Loading,        // Loading data into page buffer
            WriteStep1,     // Received 0xAA at addr1
            WriteConfirm,   // Received 0x55 at addr2, waiting for target address
            EraseStep1,     // Erase: received 0x20, waiting for 0xAA
            EraseStep2,     // Erase: received 0xAA, waiting for 0x55
            EraseConfirm,   // Erase: waiting for sector address
        }

        // Register offsets (relative to peripheral base = 0x58001000)
        // SVD FLASH0 base is 0x58001000, and offsets are from there.
        // The registers in the SVD use offsets like 0x1008, 0x1010, etc.
        // which are relative to the parent PMU block at 0x58000000.
        // Since our peripheral is registered at 0x58001000, we subtract 0x1000.
        private enum Registers
        {
            Id               = 0x08,  // SVD: 0x1008
            FlashStatus      = 0x10,  // SVD: 0x1010
            FlashConfig      = 0x14,  // SVD: 0x1014
            MarginControl    = 0x18,  // SVD: 0x1018
            ProtectionConfig0 = 0x20, // SVD: 0x1020
            ProtectionConfig1 = 0x24, // SVD: 0x1024
            ProtectionConfig2 = 0x28, // SVD: 0x1028
        }
    }
}
