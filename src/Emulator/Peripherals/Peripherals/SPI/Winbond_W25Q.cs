//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;

using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Peripherals.SPI.NORFlash;
using Antmicro.Renode.Utilities;

using Range = Antmicro.Renode.Core.Range;

namespace Antmicro.Renode.Peripherals.SPI
{
    // Winbond W25Q32/W25Q64/W25Q128 serial NOR flash.
    //
    // Adds the Winbond Status Register-2/-3 (read 0x35/0x15, write 0x31/0x11, or SR1+SR2
    // in one 0x01 command) and enforces the WPS=0 status-register memory protection:
    // SEC/TB/BP[2:0] in SR1 select the protected area and CMP in SR2 complements it.
    // Program and erase of protected addresses are ignored, as on the real part.
    //
    // The protection bits are non-volatile: they are kept across Reset(), so a lock set
    // by the firmware survives a system reset like it does on hardware.
    public class Winbond_W25Q : GenericSpiFlash
    {
        public Winbond_W25Q(MappedMemory underlyingMemory, byte memoryType = DefaultMemoryType)
            : base(underlyingMemory, manufacturerId: ManufacturerId, memoryType: memoryType,
                   writeStatusCanSetWriteEnable: false, useStatusRegisterStubs: false)
        {
            if(underlyingMemory.Size < 4.MB() || underlyingMemory.Size > 16.MB())
            {
                // Smaller parts use a different BP table, larger ones add BP3 and 4-byte addressing
                throw new ConstructionException("Size of the underlying memory must be in range 4MB - 16MB (W25Q32 - W25Q128)");
            }

            statusRegister
                .WithValueField(2, 3, out blockProtect, name: "BP")
                .WithFlag(5, out topBottom, name: "TB")
                .WithFlag(6, out sectorProtect, name: "SEC")
                .WithFlag(7, name: "SRP");

            statusRegister2 = new ByteRegister(this)
                .WithFlag(0, name: "SRL")
                .WithFlag(1, name: "QE")
                .WithReservedBits(2, 1)
                // Security Register lock bits are one-time programmable: they can only be set
                .WithValueField(3, 3, valueProviderCallback: _ => securityRegisterLocks,
                    writeCallback: (_, value) => securityRegisterLocks |= (byte)value, name: "LB")
                .WithFlag(6, out complementProtect, name: "CMP")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => false, name: "SUS");

            statusRegister3 = new ByteRegister(this, DefaultStatusRegister3)
                .WithReservedBits(0, 2)
                .WithFlag(2, out writeProtectSelection, name: "WPS")
                .WithReservedBits(3, 2)
                .WithValueField(5, 2, name: "DRV")
                .WithReservedBits(7, 1);
        }

        public override void Reset()
        {
            // The base reset clears SR1 and the locked range, but the protection bits are non-volatile
            var protectionBits = (byte)(statusRegister.Read() & StatusRegisterNonVolatileMask);
            base.Reset();
            statusRegister.Write(0, protectionBits);
            UpdateLockedRange();
        }

        protected override void RecognizeOperation(byte firstByte)
        {
            if(!Enum.IsDefined(typeof(InternalCommands), firstByte))
            {
                base.RecognizeOperation(firstByte);
                return;
            }

            currentOperation.CommandBytesHandled = 0;
            currentOperation.State = DecodedOperation.OperationState.HandleCommand;

            switch((InternalCommands)firstByte)
            {
            case InternalCommands.ReadStatusRegister2:
                currentOperation.Operation = DecodedOperation.OperationType.ReadRegister;
                currentOperation.Register = (uint)InternalRegisters.Status2;
                break;
            case InternalCommands.ReadStatusRegister3:
                currentOperation.Operation = DecodedOperation.OperationType.ReadRegister;
                currentOperation.Register = (uint)InternalRegisters.Status3;
                break;
            case InternalCommands.WriteStatusRegister2:
                currentOperation.Operation = DecodedOperation.OperationType.WriteRegister;
                currentOperation.Register = (uint)InternalRegisters.Status2;
                break;
            case InternalCommands.WriteStatusRegister3:
                currentOperation.Operation = DecodedOperation.OperationType.WriteRegister;
                currentOperation.Register = (uint)InternalRegisters.Status3;
                break;
            }
        }

        protected override byte ReadRegister(Register register)
        {
            switch((uint)register)
            {
            case (uint)InternalRegisters.Status2:
                return statusRegister2.Read();
            case (uint)InternalRegisters.Status3:
                return statusRegister3.Read();
            default:
                return base.ReadRegister(register);
            }
        }

        protected override void WriteRegister(Register register, byte data)
        {
            if(register != Register.Status && !Enum.IsDefined(typeof(InternalRegisters), (uint)register))
            {
                base.WriteRegister(register, data);
                return;
            }

            if(!WriteEnable)
            {
                this.Log(LogLevel.Error, "Trying to write a register, but write enable latch is not set");
                return;
            }

            // 0x01 writes SR1 and, optionally, SR2 as the second byte; 0x31/0x11 write one register
            var target = register == Register.Status && currentOperation.CommandBytesHandled == 1
                ? (uint)InternalRegisters.Status2
                : (uint)register;
            if(currentOperation.CommandBytesHandled > (register == Register.Status ? 1 : 0))
            {
                this.Log(LogLevel.Warning, "Ignoring extra byte 0x{0:X2} written to {1}", data, register);
                return;
            }

            switch(target)
            {
            case (uint)Register.Status:
                statusRegister.Write(0, data);
                break;
            case (uint)InternalRegisters.Status2:
                statusRegister2.Write(0, data);
                break;
            case (uint)InternalRegisters.Status3:
                statusRegister3.Write(0, data);
                break;
            }
            UpdateLockedRange();
        }

        protected override void WriteToMemory(byte val)
        {
            if(!TryVerifyWriteToMemory(out var position))
            {
                return;
            }
            // Programming can only clear bits; erased cells read 0xFF
            var currentVal = underlyingMemory.ReadByte(position);
            underlyingMemory.WriteByte(position, (byte)(val & currentVal));
        }

        private void UpdateLockedRange()
        {
            if(writeProtectSelection.Value)
            {
                this.Log(LogLevel.Warning, "Individual block protection (WPS=1) is not supported, the memory is left unprotected");
                lockedRange = null;
                return;
            }

            var size = (ulong)UnderlyingMemory.Size;
            var level = (int)blockProtect.Value;
            Range? range;
            if(level == 0)
            {
                range = null;
            }
            else if(level == 7)
            {
                range = new Range(0, size);
            }
            else
            {
                // SEC=0: 1/64 .. 1/2 of the memory; SEC=1: 4KB, 8KB, 16KB, then 32KB
                var protectedSize = sectorProtect.Value
                    ? (ulong)4.KB() << Math.Min(level - 1, 3)
                    : size >> (7 - level);
                range = new Range(topBottom.Value ? 0 : size - protectedSize, protectedSize);
            }

            if(complementProtect.Value)
            {
                if(range == null)
                {
                    range = new Range(0, size);
                }
                else if(range.Value.Size == size)
                {
                    range = null;
                }
                else
                {
                    var unprotectedSize = size - range.Value.Size;
                    range = new Range(range.Value.StartAddress == 0 ? range.Value.Size : 0, unprotectedSize);
                }
            }

            lockedRange = range;
            this.Log(LogLevel.Debug, "Protected range: {0}", lockedRange?.ToString() ?? "none");
        }

        private byte securityRegisterLocks;

        private readonly ByteRegister statusRegister2;
        private readonly ByteRegister statusRegister3;
        private readonly IValueRegisterField blockProtect;
        private readonly IFlagRegisterField topBottom;
        private readonly IFlagRegisterField sectorProtect;
        private readonly IFlagRegisterField complementProtect;
        private readonly IFlagRegisterField writeProtectSelection;

        private const byte ManufacturerId = 0xEF;
        private const byte DefaultMemoryType = 0x40; // W25Q128JV-IQ/JQ; the -IM/JM parts report 0x70
        private const byte DefaultStatusRegister3 = 0x60; // DRV = 25%
        // BP[2:0], TB, SEC, SRP
        private const byte StatusRegisterNonVolatileMask = 0xFC;

        private enum InternalCommands : byte
        {
            ReadStatusRegister2 = 0x35,
            ReadStatusRegister3 = 0x15,
            WriteStatusRegister2 = 0x31,
            WriteStatusRegister3 = 0x11,
        }

        private enum InternalRegisters : uint
        {
            Status2 = Register.FirstNonstandardRegister,
            Status3,
        }
    }
}
