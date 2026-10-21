//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.SPI
{
    // STMicroelectronics M95640 — 64 Kbit (8 KB) SPI EEPROM.
    // Protocol: WREN, WRDI, RDSR, WRSR, READ, WRITE with 16-bit addresses.
    // Page size: 32 bytes.
    //
    // Optional underlyingMemory backs storage with a sysbus-mapped MappedMemory,
    // so the contents can be pre-seeded with `sysbus LoadBinary <file> <addr>`
    // and persist across machine resets within an emulation session.
    public class STM_M95640 : ISPIPeripheral, IGPIOReceiver
    {
        public STM_M95640(MappedMemory underlyingMemory = null)
        {
            this.underlyingMemory = underlyingMemory;
            if(underlyingMemory == null)
            {
                storage = new byte[Capacity];
                for(int i = 0; i < storage.Length; i++)
                {
                    storage[i] = 0xFF;
                }
            }
            Reset();
        }

        public void Reset()
        {
            state = State.Idle;
            statusRegister = 0;
            writeEnabled = false;
            address = 0;
        }

        public void OnGPIO(int number, bool value)
        {
            // Active-low chip select wired directly to a GPIO: end the transaction
            // when CS is deasserted (goes high).  Behind an SPIMultiplexer this is
            // not called and the multiplexer calls FinishTransmission itself.
            // Without this, a directly-wired EEPROM stays in the READ/WRITE state
            // after its first transaction and swallows every later command.
            if(number == 0 && value)
            {
                FinishTransmission();
            }
        }

        public void FinishTransmission()
        {
            if(state == State.Writing)
            {
                // Commit the write — WIP would be set on real hardware,
                // but we complete instantly.
                writeEnabled = false;
                statusRegister &= unchecked((byte)~StatusWel);
            }
            if((state == State.Reading || state == State.Writing) && currentOpBytes > 0)
            {
                this.Log(LogLevel.Debug, "{0} @ 0x{1:X4} +{2}", state == State.Reading ? "READ" : "WRITE", currentOpStartAddr, currentOpBytes);
            }
            currentOpBytes = 0;
            state = State.Idle;
        }

        public byte Transmit(byte data)
        {
            switch(state)
            {
                case State.Idle:
                    return HandleCommand(data);

                case State.ReadingStatus:
                    state = State.Idle;
                    return GetStatusRegister();

                case State.WritingStatus:
                    statusRegister = (byte)(data & StatusWritableMask);
                    writeEnabled = false;
                    state = State.Idle;
                    return 0xFF;

                case State.AddressHigh:
                    address = (ushort)(data << 8);
                    state = State.AddressLow;
                    return 0xFF;

                case State.AddressLow:
                    address |= data;
                    state = nextStateAfterAddress;
                    currentOpStartAddr = address;
                    currentOpBytes = 0;
                    return 0xFF;

                case State.Reading:
                    var result = ReadByte(address);
                    address = (ushort)((address + 1) % Capacity);
                    currentOpBytes++;
                    return result;

                case State.Writing:
                    WriteByte(address, data);
                    // Address increments within page boundary
                    address = (ushort)((address & ~(PageSize - 1)) | ((address + 1) & (PageSize - 1)));
                    currentOpBytes++;
                    return 0xFF;

                default:
                    this.Log(LogLevel.Warning, "Unexpected state {0}", state);
                    state = State.Idle;
                    return 0xFF;
            }
        }

        private byte HandleCommand(byte cmd)
        {
            switch(cmd)
            {
                case CmdWren:
                    writeEnabled = true;
                    return 0xFF;

                case CmdWrdi:
                    writeEnabled = false;
                    return 0xFF;

                case CmdRdsr:
                    state = State.ReadingStatus;
                    return GetStatusRegister();

                case CmdWrsr:
                    if(!writeEnabled)
                    {
                        this.Log(LogLevel.Warning, "WRSR without WREN — ignoring");
                        return 0xFF;
                    }
                    state = State.WritingStatus;
                    return 0xFF;

                case CmdRead:
                    state = State.AddressHigh;
                    nextStateAfterAddress = State.Reading;
                    return 0xFF;

                case CmdWrite:
                    if(!writeEnabled)
                    {
                        this.Log(LogLevel.Warning, "WRITE without WREN — ignoring");
                        return 0xFF;
                    }
                    state = State.AddressHigh;
                    nextStateAfterAddress = State.Writing;
                    return 0xFF;

                default:
                    this.Log(LogLevel.Warning, "Unknown command 0x{0:X2}", cmd);
                    return 0xFF;
            }
        }

        private byte GetStatusRegister()
        {
            byte sr = statusRegister;
            if(writeEnabled)
            {
                sr |= StatusWel;
            }
            // WIP is always 0 — writes complete instantly
            return sr;
        }

        private byte ReadByte(ushort addr)
        {
            if(addr >= Capacity)
            {
                return 0xFF;
            }
            if(underlyingMemory != null)
            {
                return underlyingMemory.ReadByte(addr);
            }
            return storage[addr];
        }

        private void WriteByte(ushort addr, byte value)
        {
            if(addr >= Capacity)
            {
                return;
            }
            if(underlyingMemory != null)
            {
                underlyingMemory.WriteByte(addr, value);
                return;
            }
            storage[addr] = value;
        }

        private readonly MappedMemory underlyingMemory;
        private readonly byte[] storage;
        private State state;
        private State nextStateAfterAddress;
        private byte statusRegister;
        private bool writeEnabled;
        private ushort address;
        private ushort currentOpStartAddr;
        private int currentOpBytes;

        private const int Capacity = 8192;   // 8 KB
        private const int PageSize = 32;

        // Commands
        private const byte CmdWren  = 0x06;
        private const byte CmdWrdi  = 0x04;
        private const byte CmdRdsr  = 0x05;
        private const byte CmdWrsr  = 0x01;
        private const byte CmdRead  = 0x03;
        private const byte CmdWrite = 0x02;

        // Status register bits
        private const byte StatusWip = 0x01;
        private const byte StatusWel = 0x02;
        private const byte StatusWritableMask = 0x8C;  // BP0, BP1, SRWD

        private enum State
        {
            Idle,
            ReadingStatus,
            WritingStatus,
            AddressHigh,
            AddressLow,
            Reading,
            Writing,
        }
    }
}
