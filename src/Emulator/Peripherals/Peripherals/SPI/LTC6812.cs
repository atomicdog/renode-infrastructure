//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.SPI
{
    // Analog Devices / Linear Technology LTC6812-1 multicell battery
    // stack monitor.  Implements the isoSPI command protocol over a
    // standard ISPIPeripheral interface.
    //
    // Supports daisy chain emulation: set numberOfDevices > 1 to model
    // multiple ICs sharing one isoSPI bus.  Cell voltages for each IC
    // are set via SetCellVoltage(device, cell, value).
    //
    // The values set from the monitor are the analog inputs.  The read
    // commands return the result registers, which hold 0xFFFF after reset
    // or a CLRCELL/CLRAUX and take the inputs only when a conversion
    // command (ADCV, ADAX, ADCVAX, ...) measures that channel.
    //
    // A conversion takes the datasheet's total conversion time for its command,
    // channel selection and ADC mode (MD bits, with IC0's ADCOPT).  The results
    // are latched when the command arrives; the time only shows in PLADC: after
    // it, SDO reads low (0x00) until the conversion ends and high (0xFF) after.
    public class LTC6812 : ISPIPeripheral, IGPIOReceiver
    {
        public LTC6812(IMachine machine, int numberOfDevices = 1)
        {
            this.machine = machine;
            this.numberOfDevices = numberOfDevices;
            cellVoltages = new ushort[numberOfDevices * NumberOfCells];
            gpioVoltages = new ushort[numberOfDevices * NumberOfGpios];
            cellRegisters = new ushort[numberOfDevices * NumberOfCells];
            gpioRegisters = new ushort[numberOfDevices * NumberOfGpios];
            ref2Registers = new ushort[numberOfDevices];
            sumOfCells = new ushort[numberOfDevices];
            configA = new byte[numberOfDevices * 6];
            configB = new byte[numberOfDevices * 6];
            commReg = new byte[numberOfDevices * 6];
            cmdBuffer = new byte[4];
            responseQueue = new Queue<byte>();

            Reset();
        }

        public void Reset()
        {
            state = State.Idle;
            cmdByteIndex = 0;
            writeByteIndex = 0;
            responseQueue.Clear();
            conversionEnd = TimeInterval.Empty;
            polling = false;
            Array.Clear(configA, 0, configA.Length);
            Array.Clear(configB, 0, configB.Length);
            Array.Clear(commReg, 0, commReg.Length);
            i2cReadMode = false;
            i2cRecordIndex = 0;

            // Default cell voltages: 3.6V each (100µV/LSB → 36000)
            for(int i = 0; i < cellVoltages.Length; i++)
            {
                cellVoltages[i] = DefaultCellVoltageRaw;
            }
            // Default GPIO: 0V
            Array.Clear(gpioVoltages, 0, gpioVoltages.Length);
            ClearCellRegisters();
            ClearAuxRegisters();
            // Default internal die temp: ~25°C
            // ITMP = measured_value / 100µV/LSB, T(°C) = ITMP * 100µV / 7.6mV/°C - 276°C
            // For 25°C: (25+276) * 7.6mV / 100µV = 301 * 76 = 22876 = 0x595C
            internalTemp = 0x595C;
            UpdateSumOfCells();
            analogSupply = 50000;
            digitalSupply = 33000;
            revisionCode = 0x01;
        }

        public byte Transmit(byte data)
        {
            // If we have a response queued, return it regardless of incoming data
            if(responseQueue.Count > 0)
            {
                var resp = responseQueue.Dequeue();
                if(responseQueue.Count == 0)
                {
                    // Response fully drained — ready for next command
                    state = State.Idle;
                }
                return resp;
            }

            switch(state)
            {
                case State.Idle:
                case State.ReceivingCommand:
                    // After PLADC, SDO is low while a conversion is running
                    var output = polling && Now < conversionEnd ? (byte)0x00 : (byte)0xFF;
                    state = State.ReceivingCommand;
                    // The LTC isoSPI has no chip-select frame, and the driver prefixes
                    // transactions with wake-up pad bytes, so a fixed 4-byte boundary
                    // desyncs.  Slide a 4-byte window and lock onto the first group
                    // whose command PEC checks out — the PEC is the frame sync.
                    cmdBuffer[0] = cmdBuffer[1];
                    cmdBuffer[1] = cmdBuffer[2];
                    cmdBuffer[2] = cmdBuffer[3];
                    cmdBuffer[3] = data;
                    if(cmdByteIndex < 4)
                    {
                        cmdByteIndex++;
                    }
                    if(cmdByteIndex >= 4
                        && CalculatePec15(cmdBuffer, 0, 2) == (ushort)((cmdBuffer[2] << 8) | cmdBuffer[3]))
                    {
                        cmdByteIndex = 0;
                        ProcessCommand();
                        // If this was a read command, state is now SendingResponse
                        // If write, state is ReceivingData; poll/action, back to Idle
                    }
                    return output;

                case State.ReceivingData:
                    writeBuffer[writeByteIndex++] = data;
                    if(writeByteIndex >= writeTotalBytes)
                    {
                        ProcessWriteData();
                        state = State.Idle;
                    }
                    return 0xFF;

                case State.SendingResponse:
                    // Master is clocking out a byte but we already queued the response
                    // This shouldn't normally happen (response queue was empty)
                    return 0xFF;

                default:
                    return 0xFF;
            }
        }

        public void FinishTransmission()
        {
            state = State.Idle;
            cmdByteIndex = 0;
            writeByteIndex = 0;
            responseQueue.Clear();
            polling = false;
        }

        public void OnGPIO(int number, bool value)
        {
            // CS handling — not used when connected directly to USIC SELO
        }

        // Bytes returned by the I2C slave on the LTC's I2C-over-COMM port.  Each
        // read transaction (START with R/W=1) returns them from index 0; reads past
        // the end return 0xFF, as an absent slave would.  Applies to every IC on
        // this chain.  Kept across Reset, like the rest of the board configuration.
        public void SetCommI2CReadByte(int index, byte value)
        {
            if(index < 0)
            {
                this.Log(LogLevel.Warning, "Invalid COMM I2C read index {0}", index);
                return;
            }
            while(commI2CReadData.Count <= index)
            {
                commI2CReadData.Add(0xFF);
            }
            commI2CReadData[index] = value;
        }

        public void ClearCommI2CReadData()
        {
            commI2CReadData.Clear();
        }

        // Monitor commands for configuring cell voltages.
        // Device index is 0-based (0 = first IC in chain).
        public void SetCellVoltage(int cell, ushort rawValue, int device = 0)
        {
            if(device < 0 || device >= numberOfDevices)
            {
                this.Log(LogLevel.Warning, "Invalid device {0}, must be 0-{1}", device, numberOfDevices - 1);
                return;
            }
            if(cell < 1 || cell > NumberOfCells)
            {
                this.Log(LogLevel.Warning, "Invalid cell number {0}, must be 1-{1}", cell, NumberOfCells);
                return;
            }
            cellVoltages[device * NumberOfCells + cell - 1] = rawValue;
            UpdateSumOfCells();
            this.Log(LogLevel.Info, "IC{0} Cell {1} voltage set to {2} ({3:F4}V)", device, cell, rawValue, rawValue * 0.0001);
        }

        public void SetCellMillivolts(int cell, int millivolts, int device = 0)
        {
            SetCellVoltage(cell, (ushort)(millivolts * 10), device);
        }

        public void SetAllCellMillivolts(int millivolts)
        {
            for(int d = 0; d < numberOfDevices; d++)
            {
                for(int i = 1; i <= NumberOfCells; i++)
                {
                    SetCellMillivolts(i, millivolts, d);
                }
            }
        }

        public void SetGpioVoltage(int gpio, ushort rawValue, int device = 0)
        {
            if(device < 0 || device >= numberOfDevices)
            {
                this.Log(LogLevel.Warning, "Invalid device {0}, must be 0-{1}", device, numberOfDevices - 1);
                return;
            }
            if(gpio < 1 || gpio > NumberOfGpios)
            {
                this.Log(LogLevel.Warning, "Invalid GPIO number {0}, must be 1-{1}", gpio, NumberOfGpios);
                return;
            }
            gpioVoltages[device * NumberOfGpios + gpio - 1] = rawValue;
            this.Log(LogLevel.Info, "IC{0} GPIO{1} voltage set to {2} ({3:F4}V)", device, gpio, rawValue, rawValue * 0.0001);
        }

        public void SetGpioMillivolts(int gpio, int millivolts, int device = 0)
        {
            SetGpioVoltage(gpio, (ushort)(millivolts * 10), device);
        }

        private void ProcessCommand()
        {
            var cmd0 = cmdBuffer[0];
            var cmd1 = cmdBuffer[1];
            var pec0 = cmdBuffer[2];
            var pec1 = cmdBuffer[3];

            // Verify command PEC
            var expectedPec = CalculatePec15(cmdBuffer, 0, 2);
            var receivedPec = (ushort)((pec0 << 8) | pec1);
            if(expectedPec != receivedPec)
            {
                this.Log(LogLevel.Warning, "Command PEC mismatch: expected 0x{0:X4}, received 0x{1:X4}", expectedPec, receivedPec);
                state = State.Idle;
                return;
            }

            // Extract 11-bit command code: CMD0[2:0] << 8 | CMD1[7:0]
            var cc = ((cmd0 & 0x07) << 8) | cmd1;

            this.Log(LogLevel.Debug, "Command: CC=0x{0:X3} (CMD0=0x{1:X2} CMD1=0x{2:X2})", cc, cmd0, cmd1);

            polling = false;
            switch(cc)
            {
                case 0x001: // WRCFGA
                    StartWrite(WriteTarget.ConfigA);
                    break;
                case 0x024: // WRCFGB
                    StartWrite(WriteTarget.ConfigB);
                    break;
                case 0x002: // RDCFGA
                    for(int dev = 0; dev < numberOfDevices; dev++)
                    {
                        var data = new byte[6];
                        Array.Copy(configA, dev * 6, data, 0, 6);
                        QueueReadResponseSingle(data);
                    }
                    state = State.SendingResponse;
                    break;
                case 0x026: // RDCFGB
                    for(int dev = 0; dev < numberOfDevices; dev++)
                    {
                        var data = new byte[6];
                        Array.Copy(configB, dev * 6, data, 0, 6);
                        QueueReadResponseSingle(data);
                    }
                    state = State.SendingResponse;
                    break;
                case 0x004: // RDCVA (cells 1-3)
                    QueueCellVoltageGroup(0);
                    break;
                case 0x006: // RDCVB (cells 4-6)
                    QueueCellVoltageGroup(3);
                    break;
                case 0x008: // RDCVC (cells 7-9)
                    QueueCellVoltageGroup(6);
                    break;
                case 0x00A: // RDCVD (cells 10-12)
                    QueueCellVoltageGroup(9);
                    break;
                case 0x009: // RDCVE (cells 13-15)
                    QueueCellVoltageGroup(12);
                    break;
                case 0x00C: // RDAUXA (GPIO 1-3)
                    QueueGpioGroup(0);
                    break;
                case 0x00E: // RDAUXB (GPIO 4-5 + REF2)
                    QueueGpioGroupB();
                    break;
                case 0x00D: // RDAUXC (GPIO 6-8)
                    QueueGpioGroup(5);
                    break;
                case 0x00F: // RDAUXD (GPIO 9 + spare)
                    QueueGpioGroupD();
                    break;
                case 0x010: // RDSTATA
                    QueueStatusA();
                    break;
                case 0x012: // RDSTATB
                    QueueStatusB();
                    break;
                case 0x711: // CLRCELL
                    ClearCellRegisters();
                    state = State.Idle;
                    break;
                case 0x712: // CLRAUX
                    ClearAuxRegisters();
                    state = State.Idle;
                    break;
                case 0x713: // CLRSTAT
                    state = State.Idle;
                    break;
                case 0x714: // PLADC
                    polling = true;
                    state = State.Idle;
                    break;
                case 0x721: // WRCOMM — write the I2C/SPI COMM registers
                    StartWrite(WriteTarget.Comm);
                    break;
                case 0x723: // STCOMM — clock the COMM registers out on the I2C port
                    // The master follows STCOMM with extra clock pulses (up to 72 SCK)
                    // to drive the I2C lines.  Execute the queued I2C transaction now.
                    // The trailing clock bytes then slide through the command window
                    // like wake-up padding until the next command's PEC locks on, so
                    // no chip-select deassert is needed to resynchronise.
                    ExecuteCommI2C();
                    state = State.Idle;
                    break;
                case 0x722: // RDCOMM — read back the COMM registers
                    for(int dev = 0; dev < numberOfDevices; dev++)
                    {
                        var data = new byte[6];
                        Array.Copy(commReg, dev * 6, data, 0, 6);
                        QueueReadResponseSingle(data);
                    }
                    state = State.SendingResponse;
                    break;
                default:
                    // ADC conversion commands (ADCV, ADAX, ADSTAT, etc.)
                    if((cc & 0x600) != 0)
                    {
                        ConvertChannels(cc);
                        var micros = ConversionMicroseconds(cc);
                        conversionEnd = Now + TimeInterval.FromMicroseconds(micros);
                        this.Log(LogLevel.Debug, "ADC conversion started, CC=0x{0:X3}, {1} us", cc, micros);
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "Unknown command CC=0x{0:X3}", cc);
                    }
                    state = State.Idle;
                    break;
            }
        }

        // A write command is followed by 6 data bytes + 2 PEC bytes per IC in the chain.
        private void StartWrite(WriteTarget target)
        {
            currentWriteTarget = target;
            writeByteIndex = 0;
            writeTotalBytes = numberOfDevices * 8;
            writeBuffer = new byte[writeTotalBytes];
            state = State.ReceivingData;
        }

        private void ProcessWriteData()
        {
            // writeBuffer contains (6 data + 2 PEC) × numberOfDevices
            // In a daisy chain write, data for the last IC comes first
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var offset = dev * 8;
                var dataPec = CalculatePec15(writeBuffer, offset, 6);
                var receivedPec = (ushort)((writeBuffer[offset + 6] << 8) | writeBuffer[offset + 7]);
                if(dataPec != receivedPec)
                {
                    this.Log(LogLevel.Warning, "IC{0} write data PEC mismatch: expected 0x{1:X4}, received 0x{2:X4}",
                        dev, dataPec, receivedPec);
                    continue;
                }

                // In daisy chain, data is shifted through: first 8 bytes = last IC, last 8 = first IC
                // Map: buffer device index → actual IC (reversed)
                var targetDev = numberOfDevices - 1 - dev;
                var cfgOffset = targetDev * 6;
                switch(currentWriteTarget)
                {
                    case WriteTarget.ConfigA:
                        Array.Copy(writeBuffer, offset, configA, cfgOffset, 6);
                        this.Log(LogLevel.Debug, "WRCFGA IC{0}: [{1:X2} {2:X2} {3:X2} {4:X2} {5:X2} {6:X2}]",
                            targetDev, writeBuffer[offset], writeBuffer[offset+1], writeBuffer[offset+2],
                            writeBuffer[offset+3], writeBuffer[offset+4], writeBuffer[offset+5]);
                        break;
                    case WriteTarget.ConfigB:
                        Array.Copy(writeBuffer, offset, configB, cfgOffset, 6);
                        break;
                    case WriteTarget.Comm:
                        Array.Copy(writeBuffer, offset, commReg, cfgOffset, 6);
                        break;
                }
            }
        }

        // Decode the COMM registers as an I2C transaction with the COMM I2C slave.
        // Each IC's 6 COMM bytes hold three comm slots: [ICOM(4b) | D(8b) | FCOM(4b)].
        // ICOM 0x6=START, 0x0=BLANK(data), 0x1=STOP, 0x7=NO-TRANSMIT.  On a read the
        // slave byte is written back into the D field so RDCOMM returns it.  A byte
        // the master sends (the START control byte, or data in a write) reads back
        // FCOM 0111 when the slave ACKs it and 1111 when it NACKs (datasheet read
        // codes); the slave is present when it has read data configured.
        private void ExecuteCommI2C()
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var baseIdx = dev * 6;
                for(int slot = 0; slot < 3; slot++)
                {
                    var hi = baseIdx + slot * 2;
                    var b0 = commReg[hi];
                    var b1 = commReg[hi + 1];
                    var icom = (b0 >> 4) & 0xF;
                    var data = (byte)(((b0 & 0xF) << 4) | (b1 >> 4));
                    switch(icom)
                    {
                        case 0x6: // START/repeated-START: D is the control byte (addr<<1|RW)
                            i2cReadMode = (data & 1) != 0;
                            i2cRecordIndex = 0;
                            this.Log(LogLevel.Debug, "COMM I2C START dev{0} addr=0x{1:X2} {2}", dev, data >> 1, i2cReadMode ? "R" : "W");
                            SetSlaveAcknowledge(hi);
                            break;
                        case 0x0: // BLANK: a data byte
                            if(i2cReadMode)
                            {
                                // Master clocks 0xFF and reads the slave's byte back
                                var val = ReadCommI2CByte(i2cRecordIndex++);
                                this.Log(LogLevel.Debug, "COMM I2C read[{0}] dev{1} -> 0x{2:X2}", i2cRecordIndex - 1, dev, val);
                                commReg[hi] = (byte)((b0 & 0xF0) | (val >> 4));
                                commReg[hi + 1] = (byte)(((val & 0xF) << 4) | (b1 & 0xF));
                            }
                            else
                            {
                                // write byte (e.g. a memory address) — not stored
                                SetSlaveAcknowledge(hi);
                            }
                            break;
                        case 0x1: // STOP
                        case 0x7: // NO TRANSMIT — slot unused
                        default:
                            break;
                    }
                }
            }
        }

        private void SetSlaveAcknowledge(int slotIndex)
        {
            var fcom = commI2CReadData.Count > 0 ? 0x7 : 0xF;
            commReg[slotIndex + 1] = (byte)((commReg[slotIndex + 1] & 0xF0) | fcom);
        }

        private byte ReadCommI2CByte(int index)
        {
            return index < commI2CReadData.Count ? commI2CReadData[index] : (byte)0xFF;
        }

        // Total conversion time of a conversion command, from the datasheet's
        // conversion time tables (Tables 5-12 and Table 37).  Commands whose
        // time is not listed (DIAGN, invalid selections) take none.
        private ulong ConversionMicroseconds(int cc)
        {
            var code = cc & ~0x180;
            uint[] times = null;
            if(code == 0x467 || code == 0x477) // ADCVSC
            {
                times = CellsAndSumTimes;
            }
            else if(code == 0x46F || code == 0x47F) // ADCVAX
            {
                times = CellsAndGpio12Times;
            }
            else if((code & ~0x017) == 0x260 || (code & ~0x057) == 0x228) // ADCV, ADOW
            {
                var ch = code & 0x7;
                times = ch == 0 ? AllCellsTimes : (ch <= 5 ? OneChannelPerAdcTimes : null);
            }
            else if((code & ~0x060) == 0x207) // CVST
            {
                times = AllCellsTimes;
            }
            else if((code & ~0x010) == 0x201) // ADOL
            {
                times = OverlapTimes;
            }
            else if((code & ~0x060) == 0x407) // AXST
            {
                times = AllAuxTimes;
            }
            else if((code & ~0x060) == 0x40F) // STATST
            {
                times = AllStatusTimes;
            }
            else if((code & ~0x007) == 0x460 || (code & ~0x007) == 0x400 || (code & ~0x047) == 0x410) // ADAX, ADAXD, AXOW
            {
                var chg = code & 0x7;
                times = chg == 0 ? AllAuxTimes : (chg <= 4 ? TwoAuxTimes : (chg <= 6 ? OneChannelTimes : null));
            }
            else if((code & ~0x007) == 0x468 || (code & ~0x007) == 0x408) // ADSTAT, ADSTATD
            {
                var chst = code & 0x7;
                times = chst == 0 ? AllStatusTimes : (chst <= 4 ? OneChannelTimes : null);
            }
            return times == null ? 0 : times[AdcModeIndex(cc)];
        }

        // Column of the conversion time tables: 27kHz, 14kHz, 7kHz, 3kHz, 2kHz,
        // 1kHz, 422Hz, 26Hz.  MD picks the mode, ADCOPT (CFGAR0[0]) its variant.
        private int AdcModeIndex(int cc)
        {
            var adcopt = (configA[0] & 0x1) != 0;
            switch((cc >> 7) & 0x3)
            {
                case 1:
                    return adcopt ? 1 : 0;
                case 2:
                    return adcopt ? 3 : 2;
                case 3:
                    return adcopt ? 4 : 7;
                default:
                    return adcopt ? 5 : 6;
            }
        }

        // Latch the analog inputs into the result registers for the channels a
        // conversion command measures.  The MD (speed) bits only change timing.
        private void ConvertChannels(int cc)
        {
            var code = cc & ~0x180;
            if((code & ~0x017) == 0x260 || (code & ~0x057) == 0x228) // ADCV, ADOW: CH selects cells
            {
                ConvertCells(code & 0x7);
            }
            else if(code == 0x467 || code == 0x477) // ADCVSC: all cells + sum of cells
            {
                ConvertCells(0);
            }
            else if(code == 0x46F || code == 0x47F) // ADCVAX: all cells + GPIO1, GPIO2
            {
                ConvertCells(0);
                for(int dev = 0; dev < numberOfDevices; dev++)
                {
                    ConvertGpio(dev, 1);
                    ConvertGpio(dev, 2);
                }
            }
            else if((code >= 0x460 && code <= 0x466) || (code >= 0x400 && code <= 0x406)) // ADAX, ADAXD: CHG selects
            {
                ConvertAux(code & 0x7);
            }
        }

        // CH = 0: all cells; CH = n (1-5): cells n, n+5 and n+10.
        private void ConvertCells(int ch)
        {
            if(ch > 5)
            {
                return;
            }
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                for(int cell = 1; cell <= NumberOfCells; cell++)
                {
                    if(ch == 0 || (cell - 1) % 5 == ch - 1)
                    {
                        var i = dev * NumberOfCells + cell - 1;
                        cellRegisters[i] = cellVoltages[i];
                    }
                }
            }
        }

        // CHG = 0: GPIO1-9 and REF2; 1-4: GPIOn and GPIOn+5; 5: GPIO5; 6: REF2.
        private void ConvertAux(int chg)
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                for(int gpio = 1; gpio <= NumberOfGpios; gpio++)
                {
                    if(chg == 0 || (chg <= 5 && (gpio == chg || gpio == chg + 5)))
                    {
                        ConvertGpio(dev, gpio);
                    }
                }
                if(chg == 0 || chg == 6)
                {
                    ref2Registers[dev] = Ref2VoltageRaw;
                }
            }
        }

        private void ConvertGpio(int device, int gpio)
        {
            var i = device * NumberOfGpios + gpio - 1;
            gpioRegisters[i] = gpioVoltages[i];
        }

        private void ClearCellRegisters()
        {
            for(int i = 0; i < cellRegisters.Length; i++)
            {
                cellRegisters[i] = ClearedRegisterValue;
            }
        }

        private void ClearAuxRegisters()
        {
            for(int i = 0; i < gpioRegisters.Length; i++)
            {
                gpioRegisters[i] = ClearedRegisterValue;
            }
            for(int i = 0; i < ref2Registers.Length; i++)
            {
                ref2Registers[i] = ClearedRegisterValue;
            }
        }

        private void QueueReadResponseSingle(byte[] data)
        {
            // Queue 6 data bytes + 2 PEC bytes for one IC
            var pec = CalculatePec15(data, 0, 6);
            foreach(var b in data)
            {
                responseQueue.Enqueue(b);
            }
            responseQueue.Enqueue((byte)(pec >> 8));
            responseQueue.Enqueue((byte)(pec & 0xFF));
        }

        private void QueueCellVoltageGroup(int startCell)
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var data = new byte[6];
                for(int i = 0; i < 3; i++)
                {
                    var cellIdx = dev * NumberOfCells + startCell + i;
                    var voltage = cellIdx < cellRegisters.Length ? cellRegisters[cellIdx] : (ushort)0;
                    data[i * 2] = (byte)(voltage & 0xFF);
                    data[i * 2 + 1] = (byte)(voltage >> 8);
                }
                QueueReadResponseSingle(data);
            }
            state = State.SendingResponse;
        }

        private void QueueGpioGroup(int startGpio)
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var data = new byte[6];
                for(int i = 0; i < 3; i++)
                {
                    var gpioIdx = dev * NumberOfGpios + startGpio + i;
                    var voltage = gpioIdx < gpioRegisters.Length ? gpioRegisters[gpioIdx] : (ushort)0;
                    data[i * 2] = (byte)(voltage & 0xFF);
                    data[i * 2 + 1] = (byte)(voltage >> 8);
                }
                QueueReadResponseSingle(data);
            }
            state = State.SendingResponse;
        }

        private void QueueGpioGroupB()
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var data = new byte[6];
                var g4 = gpioRegisters[dev * NumberOfGpios + 3];
                var g5 = gpioRegisters[dev * NumberOfGpios + 4];
                var ref2 = ref2Registers[dev];
                data[0] = (byte)(g4 & 0xFF); data[1] = (byte)(g4 >> 8);
                data[2] = (byte)(g5 & 0xFF); data[3] = (byte)(g5 >> 8);
                data[4] = (byte)(ref2 & 0xFF); data[5] = (byte)(ref2 >> 8);
                QueueReadResponseSingle(data);
            }
            state = State.SendingResponse;
        }

        private void QueueGpioGroupD()
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var data = new byte[6];
                var g9 = gpioRegisters[dev * NumberOfGpios + 8];
                data[0] = (byte)(g9 & 0xFF); data[1] = (byte)(g9 >> 8);
                QueueReadResponseSingle(data);
            }
            state = State.SendingResponse;
        }

        private void QueueStatusA()
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var data = new byte[6];
                data[0] = (byte)(sumOfCells[dev] & 0xFF);
                data[1] = (byte)(sumOfCells[dev] >> 8);
                data[2] = (byte)(internalTemp & 0xFF);
                data[3] = (byte)(internalTemp >> 8);
                data[4] = (byte)(analogSupply & 0xFF);
                data[5] = (byte)(analogSupply >> 8);
                QueueReadResponseSingle(data);
            }
            state = State.SendingResponse;
        }

        private void QueueStatusB()
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                var data = new byte[6];
                data[0] = (byte)(digitalSupply & 0xFF);
                data[1] = (byte)(digitalSupply >> 8);
                data[2] = 0x00;
                data[3] = 0x00;
                data[4] = 0x00;
                data[5] = (byte)(revisionCode & 0x0F);
                QueueReadResponseSingle(data);
            }
            state = State.SendingResponse;
        }

        // SC (status register group A) is each IC's own sum of cells at
        // 100µV * 30 per LSB (LTC6812/6813; the 12-cell LTC6811 uses * 20).
        private void UpdateSumOfCells()
        {
            for(int dev = 0; dev < numberOfDevices; dev++)
            {
                uint sum = 0;
                for(int i = 0; i < NumberOfCells; i++)
                {
                    sum += cellVoltages[dev * NumberOfCells + i];
                }
                sumOfCells[dev] = (ushort)(sum / 30);
            }
        }

        // PEC-15 calculation per LTC6812 datasheet
        // Polynomial: x^15 + x^14 + x^10 + x^8 + x^7 + x^4 + x^3 + 1
        private static ushort CalculatePec15(byte[] data, int offset, int length)
        {
            ushort pec = 16; // Initial seed = 0x0010

            for(int i = offset; i < offset + length; i++)
            {
                for(int bit = 7; bit >= 0; bit--)
                {
                    var din = (data[i] >> bit) & 1;
                    var in0 = din ^ ((pec >> 14) & 1);
                    var in3 = in0 ^ ((pec >> 2) & 1);
                    var in4 = in0 ^ ((pec >> 3) & 1);
                    var in7 = in0 ^ ((pec >> 6) & 1);
                    var in8 = in0 ^ ((pec >> 7) & 1);
                    var in10 = in0 ^ ((pec >> 9) & 1);
                    var in14 = in0 ^ ((pec >> 13) & 1);

                    pec = (ushort)(
                        (in14 << 14) |
                        ((pec >> 12) & 1) << 13 |
                        ((pec >> 11) & 1) << 12 |
                        ((pec >> 10) & 1) << 11 |
                        (in10 << 10) |
                        ((pec >> 8) & 1) << 9 |
                        (in8 << 8) |
                        (in7 << 7) |
                        ((pec >> 5) & 1) << 6 |
                        ((pec >> 4) & 1) << 5 |
                        (in4 << 4) |
                        (in3 << 3) |
                        ((pec >> 1) & 1) << 2 |
                        ((pec >> 0) & 1) << 1 |
                        in0
                    );
                }
            }

            // Final PEC is 15-bit value shifted left by 1 (0 appended to LSB)
            return (ushort)((pec << 1) & 0xFFFE);
        }

        private const int NumberOfCells = 15;
        private const int NumberOfGpios = 9;
        private const ushort DefaultCellVoltageRaw = 36000; // 3.6V at 100µV/LSB
        private const ushort Ref2VoltageRaw = 30000; // 3.0V second reference
        private const ushort ClearedRegisterValue = 0xFFFF;

        private TimeInterval Now => machine.ElapsedVirtualTime.TimeElapsed;

        private readonly IMachine machine;
        private readonly int numberOfDevices;
        private readonly ushort[] cellVoltages;
        private readonly ushort[] gpioVoltages;
        private readonly ushort[] cellRegisters;
        private readonly ushort[] gpioRegisters;
        private readonly ushort[] ref2Registers;
        private readonly byte[] configA;
        private readonly byte[] configB;
        private readonly byte[] commReg;
        private readonly byte[] cmdBuffer;
        private readonly Queue<byte> responseQueue;
        private readonly List<byte> commI2CReadData = new List<byte>();

        private byte[] writeBuffer;
        private int cmdByteIndex;
        private int writeByteIndex;
        private int writeTotalBytes;
        private State state;
        private WriteTarget currentWriteTarget;
        private TimeInterval conversionEnd;
        private bool polling;
        private bool i2cReadMode;
        private int i2cRecordIndex;
        private readonly ushort[] sumOfCells;
        private ushort internalTemp;
        private ushort analogSupply;
        private ushort digitalSupply;
        private byte revisionCode;

        // Total conversion times in us, in the column order of AdcModeIndex
        private static readonly uint[] AllCellsTimes = { 937, 1083, 1956, 2537, 3701, 6028, 10683, 167774 };
        private static readonly uint[] OneChannelPerAdcTimes = { 203, 232, 407, 523, 756, 1221, 2152, 33570 };
        private static readonly uint[] CellsAndSumTimes = { 1147, 1322, 2369, 3067, 4463, 7256, 12842, 201351 };
        private static readonly uint[] CellsAndGpio12Times = { 1328, 1531, 2753, 3568, 5197, 8455, 14971, 234899 };
        private static readonly uint[] OverlapTimes = { 384, 442, 791, 1024, 1490, 2420, 4282, 67119 };
        private static readonly uint[] AllAuxTimes = { 1825, 2116, 3862, 5025, 7353, 12007, 21316, 335498 };
        private static readonly uint[] AllStatusTimes = { 742, 858, 1556, 2022, 2953, 4814, 8538, 134211 };
        // Table 37 gives these two rounded
        private static readonly uint[] TwoAuxTimes = { 380, 439, 788, 1000, 1500, 2400, 4300, 67100 };
        private static readonly uint[] OneChannelTimes = { 200, 229, 403, 520, 753, 1200, 2100, 34000 };

        private enum State
        {
            Idle,
            ReceivingCommand,
            ReceivingData,
            SendingResponse,
        }

        private enum WriteTarget
        {
            ConfigA,
            ConfigB,
            Comm,
        }
    }
}
