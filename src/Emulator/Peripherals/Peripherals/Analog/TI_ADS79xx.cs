//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;

using Antmicro.Renode.Core;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;

namespace Antmicro.Renode.Peripherals.Analog
{
    // Texas Instruments ADS7950-ADS7961 family: 4/8/12/16-channel, 12/10/8-bit
    // SAR ADCs with a 16-bit SPI frame (e.g. ADS7952: 12 channels, 12 bits).
    //
    // Each frame the master shifts in a mode-control word (DI15-0) and the device
    // shifts out a result: DO15-12 hold the channel address (or the GPIO pins when
    // DI04 was set), DO11-0 the conversion result, MSB-aligned.  The channel
    // selected in frame n is sampled at the start of frame n+1 and output in frame
    // n+2.  Manual, Auto-1 and Auto-2 modes and their program registers are
    // modelled.
    //
    // GPIO0-3 are outputs 0-3: the GPIO program register's DI03-00 make a pin an
    // output, and a mode-control frame with DI11 set drives the output pins from
    // its DI03-00.  Inputs read low; the pins' alarm, range and power-down
    // functions are not modelled.
    //
    // Frames are delimited by the active-low chip select on GPIO 0 when it is
    // connected, and otherwise by counting two bytes per frame.
    public class TI_ADS79xx : ISPIPeripheral, IGPIOReceiver, INumberedGPIOOutput
    {
        public TI_ADS79xx(int numberOfChannels = 12, int resolutionBits = 12, int referenceMillivolts = 2500)
        {
            if(numberOfChannels != 4 && numberOfChannels != 8 && numberOfChannels != 12 && numberOfChannels != 16)
            {
                throw new ConstructionException($"{nameof(numberOfChannels)} must be 4, 8, 12 or 16, got {numberOfChannels}");
            }
            if(resolutionBits != 8 && resolutionBits != 10 && resolutionBits != 12)
            {
                throw new ConstructionException($"{nameof(resolutionBits)} must be 8, 10 or 12, got {resolutionBits}");
            }
            if(referenceMillivolts <= 0)
            {
                throw new ConstructionException($"{nameof(referenceMillivolts)} must be positive, got {referenceMillivolts}");
            }
            this.numberOfChannels = numberOfChannels;
            this.resolutionBits = resolutionBits;
            this.referenceMillivolts = referenceMillivolts;
            channelMillivolts = new int[numberOfChannels];
            var gpios = new Dictionary<int, IGPIO>();
            for(var i = 0; i < NumberOfGpios; i++)
            {
                gpios[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(gpios);
            Reset();
        }

        public void Reset()
        {
            // Power-up state: manual mode, channel 0, range 1 (0 to Vref)
            mode = Mode.Manual;
            manualChannel = 0;
            autoCounter = 0;
            auto1Channels = (ushort)((1 << numberOfChannels) - 1);
            auto2LastChannel = numberOfChannels - 1;
            doubleRange = false;
            gpioOnOutput = false;
            expectingAuto1Mask = false;
            sampledWord = 0;
            outputWord = 0;
            command = 0;
            byteIndex = 0;
            gpioDirections = 0;
            gpioLevels = 0;
            UpdateGpioOutputs();
        }

        public byte Transmit(byte data)
        {
            if(byteIndex == 0)
            {
                StartFrame();
            }
            byte output;
            if(byteIndex == 0)
            {
                command = (ushort)(data << 8);
                output = (byte)(outputWord >> 8);
                byteIndex = 1;
            }
            else
            {
                command |= data;
                output = (byte)outputWord;
                byteIndex = 0;
                ProcessCommand(command);
            }
            return output;
        }

        public void FinishTransmission()
        {
            byteIndex = 0;
        }

        // GPIO 0: chip select, active low.  Either edge starts a new frame.
        public void OnGPIO(int number, bool value)
        {
            if(number != 0)
            {
                this.Log(LogLevel.Warning, "Unexpected GPIO {0}", number);
                return;
            }
            if(byteIndex != 0)
            {
                this.Log(LogLevel.Debug, "Chip select changed mid-frame, frame dropped");
            }
            byteIndex = 0;
        }

        public void SetChannelMillivolts(int channel, int millivolts)
        {
            if(channel < 0 || channel >= numberOfChannels)
            {
                this.Log(LogLevel.Warning, "Invalid channel {0}, must be 0-{1}", channel, numberOfChannels - 1);
                return;
            }
            channelMillivolts[channel] = millivolts;
            this.Log(LogLevel.Info, "Channel {0} set to {1} mV", channel, millivolts);
        }

        public int GetChannelMillivolts(int channel)
        {
            if(channel < 0 || channel >= numberOfChannels)
            {
                this.Log(LogLevel.Warning, "Invalid channel {0}, must be 0-{1}", channel, numberOfChannels - 1);
                return 0;
            }
            return channelMillivolts[channel];
        }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        // At the start of a frame the device outputs the conversion sampled at the
        // start of the previous frame, then samples the channel selected for this one.
        private void StartFrame()
        {
            outputWord = sampledWord;
            int channel;
            switch(mode)
            {
                case Mode.Auto1:
                case Mode.Auto2:
                    channel = autoCounter;
                    AdvanceAutoCounter();
                    break;
                default:
                    channel = manualChannel;
                    break;
            }
            sampledWord = (ushort)(((gpioOnOutput ? gpioLevels & gpioDirections : channel) << 12) | Convert(channel));
        }

        private void ProcessCommand(ushort word)
        {
            if(expectingAuto1Mask)
            {
                // Second frame of the Auto-1 program sequence: one bit per channel
                expectingAuto1Mask = false;
                auto1Channels = (ushort)(word & ((1 << numberOfChannels) - 1));
                this.Log(LogLevel.Debug, "Auto-1 channels set to 0x{0:X4}", auto1Channels);
                return;
            }

            var program = (word & ProgramBit) != 0;
            switch(word >> 12)
            {
                case 0x0: // continue in the selected mode
                    return;
                case 0x1:
                    mode = Mode.Manual;
                    if(program)
                    {
                        manualChannel = (word >> 7) & 0xF;
                        if(manualChannel >= numberOfChannels)
                        {
                            this.Log(LogLevel.Warning, "Manual mode selected channel {0}, the device has {1}", manualChannel, numberOfChannels);
                        }
                    }
                    break;
                case 0x2:
                    if(mode != Mode.Auto1 || (program && (word & CounterResetBit) != 0))
                    {
                        autoCounter = FirstAuto1Channel();
                    }
                    mode = Mode.Auto1;
                    break;
                case 0x3:
                    if(mode != Mode.Auto2 || (program && (word & CounterResetBit) != 0))
                    {
                        autoCounter = 0;
                    }
                    mode = Mode.Auto2;
                    break;
                case 0x4: // GPIO program register: DI03-00 pin directions (1 = output)
                    gpioDirections = word & GpioMask;
                    this.Log(LogLevel.Debug, "GPIO directions set to 0x{0:X}", gpioDirections);
                    UpdateGpioOutputs();
                    return;
                case 0x8:
                    expectingAuto1Mask = true;
                    return;
                case 0x9:
                    auto2LastChannel = Math.Min((word >> 6) & 0xF, numberOfChannels - 1);
                    return;
                default:
                    this.Log(LogLevel.Warning, "Unknown mode control word 0x{0:X4}", word);
                    return;
            }

            if(program)
            {
                doubleRange = (word & RangeBit) != 0;
                gpioOnOutput = (word & GpioOnOutputBit) != 0;
                gpioLevels = word & GpioMask;
                UpdateGpioOutputs();
                if((word & PowerDownBit) != 0)
                {
                    this.Log(LogLevel.Debug, "Power-down requested, conversions continue");
                }
            }
        }

        private void UpdateGpioOutputs()
        {
            for(var i = 0; i < NumberOfGpios; i++)
            {
                Connections[i].Set((gpioDirections & gpioLevels & (1 << i)) != 0);
            }
        }

        private int FirstAuto1Channel()
        {
            for(var i = 0; i < numberOfChannels; i++)
            {
                if((auto1Channels & (1 << i)) != 0)
                {
                    return i;
                }
            }
            return 0;
        }

        private void AdvanceAutoCounter()
        {
            if(mode == Mode.Auto2)
            {
                autoCounter = autoCounter >= auto2LastChannel ? 0 : autoCounter + 1;
                return;
            }
            for(var i = 1; i <= numberOfChannels; i++)
            {
                var next = (autoCounter + i) % numberOfChannels;
                if((auto1Channels & (1 << next)) != 0)
                {
                    autoCounter = next;
                    return;
                }
            }
        }

        // The result, MSB-aligned in DO11-0, clamped to the selected range.
        private ushort Convert(int channel)
        {
            if(channel >= numberOfChannels)
            {
                return 0;
            }
            var fullScale = referenceMillivolts * (doubleRange ? 2 : 1);
            var maxCode = (1 << resolutionBits) - 1;
            var code = (int)Math.Round((double)channelMillivolts[channel] * (1 << resolutionBits) / fullScale);
            code = Math.Max(0, Math.Min(maxCode, code));
            return (ushort)(code << (12 - resolutionBits));
        }

        private Mode mode;
        private int manualChannel;
        private int autoCounter;
        private ushort auto1Channels;
        private int auto2LastChannel;
        private bool doubleRange;
        private bool gpioOnOutput;
        private bool expectingAuto1Mask;
        private ushort sampledWord;
        private ushort outputWord;
        private ushort command;
        private int byteIndex;
        private int gpioDirections;
        private int gpioLevels;

        private readonly int numberOfChannels;
        private readonly int resolutionBits;
        private readonly int referenceMillivolts;
        private readonly int[] channelMillivolts;

        private const int ProgramBit = 1 << 11;
        private const int CounterResetBit = 1 << 10;
        private const int RangeBit = 1 << 6;
        private const int PowerDownBit = 1 << 5;
        private const int GpioOnOutputBit = 1 << 4;
        private const int GpioMask = 0xF;
        private const int NumberOfGpios = 4;

        private enum Mode
        {
            Manual,
            Auto1,
            Auto2,
        }
    }
}
