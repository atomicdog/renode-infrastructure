//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Linq;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CPU;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.UART
{
    public class XMC4XXX_UART : UARTBase, IDoubleWordPeripheral, IKnownSize,
        IPeripheralContainer<ISPIPeripheral, NullRegistrationPoint>
    {
        // peripheralClockFrequency is fPB, the clock the channel's fractional
        // divider runs from. When set, SPI master words take the time the
        // programmed FDR/BRG/SCTR/PCR settings give them; when 0 they are
        // instant.
        //
        // By default a timed word completes asynchronously, with the receive
        // event when its last bit has been shifted in, while the CPU runs on.
        // The trailing and inter-word delays after it only hold off the next
        // word. With SpiTimingStallsCpu the CPU waits in the TBUF write
        // instead, until the receive event. Firmware that starts a
        // byte, then blocks on a completion its receive ISR never yields to,
        // relies on its code path outlasting the wire; the emulated CPU is too
        // fast for that, and each byte would wait for the next scheduler tick.
        public XMC4XXX_UART(IMachine machine, long peripheralClockFrequency = 0) : base(machine)
        {
            this.machine = machine;
            this.peripheralClockFrequency = peripheralClockFrequency;
            irqs = Enumerable.Range(0, NumberOfInterruptNodes).Select(_ => new GPIO()).ToArray();

            var registersMap = new Dictionary<long, DoubleWordRegister>();
            DefineRegisters(registersMap);
            registers = new DoubleWordRegisterCollection(this, registersMap);
        }

        public override void Reset()
        {
            base.Reset();
            stbiFlag = false;
            spiCsAsserted = false;
            spiBusyUntil = TimeInterval.Empty;
            // Words still in flight belong to the old state: drop them.
            spiGeneration++;
            registers.Reset();
            // Ensure all IRQ lines are low and tracking is synchronized
            // after register reset (which may trigger UpdateInterrupts callbacks).
            for(int i = 0; i < NumberOfInterruptNodes; i++)
            {
                irqs[i].Set(false);
                irqLastState[i] = false;
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public bool SpiTimingStallsCpu { get; set; }

        public void WriteDoubleWord(long offset, uint val)
        {
            registers.Write(offset, val);
        }

        // Six service request outputs, matching the USIC's SR0-SR5 nodes.
        // Connect in .repl as: [0-5] -> nvic@[84-89]  (for USIC0)
        public GPIO IRQ0 => irqs[0];
        public GPIO IRQ1 => irqs[1];
        public GPIO IRQ2 => irqs[2];
        public GPIO IRQ3 => irqs[3];
        public GPIO IRQ4 => irqs[4];
        public GPIO IRQ5 => irqs[5];

        public override Bits StopBits => stopBits.Value ? Bits.Two : Bits.One;
        public override Parity ParityBit => Parity.None;
        public override uint BaudRate => 115200;

        public long Size => 0x200;

        // IPeripheralContainer<ISPIPeripheral> — allows attaching an SPI slave
        // (or SPIMultiplexer) when the USIC channel operates in SPI master mode.
        public void Register(ISPIPeripheral peripheral, NullRegistrationPoint registrationPoint)
        {
            if(spiSlaveDevice != null)
            {
                throw new RegistrationException("Cannot register more than one SPI peripheral.");
            }
            Machine.RegisterAsAChildOf(this, peripheral, registrationPoint);
            spiSlaveDevice = peripheral;
        }

        public void Unregister(ISPIPeripheral peripheral)
        {
            if(peripheral != spiSlaveDevice)
            {
                throw new RegistrationException("Trying to unregister not registered device.");
            }
            Machine.UnregisterAsAChildOf(this, peripheral);
            spiSlaveDevice = null;
        }

        public IEnumerable<NullRegistrationPoint> GetRegistrationPoints(ISPIPeripheral peripheral)
        {
            if(peripheral != spiSlaveDevice)
            {
                throw new RegistrationException("Trying to obtain a registration point for a not registered device.");
            }
            return new[] { NullRegistrationPoint.Instance };
        }

        IEnumerable<IRegistered<ISPIPeripheral, NullRegistrationPoint>> IPeripheralContainer<ISPIPeripheral, NullRegistrationPoint>.Children
        {
            get
            {
                if(spiSlaveDevice == null)
                {
                    return Enumerable.Empty<IRegistered<ISPIPeripheral, NullRegistrationPoint>>();
                }
                return new[] { Registered.Create(spiSlaveDevice, NullRegistrationPoint.Instance) };
            }
        }

        // Some firmware processes commands on LF (0x0A), but most terminal
        // emulators (including Renode's GUI) send only CR (0x0D) on Enter.
        // When set, every received CR is followed by an LF.
        public bool AppendLineFeedAfterCarriageReturn { get; set; }

        public override void WriteChar(byte value)
        {
            base.WriteChar(value);
            if(AppendLineFeedAfterCarriageReturn && value == 0x0D)
            {
                base.WriteChar(0x0A);
            }
        }

        protected override void CharWritten()
        {
            rxIndicationFlag.Value = true;
            // Also kick the TX path — receiving a character often means the
            // firmware will produce output (command response, echo).  The
            // STBI interrupt chain may have stopped since the last TX drain;
            // re-assert it so the ISR picks up any queued TX data.
            stbiFlag = true;
            this.NoisyLog("CharWritten: RX char queued, STBI kicked");
            UpdateInterrupts();
        }

        protected override void QueueEmptied()
        {
            UpdateInterrupts();
        }

        private void UpdateInterrupts()
        {
            var activeMode = channelMode.Value == 1 || channelMode.Value == 2;

            // Compute desired state for each SR node.  Multiple interrupt
            // sources can map to the same node, so OR all contributions.
            var nodeState = new bool[NumberOfInterruptNodes];

            if(receiveIndicationInterruptEnable.Value && rxIndicationFlag.Value)
            {
                var node = (int)rxInterruptNode.Value;
                if(node < NumberOfInterruptNodes)
                {
                    nodeState[node] = true;
                }
            }

            if(transmitBufferInterruptEnable.Value)
            {
                // TX buffer is always ready (instant transmit) — fire TBINP while enabled
                var node = (int)txBufferInterruptNode.Value;
                if(node < NumberOfInterruptNodes)
                {
                    nodeState[node] = true;
                }
            }

            // FIFO mode: TX buffer event (STBI) → STBINP node.
            if(txFifoSize.Value > 0 && stbiFlag && activeMode)
            {
                var node = (int)txFifoInterruptNode.Value;
                if(node < NumberOfInterruptNodes)
                {
                    nodeState[node] = true;
                }
            }

            // FIFO mode: RX data available (SRBI) → SRBINP node
            if(rxFifoSize.Value > 0 && RxCount > 0 && activeMode)
            {
                var node = (int)rxFifoInterruptNode.Value;
                if(node < NumberOfInterruptNodes)
                {
                    nodeState[node] = true;
                }
            }

            // Apply computed state to each IRQ line.  Only call Set when
            // the value differs from what we last drove to avoid spurious
            // NVIC Running-flag transitions on shared interrupt lines.
            for(int i = 0; i < NumberOfInterruptNodes; i++)
            {
                if(nodeState[i] != irqLastState[i])
                {
                    irqs[i].Set(nodeState[i]);
                    irqLastState[i] = nodeState[i];
                }
            }
        }

        // Manually trigger the TX FIFO buffer event (STBI), causing the
        // STBINP interrupt to fire.  Use from the monitor after the firmware
        // has initialised the UART to kick-start interrupt-driven TX.
        public void KickTx()
        {
            this.Log(LogLevel.Info, "KickTx: manually asserting STBI");
            stbiFlag = true;
            UpdateInterrupts();
        }

        // Helpers that read from either the UART base queue or the SPI response queue
        // depending on the active channel mode.
        private int RxCount => channelMode.Value == 1 ? spiResponseQueue.Count : Count;

        private byte RxDequeue()
        {
            if(channelMode.Value == 1 && spiResponseQueue.Count > 0)
            {
                return spiResponseQueue.Dequeue();
            }
            TryGetCharacter(out var b);
            return b;
        }

        private byte RxPeek()
        {
            if(channelMode.Value == 1 && spiResponseQueue.Count > 0)
            {
                return spiResponseQueue.Peek();
            }
            if(Count > 0)
            {
                TryGetCharacter(out var b, true);
                return b;
            }
            return 0;
        }

        private void DoTransmit(byte value)
        {
            if(channelMode.Value == 1 && spiSlaveDevice != null)
            {
                // SPI master mode: full-duplex exchange with attached slave.
                // The slave sees the byte now; the firmware sees it received
                // once the word has been shifted out.
                var response = spiSlaveDevice.Transmit(value);
                this.NoisyLog("SPI TX=0x{0:X2} RX=0x{1:X2}", value, response);
                var wordTime = GetSpiWordTime(out var receiveTime);
                if(wordTime != lastSpiWordTime)
                {
                    this.Log(LogLevel.Debug, "SPI word time {0:F2} us, received after {1:F2} us", wordTime.TotalMicroseconds, receiveTime.TotalMicroseconds);
                    lastSpiWordTime = wordTime;
                }
                if(wordTime == TimeInterval.Empty)
                {
                    CompleteSpiWord(response);
                    return;
                }
                var onCpu = machine.SystemBus.TryGetCurrentCPU(out var cpu);
                if(onCpu)
                {
                    cpu.SyncTime();
                }
                // Words go out back to back: this one starts when the shifter
                // is free. Measured on the clock source ScheduleAction counts
                // its delay from.
                var now = machine.ClockSource.CurrentValue;
                var stall = SpiTimingStallsCpu && onCpu && cpu is BaseCPU;
                var baseCpu = cpu as BaseCPU;
                if(stall)
                {
                    // An earlier stall the CPU has not served yet is still
                    // ahead of it: the CPU's own time includes it.
                    now += TimeInterval.FromCPUCycles(baseCpu.SkipInstructions, baseCpu.PerformanceInMips, out var _);
                }
                var start = spiBusyUntil > now ? spiBusyUntil : now;
                spiBusyUntil = start + wordTime;
                var untilReceived = start + receiveTime - now;
                if(stall)
                {
                    // Whole instruction slots, so the skip is exact
                    var cycles = (ulong)Math.Ceiling(untilReceived.TotalMicroseconds * baseCpu.PerformanceInMips);
                    baseCpu.SkipTime(TimeInterval.FromCPUCycles(cycles, baseCpu.PerformanceInMips, out var _));
                    CompleteSpiWord(response);
                    return;
                }
                var generation = spiGeneration;
                machine.ScheduleAction(untilReceived, _ =>
                {
                    if(generation == spiGeneration)
                    {
                        CompleteSpiWord(response);
                    }
                });
            }
            else if(channelMode.Value == 1)
            {
                // SPI mode but no slave device — silently discard.
                // Do NOT set rxIndicationFlag or fire interrupts, as that
                // creates a tight TX→IRQ→TX loop on unconnected SPI channels.
                return;
            }
            else
            {
                // UART mode: send character to console.
                TransmitCharacter(value);
                // Re-assert STBI for interrupt-driven UART TX.
                // On real hardware, STBI fires when the FIFO fill level drops
                // below LIMIT.  Since we consume bytes instantly (level always 0),
                // every transmit is a level-crossing event.
                stbiFlag = true;
                this.NoisyLog("TX byte 0x{0:X2} '{1}', STBI re-asserted", value,
                    value >= 0x20 && value < 0x7F ? (char)value : '.');
                UpdateInterrupts();
            }
        }

        private void CompleteSpiWord(byte response)
        {
            spiResponseQueue.Enqueue(response);
            // Set PSR.RIF to signal data received; firmware clears via PSCR.
            rxIndicationFlag.Value = true;
            UpdateInterrupts();
        }

        // Time to shift one SPI word, from the clock tree in the reference
        // manual: fFD from FDR (normal or fractional divider), fPIN = fFD
        // (BRG.CLKSEL = 0), fPPP = fPIN, or fPIN / 2 with BRG.PPPEN,
        // fPDIV = fPPP / (PDIV + 1), fSCLK = fPDIV / 2. A word is SCTR.WLE + 1
        // SCLK periods, plus:
        // - the inter-word (or next-frame) delay when PCR.TIWEN is set:
        //   (PCTQ1 + 1) * (DCTQ1 + 1) periods of the clock PCR.CTQSEL1 picks;
        // - when the master generates MSLS (PCR.MSLSEN) and a frame is one
        //   word long (SCTR.FLE <= WLE), a leading and a trailing delay of
        //   (BRG.PCTQ + 1) * (BRG.DCTQ + 1) periods of the BRG.CTQSEL clock.
        // Longer frames get their leading/trailing delays only on the first
        // and last word on hardware; that is not modelled.
        // receiveTime is when the receive event comes: after the leading delay
        // and the bits. On an XMC4500 MBB (1 MHz SCLK, 2 us delays) the
        // receive ISR started ~10.5 us after the TBUF write, not the 14 us of
        // the whole word.
        // Empty when the timing is unknown, which keeps the transfer instant.
        private TimeInterval GetSpiWordTime(out TimeInterval receiveTime)
        {
            receiveTime = TimeInterval.Empty;
            if(peripheralClockFrequency <= 0 || clockSelect.Value != 0)
            {
                return TimeInterval.Empty;
            }
            double fd;
            switch(dividerMode.Value)
            {
                case 1:
                    fd = peripheralClockFrequency / (1024.0 - dividerStep.Value);
                    break;
                case 2:
                    fd = (double)peripheralClockFrequency * dividerStep.Value / 1024.0;
                    break;
                default:
                    return TimeInterval.Empty;
            }
            var fppp = prescalerEnable.Value ? fd / 2 : fd;
            var fpdiv = fppp / (preDivider.Value + 1);
            var fsclk = fpdiv / 2;
            if(fsclk <= 0)
            {
                return TimeInterval.Empty;
            }
            var seconds = (wordLength.Value + 1) / fsclk;

            var pcr = protocolControl.Value;
            double frameDelay = 0;
            if((pcr & 1u) != 0 && frameLength.Value <= wordLength.Value)
            {
                var ftq = TimeQuantumClock(timeQuantumSelect.Value, fpdiv, fppp, fsclk);
                frameDelay = (frameDelayPrescaler.Value + 1) * (frameDelayCount.Value + 1) / ftq;
            }
            receiveTime = TimeInterval.FromSeconds(frameDelay + seconds);
            seconds += 2 * frameDelay;

            // In SSC mode PCR holds MSLSEN [0], CTQSEL1 [5:4], PCTQ1 [7:6],
            // DCTQ1 [12:8] and TIWEN [24]; its fields are named for ASC mode.
            if((pcr & (1u << 24)) != 0)
            {
                var ftq = TimeQuantumClock((pcr >> 4) & 0x3, fpdiv, fppp, fsclk);
                seconds += (((pcr >> 6) & 0x3) + 1) * (((pcr >> 8) & 0x1F) + 1) / ftq;
            }
            return TimeInterval.FromSeconds(seconds);
        }

        // The clock a CTQSEL field selects: fPDIV, fPPP, fSCLK or fMCLK.
        private static double TimeQuantumClock(ulong select, double fpdiv, double fppp, double fsclk)
        {
            switch(select)
            {
                case 0:
                    return fpdiv;
                case 1:
                    return fppp;
                case 2:
                    return fsclk;
                default:
                    return fppp / 2;
            }
        }

        private void DefineRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // KSCFG @ 0x00C — Kernel State Configuration
            map.Add((long)Registers.KernelStateConfiguration, new DoubleWordRegister(this)
                .WithFlag(0, name: "MODEN")
                .WithTaggedFlag("BPMODEN", 1)
                .WithReservedBits(2, 2)
                .WithTag("NOMCFG", 4, 2)
                .WithReservedBits(6, 1)
                .WithTaggedFlag("BPNOM", 7)
                .WithTag("SUMCFG", 8, 2)
                .WithReservedBits(10, 1)
                .WithTaggedFlag("BPSUM", 11)
                .WithReservedBits(12, 20)
            );

            // FDR @ 0x010 — Fractional Divider
            map.Add((long)Registers.FractionalDivider, new DoubleWordRegister(this)
                .WithValueField(0, 10, out dividerStep, name: "STEP")
                .WithReservedBits(10, 4)
                .WithValueField(14, 2, out dividerMode, name: "DM")
                .WithTag("RESULT", 16, 10)
                .WithReservedBits(26, 6)
            );

            // BRG @ 0x014 — Baud Rate Generator
            map.Add((long)Registers.BaudRateGenerator, new DoubleWordRegister(this)
                .WithValueField(0, 2, out clockSelect, name: "CLKSEL")
                .WithReservedBits(2, 1)
                .WithTaggedFlag("TMEN", 3)
                .WithFlag(4, out prescalerEnable, name: "PPPEN")
                .WithReservedBits(5, 1)
                .WithValueField(6, 2, out timeQuantumSelect, name: "CTQSEL")
                .WithValueField(8, 2, out frameDelayPrescaler, name: "PCTQ")
                .WithValueField(10, 5, out frameDelayCount, name: "DCTQ")
                .WithValueField(16, 10, out preDivider, name: "PDIV")
                .WithReservedBits(26, 2)
                .WithTaggedFlag("SCLKOSEL", 28)
                .WithTaggedFlag("MCLKCFG", 29)
                .WithTag("SCLKCFG", 30, 2)
            );

            // INPR @ 0x018 — Interrupt Node Pointer
            map.Add((long)Registers.InterruptNodePointer, new DoubleWordRegister(this)
                .WithTag("TSINP", 0, 3)
                .WithReservedBits(3, 1)
                .WithValueField(4, 3, out txBufferInterruptNode, name: "TBINP")
                .WithReservedBits(7, 1)
                .WithValueField(8, 3, out rxInterruptNode, name: "RINP")
                .WithReservedBits(11, 1)
                .WithTag("AINP", 12, 3)
                .WithReservedBits(15, 1)
                .WithTag("PINP", 16, 3)
                .WithReservedBits(19, 13)
                .WithWriteCallback((_, __) => UpdateInterrupts())
            );

            // DX0CR @ 0x01C — Input Control Register 0
            map.Add((long)Registers.InputControl0, new DoubleWordRegister(this)
                .WithTag("DSEL", 0, 3)
                .WithReservedBits(3, 1)
                .WithTaggedFlag("INSW", 4)
                .WithTaggedFlag("DFEN", 5)
                .WithTaggedFlag("DSEN", 6)
                .WithReservedBits(7, 1)
                .WithTaggedFlag("DPOL", 8)
                .WithTaggedFlag("SFSEL", 9)
                .WithTag("CM", 10, 2)
                .WithReservedBits(12, 3)
                .WithTaggedFlag("DXS", 15)
                .WithReservedBits(16, 16)
            );

            // DX1CR–DX5CR @ 0x020–0x030
            for(int i = 1; i <= 5; i++)
            {
                var idx = i;
                map.Add((long)Registers.InputControl0 + idx * 4, new DoubleWordRegister(this)
                    .WithTag($"DX{idx}CR", 0, 32)
                );
            }

            // SCTR @ 0x034 — Shift Control
            map.Add((long)Registers.ShiftControl, new DoubleWordRegister(this)
                .WithTaggedFlag("SDIR", 0)
                .WithTaggedFlag("PDL", 1)
                .WithTag("DSM", 2, 2)
                .WithTaggedFlag("HPCDIR", 4)
                .WithReservedBits(5, 1)
                .WithTag("DOCFG", 6, 2)
                .WithTag("TRM", 8, 2)
                .WithReservedBits(10, 6)
                .WithValueField(16, 6, out frameLength, name: "FLE")
                .WithReservedBits(22, 2)
                .WithValueField(24, 4, out wordLength, name: "WLE")
                .WithReservedBits(28, 4)
            );

            // TCSR @ 0x038 — Transmit Control/Status
            // TDV (bit 7) = 0: transmit buffer ready to accept data
            map.Add((long)Registers.TransmitControlStatus, new DoubleWordRegister(this)
                .WithTaggedFlag("WLEMD", 0)
                .WithTaggedFlag("SELMD", 1)
                .WithTaggedFlag("FLEMD", 2)
                .WithTaggedFlag("WAMD", 3)
                .WithTaggedFlag("HPCMD", 4)
                .WithTaggedFlag("SOF", 5)
                .WithTaggedFlag("EOF", 6)
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => false, name: "TDV")
                .WithTaggedFlag("TDSSM", 8)
                .WithReservedBits(9, 1)
                .WithTag("TDEN", 10, 2)
                .WithTaggedFlag("TDVTR", 12)
                .WithReservedBits(13, 19)
            );

            // PCR @ 0x03C — Protocol Control
            // In UART mode: SMD, STPB, SP, PL, etc.
            // In SPI mode: MSLSEN, SELCTR, SELINV, FEM, SELO[0-7].
            // All lower bits must be stored so read-modify-write works
            // (the DAVE SPI driver does RMW to set SELO, RNIEN, etc.).
            protocolControl = new DoubleWordRegister(this);
            map.Add((long)Registers.ProtocolControl, protocolControl
                .WithFlag(0, name: "SMD_MSLSEN")
                .WithFlag(1, out stopBits, name: "STPB_SELCTR")
                .WithFlag(2, name: "IDM_SELINV")
                .WithFlag(3, name: "SBIEN_FEM")
                .WithFlag(4, name: "CDEN")
                .WithFlag(5, name: "RNIEN")
                .WithFlag(6, name: "FEIEN")
                .WithFlag(7, name: "FFIEN")
                .WithValueField(8, 5, name: "SP")
                .WithValueField(13, 3, name: "PL")
                .WithFlag(16, name: "RSTEN")
                .WithFlag(17, name: "TSTEN")
                .WithReservedBits(18, 6)
                .WithValueField(24, 8, name: "SELO", writeCallback: (oldVal, newVal) =>
                {
                    if(channelMode.Value != 1)
                    {
                        return;
                    }
                    if((uint)oldVal != (uint)newVal)
                    {
                        this.NoisyLog("SELO changed 0x{0:X2}→0x{1:X2}", (uint)oldVal, (uint)newVal);
                    }
                    if(spiSlaveDevice == null)
                    {
                        return;
                    }
                    // Bits that transitioned from 1→0 = CS deassert → FinishTransmission
                    var deasserted = (uint)oldVal & ~(uint)newVal;
                    if(deasserted != 0)
                    {
                        spiSlaveDevice.FinishTransmission();
                    }
                })
            );

            // CCR @ 0x040 — Channel Control
            map.Add((long)Registers.ChannelControl, new DoubleWordRegister(this)
                .WithValueField(0, 4, out channelMode, name: "MODE")
                .WithReservedBits(4, 2)
                .WithTag("HPCEN", 6, 2)
                .WithTag("PM", 8, 2)
                .WithTaggedFlag("RSIEN", 10)
                .WithTaggedFlag("DLIEN", 11)
                .WithTaggedFlag("TSIEN", 12)
                .WithFlag(13, out transmitBufferInterruptEnable, name: "TBIEN")
                .WithFlag(14, out receiveIndicationInterruptEnable, name: "RIEN")
                .WithTaggedFlag("AIEN", 15)
                .WithTaggedFlag("BRGIEN", 16)
                .WithReservedBits(17, 15)
                .WithWriteCallback((_, __) =>
                {
                    // When UART mode is enabled with TX FIFO configured,
                    // assert STBI (TX buffer empty = ready for data).
                    // Only auto-init STBI for UART mode. SPI mode STBI is managed
                    // by the SPI master driver which handles its own TX kickstart.
                    if(channelMode.Value == 2 && txFifoSize.Value > 0)
                    {
                        stbiFlag = true;
                    }
                    UpdateInterrupts();
                })
            );

            // CMTR @ 0x044 — Capture Mode Timer
            map.Add((long)Registers.CaptureModeTimer, new DoubleWordRegister(this)
                .WithTag("CTV", 0, 10)
                .WithReservedBits(10, 22)
            );

            // PSR_ASCMode @ 0x048 — Protocol Status (UART)
            map.Add((long)Registers.ProtocolStatus, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => true, name: "TXIDLE")
                .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => true, name: "RXIDLE")
                .WithTaggedFlag("SBD", 2)
                .WithTaggedFlag("COL", 3)
                .WithTaggedFlag("RNS", 4)
                .WithTaggedFlag("FER0", 5)
                .WithTaggedFlag("FER1", 6)
                .WithTaggedFlag("RFF", 7)
                .WithFlag(8, FieldMode.Read, valueProviderCallback: _ => true, name: "TFF")
                .WithTaggedFlag("BUSY", 9)
                .WithTaggedFlag("RSIF", 10)
                .WithTaggedFlag("DLIF", 11)
                .WithTaggedFlag("TSIF", 12)
                .WithTaggedFlag("TBIF", 13)
                .WithFlag(14, out rxIndicationFlag, FieldMode.Read | FieldMode.WriteZeroToClear, name: "RIF")
                .WithTaggedFlag("AIF", 15)
                .WithTaggedFlag("BRGIF", 16)
                .WithReservedBits(17, 15)
                .WithWriteCallback((_, __) => UpdateInterrupts())
            );

            // PSCR @ 0x04C — Protocol Status Clear
            // Write-only register: writing 1 to any bit clears the
            // corresponding PSR flag.  We track RIF (bit 14) and
            // IO0 (bit 16) for SPI CS deassert.
            map.Add((long)Registers.ProtocolStatusClear, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PSCR", writeCallback: (_, val) =>
                {
                    if((val & (1u << 14)) != 0)
                    {
                        rxIndicationFlag.Value = false;
                        UpdateInterrupts();
                    }
                    // Bits 16-21 clear IO0-IO5 flags.  When IO0 transitions
                    // from asserted to cleared in SPI mode, that is a CS
                    // deassert → end of SPI transaction.
                    if((val & 0x3F0000u) != 0 && spiCsAsserted && channelMode.Value == 1 && spiSlaveDevice != null)
                    {
                        spiCsAsserted = false;
                        this.NoisyLog("PSCR: IO clear 0x{0:X} — SPI CS deasserted, FinishTransmission", (uint)(val >> 16) & 0x3F);
                        spiSlaveDevice.FinishTransmission();
                    }
                })
            );

            // FMR @ 0x068 — Flag Modification Register
            // Write-only: sets/clears protocol status flags.
            // In SPI mode, the DAVE driver uses SIO0 (bit 16) to assert CS
            // before a transfer, and PSCR bit 16 to deassert CS after.
            // We track IO0 state here so FinishTransmission fires on deassert.
            map.Add((long)Registers.FlagModification, new DoubleWordRegister(this)
                .WithTag("MTDV", 0, 2)
                .WithReservedBits(2, 2)
                .WithTag("ATVC", 4, 2)
                .WithReservedBits(6, 8)
                .WithFlag(14, FieldMode.Write, name: "CRDV0")
                .WithFlag(15, FieldMode.Write, name: "CRDV1")
                .WithValueField(16, 6, FieldMode.Write, name: "SIO", writeCallback: (_, val) =>
                {
                    if(channelMode.Value == 1 && spiSlaveDevice != null && val != 0)
                    {
                        spiCsAsserted = true;
                        this.NoisyLog("FMR: SIO=0x{0:X} — SPI CS asserted", (uint)val);
                    }
                })
                .WithReservedBits(22, 10)
            );

            // RBUFSR @ 0x050 — Receive Buffer Status (combined RBUF0/RBUF1 status)
            map.Add((long)Registers.ReceiveBufferStatus, new DoubleWordRegister(this)
                .WithTag("WLEN", 0, 4)
                .WithReservedBits(4, 2)
                .WithTaggedFlag("SOF", 6)
                .WithReservedBits(7, 1)
                .WithTaggedFlag("PAR", 8)
                .WithTaggedFlag("PERR", 9)
                .WithReservedBits(10, 3)
                .WithFlag(13, FieldMode.Read, valueProviderCallback: _ => RxCount > 0, name: "RDV0")
                .WithTag("DS0", 14, 2)
                .WithReservedBits(16, 16)
            );

            // RBUF @ 0x054 — Receive Buffer (non-FIFO read, dequeues one byte)
            map.Add((long)Registers.ReceiveBuffer, new DoubleWordRegister(this)
                .WithValueField(0, 16, FieldMode.Read, valueProviderCallback: _ =>
                {
                    var b = RxDequeue();
                    UpdateInterrupts();
                    return b;
                }, name: "DSR")
                .WithReservedBits(16, 16)
            );

            // RBUFD @ 0x058 — Receive Buffer Debug (non-destructive peek)
            map.Add((long)Registers.ReceiveBufferDebug, new DoubleWordRegister(this)
                .WithValueField(0, 16, FieldMode.Read, valueProviderCallback: _ =>
                {
                    if(RxCount > 0)
                    {
                        return RxPeek();
                    }
                    return 0;
                }, name: "DSR")
                .WithReservedBits(16, 16)
            );

            // TBUF[0..31] @ 0x080–0x0FC — Transmit Buffer direct registers
            for(int i = 0; i < 32; i++)
            {
                map.Add((long)Registers.TransmitBufferInput + i * 4, new DoubleWordRegister(this)
                    .WithValueField(0, 16, name: "TDATA", writeCallback: (_, v) => DoTransmit((byte)v))
                    .WithReservedBits(16, 16)
                );
            }

            // BYP @ 0x100 — Bypass Register
            map.Add((long)Registers.Bypass, new DoubleWordRegister(this)
                .WithTag("BDATA", 0, 16)
                .WithReservedBits(16, 16)
            );

            // BYPCR @ 0x104 — Bypass Control
            map.Add((long)Registers.BypassControl, new DoubleWordRegister(this)
                .WithTag("BWLE", 0, 4)
                .WithReservedBits(4, 4)
                .WithTaggedFlag("BDSSM", 8)
                .WithReservedBits(9, 1)
                .WithTag("BDEN", 10, 2)
                .WithTaggedFlag("BDVTR", 12)
                .WithTaggedFlag("BPRIO", 13)
                .WithReservedBits(14, 1)
                .WithTaggedFlag("BDV", 15)
                .WithTag("BSELO", 16, 5)
                .WithTaggedFlag("BHPC", 21)
                .WithReservedBits(22, 10)
            );

            // TBCTR @ 0x108 — Transmit Buffer Control
            // XMC4500 field layout (from SVD):
            //   0-5: DPTR, 8-13: LIMIT, 15: STBTSEL,
            //   16-18: STBINP, 19-21: ATBINP, 24-26: SIZE, 30: LOF
            map.Add((long)Registers.TransmitBufferControl, new DoubleWordRegister(this)
                .WithTag("DPTR", 0, 6)
                .WithReservedBits(6, 2)
                .WithTag("LIMIT", 8, 6)
                .WithReservedBits(14, 1)
                .WithTaggedFlag("STBTSEL", 15)
                .WithValueField(16, 3, out txFifoInterruptNode, name: "STBINP")
                .WithTag("ATBINP", 19, 3)
                .WithReservedBits(22, 2)
                .WithValueField(24, 3, out txFifoSize, name: "SIZE")
                .WithReservedBits(27, 3)
                .WithTaggedFlag("LOF", 30)
                .WithTaggedFlag("TBFLVL", 31)
            );

            // RBCTR @ 0x10C — Receive Buffer Control
            // XMC4500 field layout (from SVD):
            //   0-5: DPTR, 8-13: LIMIT, 14: SRBTM, 15: SRBTEN,
            //   16-18: SRBINP, 19-21: ARBINP, 24-26: SIZE, 30: LOF, 31: RCIM
            map.Add((long)Registers.ReceiveBufferControl, new DoubleWordRegister(this)
                .WithTag("DPTR", 0, 6)
                .WithReservedBits(6, 2)
                .WithTag("LIMIT", 8, 6)
                .WithTaggedFlag("SRBTM", 14)
                .WithTaggedFlag("SRBTEN", 15)
                .WithValueField(16, 3, out rxFifoInterruptNode, name: "SRBINP")
                .WithTag("ARBINP", 19, 3)
                .WithReservedBits(22, 2)
                .WithValueField(24, 3, out rxFifoSize, name: "SIZE")
                .WithReservedBits(27, 3)
                .WithTaggedFlag("LOF", 30)
                .WithTaggedFlag("RCIM", 31)
                .WithWriteCallback((_, __) => UpdateInterrupts())
            );

            // TRBPTR @ 0x110 — Transmit/Receive Buffer Pointer
            map.Add((long)Registers.TransmitReceiveBufferPointer, new DoubleWordRegister(this)
                .WithTag("TDIPTR", 0, 6)
                .WithReservedBits(6, 2)
                .WithTag("TDOPTR", 8, 6)
                .WithReservedBits(14, 2)
                .WithTag("RDIPTR", 16, 6)
                .WithReservedBits(22, 2)
                .WithTag("RDOPTR", 24, 6)
                .WithReservedBits(30, 2)
            );

            // TRBSR @ 0x114 — Transmit/Receive Buffer Status
            map.Add((long)Registers.TransmitReceiveBufferStatus, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => RxCount > 0, name: "SRBI")
                .WithTaggedFlag("RBERI", 1)
                .WithTaggedFlag("ARBI", 2)
                .WithFlag(3, FieldMode.Read, valueProviderCallback: _ => RxCount == 0, name: "REMPTY")
                .WithFlag(4, FieldMode.Read, valueProviderCallback: _ => false, name: "RFULL")
                .WithTaggedFlag("RBUS", 5)
                .WithTaggedFlag("SRBT", 6)
                .WithReservedBits(7, 1)
                .WithFlag(8, FieldMode.Read, valueProviderCallback: _ => stbiFlag, name: "STBI")
                .WithTaggedFlag("TBERI", 9)
                .WithReservedBits(10, 1)
                .WithFlag(11, FieldMode.Read, valueProviderCallback: _ => true, name: "TEMPTY")
                .WithFlag(12, FieldMode.Read, valueProviderCallback: _ => false, name: "TFULL")
                .WithTaggedFlag("TBUS", 13)
                .WithTaggedFlag("STBT", 14)
                .WithReservedBits(15, 1)
                .WithValueField(16, 7, FieldMode.Read, valueProviderCallback: _ => (ulong)RxCount, name: "RBFLVL")
                .WithReservedBits(23, 1)
                .WithValueField(24, 7, FieldMode.Read, valueProviderCallback: _ => 0, name: "TBFLVL")
                .WithReservedBits(31, 1)
            );

            // TRBSCR @ 0x118 — Transmit/Receive Buffer Status Clear
            // All bits are write-1-to-clear.  SRBI/RBERI/ARBI are computed
            // dynamically from queue state so clearing is a no-op.
            map.Add((long)Registers.TransmitReceiveBufferStatusClear, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Write, name: "CSRBI", writeCallback: (_, __) => { })
                .WithFlag(1, FieldMode.Write, name: "CRBERI", writeCallback: (_, __) => { })
                .WithFlag(2, FieldMode.Write, name: "CARBI", writeCallback: (_, __) => { })
                .WithReservedBits(3, 5)
                .WithFlag(8, FieldMode.Write, name: "CSTBI", writeCallback: (_, val) =>
                {
                    if(val)
                    {
                        this.NoisyLog("CSTBI: clearing STBI flag");
                        stbiFlag = false;
                        UpdateInterrupts();
                    }
                })
                .WithFlag(9, FieldMode.Write, name: "CTBERI", writeCallback: (_, __) => { })
                .WithReservedBits(10, 4)
                .WithFlag(14, FieldMode.Write, name: "FLUSHTB", writeCallback: (_, val) =>
                {
                    if(val)
                    {
                        this.NoisyLog("FLUSHTB: flushing TX buffer");
                    }
                })
                .WithFlag(15, FieldMode.Write, name: "FLUSHRB", writeCallback: (_, val) =>
                {
                    if(val)
                    {
                        this.NoisyLog("FLUSHRB: flushing RX buffer");
                    }
                })
                .WithReservedBits(16, 16)
            );

            // OUTR @ 0x11C — Receive FIFO Output (dequeues)
            map.Add((long)Registers.ReceiveFIFOOutput, new DoubleWordRegister(this)
                .WithValueField(0, 16, FieldMode.Read, valueProviderCallback: _ =>
                {
                    var b = RxDequeue();
                    UpdateInterrupts();
                    return b;
                }, name: "DSR")
                .WithValueField(16, 5, FieldMode.Read, valueProviderCallback: _ => 0, name: "RCI")
                .WithReservedBits(21, 11)
            );

            // OUTDR @ 0x120 — Receive FIFO Output Debug (non-destructive)
            map.Add((long)Registers.ReceiveFIFODebug, new DoubleWordRegister(this)
                .WithValueField(0, 16, FieldMode.Read, valueProviderCallback: _ =>
                {
                    if(RxCount > 0)
                    {
                        return RxPeek();
                    }
                    return 0;
                }, name: "DSR")
                .WithValueField(16, 5, FieldMode.Read, valueProviderCallback: _ => 0, name: "RCI")
                .WithReservedBits(21, 11)
            );

            // IN[0..31] @ 0x180–0x1FC — Transmit FIFO Input
            for(int i = 0; i < 32; i++)
            {
                map.Add((long)Registers.TransmitFIFOInput + i * 4, new DoubleWordRegister(this)
                    .WithValueField(0, 16, name: "TDATA", writeCallback: (_, v) => DoTransmit((byte)v))
                    .WithReservedBits(16, 16)
                );
            }
        }

        private readonly GPIO[] irqs;
        private readonly DoubleWordRegisterCollection registers;

        private IFlagRegisterField receiveIndicationInterruptEnable;  // CCR.RIEN
        private IFlagRegisterField transmitBufferInterruptEnable;      // CCR.TBIEN
        private IFlagRegisterField rxIndicationFlag;                    // PSR.RIF
        private IFlagRegisterField stopBits;                            // PCR.STPB
        private IValueRegisterField rxInterruptNode;                    // INPR.RINP
        private IValueRegisterField txBufferInterruptNode;              // INPR.TBINP
        private IValueRegisterField channelMode;                         // CCR.MODE
        private IValueRegisterField txFifoInterruptNode;                // TBCTR.STBINP
        private IValueRegisterField rxFifoInterruptNode;                // RBCTR.SRBINP
        private IValueRegisterField txFifoSize;                         // TBCTR.SIZE
        private IValueRegisterField rxFifoSize;                         // RBCTR.SIZE
        private bool stbiFlag;                                           // TX buffer event flag
        private readonly bool[] irqLastState = new bool[NumberOfInterruptNodes]; // last driven GPIO state
        private bool spiCsAsserted;                                          // SPI CS state from FMR SIO / PSCR IO
        private ISPIPeripheral spiSlaveDevice;                           // attached SPI slave (or mux)
        private readonly Queue<byte> spiResponseQueue = new Queue<byte>(); // SPI RX responses
        private readonly IMachine machine;
        private readonly long peripheralClockFrequency;                  // fPB, 0 = untimed SPI
        private IValueRegisterField dividerStep;                         // FDR.STEP
        private IValueRegisterField dividerMode;                         // FDR.DM
        private IValueRegisterField clockSelect;                         // BRG.CLKSEL
        private IFlagRegisterField prescalerEnable;                      // BRG.PPPEN
        private IValueRegisterField preDivider;                          // BRG.PDIV
        private IValueRegisterField timeQuantumSelect;                   // BRG.CTQSEL
        private IValueRegisterField frameDelayPrescaler;                 // BRG.PCTQ
        private IValueRegisterField frameDelayCount;                     // BRG.DCTQ
        private IValueRegisterField frameLength;                         // SCTR.FLE
        private IValueRegisterField wordLength;                          // SCTR.WLE
        private DoubleWordRegister protocolControl;                      // PCR
        private TimeInterval spiBusyUntil;                               // end of the last SPI word in flight
        private TimeInterval lastSpiWordTime;                            // last word time logged
        private int spiGeneration;                                       // bumped on reset

        private const int NumberOfInterruptNodes = 6;

        private enum Registers : long
        {
            KernelStateConfiguration         = 0x00C,
            FractionalDivider                = 0x010,
            BaudRateGenerator                = 0x014,
            InterruptNodePointer             = 0x018,
            InputControl0                    = 0x01C,
            // DX1CR–DX5CR: 0x020–0x030 (dynamic)
            ShiftControl                     = 0x034,
            TransmitControlStatus            = 0x038,
            ProtocolControl                  = 0x03C,
            ChannelControl                   = 0x040,
            CaptureModeTimer                 = 0x044,
            ProtocolStatus                   = 0x048,
            ProtocolStatusClear              = 0x04C,
            ReceiveBufferStatus              = 0x050,
            ReceiveBuffer                    = 0x054,
            ReceiveBufferDebug               = 0x058,
            // RBUF0/RBUF1/RBUF01SR: 0x05C–0x064 (not mapped — firmware uses OUTR)
            FlagModification                 = 0x068,
            TransmitBufferInput              = 0x080,  // [0..31] × 4 bytes
            Bypass                           = 0x100,
            BypassControl                    = 0x104,
            TransmitBufferControl            = 0x108,
            ReceiveBufferControl             = 0x10C,
            TransmitReceiveBufferPointer     = 0x110,
            TransmitReceiveBufferStatus      = 0x114,
            TransmitReceiveBufferStatusClear = 0x118,
            ReceiveFIFOOutput                = 0x11C,
            ReceiveFIFODebug                 = 0x120,
            TransmitFIFOInput                = 0x180,  // [0..31] × 4 bytes
        }
    }
}
