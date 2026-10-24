//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Globalization;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    // Infineon XMC4xxx System Control Unit (SCU).
    // Covers SCU_GENERAL, SCU_INTERRUPT, SCU_HIBERNATE, SCU_RESET,
    // SCU_CLK, SCU_OSC, SCU_PLL sub-blocks within a single 4 KB region
    // starting at 0x50004000.
    //
    // This is a minimal model that fakes PLL lock, oscillator ready,
    // clock mux status, and peripheral reset deassert — enough to get
    // XMClib / DAVE-generated startup code through SystemInit.
    public class XMC4XXX_SCU : IDoubleWordPeripheral, IKnownSize, IProvidesRegisterCollection<DoubleWordRegisterCollection>
    {
        public XMC4XXX_SCU(IMachine machine)
        {
            this.machine = machine;
            var registersMap = new Dictionary<long, DoubleWordRegister>();

            DefineGeneralRegisters(registersMap);
            DefineInterruptRegisters(registersMap);
            DefinePowerRegisters(registersMap);
            DefineHibernateRegisters(registersMap);
            DefineResetRegisters(registersMap);
            DefineClockRegisters(registersMap);
            DefineOscRegisters(registersMap);
            DefinePllRegisters(registersMap);
            DefineTrapRegisters(registersMap);
            DefineRtcRegisters(registersMap);

            RegistersCollection = new DoubleWordRegisterCollection(this, registersMap);
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            // After reset, peripheral resets are asserted
            prstat0 = 0x00010F9FuL;
            prstat1 = 0x000001F9uL;
            prstat2 = 0x000000F6uL;
            prstat3 = 0x00000004uL;
            // PLL starts powered down
            pllPoweredDown = true;
            usbPllPoweredDown = true;
            systemPllVcoBypass = true;
            oscHpEnabled = false;
            // Clock gate status: all gated by default
            clkstat = 0u;
            // Hibernate domain initially off
            hibDomainEnabled = false;
            // Service requests; the RTC (hibernate domain) keeps its own state
            srraw = 0;
            srmsk = 0;
            UpdateInterrupt();
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public long Size => 0x1000;

        // SCU service request (SCU_0, NVIC IRQ 0): asserted while SRSTAT
        // (SRRAW & SRMSK) is not zero.
        public GPIO IRQ { get; } = new GPIO();

        // The RTC's calendar, "yyyy-MM-dd HH:mm:ss". The RTC is in the hibernate
        // domain: it keeps its time and its enable across a system reset. Setting it
        // also starts it, as on a board whose RTC was already running.
        public string RtcTime
        {
            get
            {
                var now = RtcNow();
                return string.Format("{0:D4}-{1:MM-dd HH:mm:ss}", now.Year - RtcYearOffset, now);
            }
            set
            {
                var parsed = DateTime.ParseExact(value, "yyyy-MM-dd HH:mm:ss", CultureInfo.InvariantCulture);
                rtcEnabled = true;
                RebaseRtc(parsed.AddYears(RtcYearOffset));
            }
        }

        public DoubleWordRegisterCollection RegistersCollection { get; }

        // ────────────────────────────────────────────
        //  SCU_GENERAL  (base + 0x000)
        // ────────────────────────────────────────────
        private void DefineGeneralRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // ID @ 0x000 — Module ID (read-only, reset = 0x00A0C000)
            map.Add(0x000, new DoubleWordRegister(this, 0x00A0C000u)
                .WithTag("MOD_REV", 0, 8)
                .WithTag("MOD_TYPE", 8, 8)
                .WithTag("MOD_NUMBER", 16, 16)
            );

            // IDCHIP @ 0x004 — Chip ID
            // XMC4500-F144K1024: 0x00004B01 (from reference manual)
            map.Add(0x004, new DoubleWordRegister(this, 0x00004B01u)
                .WithValueField(0, 32, FieldMode.Read, name: "IDCHIP")
            );

            // IDMANUF @ 0x008 — Manufacturer ID (Infineon = 0x00001820)
            map.Add(0x008, new DoubleWordRegister(this, 0x00001820u)
                .WithValueField(0, 32, FieldMode.Read, name: "IDMANUF")
            );

            // STCON @ 0x010 — Startup Configuration
            map.Add(0x010, new DoubleWordRegister(this)
                .WithValueField(0, 2, FieldMode.Read, name: "HWCON")  // boot mode pins
                .WithReservedBits(2, 6)
                .WithValueField(8, 4, name: "SWCON",
                    writeCallback: (_, val) =>
                    {
                        swconValue = (uint)val;
                        if(val != 0)
                        {
                            this.Log(LogLevel.Info, "SWCON set to 0x{0:X} — bootloader/alternate boot requested", val);
                        }
                    },
                    valueProviderCallback: _ => swconValue)
                .WithReservedBits(12, 20)
            );

            // GPR0 @ 0x02C — General Purpose Register 0
            map.Add(0x02C, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "GPR0")
            );

            // GPR1 @ 0x030 — General Purpose Register 1
            map.Add(0x030, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "GPR1")
            );

            // CCUCON @ 0x04C — CCU Control Register
            map.Add(0x04C, new DoubleWordRegister(this)
                .WithTaggedFlag("GSC40", 0)
                .WithTaggedFlag("GSC41", 1)
                .WithTaggedFlag("GSC42", 2)
                .WithTaggedFlag("GSC43", 3)
                .WithReservedBits(4, 4)
                .WithTaggedFlag("GSC80", 8)
                .WithTaggedFlag("GSC81", 9)
                .WithReservedBits(10, 22)
            );

            // DTSCON @ 0x08C — Die Temperature Sensor Control
            map.Add(0x08C, new DoubleWordRegister(this)
                .WithTaggedFlag("PWD", 0)
                .WithTaggedFlag("START", 1)
                .WithReservedBits(2, 14)
                .WithTag("OFFSET", 16, 7)
                .WithTag("GAIN", 24, 6)
                .WithTaggedFlag("RINTSEL", 30)
                .WithReservedBits(31, 1)
            );

            // DTSSTAT @ 0x090 — Die Temperature Sensor Status
            map.Add(0x090, new DoubleWordRegister(this)
                .WithValueField(0, 10, FieldMode.Read, valueProviderCallback: _ => 0x180, name: "RESULT") // ~25°C
                .WithReservedBits(10, 4)
                .WithFlag(14, FieldMode.Read, valueProviderCallback: _ => true, name: "RDY")
                .WithFlag(15, FieldMode.Read, valueProviderCallback: _ => false, name: "BUSY")
                .WithReservedBits(16, 16)
            );

            // SDMMCDEL @ 0x09C — SD-MMC Delay Control
            map.Add(0x09C, new DoubleWordRegister(this)
                .WithTag("TAPEN", 0, 4)
                .WithTag("TAESSION", 4, 4)
                .WithReservedBits(8, 24)
            );

            // G0ORCEN @ 0x0A0 — Out of Range Comparator Enable 0
            map.Add(0x0A0, new DoubleWordRegister(this)
                .WithTag("G0ORCEN", 0, 32)
            );

            // G1ORCEN @ 0x0A4 — Out of Range Comparator Enable 1
            map.Add(0x0A4, new DoubleWordRegister(this)
                .WithTag("G1ORCEN", 0, 32)
            );

            // MIRRSTS @ 0x0C4 — Mirror Write Status (all ready = 0)
            // Firmware polls this waiting for bits to clear before writing HIB regs.
            map.Add(0x0C4, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => false, name: "HDCLR_MIR")
                .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => false, name: "HDSET_MIR")
                .WithFlag(2, FieldMode.Read, valueProviderCallback: _ => false, name: "HDCR_MIR")
                .WithReservedBits(3, 1)
                .WithFlag(4, FieldMode.Read, valueProviderCallback: _ => false, name: "OSCSICTRL_MIR")
                .WithReservedBits(5, 1)
                .WithFlag(6, FieldMode.Read, valueProviderCallback: _ => false, name: "OSCULCTRL_MIR")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_CTR_MIR")
                .WithFlag(8, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_ATIM0_MIR")
                .WithFlag(9, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_ATIM1_MIR")
                .WithFlag(10, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_TIM0_MIR")
                .WithFlag(11, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_TIM1_MIR")
                .WithFlag(12, FieldMode.Read, valueProviderCallback: _ => false, name: "RMX_MIR")
                .WithFlag(13, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_MSKSR_MIR")
                .WithFlag(14, FieldMode.Read, valueProviderCallback: _ => false, name: "RTC_CLRSR_MIR")
                .WithReservedBits(15, 17)
            );

            // RMACR @ 0x0C8 — Retention Memory Access Control
            map.Add(0x0C8, new DoubleWordRegister(this)
                .WithTag("RMACR", 0, 32)
            );

            // RMDATA @ 0x0CC — Retention Memory Access Data
            map.Add(0x0CC, new DoubleWordRegister(this)
                .WithTag("RMDATA", 0, 32)
            );
        }

        // ────────────────────────────────────────────
        //  SCU_INTERRUPT  (base + 0x074)
        //  Event bits in SRRAW (from XMClib):
        //    bit 17: HDCLR mirror write done
        //    bit 18: HDSET mirror write done
        //    bit 19: HDCR mirror write done
        //    bit 21: OSCSICTRL mirror write done
        //    bit 23: OSCULCTRL mirror write done
        //    bit 24: OSCULSTAT event
        //    bit 25: RTC_CTR mirror write done
        //    bit 26: RTC_ATIM0 mirror write done
        //    bit 27: RTC_ATIM1 mirror write done
        //    bit 28: RTC_TIM0 mirror write done
        //    bit 29: RTC_TIM1 mirror write done
        // ────────────────────────────────────────────
        private void DefineInterruptRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // SCU_INTERRUPT base = 0x50004074; register offsets from that base:
            //   SRSTAT: +0x00 (= 0x074), SRRAW: +0x04 (= 0x078),
            //   SRMSK: +0x08 (= 0x07C), SRCLR: +0x0C (= 0x080), SRSET: +0x10 (= 0x084)

            // SRSTAT @ 0x074 — Service Request Status (SRRAW & SRMSK)
            map.Add(0x074, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "SRSTAT",
                    valueProviderCallback: _ => srraw & srmsk)
            );

            // SRRAW @ 0x078 — Raw Service Request Status (read-only)
            map.Add(0x078, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "SRRAW",
                    valueProviderCallback: _ => srraw)
            );

            // SRMSK @ 0x07C — Service Request Mask
            map.Add(0x07C, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "SRMSK",
                    valueProviderCallback: _ => srmsk,
                    writeCallback: (_, val) => { srmsk = val; UpdateInterrupt(); })
            );

            // SRCLR @ 0x080 — Service Request Clear (write-1-to-clear SRRAW bits)
            map.Add(0x080, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SRCLR",
                    writeCallback: (_, val) => { srraw &= ~val; UpdateInterrupt(); })
            );

            // SRSET @ 0x084 — Service Request Set (write-1-to-set SRRAW bits)
            map.Add(0x084, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "SRSET",
                    writeCallback: (_, val) => { srraw |= val; UpdateInterrupt(); })
            );
        }

        // Signal that a hibernate mirror register write completed instantly.
        private void SignalMirrorWriteComplete(ulong eventBit)
        {
            srraw |= eventBit;
            UpdateInterrupt();
        }

        private void UpdateInterrupt()
        {
            IRQ.Set((srraw & srmsk) != 0);
        }

        // ────────────────────────────────────────────
        //  SCU_POWER  (base + 0x200)
        // ────────────────────────────────────────────
        private void DefinePowerRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // PWRSTAT @ 0x200 — Power Status
            map.Add(0x200, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hibDomainEnabled, name: "HIBEN")
                .WithReservedBits(1, 15)
                .WithFlag(16, FieldMode.Read, valueProviderCallback: _ => usbPhyActive, name: "USBPHYPDQ")
                .WithReservedBits(17, 15)
            );

            // PWRSET @ 0x204 — Power Set Control (write-1-to-enable)
            map.Add(0x204, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Write, name: "HIB",
                    writeCallback: (_, val) => { if(val) hibDomainEnabled = true; })
                .WithReservedBits(1, 15)
                .WithFlag(16, FieldMode.Write, name: "USBPHYPDQ",
                    writeCallback: (_, val) => { if(val) usbPhyActive = true; })
                .WithReservedBits(17, 15)
            );

            // PWRCLR @ 0x208 — Power Clear Control (write-1-to-disable)
            map.Add(0x208, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Write, name: "HIB",
                    writeCallback: (_, val) => { if(val) hibDomainEnabled = false; })
                .WithReservedBits(1, 15)
                .WithFlag(16, FieldMode.Write, name: "USBPHYPDQ",
                    writeCallback: (_, val) => { if(val) usbPhyActive = false; })
                .WithReservedBits(17, 15)
            );
        }

        // ────────────────────────────────────────────
        //  SCU_HIBERNATE  (base + 0x300)
        // ────────────────────────────────────────────
        private void DefineHibernateRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // HDSTAT @ 0x300 — Hibernate Domain Status
            map.Add(0x300, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => hibDomainEnabled, name: "EPEV")
                .WithTaggedFlag("ENEV", 1)
                .WithTaggedFlag("RTCEV", 2)
                .WithTaggedFlag("ULPWDG", 3)
                .WithReservedBits(4, 28)
            );

            // HDCLR @ 0x304 — Hibernate Domain Status Clear (mirror register)
            map.Add(0x304, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "HDCLR",
                    writeCallback: (_, __) => SignalMirrorWriteComplete(SrrawHdclr))
            );

            // HDSET @ 0x308 — Hibernate Domain Status Set (mirror register)
            map.Add(0x308, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "HDSET",
                    writeCallback: (_, __) => SignalMirrorWriteComplete(SrrawHdset))
            );

            // HDCR @ 0x30C — Hibernate Domain Control (mirror register)
            map.Add(0x30C, new DoubleWordRegister(this)
                .WithFlag(0, name: "WKPEP")
                .WithWriteCallback((_, __) => SignalMirrorWriteComplete(SrrawHdcr))
                .WithTag("WKPEP_SEL", 1, 3)
                .WithTaggedFlag("ENEV", 4)
                .WithTaggedFlag("RTCEV", 5)
                .WithReservedBits(6, 2)
                .WithTaggedFlag("RTCF", 8)
                .WithTaggedFlag("STDBYF", 9)
                .WithTaggedFlag("WAKESEL", 10)
                .WithReservedBits(11, 1)
                .WithTaggedFlag("GPI0SEL", 12)
                .WithReservedBits(13, 3)
                .WithFlag(16, name: "HIBIO0SEL")
                .WithFlag(17, name: "HIBIO1SEL")
                .WithReservedBits(18, 2)
                .WithTaggedFlag("HIB_IO_0_POL", 20)
                .WithTaggedFlag("HIB_IO_1_POL", 21)
                .WithReservedBits(22, 10)
            );

            // OSCSICTRL @ 0x314 — fOSI Control (mirror register)
            map.Add(0x314, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "OSCSICTRL",
                    writeCallback: (_, __) => SignalMirrorWriteComplete(SrrawOscsictrl))
            );

            // OSCULSTAT @ 0x318 — OSC_ULP Status
            map.Add(0x318, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => true, name: "X1D")
                .WithReservedBits(1, 31)
            );

            // OSCULCTRL @ 0x31C — OSC_ULP Control (mirror register)
            map.Add(0x31C, new DoubleWordRegister(this)
                .WithTag("X1DEN", 0, 1)
                .WithReservedBits(1, 3)
                .WithTag("MODE", 4, 2)
                .WithReservedBits(6, 26)
                .WithWriteCallback((_, __) => SignalMirrorWriteComplete(SrrawOsculctrl))
            );

            // 0x320 — reserved/unused in this model
        }

        // ────────────────────────────────────────────
        //  SCU_RESET  (base + 0x400)
        // ────────────────────────────────────────────
        private void DefineResetRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // RSTSTAT @ 0x400 — Reset Status
            map.Add(0x400, new DoubleWordRegister(this, 0x01u)  // PORST as default reset cause
                .WithValueField(0, 8, name: "RSTSTAT")
                .WithFlag(8, name: "HIBWK")
                .WithFlag(9, name: "HIBRS")
                .WithFlag(10, name: "LCKEN")
                .WithReservedBits(11, 21)
            );

            // RSTSET @ 0x404 — Reset Set
            map.Add(0x404, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "RSTSET")
            );

            // RSTCLR @ 0x408 — Reset Clear
            map.Add(0x408, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "RSTCLR")
            );

            // PRSTAT0 @ 0x40C — Peripheral 0 Reset Status (read-only)
            map.Add(0x40C, new DoubleWordRegister(this, 0x00010F9Fu)
                .WithValueField(0, 32, FieldMode.Read, name: "PRSTAT0",
                    valueProviderCallback: _ => prstat0)
            );

            // PRSET0 @ 0x410 — Peripheral 0 Reset Set (write-1-to-assert)
            map.Add(0x410, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRSET0",
                    writeCallback: (_, val) => prstat0 |= val)
            );

            // PRCLR0 @ 0x414 — Peripheral 0 Reset Clear (write-1-to-deassert)
            map.Add(0x414, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRCLR0",
                    writeCallback: (_, val) => prstat0 &= ~val)
            );

            // PRSTAT1 @ 0x418
            map.Add(0x418, new DoubleWordRegister(this, 0x000001F9u)
                .WithValueField(0, 32, FieldMode.Read, name: "PRSTAT1",
                    valueProviderCallback: _ => prstat1)
            );

            // PRSET1 @ 0x41C
            map.Add(0x41C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRSET1",
                    writeCallback: (_, val) => prstat1 |= val)
            );

            // PRCLR1 @ 0x420
            map.Add(0x420, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRCLR1",
                    writeCallback: (_, val) => prstat1 &= ~val)
            );

            // PRSTAT2 @ 0x424
            map.Add(0x424, new DoubleWordRegister(this, 0x000000F6u)
                .WithValueField(0, 32, FieldMode.Read, name: "PRSTAT2",
                    valueProviderCallback: _ => prstat2)
            );

            // PRSET2 @ 0x428
            map.Add(0x428, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRSET2",
                    writeCallback: (_, val) => prstat2 |= val)
            );

            // PRCLR2 @ 0x42C
            map.Add(0x42C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRCLR2",
                    writeCallback: (_, val) => prstat2 &= ~val)
            );

            // PRSTAT3 @ 0x430
            map.Add(0x430, new DoubleWordRegister(this, 0x00000004u)
                .WithValueField(0, 32, FieldMode.Read, name: "PRSTAT3",
                    valueProviderCallback: _ => prstat3)
            );

            // PRSET3 @ 0x434
            map.Add(0x434, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRSET3",
                    writeCallback: (_, val) => prstat3 |= val)
            );

            // PRCLR3 @ 0x438
            map.Add(0x438, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "PRCLR3",
                    writeCallback: (_, val) => prstat3 &= ~val)
            );
        }

        // ────────────────────────────────────────────
        //  SCU_CLK  (base + 0x600)
        // ────────────────────────────────────────────
        private void DefineClockRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // CLKSTAT @ 0x600 — Clock Status (read-only)
            map.Add(0x600, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "CLKSTAT",
                    valueProviderCallback: _ => clkstat)
            );

            // CLKSET @ 0x604 — Clock Enable (write-1-to-enable)
            map.Add(0x604, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CLKSET",
                    writeCallback: (_, val) => clkstat |= val)
            );

            // CLKCLR @ 0x608 — Clock Disable (write-1-to-disable)
            map.Add(0x608, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CLKCLR",
                    writeCallback: (_, val) => clkstat &= ~val)
            );

            // SYSCLKCR @ 0x60C — System Clock Control
            map.Add(0x60C, new DoubleWordRegister(this)
                .WithValueField(0, 8, out sysclkDiv, name: "SYSDIV")
                .WithReservedBits(8, 8)
                .WithFlag(16, out sysclkSelPll, name: "SYSSEL")
                .WithReservedBits(17, 15)
            );

            // CPUCLKCR @ 0x610
            map.Add(0x610, new DoubleWordRegister(this)
                .WithFlag(0, name: "CPUDIV")
                .WithReservedBits(1, 31)
            );

            // PBCLKCR @ 0x614
            map.Add(0x614, new DoubleWordRegister(this)
                .WithFlag(0, name: "PBDIV")
                .WithReservedBits(1, 31)
            );

            // USBCLKCR @ 0x618
            map.Add(0x618, new DoubleWordRegister(this)
                .WithValueField(0, 3, name: "USBDIV")
                .WithReservedBits(3, 13)
                .WithFlag(16, name: "USBSEL")
                .WithReservedBits(17, 15)
            );

            // EBUCLKCR @ 0x61C
            map.Add(0x61C, new DoubleWordRegister(this)
                .WithValueField(0, 6, name: "EBUDIV")
                .WithReservedBits(6, 26)
            );

            // CCUCLKCR @ 0x620
            map.Add(0x620, new DoubleWordRegister(this)
                .WithFlag(0, name: "CCUDIV")
                .WithReservedBits(1, 31)
            );

            // WDTCLKCR @ 0x624
            map.Add(0x624, new DoubleWordRegister(this)
                .WithValueField(0, 8, name: "WDTDIV")
                .WithReservedBits(8, 8)
                .WithValueField(16, 2, name: "WDTSEL")
                .WithReservedBits(18, 14)
            );

            // EXTCLKCR @ 0x628
            map.Add(0x628, new DoubleWordRegister(this)
                .WithValueField(0, 2, name: "ECKSEL")
                .WithReservedBits(2, 14)
                .WithValueField(16, 9, name: "ECKDIV")
                .WithReservedBits(25, 7)
            );

            // SLEEPCR @ 0x630
            map.Add(0x630, new DoubleWordRegister(this)
                .WithFlag(0, name: "SYSSEL")
                .WithReservedBits(1, 15)
                .WithTaggedFlag("USBCR", 16)
                .WithTaggedFlag("MMCCR", 17)
                .WithTaggedFlag("ETH0CR", 18)
                .WithTaggedFlag("EBUCR", 19)
                .WithTaggedFlag("CCUCR", 20)
                .WithTaggedFlag("WDTCR", 21)
                .WithReservedBits(22, 10)
            );

            // DSLEEPCR @ 0x634
            map.Add(0x634, new DoubleWordRegister(this)
                .WithValueField(0, 2, name: "SYSSEL")
                .WithReservedBits(2, 9)
                .WithTaggedFlag("FPDN", 11)
                .WithTaggedFlag("PLLPDN", 12)
                .WithTaggedFlag("VCOPDN", 13)
                .WithReservedBits(14, 2)
                .WithTaggedFlag("USBCR", 16)
                .WithTaggedFlag("MMCCR", 17)
                .WithTaggedFlag("ETH0CR", 18)
                .WithTaggedFlag("EBUCR", 19)
                .WithTaggedFlag("CCUCR", 20)
                .WithTaggedFlag("WDTCR", 21)
                .WithReservedBits(22, 10)
            );

            // CGATSTAT0 @ 0x640 — Clock Gating Status 0
            map.Add(0x640, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "CGATSTAT0",
                    valueProviderCallback: _ => cgatstat0)
            );

            // CGATSET0 @ 0x644 — Clock Gating Set 0 (write-1-to-gate)
            map.Add(0x644, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATSET0",
                    writeCallback: (_, val) => cgatstat0 |= val)
            );

            // CGATCLR0 @ 0x648 — Clock Gating Clear 0 (write-1-to-ungate)
            map.Add(0x648, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATCLR0",
                    writeCallback: (_, val) => cgatstat0 &= ~val)
            );

            // CGATSTAT1 @ 0x64C
            map.Add(0x64C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "CGATSTAT1",
                    valueProviderCallback: _ => cgatstat1)
            );

            // CGATSET1 @ 0x650
            map.Add(0x650, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATSET1",
                    writeCallback: (_, val) => cgatstat1 |= val)
            );

            // CGATCLR1 @ 0x654
            map.Add(0x654, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATCLR1",
                    writeCallback: (_, val) => cgatstat1 &= ~val)
            );

            // CGATSTAT2 @ 0x658
            map.Add(0x658, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "CGATSTAT2",
                    valueProviderCallback: _ => cgatstat2)
            );

            // CGATSET2 @ 0x65C
            map.Add(0x65C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATSET2",
                    writeCallback: (_, val) => cgatstat2 |= val)
            );

            // CGATCLR2 @ 0x660
            map.Add(0x660, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATCLR2",
                    writeCallback: (_, val) => cgatstat2 &= ~val)
            );

            // CGATSTAT3 @ 0x664
            map.Add(0x664, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "CGATSTAT3",
                    valueProviderCallback: _ => cgatstat3)
            );

            // CGATSET3 @ 0x668
            map.Add(0x668, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATSET3",
                    writeCallback: (_, val) => cgatstat3 |= val)
            );

            // CGATCLR3 @ 0x66C
            map.Add(0x66C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CGATCLR3",
                    writeCallback: (_, val) => cgatstat3 &= ~val)
            );
        }

        // ────────────────────────────────────────────
        //  SCU_OSC  (base + 0x700)
        // ────────────────────────────────────────────
        private void DefineOscRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // OSCHPSTAT @ 0x700
            map.Add(0x700, new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => oscHpEnabled, name: "X1D")
                .WithReservedBits(1, 31)
            );

            // OSCHPCTRL @ 0x704
            map.Add(0x704, new DoubleWordRegister(this, 0x00000033u)  // MODE=11 (disabled) after reset
                .WithFlag(0, name: "X1DEN")
                .WithFlag(1, name: "SHBY")
                .WithReservedBits(2, 2)
                .WithValueField(4, 2, name: "MODE", writeCallback: (_, val) =>
                {
                    // MODE=00 or MODE=10: oscillator enabled; MODE=01/11: disabled
                    oscHpEnabled = (val & 0x1) == 0;
                })
                .WithReservedBits(6, 10)
                .WithValueField(16, 4, name: "OSCVAL")
                .WithReservedBits(20, 12)
            );

            // CLKCALCONST @ 0x70C
            map.Add(0x70C, new DoubleWordRegister(this)
                .WithValueField(0, 4, name: "CALIBCONST")
                .WithReservedBits(4, 28)
            );
        }

        // ────────────────────────────────────────────
        //  SCU_PLL  (base + 0x710)
        // ────────────────────────────────────────────
        private void DefinePllRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // PLLSTAT @ 0x710 — PLL Status (read-only)
            map.Add(0x710, new DoubleWordRegister(this, 0x00000002u)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => systemPllVcoBypass, name: "VCOBYST")
                .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => pllPoweredDown, name: "PWDSTAT")
                // VCOLOCK: report locked when high-performance oscillator is enabled.
                // Real hardware allows lock even with VCOPWD set during certain startup sequences.
                .WithFlag(2, FieldMode.Read, valueProviderCallback: _ => oscHpEnabled, name: "VCOLOCK")
                .WithReservedBits(3, 1)
                .WithFlag(4, FieldMode.Read, valueProviderCallback: _ => true, name: "K1RDY")
                .WithFlag(5, FieldMode.Read, valueProviderCallback: _ => true, name: "K2RDY")
                .WithFlag(6, FieldMode.Read, valueProviderCallback: _ => systemPllVcoBypass, name: "BY")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => oscHpEnabled, name: "PLLLV")
                .WithFlag(8, FieldMode.Read, valueProviderCallback: _ => oscHpEnabled, name: "PLLHV")
                .WithFlag(9, FieldMode.Read, valueProviderCallback: _ => oscHpEnabled, name: "PLLSP")
                .WithReservedBits(10, 22)
            );

            // PLLCON0 @ 0x714 — PLL Configuration 0
            map.Add(0x714, new DoubleWordRegister(this, 0x00030003u)
                .WithFlag(0, name: "VCOBYP", writeCallback: (_, val) =>
                {
                    systemPllVcoBypass = val;
                })
                .WithFlag(1, name: "VCOPWD", writeCallback: (_, val) =>
                {
                    pllPoweredDown = val;
                })
                .WithFlag(2, name: "VCOTR")
                .WithReservedBits(3, 1)
                .WithFlag(4, name: "FINDIS")
                .WithReservedBits(5, 1)
                .WithFlag(6, name: "OSCDISCDIS")
                .WithReservedBits(7, 9)
                .WithFlag(16, name: "PLLPWD")
                .WithFlag(17, name: "OSCRES")
                .WithFlag(18, FieldMode.Write, name: "RESLD")  // write-only, restarts lock detection
                .WithFlag(19, name: "AOTREN")
                .WithFlag(20, name: "FOTR")
                .WithReservedBits(21, 11)
            );

            // PLLCON1 @ 0x718 — PLL Configuration 1 (divider values)
            map.Add(0x718, new DoubleWordRegister(this)
                .WithValueField(0, 7, name: "K1DIV")
                .WithReservedBits(7, 1)
                .WithValueField(8, 7, name: "NDIV")
                .WithReservedBits(15, 1)
                .WithValueField(16, 7, name: "K2DIV")
                .WithReservedBits(23, 1)
                .WithValueField(24, 4, name: "PDIV")
                .WithReservedBits(28, 4)
            );

            // PLLCON2 @ 0x71C — PLL Configuration 2
            map.Add(0x71C, new DoubleWordRegister(this, 0x00000001u)
                .WithFlag(0, name: "PINSEL")
                .WithReservedBits(1, 7)
                .WithFlag(8, name: "K1INSEL")
                .WithReservedBits(9, 23)
            );

            // USBPLLSTAT @ 0x720 — USB PLL Status (read-only)
            map.Add(0x720, new DoubleWordRegister(this, 0x00000002u)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => usbPllVcoBypass, name: "VCOBYST")
                .WithFlag(1, FieldMode.Read, valueProviderCallback: _ => usbPllPoweredDown, name: "PWDSTAT")
                .WithFlag(2, FieldMode.Read, valueProviderCallback: _ => !usbPllPoweredDown && !usbPllVcoBypass, name: "VCOLOCK")
                .WithReservedBits(3, 3)
                .WithFlag(6, FieldMode.Read, valueProviderCallback: _ => usbPllVcoBypass, name: "BY")
                .WithFlag(7, FieldMode.Read, valueProviderCallback: _ => !usbPllPoweredDown, name: "VCOLOCKED")
                .WithReservedBits(8, 24)
            );

            // USBPLLCON @ 0x724 — USB PLL Configuration
            map.Add(0x724, new DoubleWordRegister(this, 0x00010003u)
                .WithFlag(0, name: "VCOBYP", writeCallback: (_, val) =>
                {
                    usbPllVcoBypass = val;
                })
                .WithFlag(1, name: "VCOPWD", writeCallback: (_, val) =>
                {
                    usbPllPoweredDown = val;
                })
                .WithFlag(2, name: "VCOTR")
                .WithReservedBits(3, 1)
                .WithFlag(4, name: "FINDIS")
                .WithReservedBits(5, 1)
                .WithFlag(6, name: "OSCDISCDIS")
                .WithReservedBits(7, 1)
                .WithValueField(8, 7, name: "NDIV")
                .WithReservedBits(15, 1)
                .WithFlag(16, name: "PLLPWD")
                .WithReservedBits(17, 1)
                .WithFlag(18, FieldMode.Write, name: "RESLD")
                .WithReservedBits(19, 5)
                .WithValueField(24, 4, name: "PDIV")
                .WithReservedBits(28, 4)
            );

            // CLKMXSTAT @ 0x738 — Clock Multiplexing Status
            map.Add(0x738, new DoubleWordRegister(this, 0x00000001u)
                .WithValueField(0, 2, FieldMode.Read, name: "SYSCLKMUX",
                    valueProviderCallback: _ =>
                    {
                        // 01 = fOFI clock active, 10 = fPLL clock active
                        if(sysclkSelPll != null && sysclkSelPll.Value && !pllPoweredDown)
                        {
                            return 0x2u;  // PLL active
                        }
                        return 0x1u;  // fOFI active
                    })
                .WithReservedBits(2, 30)
            );
        }

        // ────────────────────────────────────────────
        //  SCU_TRAP  (base + 0x160)
        // ────────────────────────────────────────────
        private void DefineTrapRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // TRAPSTAT @ 0x160 — Trap Status (read-only, all traps cleared = 0)
            map.Add(0x160, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "TRAPSTAT")
            );

            // TRAPRAW @ 0x164 — Trap Raw Status (read-only)
            map.Add(0x164, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "TRAPRAW")
            );

            // TRAPDIS @ 0x168 — Trap Disable (r/w, all traps disabled by default)
            map.Add(0x168, new DoubleWordRegister(this, 0x000001FFu)
                .WithValueField(0, 32, name: "TRAPDIS")
            );

            // TRAPCLR @ 0x16C — Trap Clear (write-only)
            map.Add(0x16C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "TRAPCLR")
            );

            // TRAPSET @ 0x170 — Trap Set (write-only)
            map.Add(0x170, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "TRAPSET")
            );
        }

        // ────────────────────────────────────────────
        //  RTC  (base + 0xA00)
        //  The RTC lives within the SCU address space.
        // ────────────────────────────────────────────
        private void DefineRtcRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // ID @ 0xA00
            map.Add(0xA00, new DoubleWordRegister(this, 0x00A1C000u)
                .WithValueField(0, 32, FieldMode.Read, name: "ID")
            );

            // CTR @ 0xA04 — RTC Control (mirror register)
            map.Add(0xA04, new DoubleWordRegister(this)
                .WithFlag(0, name: "ENB",  // RTC enable
                    valueProviderCallback: _ => rtcEnabled,
                    writeCallback: (_, value) =>
                    {
                        if(value != rtcEnabled)
                        {
                            var now = RtcNow();
                            rtcEnabled = value;
                            RebaseRtc(now);
                        }
                    })
                .WithFlag(1, name: "TAE")  // Timer alarm enable
                .WithReservedBits(2, 14)
                .WithValueField(16, 16, name: "DIV",
                    valueProviderCallback: _ => rtcDivider,
                    writeCallback: (_, value) =>
                    {
                        var now = RtcNow();
                        rtcDivider = (uint)value;
                        RebaseRtc(now);
                    })
                .WithWriteCallback((_, __) => SignalMirrorWriteComplete(SrrawRtcCtr))
            );

            // RAWSTAT @ 0xA08 — RTC Raw Service Request Status
            map.Add(0xA08, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "RAWSTAT", valueProviderCallback: _ => rtcRawStatus)
            );

            // STSSR @ 0xA0C — RTC Service Request Status
            map.Add(0xA0C, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, name: "STSSR", valueProviderCallback: _ => rtcRawStatus & rtcMask)
            );

            // MSKSR @ 0xA10 — RTC Service Request Mask (mirror register)
            map.Add(0xA10, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "MSKSR",
                    valueProviderCallback: _ => rtcMask,
                    writeCallback: (_, val) => { rtcMask = (uint)val; SignalMirrorWriteComplete(SrrawRtcMsksr); })
            );

            // CLRSR @ 0xA14 — RTC Clear Service Request (mirror register)
            map.Add(0xA14, new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "CLRSR",
                    writeCallback: (_, val) => { rtcRawStatus &= ~(uint)val; SignalMirrorWriteComplete(SrrawRtcClrsr); })
            );

            // ATIM0 @ 0xA18 — RTC Alarm Time Register 0 (mirror register)
            map.Add(0xA18, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "ATIM0", valueProviderCallback: _ => rtcAlarm0, changeCallback: (_, val) => rtcAlarm0 = (uint)val,
                    writeCallback: (_, __) => SignalMirrorWriteComplete(SrrawRtcAtim0))
            );

            // ATIM1 @ 0xA1C — RTC Alarm Time Register 1 (mirror register)
            map.Add(0xA1C, new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "ATIM1", valueProviderCallback: _ => rtcAlarm1, changeCallback: (_, val) => rtcAlarm1 = (uint)val,
                    writeCallback: (_, __) => SignalMirrorWriteComplete(SrrawRtcAtim1))
            );

            // TIM0 @ 0xA20 — RTC Time Register 0: seconds, minutes, hours and
            // days (day of month - 1). The RTC counts one second per (DIV + 1)
            // periods of its 32.768 kHz clock while CTR.ENB is set; a write sets
            // the fields it holds.
            map.Add(0xA20, new DoubleWordRegister(this)
                .WithValueField(0, 6, name: "SE", valueProviderCallback: _ => (ulong)RtcNow().Second)
                .WithReservedBits(6, 2)
                .WithValueField(8, 6, name: "MI", valueProviderCallback: _ => (ulong)RtcNow().Minute)
                .WithReservedBits(14, 2)
                .WithValueField(16, 5, name: "HO", valueProviderCallback: _ => (ulong)RtcNow().Hour)
                .WithReservedBits(21, 3)
                .WithValueField(24, 5, name: "DA", valueProviderCallback: _ => (ulong)RtcNow().Day - 1)
                .WithReservedBits(29, 3)
                .WithWriteCallback((_, value) =>
                {
                    var now = RtcNow();
                    SetRtcFields(now.Year, now.Month, (int)((value >> 24) & 0x1F) + 1,
                        (int)((value >> 16) & 0x1F), (int)((value >> 8) & 0x3F), (int)(value & 0x3F));
                    SignalMirrorWriteComplete(SrrawRtcTim0);
                })
            );

            // TIM1 @ 0xA24 — RTC Time Register 1: day of week, month - 1, year
            map.Add(0xA24, new DoubleWordRegister(this)
                .WithValueField(0, 3, name: "DAWE", valueProviderCallback: _ => (ulong)(int)RtcNow().DayOfWeek)
                .WithReservedBits(3, 5)
                .WithValueField(8, 4, name: "MO", valueProviderCallback: _ => (ulong)RtcNow().Month - 1)
                .WithReservedBits(12, 4)
                .WithValueField(16, 16, name: "YE", valueProviderCallback: _ => (ulong)(RtcNow().Year - RtcYearOffset))
                .WithWriteCallback((_, value) =>
                {
                    var now = RtcNow();
                    SetRtcFields((int)(value >> 16) + RtcYearOffset, (int)((value >> 8) & 0xF) + 1, now.Day,
                        now.Hour, now.Minute, now.Second);
                    SignalMirrorWriteComplete(SrrawRtcTim1);
                })
            );
        }

        // The calendar is kept as a DateTime whose year is the RTC's YE plus
        // RtcYearOffset, so the reset value (year 0) has a DateTime too.
        private DateTime RtcNow()
        {
            if(!rtcEnabled)
            {
                return rtcBase;
            }
            var elapsed = machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds - rtcBaseVirtualSeconds;
            var secondsPerTick = (rtcDivider + 1) / 32768.0;
            return rtcBase.AddSeconds(Math.Floor(elapsed / secondsPerTick));
        }

        private void RebaseRtc(DateTime calendar)
        {
            rtcBase = calendar;
            rtcBaseVirtualSeconds = machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds;
            ScheduleRtcTick();
        }

        // Each RTC second raises the periodic events (seconds, and minutes to
        // years when they roll over) and the alarm when the time matches
        // ATIM0/ATIM1 with CTR.TAE set. Their STSSR bits (RAWSTAT & MSKSR) set
        // SRRAW.PI and SRRAW.AI.
        private void ScheduleRtcTick()
        {
            var generation = ++rtcGeneration;
            if(!rtcEnabled)
            {
                return;
            }
            var secondsPerTick = (rtcDivider + 1) / 32768.0;
            var elapsed = machine.ElapsedVirtualTime.TimeElapsed.TotalSeconds - rtcBaseVirtualSeconds;
            var next = (Math.Floor(elapsed / secondsPerTick) + 1) * secondsPerTick;
            var delay = TimeInterval.FromMicroseconds((ulong)Math.Max(1, Math.Ceiling((next - elapsed) * 1e6)));
            machine.ScheduleAction(delay, _ =>
            {
                if(generation != rtcGeneration)
                {
                    return;
                }
                RtcTick();
                ScheduleRtcTick();
            });
        }

        private void RtcTick()
        {
            var now = RtcNow();
            var events = RtcPeriodicSeconds;
            if(now.Second == 0)
            {
                events |= RtcPeriodicMinutes;
                if(now.Minute == 0)
                {
                    events |= RtcPeriodicHours;
                    if(now.Hour == 0)
                    {
                        events |= RtcPeriodicDays;
                        if(now.Day == 1)
                        {
                            events |= RtcPeriodicMonths;
                            if(now.Month == 1)
                            {
                                events |= RtcPeriodicYears;
                            }
                        }
                    }
                }
            }
            var ctr = RegistersCollection.Read(0xA04);
            if((ctr & 2) != 0
                && (rtcAlarm0 & 0x1F1F3F3F) == (((uint)(now.Day - 1) << 24) | ((uint)now.Hour << 16) | ((uint)now.Minute << 8) | (uint)now.Second)
                && (rtcAlarm1 & 0xFFFF0F00) == (((uint)(now.Year - RtcYearOffset) << 16) | ((uint)(now.Month - 1) << 8)))
            {
                events |= RtcAlarm;
            }
            rtcRawStatus |= events;
            var pending = rtcRawStatus & rtcMask;
            if((pending & RtcPeriodicMask) != 0)
            {
                srraw |= SrrawPeriodic;
            }
            if((pending & RtcAlarm) != 0)
            {
                srraw |= SrrawAlarm;
            }
            UpdateInterrupt();
        }

        private void SetRtcFields(int year, int month, int day, int hour, int minute, int second)
        {
            // Out-of-range fields are clamped, so a partly written calendar stays valid
            year = Math.Min(year, DateTime.MaxValue.Year);
            month = Math.Max(1, Math.Min(12, month));
            day = Math.Max(1, Math.Min(DateTime.DaysInMonth(year, month), day));
            RebaseRtc(new DateTime(year, month, day, Math.Min(hour, 23), Math.Min(minute, 59), Math.Min(second, 59)));
        }

        private readonly IMachine machine;

        // PLL state
        private bool pllPoweredDown;
        private bool usbPllPoweredDown;
        private bool systemPllVcoBypass;
        private bool usbPllVcoBypass;

        // Oscillator state
        private bool oscHpEnabled;

        // Power / Hibernate domain
        private bool hibDomainEnabled;
        private bool usbPhyActive;

        // SCU_INTERRUPT raw status (managed by set/clear + mirror write events)
        private ulong srraw;

        // Peripheral reset status (managed by set/clear registers)
        private ulong prstat0;
        private ulong prstat1;
        private ulong prstat2;
        private ulong prstat3;

        // Boot configuration (SWCON field of STCON register)
        private uint swconValue;

        /// <summary>
        /// Current software boot configuration (SWCON field).
        /// Non-zero values indicate bootloader or alternate boot mode requested.
        /// Readable from Python scripts for bootloader simulation.
        /// </summary>
        public uint SoftwareBootMode => swconValue;

        // Clock status (managed by set/clear registers)
        private ulong clkstat;

        // Clock gating status (managed by set/clear registers)
        private ulong cgatstat0;
        private ulong cgatstat1;
        private ulong cgatstat2;
        private ulong cgatstat3;

        // RTC read counter — ensures timeout loops see time advancing
        // RTC (hibernate domain): not reset by Reset()
        private bool rtcEnabled;
        private uint rtcDivider = 0x7FFF;
        private DateTime rtcBase = new DateTime(RtcYearOffset, 1, 1);
        private double rtcBaseVirtualSeconds;
        private uint rtcRawStatus;
        private uint rtcMask;
        private uint rtcAlarm0;
        private uint rtcAlarm1;
        private ulong rtcGeneration;
        private ulong srmsk;

        // RTC RAWSTAT/STSSR/MSKSR bits: periodic seconds, minutes, hours,
        // days, months, years, and the alarm
        private const uint RtcPeriodicSeconds = 1u << 0;
        private const uint RtcPeriodicMinutes = 1u << 1;
        private const uint RtcPeriodicHours = 1u << 2;
        private const uint RtcPeriodicDays = 1u << 3;
        private const uint RtcPeriodicMonths = 1u << 5;
        private const uint RtcPeriodicYears = 1u << 6;
        private const uint RtcPeriodicMask = 0x6F;
        private const uint RtcAlarm = 1u << 8;
        // SRRAW.PI and SRRAW.AI (bit 0 is PRWARN, the WDT pre-warning)
        private const ulong SrrawPeriodic = 1uL << 1;
        private const ulong SrrawAlarm = 1uL << 2;

        private const int RtcYearOffset = 2000;

        // System clock control fields
        private IValueRegisterField sysclkDiv;
        private IFlagRegisterField sysclkSelPll;

        // SRRAW event bit positions for hibernate mirror write completion
        private const ulong SrrawHdclr     = 1uL << 17;
        private const ulong SrrawHdset     = 1uL << 18;
        private const ulong SrrawHdcr      = 1uL << 19;
        private const ulong SrrawOscsictrl = 1uL << 21;
        private const ulong SrrawOsculctrl = 1uL << 23;
        private const ulong SrrawOsculstat = 1uL << 24;
        private const ulong SrrawRtcCtr    = 1uL << 25;
        private const ulong SrrawRtcAtim0  = 1uL << 26;
        private const ulong SrrawRtcAtim1  = 1uL << 27;
        private const ulong SrrawRtcTim0   = 1uL << 28;
        private const ulong SrrawRtcTim1   = 1uL << 29;
        private const ulong SrrawRtcMsksr  = 1uL << 30;
        private const ulong SrrawRtcClrsr  = 1uL << 31;
    }
}
