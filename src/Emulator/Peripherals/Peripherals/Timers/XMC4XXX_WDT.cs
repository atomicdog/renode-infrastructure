//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    // Infineon XMC4xxx Watchdog Timer (WDT).
    //
    // The WDT is a 32-bit up-counter with a windowed service mechanism.
    // The timer counts from 0 toward the upper bound (WUB). A service
    // write (kick) to the SRV register resets TIM to 0, but only if
    // TIM >= WLB (lower bound). If TIM reaches WUB, the WDT triggers
    // a pre-warning alarm and/or system reset.
    //
    // This model implements all documented registers from the XMC4500
    // SVD. The timer uses a LimitTimer counting up from 0 to WUB.
    public class XMC4XXX_WDT : IDoubleWordPeripheral, IKnownSize, IProvidesRegisterCollection<DoubleWordRegisterCollection>
    {
        public XMC4XXX_WDT(IMachine machine, ulong frequency = 32768)
        {
            this.machine = machine;

            timer = new LimitTimer(machine.ClockSource, frequency, this, "wdt",
                limit: uint.MaxValue,
                direction: Direction.Ascending,
                enabled: false,
                eventEnabled: true,
                autoUpdate: true,
                workMode: WorkMode.OneShot);

            timer.LimitReached += OnLimitReached;

            var registersMap = new Dictionary<long, DoubleWordRegister>();
            DefineRegisters(registersMap);
            RegistersCollection = new DoubleWordRegisterCollection(this, registersMap);
        }

        public void Reset()
        {
            RegistersCollection.Reset();
            timer.Reset();
            alarmStatus = false;
        }

        public uint ReadDoubleWord(long offset)
        {
            return RegistersCollection.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            RegistersCollection.Write(offset, value);
        }

        public long Size => 0x4000;

        public DoubleWordRegisterCollection RegistersCollection { get; }

        private void DefineRegisters(Dictionary<long, DoubleWordRegister> registersMap)
        {
            // 0x00 — ID (read-only, reset 0x00ADC000)
            registersMap[(long)Register.ID] = new DoubleWordRegister(this, resetValue: 0x00ADC000u)
                .WithValueField(0, 8, FieldMode.Read, name: "MOD_REV")
                .WithValueField(8, 8, FieldMode.Read, name: "MOD_TYPE")
                .WithValueField(16, 16, FieldMode.Read, name: "MOD_NUMBER");

            // 0x04 — CTR (control)
            registersMap[(long)Register.CTR] = new DoubleWordRegister(this)
                .WithFlag(0, out enableFlag, writeCallback: (_, val) =>
                {
                    timer.Enabled = val;
                    if(val)
                    {
                        this.Log(LogLevel.Debug, "WDT enabled");
                    }
                }, name: "ENB")
                .WithFlag(1, out preWarningEnable, name: "PRE")
                .WithReservedBits(2, 2)
                .WithFlag(4, name: "DSP")
                .WithReservedBits(5, 3)
                .WithValueField(8, 8, name: "SPW")
                .WithReservedBits(16, 16);

            // 0x08 — SRV (write-only service/kick register)
            registersMap[(long)Register.SRV] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, writeCallback: (_, __) =>
                {
                    if(!enableFlag.Value)
                    {
                        return;
                    }
                    var currentValue = (uint)timer.Value;
                    if(currentValue >= windowLowerBound.Value)
                    {
                        timer.Value = 0;
                        this.Log(LogLevel.Debug, "WDT serviced (kicked) at TIM={0}", currentValue);
                    }
                    else
                    {
                        this.Log(LogLevel.Warning, "WDT service rejected: TIM={0} < WLB={1}", currentValue, windowLowerBound.Value);
                    }
                }, name: "SRV");

            // 0x0C — TIM (read-only current timer value)
            registersMap[(long)Register.TIM] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Read, valueProviderCallback: _ =>
                {
                    return (uint)timer.Value;
                }, name: "TIM");

            // 0x10 — WLB (window lower bound)
            registersMap[(long)Register.WLB] = new DoubleWordRegister(this)
                .WithValueField(0, 32, out windowLowerBound, name: "WLB");

            // 0x14 — WUB (window upper bound, reset 0xFFFFFFFF)
            registersMap[(long)Register.WUB] = new DoubleWordRegister(this, resetValue: 0xFFFFFFFFu)
                .WithValueField(0, 32, out windowUpperBound, writeCallback: (_, val) =>
                {
                    timer.Limit = val == 0 ? 1 : val;
                }, name: "WUB");

            // 0x18 — WDTSTS (status, read-only)
            registersMap[(long)Register.WDTSTS] = new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Read, valueProviderCallback: _ => alarmStatus, name: "ALMS")
                .WithReservedBits(1, 31);

            // 0x1C — WDTCLR (clear, write-only)
            registersMap[(long)Register.WDTCLR] = new DoubleWordRegister(this)
                .WithFlag(0, FieldMode.Write, writeCallback: (_, val) =>
                {
                    if(val)
                    {
                        alarmStatus = false;
                        this.Log(LogLevel.Debug, "WDT alarm cleared");
                    }
                }, name: "ALMC")
                .WithReservedBits(1, 31);
        }

        private void OnLimitReached()
        {
            this.Log(LogLevel.Warning, "WDT timeout reached (WUB={0})", windowUpperBound.Value);
            if(preWarningEnable.Value)
            {
                alarmStatus = true;
                this.Log(LogLevel.Info, "WDT pre-warning alarm set");
            }
            else
            {
                this.Log(LogLevel.Warning, "WDT requesting system reset");
                machine.RequestReset();
            }
        }

        private IFlagRegisterField enableFlag;
        private IFlagRegisterField preWarningEnable;
        private IValueRegisterField windowLowerBound;
        private IValueRegisterField windowUpperBound;
        private bool alarmStatus;

        private readonly LimitTimer timer;
        private readonly IMachine machine;

        private enum Register : long
        {
            ID     = 0x00,
            CTR    = 0x04,
            SRV    = 0x08,
            TIM    = 0x0C,
            WLB    = 0x10,
            WUB    = 0x14,
            WDTSTS = 0x18,
            WDTCLR = 0x1C,
        }
    }
}
