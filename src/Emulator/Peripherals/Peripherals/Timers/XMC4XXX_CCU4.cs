//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;

using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.Timers
{
    // XMC4000 CCU4 capture/compare unit: four 16-bit timer slices.
    //
    // The timers do not count.  Registers keep what is written, like a memory
    // stub, so software that reads back its own configuration or a counter value
    // sees no change, with these exceptions:
    //   - GCSS requests a shadow transfer for a slice (S0SE, bit 4n), which is
    //     done at once: PRS -> PR, CRS -> CR.  GCSS, GCSC and GCST read 0.
    //   - TCSET/TCCLR set and clear the run bit in TCST; TCCLR.TCC clears TIMER.
    //     TCSET and TCCLR read 0.
    //   - GIDLS/GIDLC set and clear the idle bits in GSTAT (reset: all idle).
    // GetOutputDuty reports the fraction of time a slice's output is high in
    // edge-aligned mode (active from compare match to period match, inverted by
    // PSL.PSL), or its passive level while the timer is stopped.
    public class XMC4XXX_CCU4 : IDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_CCU4()
        {
            registers = new uint[Size / 4];
            Reset();
        }

        public void Reset()
        {
            Array.Clear(registers, 0, registers.Length);
            registers[GlobalStatus / 4] = AllSlicesIdle;
        }

        public uint ReadDoubleWord(long offset)
        {
            var index = offset / 4;
            if(offset < 0 || index >= registers.Length)
            {
                return 0;
            }
            switch(offset)
            {
                case GlobalShadowSet:
                case GlobalShadowClear:
                case GlobalShadowStatus:
                    return 0;
            }
            if(TryGetSliceRegister(offset, out var _, out var register)
                && (register == SliceTimerRunSet || register == SliceTimerRunClear))
            {
                return 0;
            }
            return registers[index];
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            var index = offset / 4;
            if(offset < 0 || index >= registers.Length)
            {
                this.Log(LogLevel.Warning, "Write outside the unit at 0x{0:X}", offset);
                return;
            }
            switch(offset)
            {
                case GlobalIdleSet:
                    registers[GlobalStatus / 4] |= value & AllSlicesIdle;
                    return;
                case GlobalIdleClear:
                    registers[GlobalStatus / 4] &= ~(value & AllSlicesIdle);
                    return;
                case GlobalShadowSet:
                    for(var slice = 0; slice < NumberOfSlices; slice++)
                    {
                        if((value & (1u << (4 * slice))) != 0)
                        {
                            TransferShadow(slice);
                        }
                    }
                    return;
                case GlobalShadowClear:
                case GlobalShadowStatus:
                    return;
            }
            if(TryGetSliceRegister(offset, out var sliceIndex, out var register))
            {
                switch(register)
                {
                    case SliceTimerRunSet:
                        if((value & 1) != 0)
                        {
                            SetSlice(sliceIndex, SliceTimerStatus, GetSlice(sliceIndex, SliceTimerStatus) | 1);
                        }
                        return;
                    case SliceTimerRunClear:
                        if((value & 1) != 0)
                        {
                            SetSlice(sliceIndex, SliceTimerStatus, GetSlice(sliceIndex, SliceTimerStatus) & ~1u);
                        }
                        if((value & 2) != 0)
                        {
                            SetSlice(sliceIndex, SliceTimer, 0);
                        }
                        return;
                }
            }
            registers[index] = value;
        }

        public double GetOutputDuty(int slice)
        {
            if(slice < 0 || slice >= NumberOfSlices)
            {
                this.Log(LogLevel.Warning, "Invalid slice {0}, must be 0-{1}", slice, NumberOfSlices - 1);
                return 0;
            }
            var passiveHigh = (GetSlice(slice, SlicePassiveLevel) & 1) != 0;
            if((GetSlice(slice, SliceTimerStatus) & 1) == 0)
            {
                return passiveHigh ? 1 : 0;
            }
            var periodPlusOne = (double)(GetSlice(slice, SlicePeriod) & 0xFFFF) + 1;
            var compare = GetSlice(slice, SliceCompare) & 0xFFFF;
            var active = Math.Max(0, Math.Min(1, (periodPlusOne - compare) / periodPlusOne));
            return passiveHigh ? 1 - active : active;
        }

        public long Size => 0x4000;

        private void TransferShadow(int slice)
        {
            SetSlice(slice, SlicePeriod, GetSlice(slice, SlicePeriodShadow));
            SetSlice(slice, SliceCompare, GetSlice(slice, SliceCompareShadow));
            this.Log(LogLevel.Debug, "Slice {0}: shadow transfer, PR=0x{1:X} CR=0x{2:X}", slice, GetSlice(slice, SlicePeriod), GetSlice(slice, SliceCompare));
        }

        private bool TryGetSliceRegister(long offset, out int slice, out long register)
        {
            slice = (int)((offset - FirstSlice) / SliceSize);
            register = (offset - FirstSlice) % SliceSize;
            return offset >= FirstSlice && slice < NumberOfSlices;
        }

        private uint GetSlice(int slice, long register)
        {
            return registers[(FirstSlice + slice * SliceSize + register) / 4];
        }

        private void SetSlice(int slice, long register, uint value)
        {
            registers[(FirstSlice + slice * SliceSize + register) / 4] = value;
        }

        private readonly uint[] registers;

        private const int NumberOfSlices = 4;
        private const uint AllSlicesIdle = 0xF;

        private const long GlobalStatus = 0x04;
        private const long GlobalIdleSet = 0x08;
        private const long GlobalIdleClear = 0x0C;
        private const long GlobalShadowSet = 0x10;
        private const long GlobalShadowClear = 0x14;
        private const long GlobalShadowStatus = 0x18;

        private const long FirstSlice = 0x100;
        private const long SliceSize = 0x100;
        private const long SliceTimerStatus = 0x08;
        private const long SliceTimerRunSet = 0x0C;
        private const long SliceTimerRunClear = 0x10;
        private const long SlicePassiveLevel = 0x18;
        private const long SlicePeriod = 0x30;
        private const long SlicePeriodShadow = 0x34;
        private const long SliceCompare = 0x38;
        private const long SliceCompareShadow = 0x3C;
        private const long SliceTimer = 0x70;
    }
}
