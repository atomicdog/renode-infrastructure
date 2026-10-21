//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord)]
    public class XMC4XXX_VADC : IDoubleWordPeripheral, IKnownSize, INumberedGPIOOutput
    {
        // peripheralClockFrequency is fADC (= fPERIPH). When set, a background
        // scan takes the conversion time the programmed input classes give
        // its channels; when 0 it takes a fixed ScanDelayMicroseconds.
        public XMC4XXX_VADC(IMachine machine, long peripheralClockFrequency = 0)
        {
            this.machine = machine;
            this.peripheralClockFrequency = peripheralClockFrequency;

            var innerConnections = new Dictionary<int, IGPIO>();
            for(int i = 0; i < NumberOfServiceRequestNodes; i++)
            {
                innerConnections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(innerConnections);

            groups = new VadcGroup[NumberOfGroups];
            for(int g = 0; g < NumberOfGroups; g++)
            {
                groups[g] = new VadcGroup();
            }

            registers = BuildRegisters();
            Reset();
        }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public uint ReadDoubleWord(long offset)
        {
            // Group result registers need special handling (VF flag)
            for(int g = 0; g < NumberOfGroups; g++)
            {
                long groupBase = GlobalRegionSize + g * GroupRegionSize;
                if(offset >= groupBase + ResultRegOffset
                    && offset < groupBase + ResultRegOffset + ResultRegCount * 4)
                {
                    int resIdx = (int)(offset - groupBase - ResultRegOffset) / 4;
                    return ReadResultRegister(g, resIdx);
                }
                if(offset >= groupBase + ResultDebugRegOffset
                    && offset < groupBase + ResultDebugRegOffset + ResultRegCount * 4)
                {
                    int resIdx = (int)(offset - groupBase - ResultDebugRegOffset) / 4;
                    return ReadResultRegister(g, resIdx);
                }
            }

            // Global result register
            if(offset == GlobalResultOffset)
            {
                return globalResultValue | (1u << 31);
            }
            if(offset == GlobalResultDebugOffset)
            {
                return globalResultValue | (1u << 31);
            }

            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            // Group result registers: firmware shouldn't write these, but allow it
            for(int g = 0; g < NumberOfGroups; g++)
            {
                long groupBase = GlobalRegionSize + g * GroupRegionSize;
                if(offset >= groupBase + ResultRegOffset
                    && offset < groupBase + ResultRegOffset + ResultRegCount * 4)
                {
                    int resIdx = (int)(offset - groupBase - ResultRegOffset) / 4;
                    groups[g].ResultValues[resIdx] = (ushort)(value & 0xFFFF);
                    return;
                }
            }

            if(offset == GlobalResultOffset)
            {
                globalResultValue = value & 0xFFFF;
                return;
            }

            registers.Write(offset, value);
        }

        public void Reset()
        {
            registers.Reset();

            clockControlRegister = 0x3;
            globalResultValue = 0;
            globalEventNodePointer = 0;
            backgroundScanMode = 0;
            scanActive = false;

            for(int g = 0; g < NumberOfGroups; g++)
            {
                groups[g].Reset();
            }

            foreach(var connection in Connections.Values)
            {
                connection.Unset();
            }
        }

        /// <summary>
        /// Set the raw 12-bit ADC result value for a specific group result register.
        /// Callable from the monitor: vadc SetResultValue 3 0 2048
        /// </summary>
        public void SetResultValue(int group, int resultIndex, ushort value)
        {
            if(group < 0 || group >= NumberOfGroups)
            {
                throw new Antmicro.Renode.Exceptions.RecoverableException(
                    $"Invalid group {group}, must be 0-{NumberOfGroups - 1}");
            }
            if(resultIndex < 0 || resultIndex >= ResultRegCount)
            {
                throw new Antmicro.Renode.Exceptions.RecoverableException(
                    $"Invalid result index {resultIndex}, must be 0-{ResultRegCount - 1}");
            }
            groups[group].ResultValues[resultIndex] = value;
            this.Log(LogLevel.Debug, "G{0}.RES[{1}] = {2} (0x{2:X3})", group, resultIndex, value);
        }

        /// <summary>
        /// Set a default raw value for ALL result registers across all groups.
        /// Useful for initializing a baseline voltage on all channels.
        /// </summary>
        public void SetDefaultValue(ushort value)
        {
            for(int g = 0; g < NumberOfGroups; g++)
            {
                for(int r = 0; r < ResultRegCount; r++)
                {
                    groups[g].ResultValues[r] = value;
                }
            }
            this.Log(LogLevel.Info, "All result registers set to {0} (0x{0:X3})", value);
        }

        public long Size => 0x4000;

        private uint ReadResultRegister(int group, int resIdx)
        {
            if(resIdx < 0 || resIdx >= ResultRegCount)
            {
                return 0;
            }

            uint result = groups[group].ResultValues[resIdx];
            // Bits [15:0] = RESULT, bit [31] = VF (always valid)
            // Bits [24:20] = CHNR: we could track which channel wrote this,
            // but the firmware only uses bits[15:0] so we leave CHNR=0.
            return (result & 0xFFFFu) | (1u << 31);
        }

        private void StartBackgroundScan()
        {
            if(scanActive)
            {
                return;
            }
            scanActive = true;
            var scanTime = GetBackgroundScanTime();
            if(scanTime != lastScanTime)
            {
                this.Log(LogLevel.Debug, "Background scan time {0:F2} us", scanTime.TotalMicroseconds);
                lastScanTime = scanTime;
            }
            machine.ScheduleAction(scanTime, _ =>
            {
                scanActive = false;

                // Clear LDEV (bit 9) — scan complete
                backgroundScanMode &= ~(1u << 9);

                // The last conversion of the sequence is the request source
                // event: interrupt on it with BRSMR.ENSI (bit 3).
                // GLOBEVNP bits [3:0] select the service request node (0-3).
                bool ensi = (backgroundScanMode & BrsmrEnsi) != 0;
                uint srNode = globalEventNodePointer & 0xFu;
                if(ensi && srNode < NumberOfServiceRequestNodes)
                {
                    this.NoisyLog("Background scan complete, firing SRN{0}", srNode);
                    Connections[(int)srNode].Blink();
                }

                // Autoscan (BRSMR.SCAN, bit 4): the source event is also a
                // load event, so the sequence starts again while conversion
                // requests are enabled (ENGT != 0).
                if((backgroundScanMode & BrsmrScan) != 0 && (backgroundScanMode & BrsmrEngt) != 0)
                {
                    StartBackgroundScan();
                }
            });
        }

        // Duration of one background scan, from the conversion timing in the
        // reference manual: tCN = (2 + STC + N + PC) * tADCI + 2 * tADC per
        // channel (fast compare: (2 + STC + 2) * tADCI + 2 * tADC), plus one
        // arbitration round, ARBRND slots of (DIVD + 1) / fADC, to select it.
        // Each group converts its selected channels (BRSSELx) on its own
        // converter, so the slowest group sets the scan time.
        private TimeInterval GetBackgroundScanTime()
        {
            if(peripheralClockFrequency <= 0)
            {
                return TimeInterval.FromMicroseconds(ScanDelayMicroseconds);
            }
            double fadc = peripheralClockFrequency;
            var globcfg = registers.Read(GlobalConfigOffset);
            var fadci = fadc / Math.Max(2u, (globcfg & 0x1Fu) + 1);
            var slot = (((globcfg >> 20) & 0x3u) + 1) / fadc;
            double longest = 0;
            for(int g = 0; g < NumberOfGroups; g++)
            {
                var select = registers.Read(BackgroundSelectOffset + g * 4);
                var slots = ArbitrationRoundSlots[(groups[g].ArbCfg >> 4) & 0x3u];
                // GLOBCFG.DPCALx turns post-calibration off for group x.
                var postCalibration = (globcfg & (1u << (16 + g))) != 0 ? 0 : 2;
                double groupTime = 0;
                for(int ch = 0; ch < ChannelsPerGroup; ch++)
                {
                    if((select & (1u << ch)) == 0)
                    {
                        continue;
                    }
                    // CHCTR.ICLSEL: 0-1 group input class, 2-3 global input class.
                    var iclsel = groups[g].ChannelControl[ch] & 0x3u;
                    var iclass = iclsel < 2 ? groups[g].Registers[2 + iclsel] : registers.Read(GlobalInputClassOffset + (iclsel - 2) * 4);
                    var stcs = iclass & 0x1Fu;
                    var stc = stcs <= 16 ? stcs : (stcs - 15) * 16;
                    double cycles;
                    switch((iclass >> 8) & 0x7u)
                    {
                        case 1:
                            cycles = 2 + stc + 10 + postCalibration;
                            break;
                        case 2:
                            cycles = 2 + stc + 8 + postCalibration;
                            break;
                        case 5:
                            cycles = 2 + stc + 2;
                            break;
                        default:
                            cycles = 2 + stc + 12 + postCalibration;
                            break;
                    }
                    groupTime += cycles / fadci + 2 / fadc + slots * slot;
                }
                longest = Math.Max(longest, groupTime);
            }
            return longest > 0 ? TimeInterval.FromSeconds(longest) : TimeInterval.FromMicroseconds(ScanDelayMicroseconds);
        }

        private DoubleWordRegisterCollection BuildRegisters()
        {
            var registersMap = new Dictionary<long, DoubleWordRegister>();

            BuildGlobalRegisters(registersMap);

            for(int g = 0; g < NumberOfGroups; g++)
            {
                BuildGroupRegisters(registersMap, g);
            }

            return new DoubleWordRegisterCollection(this, registersMap);
        }

        private void BuildGlobalRegisters(Dictionary<long, DoubleWordRegister> map)
        {
            // CLC @ 0x000: Clock Control Register (same pattern as MultiCAN)
            map[0x000] = new DoubleWordRegister(this, 0x3)
                .WithValueField(0, 32, name: "CLC",
                    valueProviderCallback: _ => clockControlRegister,
                    writeCallback: (_, value) =>
                    {
                        clockControlRegister = (uint)value;
                        if((value & 0x1) == 0)
                        {
                            clockControlRegister &= ~0x2u;
                        }
                        else
                        {
                            clockControlRegister |= 0x2u;
                        }
                    });

            // ID @ 0x008: Module ID
            map[0x008] = new DoubleWordRegister(this, ModuleId)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            // OCS @ 0x028
            AddStoreRegister(map, 0x028, "OCS");

            // GLOBCFG @ 0x080
            AddStoreRegister(map, 0x080, "GLOBCFG");

            // GLOBICLASS[0-1] @ 0x0A0, 0x0A4
            AddStoreRegister(map, 0x0A0, "GLOBICLASS0");
            AddStoreRegister(map, 0x0A4, "GLOBICLASS1");

            // GLOBBOUND @ 0x0B8
            AddStoreRegister(map, 0x0B8, "GLOBBOUND");

            // GLOBEFLAG @ 0x0E0
            AddStoreRegister(map, 0x0E0, "GLOBEFLAG");

            // GLOBEVNP @ 0x140: maps background scan event to a service request node
            map[0x140] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "GLOBEVNP",
                    valueProviderCallback: _ => globalEventNodePointer,
                    writeCallback: (_, value) => globalEventNodePointer = (uint)value);

            // GLOBTF @ 0x160
            AddStoreRegister(map, 0x160, "GLOBTF");

            // BRSSEL[0-3] @ 0x180-0x18C
            for(int i = 0; i < NumberOfGroups; i++)
            {
                AddStoreRegister(map, 0x180 + i * 4, $"BRSSEL{i}");
            }

            // BRSPND[0-3] @ 0x1C0-0x1CC
            for(int i = 0; i < NumberOfGroups; i++)
            {
                AddStoreRegister(map, 0x1C0 + i * 4, $"BRSPND{i}");
            }

            // BRSCTRL @ 0x200
            AddStoreRegister(map, 0x200, "BRSCTRL");

            // BRSMR @ 0x204: background scan mode register
            // Writing LDEV (bit 9) is a load event: with conversion requests
            // enabled (ENGT [1:0] != 0) the selected channels are converted.
            // The REQGT gate input is not modelled, so ENGT 10/11 count as on.
            map[0x204] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "BRSMR",
                    valueProviderCallback: _ => backgroundScanMode,
                    writeCallback: (_, value) =>
                    {
                        backgroundScanMode = (uint)value;
                        bool ldev = (value & BrsmrLdev) != 0;
                        bool engt = (value & BrsmrEngt) != 0;
                        if(ldev && engt && !scanActive)
                        {
                            StartBackgroundScan();
                        }
                    });

            // GLOBRCR @ 0x280
            AddStoreRegister(map, 0x280, "GLOBRCR");

            // GLOBRES @ 0x300 and GLOBRESD @ 0x380 handled in Read/WriteDoubleWord

            // EMUXSEL @ 0x3F0
            AddStoreRegister(map, 0x3F0, "EMUXSEL");
        }

        private void BuildGroupRegisters(Dictionary<long, DoubleWordRegister> map, int groupIndex)
        {
            long b = GlobalRegionSize + groupIndex * GroupRegionSize;
            var grp = groups[groupIndex];
            string prefix = $"G{groupIndex}";

            // ARBCFG @ +0x080: arbitration config
            // ANONS[17:16] should mirror ANONC[1:0] (converter powered = configured)
            map[b + 0x080] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: $"{prefix}_ARBCFG",
                    valueProviderCallback: _ =>
                    {
                        uint val = grp.ArbCfg;
                        // Mirror ANONC (bits 5:4) into ANONS (bits 17:16)
                        uint anonc = (val >> 4) & 0x3u;
                        val = (val & ~(0x3u << 16)) | (anonc << 16);
                        return val;
                    },
                    writeCallback: (_, value) => grp.ArbCfg = (uint)value);

            // ARBPR @ +0x084
            AddGroupStoreRegister(map, b + 0x084, $"{prefix}_ARBPR", grp, 0);

            // CHASS @ +0x088
            AddGroupStoreRegister(map, b + 0x088, $"{prefix}_CHASS", grp, 1);

            // ICLASS[0-1] @ +0x0A0, +0x0A4
            AddGroupStoreRegister(map, b + 0x0A0, $"{prefix}_ICLASS0", grp, 2);
            AddGroupStoreRegister(map, b + 0x0A4, $"{prefix}_ICLASS1", grp, 3);

            // ALIAS @ +0x0B0
            AddGroupStoreRegister(map, b + 0x0B0, $"{prefix}_ALIAS", grp, 4);

            // BOUND @ +0x0B8
            AddGroupStoreRegister(map, b + 0x0B8, $"{prefix}_BOUND", grp, 5);

            // SYNCTR @ +0x0C0
            AddGroupStoreRegister(map, b + 0x0C0, $"{prefix}_SYNCTR", grp, 6);

            // BFL @ +0x0C8
            AddGroupStoreRegister(map, b + 0x0C8, $"{prefix}_BFL", grp, 7);

            // BFL related
            AddGroupStoreRegister(map, b + 0x0CC, $"{prefix}_BFLS", grp, 8);
            AddGroupStoreRegister(map, b + 0x0D0, $"{prefix}_BFLC", grp, 9);
            AddGroupStoreRegister(map, b + 0x0D8, $"{prefix}_BFLNP", grp, 10);

            // Queue source registers
            AddGroupStoreRegister(map, b + 0x100, $"{prefix}_QCTRL0", grp, 11);
            AddGroupStoreRegister(map, b + 0x104, $"{prefix}_QMR0", grp, 12);
            AddGroupStoreRegister(map, b + 0x108, $"{prefix}_QSR0", grp, 13);
            AddGroupStoreRegister(map, b + 0x10C, $"{prefix}_Q0R0", grp, 14);
            AddGroupStoreRegister(map, b + 0x110, $"{prefix}_QINR0", grp, 15);

            // Autoscan source registers
            AddGroupStoreRegister(map, b + 0x120, $"{prefix}_ASCTRL", grp, 16);
            AddGroupStoreRegister(map, b + 0x124, $"{prefix}_ASMR", grp, 17);
            AddGroupStoreRegister(map, b + 0x128, $"{prefix}_ASSEL", grp, 18);
            AddGroupStoreRegister(map, b + 0x12C, $"{prefix}_ASPND", grp, 19);

            // Event flag registers
            AddGroupStoreRegister(map, b + 0x180, $"{prefix}_CEFLAG", grp, 20);
            AddGroupStoreRegister(map, b + 0x184, $"{prefix}_REFLAG", grp, 21);
            AddGroupStoreRegister(map, b + 0x188, $"{prefix}_SEFLAG", grp, 22);

            // Event flag clear registers (write-only, clear bits in flag register)
            map[b + 0x190] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: $"{prefix}_CEFCLR",
                    writeCallback: (_, value) => grp.Registers[20] &= ~(uint)value);

            map[b + 0x194] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: $"{prefix}_REFCLR",
                    writeCallback: (_, value) => grp.Registers[21] &= ~(uint)value);

            map[b + 0x198] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: $"{prefix}_SEFCLR",
                    writeCallback: (_, value) => grp.Registers[22] &= ~(uint)value);

            // Event node pointers
            AddGroupStoreRegister(map, b + 0x1A0, $"{prefix}_CEVNP0", grp, 23);
            AddGroupStoreRegister(map, b + 0x1B0, $"{prefix}_REVNP0", grp, 24);
            AddGroupStoreRegister(map, b + 0x1B4, $"{prefix}_REVNP1", grp, 25);
            AddGroupStoreRegister(map, b + 0x1C0, $"{prefix}_SEVNP", grp, 26);

            // SRACT @ +0x1C8 (write-only trigger)
            map[b + 0x1C8] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: $"{prefix}_SRACT");

            // EMUXCTR @ +0x1F0
            AddGroupStoreRegister(map, b + 0x1F0, $"{prefix}_EMUXCTR", grp, 27);

            // VFR @ +0x1F8
            AddGroupStoreRegister(map, b + 0x1F8, $"{prefix}_VFR", grp, 28);

            // CHCTR[0-7] @ +0x200
            for(int ch = 0; ch < ChannelsPerGroup; ch++)
            {
                var chIdx = ch;
                map[b + 0x200 + ch * 4] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"{prefix}_CHCTR{chIdx}",
                        valueProviderCallback: _ => grp.ChannelControl[chIdx],
                        writeCallback: (_, value) => grp.ChannelControl[chIdx] = (uint)value);
            }

            // RCR[0-15] @ +0x280
            for(int r = 0; r < ResultRegCount; r++)
            {
                var rIdx = r;
                map[b + 0x280 + r * 4] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"{prefix}_RCR{rIdx}",
                        valueProviderCallback: _ => grp.ResultControl[rIdx],
                        writeCallback: (_, value) => grp.ResultControl[rIdx] = (uint)value);
            }

            // RES[0-15] @ +0x300 and RESD[0-15] @ +0x380 handled in Read/WriteDoubleWord
        }

        private void AddStoreRegister(Dictionary<long, DoubleWordRegister> map, long offset, string name)
        {
            map[offset] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: name);
        }

        private void AddGroupStoreRegister(Dictionary<long, DoubleWordRegister> map, long offset, string name,
            VadcGroup grp, int regIndex)
        {
            map[offset] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: name,
                    valueProviderCallback: _ => grp.Registers[regIndex],
                    writeCallback: (_, value) => grp.Registers[regIndex] = (uint)value);
        }

        private uint clockControlRegister;
        private uint globalResultValue;
        private uint globalEventNodePointer;
        private uint backgroundScanMode;
        private bool scanActive;
        private TimeInterval lastScanTime;
        private readonly long peripheralClockFrequency;

        private readonly IMachine machine;
        private readonly VadcGroup[] groups;
        private readonly DoubleWordRegisterCollection registers;

        private const uint ModuleId = 0x00C5C000;
        private const int NumberOfGroups = 4;
        private const int NumberOfServiceRequestNodes = 4;
        private const int ChannelsPerGroup = 8;
        private const int ResultRegCount = 16;
        private const long GlobalRegionSize = 0x400;
        private const long GroupRegionSize = 0x400;
        private const long ResultRegOffset = 0x300;
        private const long ResultDebugRegOffset = 0x380;
        private const long GlobalResultOffset = 0x300;
        private const long GlobalResultDebugOffset = 0x380;
        private const int GroupRegisterCount = 29;
        private const int ScanDelayMicroseconds = 100;
        private const long GlobalConfigOffset = 0x080;
        private const long GlobalInputClassOffset = 0x0A0;
        private const long BackgroundSelectOffset = 0x180;
        private const uint BrsmrEngt = 0x3u;
        private const uint BrsmrEnsi = 1u << 3;
        private const uint BrsmrScan = 1u << 4;
        private const uint BrsmrLdev = 1u << 9;
        private static readonly uint[] ArbitrationRoundSlots = { 4, 8, 16, 20 };

        private class VadcGroup
        {
            public uint ArbCfg;
            public uint[] Registers = new uint[GroupRegisterCount];
            public uint[] ChannelControl = new uint[ChannelsPerGroup];
            public uint[] ResultControl = new uint[ResultRegCount];
            public ushort[] ResultValues = new ushort[ResultRegCount];

            public void Reset()
            {
                ArbCfg = 0;
                Array.Clear(Registers, 0, Registers.Length);
                Array.Clear(ChannelControl, 0, ChannelControl.Length);
                Array.Clear(ResultControl, 0, ResultControl.Length);
                Array.Clear(ResultValues, 0, ResultValues.Length);
            }
        }
    }
}
