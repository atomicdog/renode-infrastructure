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

namespace Antmicro.Renode.Peripherals.GPIOPort
{
    // Models the XMC4xxx PORT peripheral block at 0x48028000.
    //
    // Port layout depends on variant:
    //   XMC4500 (extendedPorts=false, default): P0–P6, P14, P15 (9 ports)
    //   XMC4800 (extendedPorts=true):           P0–P9, P14, P15 (12 ports)
    //
    // Pin numbering seen by the Renode GPIO system:
    //   port N pins 0–15  → connections  (portIndex * 16) .. (portIndex * 16 + 15)
    //
    // XMC4500: P14 → index 7 (pins 112–127), P15 → index 8 (pins 128–143)
    // XMC4800: P14 → index 10 (pins 160–175), P15 → index 11 (pins 176–191)
    public class XMC4XXX_GPIOPort : BaseGPIOPort, IDoubleWordPeripheral, IKnownSize
    {
        public XMC4XXX_GPIOPort(IMachine machine, bool extendedPorts = false)
            : base(machine, (extendedPorts ? ExtendedNumberOfPorts : BaseNumberOfPorts) * PinsPerPort)
        {
            if(extendedPorts)
            {
                numberOfPorts = ExtendedNumberOfPorts;
                firstAnalogPortIndex = ExtendedFirstAnalogPortIndex;
                portOffsets = PortOffsetsExtended;
            }
            else
            {
                numberOfPorts = BaseNumberOfPorts;
                firstAnalogPortIndex = BaseFirstAnalogPortIndex;
                portOffsets = PortOffsetsBase;
            }
            totalPins = numberOfPorts * PinsPerPort;

            outputData  = new bool[totalPins];
            isOutput    = new bool[totalPins];
            pdiscState  = new bool[totalPins];

            var map = new Dictionary<long, DoubleWordRegister>();
            for(var p = 0; p < numberOfPorts; p++)
            {
                DefinePortRegisters(map, p, portOffsets[p], isAnalogPort: p >= firstAnalogPortIndex);
            }
            registers = new DoubleWordRegisterCollection(this, map);
        }

        public override void Reset()
        {
            base.Reset();
            Array.Clear(outputData,  0, totalPins);
            Array.Clear(isOutput,    0, totalPins);
            Array.Clear(pdiscState,  0, totalPins);
            registers.Reset();
            UpdateAllConnections();
        }

        public uint ReadDoubleWord(long offset)
        {
            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            registers.Write(offset, value);
        }

        public override void OnGPIO(int number, bool value)
        {
            if(!CheckPinNumber(number))
            {
                return;
            }
            if(isOutput[number])
            {
                this.Log(LogLevel.Warning, "Pin {0} is configured as output, ignoring incoming GPIO signal.", number);
                return;
            }
            base.OnGPIO(number, value);
        }

        // The full mapped range must cover up to the end of P15 (offset 0xF00 + 0x100 = 0x1000).
        public long Size => 0x1000;

        private void DefinePortRegisters(Dictionary<long, DoubleWordRegister> map, int portIndex, long portBase, bool isAnalogPort)
        {
            var pinBase = portIndex * PinsPerPort;

            // OUT @ +0x00 — output latch
            map.Add(portBase + (long)PortReg.OUT, new DoubleWordRegister(this)
                .WithFlags(0, PinsPerPort, name: "Pn_OUT",
                    writeCallback: (pin, _, val) =>
                    {
                        outputData[pinBase + pin] = val;
                    },
                    valueProviderCallback: (pin, _) => outputData[pinBase + pin])
                .WithReservedBits(PinsPerPort, 32 - PinsPerPort)
                .WithWriteCallback((_, __) => UpdatePortConnections(portIndex))
            );

            // OMR @ +0x04 — atomic set/clear (write-only)
            // bits  0-15: PS0-PS15 set the corresponding output pin
            // bits 16-31: PR0-PR15 reset (clear) the corresponding output pin
            map.Add(portBase + (long)PortReg.OMR, new DoubleWordRegister(this)
                .WithWriteCallback((_, val) =>
                {
                    for(var pin = 0; pin < PinsPerPort; pin++)
                    {
                        var setBit   = (val >> pin)        & 1u;
                        var clearBit = (val >> (pin + 16)) & 1u;
                        if(clearBit != 0)
                        {
                            outputData[pinBase + pin] = false;
                        }
                        if(setBit != 0)
                        {
                            outputData[pinBase + pin] = true;
                        }
                    }
                    UpdatePortConnections(portIndex);
                })
            );

            // IOCR0/4/8/12 @ +0x10/0x14/0x18/0x1C — four pins per register
            // Each pin has a 5-bit PC field at bits [7:3], [15:11], [23:19], [31:27].
            // PC[4] (bit 4 of the field, i.e. the MSbit) distinguishes output (1) from input (0).
            for(var reg = 0; reg < 4; reg++)
            {
                var r = reg;
                var iocr = new DoubleWordRegister(this);
                for(var slot = 0; slot < 4; slot++)
                {
                    var s    = slot;
                    var pin  = r * 4 + s;
                    var lsb  = slot * 8 + 3;   // bits [lsb+4 : lsb]
                    iocr.WithValueField(lsb, 5, name: $"PC{pin}",
                        writeCallback: (_, val) =>
                        {
                            // bit 4 of the 5-bit field = output enable
                            isOutput[pinBase + pin] = ((val >> 4) & 1u) != 0;
                            UpdatePortConnections(portIndex);
                        },
                        valueProviderCallback: _ => isOutput[pinBase + pin] ? 0x10u : 0x00u);
                    if(lsb > 0)
                    {
                        iocr.WithReservedBits(lsb - 3, 3);
                    }
                }
                map.Add(portBase + (long)PortReg.IOCR0 + r * 4, iocr);
            }

            // IN @ +0x24 — current pad level (read-only)
            map.Add(portBase + (long)PortReg.IN, new DoubleWordRegister(this)
                .WithFlags(0, PinsPerPort, FieldMode.Read, name: "Pn_IN",
                    valueProviderCallback: (pin, _) =>
                        isOutput[pinBase + pin] ? outputData[pinBase + pin] : State[pinBase + pin])
                .WithReservedBits(PinsPerPort, 32 - PinsPerPort)
            );

            if(!isAnalogPort)
            {
                // PDR0 @ +0x40, PDR1 @ +0x44 — pad driver mode (tagged; no functional effect)
                // P14 and P15 do not have these registers.
                map.Add(portBase + (long)PortReg.PDR0, new DoubleWordRegister(this, 0x22222222)
                    .WithTag("PDR0", 0, 32));
                map.Add(portBase + (long)PortReg.PDR1, new DoubleWordRegister(this, 0x22222222)
                    .WithTag("PDR1", 0, 32));
            }

            if(isAnalogPort)
            {
                // PDISC @ +0x60 — analog/digital pin select (read-write for P14/P15).
                // Writing 1 disconnects the digital input buffer and routes the pin to the ADC.
                map.Add(portBase + (long)PortReg.PDISC, new DoubleWordRegister(this)
                    .WithFlags(0, PinsPerPort, name: "PDISn",
                        writeCallback: (pin, _, val) => pdiscState[pinBase + pin] = val,
                        valueProviderCallback: (pin, _) => pdiscState[pinBase + pin])
                    .WithReservedBits(PinsPerPort, 32 - PinsPerPort)
                );
            }
            else
            {
                // PDISC @ +0x60 — read-only for P0–P6; all pads always enabled.
                map.Add(portBase + (long)PortReg.PDISC, new DoubleWordRegister(this)
                    .WithFlags(0, PinsPerPort, FieldMode.Read, name: "PDISn",
                        valueProviderCallback: (pin, _) => false)
                    .WithReservedBits(PinsPerPort, 32 - PinsPerPort)
                );
            }

            // PPS @ +0x70 — pin power save (tagged)
            map.Add(portBase + (long)PortReg.PPS, new DoubleWordRegister(this)
                .WithTag("PPS", 0, PinsPerPort)
                .WithReservedBits(PinsPerPort, 32 - PinsPerPort)
            );

            // HWSEL @ +0x74 — hardware override select (tagged)
            map.Add(portBase + (long)PortReg.HWSEL, new DoubleWordRegister(this)
                .WithTag("HWSEL", 0, 32)
            );
        }

        private void UpdatePortConnections(int portIndex)
        {
            var pinBase = portIndex * PinsPerPort;
            for(var pin = 0; pin < PinsPerPort; pin++)
            {
                Connections[pinBase + pin].Set(isOutput[pinBase + pin] && outputData[pinBase + pin]);
            }
        }

        private void UpdateAllConnections()
        {
            for(var p = 0; p < numberOfPorts; p++)
            {
                UpdatePortConnections(p);
            }
        }

        private readonly DoubleWordRegisterCollection registers;
        private readonly bool[] outputData;
        private readonly bool[] isOutput;
        private readonly bool[] pdiscState;   // P14/P15 only: 1 = analog (digital input disabled)

        private readonly int numberOfPorts;
        private readonly int totalPins;
        private readonly int firstAnalogPortIndex;
        private readonly long[] portOffsets;

        private const int PinsPerPort = 16;

        // XMC4500: P0–P6, P14, P15 (9 ports)
        private const int BaseNumberOfPorts       = 9;
        private const int BaseFirstAnalogPortIndex = 7;  // indices 7 (P14) and 8 (P15)

        // XMC4800: P0–P9, P14, P15 (12 ports)
        private const int ExtendedNumberOfPorts       = 12;
        private const int ExtendedFirstAnalogPortIndex = 10;  // indices 10 (P14) and 11 (P15)

        // Byte offsets from the peripheral base for each port.
        private static readonly long[] PortOffsetsBase =
        {
            0x000, 0x100, 0x200, 0x300, 0x400, 0x500, 0x600,  // P0–P6
            0xE00, 0xF00,                                        // P14, P15
        };

        private static readonly long[] PortOffsetsExtended =
        {
            0x000, 0x100, 0x200, 0x300, 0x400, 0x500, 0x600,  // P0–P6
            0x700, 0x800, 0x900,                                  // P7–P9
            0xE00, 0xF00,                                          // P14, P15
        };

        private enum PortReg : long
        {
            OUT   = 0x00,
            OMR   = 0x04,
            IOCR0 = 0x10,
            IN    = 0x24,
            PDR0  = 0x40,
            PDR1  = 0x44,
            PDISC = 0x60,
            PPS   = 0x70,
            HWSEL = 0x74,
        }
    }
}
