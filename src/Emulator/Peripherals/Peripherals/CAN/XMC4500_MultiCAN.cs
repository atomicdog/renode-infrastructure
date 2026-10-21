//
// Copyright (c) 2010-2025 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.CAN;
using Antmicro.Renode.Tools.Network;
using Antmicro.Renode.Utilities;

namespace Antmicro.Renode.Peripherals.CAN
{
    [AllowedTranslations(AllowedTranslation.ByteToDoubleWord)]
    public class XMC4500_MultiCAN : IDoubleWordPeripheral, IKnownSize, ICAN, INumberedGPIOOutput
    {
        public XMC4500_MultiCAN(long peripheralClockFrequency = 120000000)
        {
            this.peripheralClockFrequency = peripheralClockFrequency;
            var innerConnections = new Dictionary<int, IGPIO>();
            for(int i = 0; i < NumberOfInterruptLines; i++)
            {
                innerConnections[i] = new GPIO();
            }
            Connections = new ReadOnlyDictionary<int, IGPIO>(innerConnections);

            messageObjects = new MessageObject[NumberOfMessageObjects];
            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                messageObjects[i] = new MessageObject(i);
            }

            nodeRegisters = new NodeRegisters[NumberOfNodes];
            for(int i = 0; i < NumberOfNodes; i++)
            {
                nodeRegisters[i] = new NodeRegisters();
            }

            listRegisters = new uint[NumberOfLists];
            messagePending = new uint[NumberOfServiceRequests];

            nodeProxies = new CANNodeProxy[NumberOfNodes];
            nodeBusyUntilTicks = new ulong[NumberOfNodes];
            pendingTransmits = new Queue<int>[NumberOfNodes];
            transmitScheduled = new bool[NumberOfNodes];
            for(int i = 0; i < NumberOfNodes; i++)
            {
                nodeProxies[i] = new CANNodeProxy(this, i);
                pendingTransmits[i] = new Queue<int>();
            }
            Node0 = nodeProxies[0];
            Node1 = nodeProxies[1];
            Node2 = nodeProxies[2];

            registers = BuildRegisters();
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            // Message object region: 0x1000 - 0x17FF
            if(offset >= MessageObjectBaseOffset && offset < MessageObjectBaseOffset + NumberOfMessageObjects * MessageObjectSize)
            {
                return ReadMessageObject(offset);
            }

            return registers.Read(offset);
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            // Message object region: 0x1000 - 0x17FF
            if(offset >= MessageObjectBaseOffset && offset < MessageObjectBaseOffset + NumberOfMessageObjects * MessageObjectSize)
            {
                WriteMessageObject(offset, value);
                return;
            }

            registers.Write(offset, value);
        }

        public void Reset()
        {
            registers.Reset();

            // Reset CLC: DISR=1, DISS=1
            clockControlRegister = 0x3;
            fractionalDivider = 0;
            moduleControl = 0;
            messageIndexMask = 0;
            // SVD reset = 0x00000301 (PANCMD=0x01 init-list-0, BUSY+RBUSY set).
            // Commands execute instantly in Renode, so clear BUSY/RBUSY.
            panelResult = 0x00000001;

            // Reset all MOs to list 0 (unallocated)
            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                messageObjects[i].Reset();
            }

            // Reset nodes
            for(int i = 0; i < NumberOfNodes; i++)
            {
                nodeRegisters[i].Reset();
                nodeBusyUntilTicks[i] = 0;
                pendingTransmits[i].Clear();
                transmitScheduled[i] = false;
            }

            // Reset message pending registers
            for(int i = 0; i < NumberOfServiceRequests; i++)
            {
                messagePending[i] = 0;
            }

            // Initialize list 0 with all 64 MOs in a linked list
            InitializeDefaultList();

            // Reset interrupt outputs
            foreach(var connection in Connections.Values)
            {
                connection.Unset();
            }
        }

        public void OnFrameReceived(CANMessageFrame message)
        {
            this.DebugLog("RX frame: ID=0x{0:X} IDE={1} DLC={2}",
                message.Id, message.ExtendedFormat, message.Data.Length);

            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                var mo = messageObjects[i];

                if(!IsRxCandidate(mo))
                {
                    continue;
                }

                if(!AcceptsFrame(mo, message))
                {
                    continue;
                }

                // Check for FIFO/gateway modes (MMC != 0) — not yet supported
                uint mmc = mo.FunctionControl & 0xFu;
                if(mmc != 0)
                {
                    this.WarningLog("MO{0}: Matched frame but MMC=0x{1:X} (FIFO/gateway) not supported, skipping",
                        i, mmc);
                    continue;
                }

                StoreFrame(mo, message);
                HandleRxInterrupt(i);

                this.DebugLog("MO{0}: Accepted RX frame ID=0x{1:X}", i, message.Id);
                return;
            }

            this.NoisyLog("RX frame ID=0x{0:X}: no matching MO found", message.Id);
        }

        public void OnFrameReceivedOnNode(int nodeIndex, CANMessageFrame message)
        {
            if(nodeIndex < 0 || nodeIndex >= NumberOfNodes)
            {
                this.WarningLog("OnFrameReceivedOnNode: invalid node index {0}", nodeIndex);
                return;
            }

            int targetList = nodeIndex + 1;
            this.DebugLog("RX frame on Node{0}: ID=0x{1:X} IDE={2} DLC={3}",
                nodeIndex, message.Id, message.ExtendedFormat, message.Data.Length);

            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                var mo = messageObjects[i];

                if(!IsRxCandidate(mo))
                {
                    if(mo.ListNumber == targetList)
                    {
                        this.NoisyLog("MO{0}: on list {1} but not RX candidate (flags=0x{2:X})",
                            i, targetList, mo.StatusFlags);
                    }
                    continue;
                }

                // Only match MOs assigned to this node's list
                if(mo.ListNumber != targetList)
                {
                    continue;
                }

                if(!AcceptsFrame(mo, message))
                {
                    this.NoisyLog("MO{0}: on list {1} RX candidate but ID 0x{2:X} rejected by filter",
                        i, targetList, message.Id);
                    continue;
                }

                uint mmc = mo.FunctionControl & 0xFu;
                if(mmc != 0)
                {
                    this.WarningLog("MO{0}: Matched frame but MMC=0x{1:X} (FIFO/gateway) not supported, skipping",
                        i, mmc);
                    continue;
                }

                StoreFrame(mo, message);
                HandleRxInterrupt(i);

                this.DebugLog("MO{0}: Accepted RX frame ID=0x{1:X} on Node{2}", i, message.Id, nodeIndex);
                return;
            }

            this.NoisyLog("RX frame ID=0x{0:X} on Node{1}: no matching MO found", message.Id, nodeIndex);
        }

        public event Action<CANMessageFrame> FrameSent;

        public CANNodeProxy Node0 { get; }
        public CANNodeProxy Node1 { get; }
        public CANNodeProxy Node2 { get; }

        /// <summary>
        /// Connect a specific CAN node to a CANHub.  Callable from the monitor:
        ///   can ConnectNodeToHub 1 dispCanHub
        /// </summary>
        public void ConnectNodeToHub(int nodeIndex, CANHub hub)
        {
            if(nodeIndex < 0 || nodeIndex >= NumberOfNodes)
            {
                throw new Antmicro.Renode.Exceptions.RecoverableException(
                    $"Invalid node index {nodeIndex}, must be 0-{NumberOfNodes - 1}");
            }
            var proxy = nodeProxies[nodeIndex];

            // Ensure the proxy is registered with the machine so the CANHub
            // can resolve GetMachine()/GetName() on it.
            var machine = this.GetMachine();
            try
            {
                machine.RegisterAsAChildOf(this, proxy,
                    Antmicro.Renode.Core.Structure.NullRegistrationPoint.Instance);
                machine.SetLocalName(proxy, $"Node{nodeIndex}");
            }
            catch(Antmicro.Renode.Exceptions.RegistrationException)
            {
                // Already registered from a previous call — that's fine
            }

            hub.AttachTo(proxy);
            this.Log(LogLevel.Info, "Node{0} connected to CANHub", nodeIndex);
        }

        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        public long Size => 0x4000;

        private DoubleWordRegisterCollection BuildRegisters()
        {
            var registersMap = new Dictionary<long, DoubleWordRegister>();

            // CLC @ 0x000: Clock Control Register
            // Reset=0x3 (DISR=1, DISS=1). When DISR is cleared, immediately clear DISS.
            registersMap[0x000] = new DoubleWordRegister(this, 0x3)
                .WithValueField(0, 32, name: "CLC",
                    valueProviderCallback: _ => clockControlRegister,
                    writeCallback: (_, value) =>
                    {
                        clockControlRegister = (uint)value;
                        if((value & 0x1) == 0)
                        {
                            // DISR cleared -> immediately clear DISS
                            clockControlRegister &= ~0x2u;
                            this.DebugLog("CLC: Module enabled (DISR cleared, DISS cleared)");
                        }
                        else
                        {
                            // DISR set -> set DISS
                            clockControlRegister |= 0x2u;
                            this.DebugLog("CLC: Module disabled (DISR set, DISS set)");
                        }
                    });

            // ID @ 0x008: Module Identification Register (read-only)
            registersMap[0x008] = new DoubleWordRegister(this, ModuleId)
                .WithValueField(0, 32, FieldMode.Read, name: "ID");

            // FDR @ 0x00C: Fractional Divider Register (store only)
            registersMap[0x00C] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "FDR",
                    valueProviderCallback: _ => fractionalDivider,
                    writeCallback: (_, value) =>
                    {
                        fractionalDivider = (uint)value;
                        this.NoisyLog("FDR: Written 0x{0:X8}", value);
                    });

            // LIST[8] @ 0x100-0x11C: List Registers (read-only, maintained by panel controller)
            for(int i = 0; i < NumberOfLists; i++)
            {
                var listIndex = i;
                registersMap[0x100 + i * 4] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, FieldMode.Read, name: $"LIST{listIndex}",
                        valueProviderCallback: _ => listRegisters[listIndex]);
            }

            // MSPND[8] @ 0x140-0x15C: Message Pending Registers (read/write-1-to-clear)
            for(int i = 0; i < NumberOfServiceRequests; i++)
            {
                var srIndex = i;
                registersMap[0x140 + i * 4] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"MSPND{srIndex}",
                        valueProviderCallback: _ => messagePending[srIndex],
                        writeCallback: (_, value) =>
                        {
                            // Write-1-to-clear semantics
                            messagePending[srIndex] &= ~(uint)value;
                        });
            }

            // MSID[8] @ 0x180-0x19C: Message Index Registers (read-only)
            for(int i = 0; i < NumberOfServiceRequests; i++)
            {
                var srIndex = i;
                registersMap[0x180 + i * 4] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, FieldMode.Read, name: $"MSID{srIndex}",
                        valueProviderCallback: _ => GetMessageIndex(srIndex));
            }

            // MSIMASK @ 0x1C0: Message Index Mask
            registersMap[0x1C0] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "MSIMASK",
                    valueProviderCallback: _ => messageIndexMask,
                    writeCallback: (_, value) => messageIndexMask = (uint)value);

            // PANCTR @ 0x1C4: Panel Control Register
            // BUSY(bit8) and RBUSY(bit9) always read 0 — commands execute instantly.
            registersMap[0x1C4] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "PANCTR",
                    valueProviderCallback: _ => panelResult,
                    writeCallback: (_, value) =>
                    {
                        ExecutePanelCommand((uint)value);
                    });

            // MCR @ 0x1C8: Module Control Register (store only)
            registersMap[0x1C8] = new DoubleWordRegister(this)
                .WithValueField(0, 32, name: "MCR",
                    valueProviderCallback: _ => moduleControl,
                    writeCallback: (_, value) =>
                    {
                        moduleControl = (uint)value;
                        this.NoisyLog("MCR: Written 0x{0:X8}", value);
                    });

            // MITR @ 0x1CC: Module Interrupt Trigger Register (write-only)
            registersMap[0x1CC] = new DoubleWordRegister(this)
                .WithValueField(0, 32, FieldMode.Write, name: "MITR",
                    writeCallback: (_, value) =>
                    {
                        var srn = (int)(value & 0x7);
                        this.DebugLog("MITR: Triggering interrupt SRN{0}", srn);
                        if(srn < NumberOfInterruptLines)
                        {
                            Connections[srn].Blink();
                        }
                    });

            // Node registers: 3 nodes, each 0x100 bytes starting at 0x200
            for(int n = 0; n < NumberOfNodes; n++)
            {
                var nodeIndex = n;
                long nodeBase = 0x200 + n * 0x100;

                // NCR @ nodeBase + 0x00: Node Control Register
                // Reset=0x01 (INIT=1)
                registersMap[nodeBase + 0x00] = new DoubleWordRegister(this, 0x01)
                    .WithValueField(0, 32, name: $"N{nodeIndex}CR",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].Control,
                        writeCallback: (_, value) =>
                        {
                            nodeRegisters[nodeIndex].Control = (uint)value;
                            this.NoisyLog("Node{0} NCR: 0x{1:X8}", nodeIndex, value);
                            UpdateInterruptLines();
                        });

                // NSR @ nodeBase + 0x04: Node Status Register
                // LEC[2:0] writable, bits 3-7 are write-1-to-clear
                registersMap[nodeBase + 0x04] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"N{nodeIndex}SR",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].Status,
                        writeCallback: (_, value) =>
                        {
                            uint val = (uint)value;
                            // Bits 3-7 are write-1-to-clear
                            uint w1cBits = val & 0xF8u;
                            nodeRegisters[nodeIndex].Status &= ~w1cBits;
                            // Bits 0-2 (LEC) are directly writable
                            nodeRegisters[nodeIndex].Status = (nodeRegisters[nodeIndex].Status & ~0x7u) | (val & 0x7u);
                            UpdateInterruptLines();
                        });

                // NIPR @ nodeBase + 0x08: Node Interrupt Pointer Register
                registersMap[nodeBase + 0x08] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"N{nodeIndex}IPR",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].InterruptPointer,
                        writeCallback: (_, value) =>
                        {
                            nodeRegisters[nodeIndex].InterruptPointer = (uint)value;
                            this.NoisyLog("Node{0} NIPR: 0x{1:X8}", nodeIndex, value);
                        });

                // NPCR @ nodeBase + 0x0C: Node Port Control Register (CCE-gated)
                registersMap[nodeBase + 0x0C] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"N{nodeIndex}PCR",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].PortControl,
                        writeCallback: (_, value) =>
                        {
                            if((nodeRegisters[nodeIndex].Control & NcrCce) != 0)
                            {
                                nodeRegisters[nodeIndex].PortControl = (uint)value;
                            }
                            else
                            {
                                this.WarningLog("Node{0}: NPCR write ignored, CCE not set", nodeIndex);
                            }
                        });

                // NBTR @ nodeBase + 0x10: Node Bit Timing Register (CCE-gated)
                registersMap[nodeBase + 0x10] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"N{nodeIndex}BTR",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].BitTiming,
                        writeCallback: (_, value) =>
                        {
                            if((nodeRegisters[nodeIndex].Control & NcrCce) != 0)
                            {
                                nodeRegisters[nodeIndex].BitTiming = (uint)value;
                                this.DebugLog("Node{0}: NBTR=0x{1:X8}, bit time {2} ns", nodeIndex, value, GetBitTimeTicks(nodeIndex));
                            }
                            else
                            {
                                this.WarningLog("Node{0}: NBTR write ignored, CCE not set", nodeIndex);
                            }
                        });

                // NECNT @ nodeBase + 0x14: Node Error Counter Register
                registersMap[nodeBase + 0x14] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"N{nodeIndex}ECNT",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].ErrorCount,
                        writeCallback: (_, value) =>
                        {
                            nodeRegisters[nodeIndex].ErrorCount = (uint)value;
                        });

                // NFCR @ nodeBase + 0x18: Node Frame Counter Register
                registersMap[nodeBase + 0x18] = new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: $"N{nodeIndex}FCR",
                        valueProviderCallback: _ => nodeRegisters[nodeIndex].FrameCounter,
                        writeCallback: (_, value) =>
                        {
                            nodeRegisters[nodeIndex].FrameCounter = (uint)value;
                        });
            }

            return new DoubleWordRegisterCollection(this, registersMap);
        }

        private uint ReadMessageObject(long offset)
        {
            long moOffset = offset - MessageObjectBaseOffset;
            int moIndex = (int)(moOffset / MessageObjectSize);
            int regOffset = (int)(moOffset % MessageObjectSize);

            if(moIndex < 0 || moIndex >= NumberOfMessageObjects)
            {
                this.WarningLog("MO read: invalid MO index {0}", moIndex);
                return 0;
            }

            var mo = messageObjects[moIndex];

            switch(regOffset)
            {
                case 0x00: // MOFCR: Function Control
                    return mo.FunctionControl;
                case 0x04: // MOFGPR: FIFO/Gateway Pointer
                    return mo.FifoGatewayPointer;
                case 0x08: // MOIPR: Interrupt Pointer
                    return mo.InterruptPointer;
                case 0x0C: // MOAMR: Acceptance Mask
                    return mo.AcceptanceMask;
                case 0x10: // MODATAL: Data Low
                    return mo.DataLow;
                case 0x14: // MODATAH: Data High
                    return mo.DataHigh;
                case 0x18: // MOAR: Arbitration
                    return mo.Arbitration;
                case 0x1C: // MOSTAT: Status (read view of dual MOCTR/MOSTAT register)
                    return mo.ReadStatus();
                default:
                    this.WarningLog("MO{0}: unhandled read at register offset 0x{1:X}", moIndex, regOffset);
                    return 0;
            }
        }

        private void WriteMessageObject(long offset, uint value)
        {
            long moOffset = offset - MessageObjectBaseOffset;
            int moIndex = (int)(moOffset / MessageObjectSize);
            int regOffset = (int)(moOffset % MessageObjectSize);

            if(moIndex < 0 || moIndex >= NumberOfMessageObjects)
            {
                this.WarningLog("MO write: invalid MO index {0}", moIndex);
                return;
            }

            var mo = messageObjects[moIndex];

            switch(regOffset)
            {
                case 0x00: // MOFCR
                    mo.FunctionControl = value;
                    UpdateInterruptLines();
                    break;
                case 0x04: // MOFGPR
                    mo.FifoGatewayPointer = value;
                    break;
                case 0x08: // MOIPR
                    mo.InterruptPointer = value;
                    break;
                case 0x0C: // MOAMR
                    mo.AcceptanceMask = value;
                    break;
                case 0x10: // MODATAL
                    mo.DataLow = value;
                    break;
                case 0x14: // MODATAH
                    mo.DataHigh = value;
                    break;
                case 0x18: // MOAR
                    mo.Arbitration = value;
                    break;
                case 0x1C: // MOCTR: Control (write view of dual MOCTR/MOSTAT register)
                    WriteMOCTR(moIndex, value);
                    break;
                default:
                    this.WarningLog("MO{0}: unhandled write at register offset 0x{1:X}, value=0x{2:X8}",
                        moIndex, regOffset, value);
                    break;
            }
        }

        private void WriteMOCTR(int moIndex, uint value)
        {
            var mo = messageObjects[moIndex];

            // Lower halfword bits [15:0]: writing 1 resets the corresponding flag (bits 0-11 only)
            // Upper halfword bits [31:16]: writing 1 sets the corresponding flag (bits 0-11 only)
            uint resetMask = value & 0x0FFFu;
            uint setMask = (value >> 16) & 0x0FFFu;

            // Apply reset first, then set (set takes priority if both are written)
            uint currentFlags = mo.StatusFlags;
            currentFlags &= ~resetMask;
            currentFlags |= setMask;
            mo.StatusFlags = currentFlags;

            // Firmware clearing NEWDAT means it has serviced the object; re-arm the
            // one-shot MSGLST warning so a later genuine overrun is logged again.
            if((resetMask & StatusNewDat) != 0)
            {
                mo.MsgLostWarned = false;
            }

            // Check if TXRQ (bit 8) was just set
            if((setMask & (1u << 8)) != 0)
            {
                this.DebugLog("MO{0}: TXRQ set, initiating transmit", moIndex);
                HandleTransmit(moIndex);
            }

            // Re-evaluate interrupt lines after status flag changes.
            // When firmware clears RXPND/TXPND, the corresponding line de-asserts.
            UpdateInterruptLines();
        }

        private void HandleTransmit(int moIndex)
        {
            var mo = messageObjects[moIndex];

            // Gate TX on node INIT bit
            int txListNum = mo.ListNumber;
            if(txListNum >= 1 && txListNum <= NumberOfNodes)
            {
                if((nodeRegisters[txListNum - 1].Control & NcrInit) != 0)
                {
                    this.DebugLog("MO{0}: TX suppressed, Node{1} INIT=1", moIndex, txListNum - 1);
                    return;
                }

                // CAN bus timing: a node can only start a frame once the previous one has
                // left the bus.  Keep TXRQ set and finish the transmit later instead of
                // dropping the frame, so TXOK is only reported for frames actually sent.
                if(DeferIfBusBusy(moIndex, txListNum - 1))
                {
                    return;
                }
            }

            // Extract CAN ID and IDE flag from MOAR
            uint arbitration = mo.Arbitration;
            bool ide = (arbitration & (1u << 29)) != 0;
            uint canId;
            if(ide)
            {
                // Extended frame: 29-bit ID in bits [28:0]
                canId = arbitration & 0x1FFFFFFFu;
            }
            else
            {
                // Standard frame: 11-bit ID in bits [28:18]
                canId = (arbitration >> 18) & 0x7FFu;
            }

            // Extract DLC from MOFCR bits [27:24]
            uint dlc = (mo.FunctionControl >> 24) & 0xFu;
            int dataLength = (int)Math.Min(dlc, 8u);

            // Extract data bytes from MODATAL (bytes 0-3) and MODATAH (bytes 4-7)
            byte[] data = new byte[dataLength];
            uint dataLow = mo.DataLow;
            uint dataHigh = mo.DataHigh;
            for(int i = 0; i < dataLength && i < 4; i++)
            {
                data[i] = (byte)((dataLow >> (i * 8)) & 0xFF);
            }
            for(int i = 4; i < dataLength && i < 8; i++)
            {
                data[i] = (byte)((dataHigh >> ((i - 4) * 8)) & 0xFF);
            }

            this.DebugLog("MO{0}: TX ID=0x{1:X} IDE={2} DLC={3} Data=[{4}]",
                moIndex, canId, ide, dlc,
                string.Join(" ", data.Select(b => $"0x{b:X2}")));

            // Clear TXRQ (bit 8), set TXPND (bit 1)
            mo.StatusFlags &= ~(1u << 8);
            mo.StatusFlags |= (1u << 1);

            // Set NSR.TXOK on the owning node (LIST 1-3 -> Node 0-2)
            int listNum = mo.ListNumber;
            if(listNum >= 1 && listNum <= NumberOfNodes)
            {
                int nodeIndex = listNum - 1;
                nodeRegisters[nodeIndex].Status |= NsrTxOk;
                this.NoisyLog("Node{0}: NSR.TXOK set", nodeIndex);
            }

            // Set MSPND bit: MSPND[TXINP] bit MPN
            uint txinp = (mo.InterruptPointer >> 4) & 0xFu;
            uint mpn = (mo.InterruptPointer >> 8) & 0xFFu;
            if(txinp < NumberOfServiceRequests && mpn < 32)
            {
                messagePending[txinp] |= (1u << (int)mpn);
            }

            // Re-evaluate all interrupt lines (level-sensitive model)
            UpdateInterruptLines();

            // Create the CAN frame
            var frame = new CANMessageFrame(canId, data, extendedFormat: ide);

            int txNodeIndex = listNum - 1;
            FrameSent?.Invoke(frame);

            // Fire per-node FrameSent (delivers to CANHub)
            if(txNodeIndex >= 0 && txNodeIndex < NumberOfNodes)
            {
                nodeProxies[txNodeIndex].RaiseFrameSent(frame);
                SchedulePendingTransmit(txNodeIndex);
            }

            // Loopback mode: TX frame loops back to RX on the same node
            if(txNodeIndex >= 0 && txNodeIndex < NumberOfNodes)
            {
                if((nodeRegisters[txNodeIndex].Control & NcrLm) != 0)
                {
                    this.DebugLog("MO{0}: Loopback on Node{1}", moIndex, txNodeIndex);
                    OnFrameReceivedOnNode(txNodeIndex, frame);
                }
            }
        }

        /// <summary>
        /// Returns true (and queues the MO) if the node's bus is still busy with the
        /// previous frame; otherwise claims the bus for this frame and returns false.
        /// </summary>
        private bool DeferIfBusBusy(int moIndex, int nodeIndex)
        {
            var nowTicks = this.GetMachine().ElapsedVirtualTime.TimeElapsed.Ticks;
            if(nowTicks >= nodeBusyUntilTicks[nodeIndex])
            {
                nodeBusyUntilTicks[nodeIndex] = nowTicks + GetFrameDurationTicks(nodeIndex, messageObjects[moIndex]);
                return false;
            }

            if(!pendingTransmits[nodeIndex].Contains(moIndex))
            {
                pendingTransmits[nodeIndex].Enqueue(moIndex);
            }
            this.NoisyLog("MO{0}: Node{1} bus busy, TX deferred", moIndex, nodeIndex);
            SchedulePendingTransmit(nodeIndex);
            return true;
        }

        /// <summary>
        /// Time the frame occupies the bus, derived from the node's bit rate (FDR + NBTR)
        /// and the frame's nominal length (stuff bits not counted).
        /// </summary>
        private ulong GetFrameDurationTicks(int nodeIndex, MessageObject mo)
        {
            var dataLength = Math.Min((mo.FunctionControl >> 24) & 0xFu, 8u);
            var extended = (mo.Arbitration & (1u << 29)) != 0;
            // Including the 3-bit interframe space
            var frameBits = (extended ? ExtendedFrameOverheadBits : StandardFrameOverheadBits) + 8 * dataLength;
            return Math.Max(1UL, frameBits * GetBitTimeTicks(nodeIndex));
        }

        private ulong GetBitTimeTicks(int nodeIndex)
        {
            var canClock = GetModuleClockFrequency();
            var bitTiming = nodeRegisters[nodeIndex].BitTiming;
            if(canClock == 0 || bitTiming == 0)
            {
                return DefaultBitTimeTicks;
            }

            // NBTR: BRP [5:0], TSEG1 [11:8], TSEG2 [14:12], DIV8 [15]
            var brp = (bitTiming & 0x3Fu) + 1;
            var div8 = (bitTiming & (1u << 15)) != 0 ? 8u : 1u;
            var tseg1 = (bitTiming >> 8) & 0xFu;
            var tseg2 = (bitTiming >> 12) & 0x7u;
            var quantaPerBit = tseg1 + tseg2 + 3;
            return (ulong)(brp * div8 * quantaPerBit * (double)Time.TimeInterval.TicksPerSecond / canClock);
        }

        private double GetModuleClockFrequency()
        {
            // FDR: STEP [9:0], DM [15:14]
            var step = fractionalDivider & 0x3FFu;
            switch((fractionalDivider >> 14) & 0x3u)
            {
            case 1: // normal divider
                return (double)peripheralClockFrequency / (1024 - step);
            case 2: // fractional divider
                return (double)peripheralClockFrequency * step / 1024;
            default: // clock off
                return 0;
            }
        }

        private void SchedulePendingTransmit(int nodeIndex)
        {
            if(transmitScheduled[nodeIndex] || pendingTransmits[nodeIndex].Count == 0)
            {
                return;
            }

            var nowTicks = this.GetMachine().ElapsedVirtualTime.TimeElapsed.Ticks;
            var busyUntil = nodeBusyUntilTicks[nodeIndex];
            var delay = Time.TimeInterval.FromTicks(busyUntil > nowTicks ? busyUntil - nowTicks : 0);
            transmitScheduled[nodeIndex] = true;
            this.GetMachine().ScheduleAction(delay, _ => TransmitPending(nodeIndex));
        }

        private void TransmitPending(int nodeIndex)
        {
            transmitScheduled[nodeIndex] = false;
            var queue = pendingTransmits[nodeIndex];
            while(queue.Count > 0)
            {
                var moIndex = queue.Dequeue();
                // The firmware may have cancelled the request while it was waiting
                if((messageObjects[moIndex].StatusFlags & StatusTxRq) == 0)
                {
                    continue;
                }
                // Sends now, or re-queues if the bus is (still) busy; either way the
                // next pending transmit gets scheduled.
                HandleTransmit(moIndex);
                return;
            }
        }

        private bool IsRxCandidate(MessageObject mo)
        {
            // MO must be valid, RX-enabled, direction=receive, and assigned to a node list (1-3)
            if((mo.StatusFlags & StatusMsgVal) == 0
                || (mo.StatusFlags & StatusRxEn) == 0
                || (mo.StatusFlags & StatusDir) != 0
                || mo.ListNumber < 1 || mo.ListNumber > NumberOfNodes)
            {
                return false;
            }

            // Gate RX on node INIT bit
            int nodeIndex = mo.ListNumber - 1;
            if((nodeRegisters[nodeIndex].Control & NcrInit) != 0)
            {
                return false;
            }

            return true;
        }

        private bool AcceptsFrame(MessageObject mo, CANMessageFrame message)
        {
            bool frameIde = message.ExtendedFormat;
            bool moIde = (mo.Arbitration & (1u << 29)) != 0;
            bool mide = (mo.AcceptanceMask & (1u << 29)) != 0;

            // If MIDE is set, IDE bits must match
            if(mide && frameIde != moIde)
            {
                return false;
            }

            uint mask = mo.AcceptanceMask & 0x1FFFFFFFu;
            uint moId;
            uint frameId;

            if(!moIde)
            {
                // Standard frame: 11-bit ID in MOAR bits [28:18]
                moId = (mo.Arbitration >> 18) & 0x7FFu;
                frameId = message.Id & 0x7FFu;
                uint effectiveMask = (mask >> 18) & 0x7FFu;
                return (frameId & effectiveMask) == (moId & effectiveMask);
            }
            else
            {
                // Extended frame: 29-bit ID in MOAR bits [28:0]
                moId = mo.Arbitration & 0x1FFFFFFFu;
                frameId = message.Id & 0x1FFFFFFFu;
                return (frameId & mask) == (moId & mask);
            }
        }

        private void StoreFrame(MessageObject mo, CANMessageFrame message)
        {
            // Pack data into MODATAL (bytes 0-3) and MODATAH (bytes 4-7)
            uint dataLow = 0;
            uint dataHigh = 0;
            int dataLength = Math.Min(message.Data.Length, 8);

            for(int i = 0; i < dataLength && i < 4; i++)
            {
                dataLow |= (uint)message.Data[i] << (i * 8);
            }
            for(int i = 4; i < dataLength && i < 8; i++)
            {
                dataHigh |= (uint)message.Data[i] << ((i - 4) * 8);
            }

            mo.DataLow = dataLow;
            mo.DataHigh = dataHigh;

            // Update MOAR: preserve PRI bits [31:30], write frame ID and IDE
            uint pri = mo.Arbitration & 0xC0000000u;
            if(message.ExtendedFormat)
            {
                mo.Arbitration = pri | (1u << 29) | (message.Id & 0x1FFFFFFFu);
            }
            else
            {
                mo.Arbitration = pri | ((message.Id & 0x7FFu) << 18);
            }

            // Update DLC in MOFCR bits [27:24]
            mo.FunctionControl = (mo.FunctionControl & 0xF0FFFFFFu) | ((uint)dataLength << 24);

            // If NEWDAT already set, mark message lost.  Warn only once per overrun
            // episode: a monitor object the firmware never drains (e.g. a wildcard
            // bus sniffer) would otherwise log on every single frame.  The warning
            // is re-armed in WriteMOCTR when the firmware clears NEWDAT.
            if((mo.StatusFlags & StatusNewDat) != 0)
            {
                mo.StatusFlags |= StatusMsgLst;
                if(!mo.MsgLostWarned)
                {
                    mo.MsgLostWarned = true;
                    this.WarningLog("MO{0}: MSGLST — new frame overwrites unread data (further overwrites on this object logged at debug level until it is serviced)", mo.Index);
                }
                else
                {
                    this.DebugLog("MO{0}: MSGLST — new frame overwrites unread data", mo.Index);
                }
            }

            // Set RXPND and NEWDAT
            mo.StatusFlags |= StatusRxPnd | StatusNewDat;
        }

        private void HandleRxInterrupt(int moIndex)
        {
            var mo = messageObjects[moIndex];

            // Set NSR.RXOK on the owning node
            int listNum = mo.ListNumber;
            if(listNum >= 1 && listNum <= NumberOfNodes)
            {
                int nodeIndex = listNum - 1;
                nodeRegisters[nodeIndex].Status |= NsrRxOk;
            }

            // Set MSPND bit: MSPND[RXINP] bit MPN
            uint rxinp = mo.InterruptPointer & 0x7u;
            uint mpn = (mo.InterruptPointer >> 8) & 0xFFu;
            if(rxinp < NumberOfServiceRequests && mpn < 32)
            {
                messagePending[rxinp] |= (1u << (int)mpn);
            }

            // Re-evaluate all interrupt lines (level-sensitive model).
            // Lines stay asserted while any MO has a pending+enabled condition.
            UpdateInterruptLines();
        }

        /// <summary>
        /// Level-sensitive interrupt line evaluation.
        /// Scans all MOs and node status registers to compute the correct
        /// state of each SRN output line. Lines stay asserted as long as
        /// any pending+enabled condition exists, matching real XMC4500 hardware.
        /// </summary>
        private void UpdateInterruptLines()
        {
            var lineState = new bool[NumberOfInterruptLines];

            // MO-level interrupt conditions
            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                var mo = messageObjects[i];

                // RX pending with RX interrupt enabled (RXIE = MOFCR bit 16)
                if((mo.StatusFlags & StatusRxPnd) != 0
                    && (mo.FunctionControl & (1u << 16)) != 0)
                {
                    uint rxinp = mo.InterruptPointer & 0x7u;
                    if(rxinp < NumberOfInterruptLines)
                    {
                        lineState[rxinp] = true;
                    }
                }

                // TX pending with TX interrupt enabled (TXIE = MOFCR bit 17)
                if((mo.StatusFlags & StatusTxPnd) != 0
                    && (mo.FunctionControl & (1u << 17)) != 0)
                {
                    uint txinp = (mo.InterruptPointer >> 4) & 0x7u;
                    if(txinp < NumberOfInterruptLines)
                    {
                        lineState[txinp] = true;
                    }
                }
            }

            // Node-level interrupt conditions
            for(int n = 0; n < NumberOfNodes; n++)
            {
                var node = nodeRegisters[n];

                // Transfer interrupt: TRIE enabled and (TXOK or RXOK) set
                if((node.Control & NcrTrie) != 0
                    && (node.Status & (NsrTxOk | NsrRxOk)) != 0)
                {
                    uint trinp = node.InterruptPointer & 0x7u;
                    if(trinp < NumberOfInterruptLines)
                    {
                        lineState[trinp] = true;
                    }
                }

                // Alert interrupt: ALIE enabled and ALERT set
                if((node.Control & NcrAlie) != 0
                    && (node.Status & NsrAlert) != 0)
                {
                    uint alinp = (node.InterruptPointer >> 4) & 0x7u;
                    if(alinp < NumberOfInterruptLines)
                    {
                        lineState[alinp] = true;
                    }
                }
            }

            // Apply computed state to GPIO outputs.
            // GPIO.Set is a no-op when the value hasn't changed,
            // so only actual transitions reach the NVIC.
            for(int i = 0; i < NumberOfInterruptLines; i++)
            {
                Connections[i].Set(lineState[i]);
            }
        }

        private void ExecutePanelCommand(uint value)
        {
            uint command = value & 0xFFu;
            // PANAR1 at bits [23:16], PANAR2 at bits [31:24] per SVD
            uint arg1 = (value >> 16) & 0xFFu;
            uint arg2 = (value >> 24) & 0xFFu;

            // Store result with BUSY (bit 8) and RBUSY (bit 9) always cleared
            panelResult = value & ~0x300u;

            switch(command)
            {
                case 0x00:
                    // No operation
                    break;

                case 0x01:
                    // Initialize list: clear the list specified by arg2
                    InitializeList((int)arg2);
                    this.DebugLog("PANCTR: Initialize list {0}", arg2);
                    break;

                case 0x02:
                    // Static allocate: move MO arg1 to list arg2
                    StaticAllocate((int)arg1, (int)arg2);
                    this.DebugLog("PANCTR: Static allocate MO{0} to list {1}", arg1, arg2);
                    break;

                default:
                    this.WarningLog("PANCTR: Unhandled command 0x{0:X2} (arg1={1}, arg2={2})",
                        command, arg1, arg2);
                    break;
            }
        }

        private void InitializeDefaultList()
        {
            // Place all 64 MOs in list 0 as a circular doubly-linked list
            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                messageObjects[i].ListNumber = 0;
                messageObjects[i].PreviousIndex = (uint)((i == 0) ? (NumberOfMessageObjects - 1) : (i - 1));
                messageObjects[i].NextIndex = (uint)((i == NumberOfMessageObjects - 1) ? 0 : (i + 1));
            }

            // List 0: BEGIN=0, END=63, SIZE=64, EMPTY=0
            listRegisters[0] = 0u              // BEGIN [7:0]
                | (63u << 8)                   // END [15:8]
                | (64u << 16)                  // SIZE [23:16]
                | (0u << 24);                  // EMPTY [24]

            // Lists 1-7: empty
            for(int i = 1; i < NumberOfLists; i++)
            {
                listRegisters[i] = (1u << 24); // EMPTY=1, everything else 0
            }
        }

        private void InitializeList(int listIndex)
        {
            if(listIndex < 0 || listIndex >= NumberOfLists)
            {
                this.WarningLog("InitializeList: invalid list index {0}", listIndex);
                return;
            }

            // Move all MOs currently in this list back to list 0
            for(int i = 0; i < NumberOfMessageObjects; i++)
            {
                if(messageObjects[i].ListNumber == listIndex)
                {
                    RemoveFromList(i);
                    AddToList(i, 0);
                }
            }

            // Mark the target list as empty
            listRegisters[listIndex] = (1u << 24);
        }

        private void StaticAllocate(int moIndex, int listIndex)
        {
            if(moIndex < 0 || moIndex >= NumberOfMessageObjects)
            {
                this.WarningLog("StaticAllocate: invalid MO index {0}", moIndex);
                return;
            }
            if(listIndex < 0 || listIndex >= NumberOfLists)
            {
                this.WarningLog("StaticAllocate: invalid list index {0}", listIndex);
                return;
            }

            if(messageObjects[moIndex].ListNumber == listIndex)
            {
                return; // Already in the target list
            }

            RemoveFromList(moIndex);
            AddToList(moIndex, listIndex);
        }

        private void RemoveFromList(int moIndex)
        {
            var mo = messageObjects[moIndex];
            int listIndex = mo.ListNumber;

            uint size = (listRegisters[listIndex] >> 16) & 0xFFu;

            if(size == 0)
            {
                return; // List is already empty
            }

            if(size == 1)
            {
                // Removing the only element
                listRegisters[listIndex] = (1u << 24); // EMPTY
            }
            else
            {
                uint begin = listRegisters[listIndex] & 0xFFu;
                uint end = (listRegisters[listIndex] >> 8) & 0xFFu;
                uint prevIdx = mo.PreviousIndex;
                uint nextIdx = mo.NextIndex;

                // Relink the chain around the removed element
                messageObjects[prevIdx].NextIndex = nextIdx;
                messageObjects[nextIdx].PreviousIndex = prevIdx;

                if(begin == (uint)moIndex)
                {
                    begin = nextIdx;
                }
                if(end == (uint)moIndex)
                {
                    end = prevIdx;
                }

                size--;
                listRegisters[listIndex] = (begin & 0xFFu)
                    | ((end & 0xFFu) << 8)
                    | ((size & 0xFFu) << 16)
                    | (size == 0 ? (1u << 24) : 0u);
            }
        }

        private void AddToList(int moIndex, int listIndex)
        {
            var mo = messageObjects[moIndex];
            mo.ListNumber = listIndex;

            uint size = (listRegisters[listIndex] >> 16) & 0xFFu;
            bool empty = (listRegisters[listIndex] & (1u << 24)) != 0;

            if(empty || size == 0)
            {
                // First element — point to self
                mo.PreviousIndex = (uint)moIndex;
                mo.NextIndex = (uint)moIndex;
                listRegisters[listIndex] = ((uint)moIndex & 0xFFu)
                    | (((uint)moIndex & 0xFFu) << 8)
                    | (1u << 16);
            }
            else
            {
                uint begin = listRegisters[listIndex] & 0xFFu;
                uint end = (listRegisters[listIndex] >> 8) & 0xFFu;

                // Append at the end of the circular doubly-linked list
                messageObjects[end].NextIndex = (uint)moIndex;
                mo.PreviousIndex = end;
                mo.NextIndex = begin;
                messageObjects[begin].PreviousIndex = (uint)moIndex;

                size++;
                listRegisters[listIndex] = (begin & 0xFFu)
                    | (((uint)moIndex & 0xFFu) << 8)
                    | ((size & 0xFFu) << 16);
            }
        }

        private uint GetMessageIndex(int srIndex)
        {
            // Return the bit position of the lowest set bit in MSPND[srIndex]
            // masked by MSIMASK. Return 0x20 if no bits are pending.
            uint pending = messagePending[srIndex] & messageIndexMask;
            if(pending == 0)
            {
                return 0x20;
            }

            for(int i = 0; i < 32; i++)
            {
                if((pending & (1u << i)) != 0)
                {
                    return (uint)i;
                }
            }

            return 0x20;
        }

        private uint clockControlRegister;
        private uint fractionalDivider;
        private readonly long peripheralClockFrequency;
        private uint moduleControl;
        private uint messageIndexMask;
        private uint panelResult;

        private readonly DoubleWordRegisterCollection registers;
        private readonly MessageObject[] messageObjects;
        private readonly NodeRegisters[] nodeRegisters;
        private readonly uint[] listRegisters;
        private readonly uint[] messagePending;
        private readonly CANNodeProxy[] nodeProxies;
        private readonly ulong[] nodeBusyUntilTicks;
        private readonly Queue<int>[] pendingTransmits;
        private readonly bool[] transmitScheduled;

        private const uint ModuleId = 0x00B0C000;
        // Used until the firmware programs FDR/NBTR: 500 kbit/s
        private const ulong DefaultBitTimeTicks = 2 * Time.TimeInterval.TicksPerMicrosecond;
        private const uint StandardFrameOverheadBits = 47;
        private const uint ExtendedFrameOverheadBits = 67;
        private const int NumberOfMessageObjects = 64;
        private const int NumberOfNodes = 3;
        private const int NumberOfLists = 8;
        private const int NumberOfServiceRequests = 8;
        private const int NumberOfInterruptLines = 8;
        private const long MessageObjectBaseOffset = 0x1000;
        private const int MessageObjectSize = 0x20;

        // MOSTAT bit definitions
        private const uint StatusRxPnd  = 1u << 0;
        private const uint StatusTxPnd  = 1u << 1;
        private const uint StatusNewDat = 1u << 3;
        private const uint StatusMsgLst = 1u << 4;
        private const uint StatusMsgVal = 1u << 5;
        private const uint StatusTxRq   = 1u << 8;
        private const uint StatusRxEn   = 1u << 7;
        private const uint StatusDir    = 1u << 11;

        // NCR bit definitions
        private const uint NcrInit  = 1u << 0;
        private const uint NcrTrie  = 1u << 1;
        private const uint NcrLecie = 1u << 2;
        private const uint NcrAlie  = 1u << 3;
        private const uint NcrCce   = 1u << 6;
        private const uint NcrLm    = 1u << 12;

        // NSR bit definitions
        private const uint NsrTxOk  = 1u << 3;
        private const uint NsrRxOk  = 1u << 4;
        private const uint NsrAlert = 1u << 5;

        /// <summary>
        /// Per-node register state for CAN nodes 0-2.
        /// </summary>
        private class NodeRegisters
        {
            public uint Control;          // NCR — reset 0x01 (INIT=1)
            public uint Status;           // NSR
            public uint InterruptPointer; // NIPR
            public uint PortControl;      // NPCR
            public uint BitTiming;        // NBTR
            public uint ErrorCount;       // NECNT
            public uint FrameCounter;     // NFCR

            public void Reset()
            {
                Control = 0x01;
                Status = 0;
                InterruptPointer = 0;
                PortControl = 0;
                BitTiming = 0;
                ErrorCount = 0;
                FrameCounter = 0;
            }
        }

        /// <summary>
        /// Represents one of 64 CAN Message Objects.
        /// </summary>
        private class MessageObject
        {
            public MessageObject(int index)
            {
                Index = index;
            }

            public void Reset()
            {
                FunctionControl = 0;
                FifoGatewayPointer = 0;
                InterruptPointer = 0;
                AcceptanceMask = 0;
                DataLow = 0;
                DataHigh = 0;
                Arbitration = 0;
                StatusFlags = 0;
                ListNumber = 0;
                PreviousIndex = 0;
                NextIndex = 0;
                MsgLostWarned = false;
            }

            /// <summary>
            /// Build the MOSTAT read value from status flags and list management fields.
            /// </summary>
            public uint ReadStatus()
            {
                return (StatusFlags & 0xFFFu)
                    | (((uint)ListNumber & 0xFu) << 12)
                    | ((PreviousIndex & 0xFFu) << 16)
                    | ((NextIndex & 0xFFu) << 24);
            }

            public readonly int Index;

            // MOFCR, MOFGPR, MOIPR, MOAMR, MODATAL, MODATAH, MOAR
            public uint FunctionControl;
            public uint FifoGatewayPointer;
            public uint InterruptPointer;
            public uint AcceptanceMask;
            public uint DataLow;
            public uint DataHigh;
            public uint Arbitration;

            // Bits [11:0] of MOSTAT — controlled via MOCTR set/reset writes
            public uint StatusFlags;

            // Managed by the panel controller, read-only in MOSTAT
            public int ListNumber;
            public uint PreviousIndex;
            public uint NextIndex;

            // Set once an unread-overwrite (MSGLST) has been logged; re-armed when
            // the firmware services the object (clears NEWDAT).  Keeps a never-drained
            // monitor object from logging a warning on every received frame.
            public bool MsgLostWarned;
        }

        /// <summary>
        /// Per-node ICAN proxy — allows connecting each CAN node to a separate CANHub.
        /// </summary>
        public class CANNodeProxy : ICAN
        {
            public CANNodeProxy(XMC4500_MultiCAN parent, int nodeIndex)
            {
                this.parent = parent;
                this.nodeIndex = nodeIndex;
            }

            public void OnFrameReceived(CANMessageFrame message)
            {
                parent.OnFrameReceivedOnNode(nodeIndex, message);
            }

            public void Reset()
            {
                // State is managed by the parent MultiCAN
            }

            internal void RaiseFrameSent(CANMessageFrame frame)
            {
                FrameSent?.Invoke(frame);
            }

            public event Action<CANMessageFrame> FrameSent;

            private readonly XMC4500_MultiCAN parent;
            private readonly int nodeIndex;
        }
    }
}
