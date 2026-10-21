//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.CAN;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;

namespace Antmicro.Renode.Peripherals.CAN
{
    /// <summary>
    /// Freescale/NXP S12 MSCAN controller (as found in MC9S12P128).
    /// Byte-addressable, 64-byte register block at offset 0x0140 from module base.
    /// Implements standard 11-bit CAN with 3 TX buffers, single RX buffer,
    /// acceptance filtering in the 32-, 16- and 8-bit modes (with IDHIT),
    /// and RX/TX interrupts.
    /// </summary>
    public class S12MSCAN : BasicBytePeripheral, IKnownSize, ICAN
    {
        public S12MSCAN(IMachine machine) : base(machine)
        {
            RxIRQ = new GPIO();
            TxIRQ = new GPIO();
            rxFifo = new Queue<ReceivedFrame>();
            DefineRegisters();
            Reset();
        }

        public override void Reset()
        {
            base.Reset();
            rxFifo.Clear();
            initMode = true;
            filterMode = 0;
            idHit = 0;
            txBufFree = 0x07; // all 3 TX buffers free
            txBufSel = 0;
            Array.Clear(rxIdr, 0, rxIdr.Length);
            Array.Clear(rxDsr, 0, rxDsr.Length);
            rxDlr = 0;
            for(int b = 0; b < 3; b++)
            {
                Array.Clear(txIdr[b], 0, txIdr[b].Length);
                Array.Clear(txDsr[b], 0, txDsr[b].Length);
                txDlr[b] = 0;
                txTbpr[b] = 0;
            }
            Array.Clear(idar, 0, idar.Length);
            Array.Clear(idmr, 0, idmr.Length);
            canctl0 = 0;
            canctl1 = 0;
            canrflg = 0;
            canrier = 0;
            cantier = 0;
            UpdateInterrupts();
        }

        public void OnFrameReceived(CANMessageFrame message)
        {
            if(initMode)
            {
                return;
            }

            if(!TryMatchFilter(message, out var hit))
            {
                this.DebugLog("RX frame ID=0x{0:X} rejected by filter", message.Id);
                return;
            }

            this.DebugLog("RX frame ID=0x{0:X} DLC={1} accepted by filter {2}", message.Id, message.Data.Length, hit);
            rxFifo.Enqueue(new ReceivedFrame(message, hit));

            if((canrflg & RXF) == 0)
            {
                DeliverNextRxFrame();
            }
        }

        public event Action<CANMessageFrame> FrameSent;

        public GPIO RxIRQ { get; }
        public GPIO TxIRQ { get; }

        public long Size => 0x40; // 64 bytes (0x0140-0x017F)

        protected override void DefineRegisters()
        {
            // All registers are handled manually via ReadByte/WriteByte
            // because the MSCAN has complex side-effects on read/write.
        }

        public override byte ReadByte(long offset)
        {
            switch(offset)
            {
            case CANCTL0_OFF: return canctl0;
            case CANCTL1_OFF:
                // INITAK (bit 0) mirrors INITRQ from CANCTL0
                byte ctl1 = canctl1;
                if(initMode) ctl1 |= 0x01; else ctl1 &= 0xFE;
                return ctl1;
            case CANBTR0_OFF: return canbtr0;
            case CANBTR1_OFF: return canbtr1;
            case CANRFLG_OFF: return canrflg;
            case CANRIER_OFF: return canrier;
            case CANTFLG_OFF: return txBufFree;
            case CANTIER_OFF: return cantier;
            case CANTARQ_OFF: return 0;
            case CANTAAK_OFF: return 0;
            case CANTBSEL_OFF: return txBufSel;
            case CANIDAC_OFF: return (byte)((filterMode << 4) | idHit); // IDAM, IDHIT
            case CANRXERR_OFF: return 0;
            case CANTXERR_OFF: return 0;

            // Acceptance filter registers
            case long n when n >= CANIDAR0_OFF && n <= CANIDMR7_OFF:
                return GetFilterRegister(offset);

            // RX buffer
            case long n when n >= CANRXIDR0_OFF && n <= CANRXIDR3_OFF:
                return rxIdr[offset - CANRXIDR0_OFF];
            case long n when n >= CANRXDSR0_OFF && n <= CANRXDSR7_OFF:
                return rxDsr[offset - CANRXDSR0_OFF];
            case CANRXDLR_OFF: return rxDlr;

            // TX buffer (selected by CANTBSEL)
            case long n when n >= CANTXIDR0_OFF && n <= CANTXIDR3_OFF:
                return txIdr[SelectedTxBuf][offset - CANTXIDR0_OFF];
            case long n when n >= CANTXDSR0_OFF && n <= CANTXDSR7_OFF:
                return txDsr[SelectedTxBuf][offset - CANTXDSR0_OFF];
            case CANTXDLR_OFF: return txDlr[SelectedTxBuf];
            case CANTXTBPR_OFF: return txTbpr[SelectedTxBuf];

            default:
                this.DebugLog("Unhandled MSCAN read at offset 0x{0:X2}", offset);
                return 0;
            }
        }

        public override void WriteByte(long offset, byte value)
        {
            switch(offset)
            {
            case CANCTL0_OFF:
                canctl0 = value;
                initMode = (value & 0x01) != 0;
                break;
            case CANCTL1_OFF:
                canctl1 = value;
                break;
            case CANBTR0_OFF:
                canbtr0 = value;
                break;
            case CANBTR1_OFF:
                canbtr1 = value;
                break;
            case CANRFLG_OFF:
                // Write 1 to clear flags
                canrflg &= (byte)~value;
                if((value & RXF) != 0 && rxFifo.Count > 0)
                {
                    DeliverNextRxFrame();
                }
                UpdateInterrupts();
                break;
            case CANRIER_OFF:
                canrier = value;
                UpdateInterrupts();
                break;
            case CANTFLG_OFF:
                // Writing 1 to a bit where txBufFree is 1 triggers transmission
                byte bitsToTx = (byte)(value & txBufFree & 0x07);
                if(bitsToTx != 0)
                {
                    TransmitFromBuffer(bitsToTx);
                }
                break;
            case CANTIER_OFF:
                cantier = value;
                UpdateInterrupts();
                break;
            case CANTBSEL_OFF:
                // Firmware writes a mask; hardware selects lowest-numbered free buffer
                txBufSel = SelectLowestBit((byte)(value & txBufFree & 0x07));
                break;
            case CANIDAC_OFF:
                filterMode = (byte)((value >> 4) & 0x3); // IDHIT is read-only
                break;

            // Acceptance filter registers
            case long n when n >= CANIDAR0_OFF && n <= CANIDMR7_OFF:
                SetFilterRegister(offset, value);
                break;

            // TX buffer (selected by CANTBSEL)
            case long n when n >= CANTXIDR0_OFF && n <= CANTXIDR3_OFF:
                txIdr[SelectedTxBuf][offset - CANTXIDR0_OFF] = value;
                break;
            case long n when n >= CANTXDSR0_OFF && n <= CANTXDSR7_OFF:
                txDsr[SelectedTxBuf][offset - CANTXDSR0_OFF] = value;
                break;
            case CANTXDLR_OFF:
                txDlr[SelectedTxBuf] = value;
                break;
            case CANTXTBPR_OFF:
                txTbpr[SelectedTxBuf] = value;
                break;

            default:
                this.DebugLog("Unhandled MSCAN write at offset 0x{0:X2} value 0x{1:X2}", offset, value);
                break;
            }
        }

        private void TransmitFromBuffer(byte bufMask)
        {
            for(int buf = 0; buf < 3; buf++)
            {
                if((bufMask & (1 << buf)) == 0)
                {
                    continue;
                }

                // Decode CAN ID from TXIDR0/1 (standard 11-bit)
                uint id = (uint)(txIdr[buf][0] << 3) | (uint)(txIdr[buf][1] >> 5);
                int dlc = txDlr[buf] & 0x0F;
                if(dlc > 8) dlc = 8;

                var data = new byte[dlc];
                Array.Copy(txDsr[buf], data, dlc);

                this.Log(LogLevel.Debug, "TX buf{0}: ID=0x{1:X3} DLC={2} data=[{3}]",
                    buf, id, dlc, BitConverter.ToString(data));

                var frame = new CANMessageFrame(id, data);
                FrameSent?.Invoke(frame);

                // Mark buffer as free again
                txBufFree |= (byte)(1 << buf);
            }
            UpdateInterrupts();
        }

        private void DeliverNextRxFrame()
        {
            if(rxFifo.Count == 0)
            {
                return;
            }

            var received = rxFifo.Dequeue();
            var frame = received.Frame;
            idHit = received.FilterHit;
            uint id = frame.Id;
            int dlc = Math.Min(frame.Data.Length, 8);

            // Encode ID into RXIDR0/1 (standard 11-bit format)
            rxIdr[0] = (byte)(id >> 3);
            rxIdr[1] = (byte)((id & 0x07) << 5);
            rxIdr[2] = 0;
            rxIdr[3] = 0;

            Array.Clear(rxDsr, 0, rxDsr.Length);
            Array.Copy(frame.Data, rxDsr, dlc);
            rxDlr = (byte)dlc;

            canrflg |= RXF; // Set RX full flag
            UpdateInterrupts();
        }

        // Acceptance filtering of a standard frame (IDR0 = ID[10:3],
        // IDR1 = ID[2:0] << 5 with RTR and IDE clear) in the mode CANIDAC.IDAM
        // selects. A filter matches when every bit its mask (IDMR) leaves
        // clear equals the acceptance code (IDAR). For standard identifiers:
        //   00: two 32-bit filters, IDAR0-1/IDAR4-5 against IDR0-1
        //   01: four 16-bit filters, IDAR(2k)-(2k+1) against IDR0-1
        //   10: eight 8-bit filters, IDARk against IDR0
        //   11: closed, nothing is accepted
        // The number of the first matching filter becomes IDHIT.
        private bool TryMatchFilter(CANMessageFrame message, out int hit)
        {
            var rxIdr0 = (byte)(message.Id >> 3);
            var rxIdr1 = (byte)((message.Id & 0x07) << 5);

            switch(filterMode)
            {
            case 0:
                for(hit = 0; hit < 2; hit++)
                {
                    if(FilterMatches(rxIdr0, hit * 4) && FilterMatches(rxIdr1, hit * 4 + 1))
                    {
                        return true;
                    }
                }
                break;
            case 1:
                for(hit = 0; hit < 4; hit++)
                {
                    if(FilterMatches(rxIdr0, hit * 2) && FilterMatches(rxIdr1, hit * 2 + 1))
                    {
                        return true;
                    }
                }
                break;
            case 2:
                for(hit = 0; hit < 8; hit++)
                {
                    if(FilterMatches(rxIdr0, hit))
                    {
                        return true;
                    }
                }
                break;
            }
            hit = 0;
            return false;
        }

        private bool FilterMatches(byte idr, int filterRegister)
        {
            return ((idr ^ idar[filterRegister]) & ~idmr[filterRegister]) == 0;
        }

        // 0x10-0x13 IDAR0-3, 0x14-0x17 IDMR0-3, 0x18-0x1B IDAR4-7, 0x1C-0x1F IDMR4-7
        private byte GetFilterRegister(long offset)
        {
            var idx = offset - CANIDAR0_OFF;
            var bankIndex = (idx / 8) * 4 + idx % 4;
            return (idx % 8) < 4 ? idar[bankIndex] : idmr[bankIndex];
        }

        private void SetFilterRegister(long offset, byte value)
        {
            var idx = offset - CANIDAR0_OFF;
            var bankIndex = (idx / 8) * 4 + idx % 4;
            if((idx % 8) < 4)
            {
                idar[bankIndex] = value;
            }
            else
            {
                idmr[bankIndex] = value;
            }
        }

        private void UpdateInterrupts()
        {
            bool rxIrq = (canrflg & canrier & RXF) != 0;
            bool txIrq = (txBufFree & cantier & 0x07) != 0;
            RxIRQ.Set(rxIrq);
            TxIRQ.Set(txIrq);
        }

        private int SelectedTxBuf
        {
            get
            {
                if((txBufSel & 0x01) != 0) return 0;
                if((txBufSel & 0x02) != 0) return 1;
                if((txBufSel & 0x04) != 0) return 2;
                return 0;
            }
        }

        private static byte SelectLowestBit(byte val)
        {
            if(val == 0) return 0;
            return (byte)(val & (~val + 1)); // isolate lowest set bit
        }

        // State
        private bool initMode;
        private byte canctl0, canctl1, canbtr0, canbtr1;
        private byte canrflg, canrier;
        private byte txBufFree; // CANTFLG: bit per free buffer
        private byte cantier;
        private byte txBufSel; // CANTBSEL: selected TX buffer
        private byte filterMode; // CANIDAC.IDAM
        private byte idHit;      // CANIDAC.IDHIT of the frame in the RX buffer

        private readonly byte[] rxIdr = new byte[4];
        private readonly byte[] rxDsr = new byte[8];
        private byte rxDlr;

        private readonly byte[][] txIdr = new byte[3][] { new byte[4], new byte[4], new byte[4] };
        private readonly byte[][] txDsr = new byte[3][] { new byte[8], new byte[8], new byte[8] };
        private readonly byte[] txDlr = new byte[3];
        private readonly byte[] txTbpr = new byte[3];

        private readonly byte[] idar = new byte[8]; // acceptance registers (2 banks of 4)
        private readonly byte[] idmr = new byte[8]; // mask registers (2 banks of 4)

        private readonly Queue<ReceivedFrame> rxFifo;

        private struct ReceivedFrame
        {
            public ReceivedFrame(CANMessageFrame frame, int filterHit)
            {
                Frame = frame;
                FilterHit = (byte)filterHit;
            }

            public CANMessageFrame Frame { get; }
            public byte FilterHit { get; }
        }

        // Register offsets (relative to module base, 0x0140 on MC9S12P128)
        private const long CANCTL0_OFF  = 0x00;
        private const long CANCTL1_OFF  = 0x01;
        private const long CANBTR0_OFF  = 0x02;
        private const long CANBTR1_OFF  = 0x03;
        private const long CANRFLG_OFF  = 0x04;
        private const long CANRIER_OFF  = 0x05;
        private const long CANTFLG_OFF  = 0x06;
        private const long CANTIER_OFF  = 0x07;
        private const long CANTARQ_OFF  = 0x08;
        private const long CANTAAK_OFF  = 0x09;
        private const long CANTBSEL_OFF = 0x0A;
        private const long CANIDAC_OFF  = 0x0B;
        private const long CANRXERR_OFF = 0x0E;
        private const long CANTXERR_OFF = 0x0F;
        private const long CANIDAR0_OFF = 0x10; // 0x10-0x13
        private const long CANIDMR0_OFF = 0x14; // 0x14-0x17
        private const long CANIDAR4_OFF = 0x18; // 0x18-0x1B
        private const long CANIDMR4_OFF = 0x1C; // 0x1C-0x1F
        private const long CANIDMR7_OFF = 0x1F;
        private const long CANRXIDR0_OFF = 0x20;
        private const long CANRXIDR3_OFF = 0x23;
        private const long CANRXDSR0_OFF = 0x24;
        private const long CANRXDSR7_OFF = 0x2B;
        private const long CANRXDLR_OFF  = 0x2C;
        private const long CANTXIDR0_OFF = 0x30;
        private const long CANTXIDR3_OFF = 0x33;
        private const long CANTXDSR0_OFF = 0x34;
        private const long CANTXDSR7_OFF = 0x3B;
        private const long CANTXDLR_OFF  = 0x3C;
        private const long CANTXTBPR_OFF = 0x3D;

        private const byte RXF = 0x01; // Receive buffer full flag
    }
}
