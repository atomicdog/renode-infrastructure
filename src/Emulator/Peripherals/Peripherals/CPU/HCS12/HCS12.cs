//
// Copyright (c) 2010-2026 Antmicro
//
// This file is licensed under the MIT License.
// Full license text is available in 'licenses/MIT.txt'.
//
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Exceptions;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Memory;
using Antmicro.Renode.Utilities;

using ELFSharp.ELF;

namespace Antmicro.Renode.Peripherals.CPU
{
    // HCS12 (CPU12) pure C# interpretive CPU for Renode.
    // Follows the MSP430X pattern: inherits BaseCPU, no tlib/TCG.
    // Reference: Motorola CPU12 Reference Manual (CPU12RM/AD)
    // Target: MC9S12P-family (big-endian, 16-bit, paged flash)
    public class HCS12 : BaseCPU, IGPIOReceiver, ICpuSupportingGdb
    {
        public HCS12(IMachine machine, string cpuType = "hcs12")
            : base(0, cpuType, machine, Endianess.BigEndian)
        {
        }

        public override void Reset()
        {
            base.Reset();

            regA = 0;
            regB = 0;
            regX = 0;
            regY = 0;
            regSP = 0;
            ccr = 0;
            ppage = PPAGEResetValue;
            executedInstructions = 0;

            hooks.Clear();
            pendingInterrupt.Clear();
            Array.Clear(ioRegs, 0, ioRegs.Length);
            timerEnabled = false;
            timerCounter = 0;
            timerPrescaler = 1;
            timerPrescaleCount = 0;

            // Set I bit (mask interrupts) on reset
            ccr |= CCR_I;

            // Port S (PTS, 0x0258): SCI RX pins idle HIGH (mark state).
            // Bits 0,2 = SCI0/SCI1 RXD. Low = break/active; High = idle.
            ioRegs[0x258] = 0x05;

            // Read reset vector from 0xFFFE
            var resetVector = ReadWord(ResetVectorAddress);
            PC = resetVector;

            this.Log(LogLevel.Info, "HCS12 reset: PC=0x{0:X4}, PPAGE=0x{1:X2}", (uint)PC, ppage);
        }

        public void OnGPIO(int number, bool value)
        {
            if(value)
            {
                pendingInterrupt.Add(number);
            }
            else
            {
                pendingInterrupt.Remove(number);
            }
        }

        public void AddHookAtInterruptBegin(Action<ulong> hook)
        {
            throw new RecoverableException("Not implemented");
        }

        public void AddHookAtInterruptEnd(Action<ulong> hook)
        {
            throw new RecoverableException("Not implemented");
        }

        public void AddHookAtWfiStateChange(Action<bool> hook)
        {
            throw new RecoverableException("Not implemented");
        }

        public void AddHook(ulong addr, CpuAddressHook hook)
        {
            if(!hooks.ContainsKey(addr))
            {
                hooks.Add(addr, new HashSet<CpuAddressHook>());
            }
            hooks[addr].Add(hook);
        }

        public void RemoveHook(ulong addr, CpuAddressHook hook)
        {
            if(hooks.ContainsKey(addr))
            {
                hooks[addr].Remove(hook);
            }
        }

        public void RemoveHooksAt(ulong addr)
        {
            if(hooks.ContainsKey(addr))
            {
                hooks[addr].Clear();
            }
        }

        public void RemoveAllHooks()
        {
            hooks.Clear();
        }

        public void RemoveHooks(CpuAddressHook hookToRemove)
        {
            foreach(var hook in hooks.Values.Where(x => x.Contains(hookToRemove)))
            {
                hook.Remove(hookToRemove);
            }
        }

        public void SetRegister(int register, RegisterValue value)
        {
            SetRegisterValue((Registers)register, (ushort)value.RawValue);
        }

        public void SetRegisterUnsafe(int register, RegisterValue value)
        {
            SetRegister(register, value);
        }

        public RegisterValue GetRegister(int register)
        {
            return RegisterValue.Create(GetRegisterValue((Registers)register), 16);
        }

        public RegisterValue GetRegisterUnsafe(int register)
        {
            return GetRegister(register);
        }

        public IEnumerable<CPURegister> GetRegisters()
        {
            return new[]
            {
                new CPURegister(0, 16, isGeneral: false, isReadonly: false, aliases: new[] { "D" }),
                new CPURegister(1, 16, isGeneral: true, isReadonly: false, aliases: new[] { "X" }),
                new CPURegister(2, 16, isGeneral: true, isReadonly: false, aliases: new[] { "Y" }),
                new CPURegister(3, 16, isGeneral: false, isReadonly: false, aliases: new[] { "SP" }),
                new CPURegister(4, 16, isGeneral: false, isReadonly: false, aliases: new[] { "PC" }),
                new CPURegister(5, 8, isGeneral: false, isReadonly: false, aliases: new[] { "CCR" }),
            };
        }

        public IEnumerable<CPURegister> GetAllRegisters()
        {
            return GetRegisters();
        }

        public void EnterSingleStepModeSafely(HaltArguments args)
        {
            ExecutionMode = ExecutionMode.SingleStep;
        }

        public override ExecutionResult ExecuteInstructions(ulong numberOfInstructionsToExecute, out ulong numberOfExecutedInstructions)
        {
            numberOfExecutedInstructions = 0;
            while(numberOfInstructionsToExecute-- > 0)
            {
                var result = ExecuteNextInstruction();
                if(result == ExecutionResult.Aborted)
                {
                    this.Log(LogLevel.Error, "Execution aborted at PC=0x{0:X4}", (uint)PC);
                    this.Log(LogLevel.Debug, "Register dump: {0}", DumpRegisters());
                    return ExecutionResult.Aborted;
                }

                numberOfExecutedInstructions++;
                executedInstructions++;

                OnInstructionExecuted();

                // Advance free-running timer counter and check output compares
                TickTimer();

                // Check interrupts: if I bit is clear, handle pending
                if((ccr & CCR_I) == 0 && pendingInterrupt.Count > 0)
                {
                    HandleInterrupt(pendingInterrupt.Min);
                }

                if(hooks.ContainsKey(PC.RawValue))
                {
                    return ExecutionResult.StoppedAtBreakpoint;
                }
            }

            return ExecutionResult.Ok;
        }

        public string DumpRegisters()
        {
            return $"A=0x{regA:X2} B=0x{regB:X2} D=0x{RegD:X4} X=0x{regX:X4} Y=0x{regY:X4} SP=0x{regSP:X4} PC=0x{(uint)PC:X4} CCR=0x{ccr:X2} [{CCRString}] PPAGE=0x{ppage:X2}";
        }

        public byte ReadIO(int address)
        {
            if(address >= 0 && address < 0x400)
                return ioRegs[address];
            return 0;
        }

        public virtual string DumpTimer()
        {
            return $"TEN={timerEnabled} TIOS=0x{ioRegs[0x40]:X2} TIE=0x{ioRegs[0x4C]:X2} TFLG1=0x{ioRegs[0x4E]:X2} " +
                   $"TSCR2=0x{ioRegs[0x4D]:X2} prescaler={timerPrescaler} counter={timerCounter} " +
                   $"TC0=0x{(ioRegs[0x50] << 8 | ioRegs[0x51]):X4} TC7=0x{(ioRegs[0x5E] << 8 | ioRegs[0x5F]):X4} " +
                   $"pending=[{string.Join(",", pendingInterrupt)}]";
        }

        public override string Architecture => "hcs12";

        public override RegisterValue PC { get; set; }

        public override ulong ExecutedInstructions => executedInstructions;

        public string GDBArchitecture => "m68hc12";

        public List<GDBFeatureDescriptor> GDBFeatures => new List<GDBFeatureDescriptor>();

        public event Action<int> InterruptAcknowledged;

        // PPAGE register -- exposed for the MMC peripheral or direct access
        public byte PPAGE
        {
            get => ppage;
            set => ppage = (byte)(value & 0x0F);
        }

        // ATD input for channel 0-15, as the 16-bit left-justified result
        // (e.g. 0x8000 = half scale). A conversion sequence copies the
        // values of the sequence's channels into ATDDR0-ATDDR7.
        public void SetAnalogValue(int channel, ushort value)
        {
            if(channel < 0 || channel >= NumberOfAnalogChannels)
            {
                this.Log(LogLevel.Warning, "Invalid ATD channel {0}", channel);
                return;
            }
            analogValues[channel] = value;
        }

        // P-Flash is loaded via LoadSRecord into sysbus at global addresses 0x020000-0x03FFFF.
        // The CPU translates local addresses to global addresses for flash reads.

        protected override bool ExecutionFinished(ExecutionResult result)
        {
            if(result == ExecutionResult.StoppedAtBreakpoint)
            {
                foreach(var hook in hooks[PC.RawValue])
                {
                    hook(this, PC.RawValue);
                }
                return true;
            }
            return false;
        }

        // Extension points for subclasses (for example firmware-specific
        // bring-up workarounds kept out of the generic model).

        // Called after every instruction, before the timer advances.
        protected virtual void OnInstructionExecuted()
        {
        }

        // Called before an instruction executes; return Aborted to stop.
        protected virtual ExecutionResult BeforeInstruction(ushort pc, byte opcode)
        {
            return ExecutionResult.Ok;
        }

        // Lets a subclass supply the value of a data or instruction read
        // before the normal memory map is consulted.
        protected virtual bool TryInterceptRead(ushort address, out byte value)
        {
            value = 0;
            return false;
        }

        protected bool TimerEnabled => timerEnabled;

        #region Register Access

        private ushort RegD
        {
            get => (ushort)((regA << 8) | regB);
            set
            {
                regA = (byte)(value >> 8);
                regB = (byte)(value & 0xFF);
            }
        }

        private ushort GetRegisterValue(Registers register)
        {
            switch(register)
            {
            case Registers.D: return RegD;
            case Registers.X: return regX;
            case Registers.Y: return regY;
            case Registers.SP: return regSP;
            case Registers.PC: return (ushort)(uint)PC;
            case Registers.CCR: return ccr;
            default: throw new Exception($"Invalid register {register}");
            }
        }

        private void SetRegisterValue(Registers register, ushort value)
        {
            switch(register)
            {
            case Registers.D: RegD = value; break;
            case Registers.X: regX = value; break;
            case Registers.Y: regY = value; break;
            case Registers.SP: regSP = value; break;
            case Registers.PC: PC = value; break;
            case Registers.CCR: ccr = (byte)(value & 0xFF); break;
            default: throw new Exception($"Invalid register {register}");
            }
        }

        private string CCRString
        {
            get
            {
                var sb = new StringBuilder(8);
                sb.Append((ccr & CCR_S) != 0 ? 'S' : '-');
                sb.Append((ccr & CCR_X) != 0 ? 'X' : '-');
                sb.Append((ccr & CCR_H) != 0 ? 'H' : '-');
                sb.Append((ccr & CCR_I) != 0 ? 'I' : '-');
                sb.Append((ccr & CCR_N) != 0 ? 'N' : '-');
                sb.Append((ccr & CCR_Z) != 0 ? 'Z' : '-');
                sb.Append((ccr & CCR_V) != 0 ? 'V' : '-');
                sb.Append((ccr & CCR_C) != 0 ? 'C' : '-');
                return sb.ToString();
            }
        }

        #endregion

        #region Memory Access

        protected byte ReadByte(ushort address)
        {
            if(TryInterceptRead(address, out var intercepted))
            {
                return intercepted;
            }

            // I/O register intercepts (0x0000-0x03FF), except ranges
            // delegated to sysbus peripherals (e.g., MSCAN at 0x0140-0x017F)
            if(address < 0x0400 && !IsDelegatedIO(address))
            {
                return ReadIORegister(address);
            }

            // Paged flash window: 0x8000-0xBFFF -> use PPAGE to select 16KB page
            // Read from the global P-Flash region mapped at 0x020000+ in sysbus
            if(address >= PagedWindowStart && address <= PagedWindowEnd)
            {
                var globalAddr = ((uint)ppage << 14) | (uint)(address & 0x3FFF);
                return machine.SystemBus.ReadByte(globalAddr);
            }

            // Unpaged flash: 0x4000-0x7FFF -> fixed page 0x0D
            if(address >= 0x4000 && address <= 0x7FFF)
            {
                var globalAddr = (0x0Du << 14) | (uint)(address & 0x3FFF);
                return machine.SystemBus.ReadByte(globalAddr);
            }

            // Unpaged flash: 0xC000-0xFFFF -> fixed page 0x0F (includes vectors)
            if(address >= 0xC000)
            {
                var globalAddr = (0x0Fu << 14) | (uint)(address & 0x3FFF);
                return machine.SystemBus.ReadByte(globalAddr);
            }

            return machine.SystemBus.ReadByte(address);
        }

        private ushort ReadWord(ushort address)
        {
            // Big-endian: high byte first
            var hi = ReadByte(address);
            var lo = ReadByte((ushort)(address + 1));
            return (ushort)((hi << 8) | lo);
        }

        private void WriteByte(ushort address, byte value)
        {
            // I/O register intercepts (0x0000-0x03FF), except delegated ranges
            if(address < 0x0400 && !IsDelegatedIO(address))
            {
                WriteIORegister(address, value);
                return;
            }

            machine.SystemBus.WriteByte(address, value);
        }

        private void WriteWord(ushort address, ushort value)
        {
            // Big-endian: high byte first
            WriteByte(address, (byte)(value >> 8));
            WriteByte((ushort)(address + 1), (byte)(value & 0xFF));
        }

        private byte FetchByte()
        {
            var val = ReadByte((ushort)(uint)PC);
            PC = (ushort)((uint)PC + 1);
            return val;
        }

        private ushort FetchWord()
        {
            var val = ReadWord((ushort)(uint)PC);
            PC = (ushort)((uint)PC + 2);
            return val;
        }

        // Push/pop for stack operations (pre-decrement push, post-increment pull)
        protected void PushByte(byte value)
        {
            regSP--;
            WriteByte(regSP, value);
        }

        protected void PushWord(ushort value)
        {
            regSP -= 2;
            WriteWord(regSP, value);
        }

        private byte PullByte()
        {
            var val = ReadByte(regSP);
            regSP++;
            return val;
        }

        private ushort PullWord()
        {
            var val = ReadWord(regSP);
            regSP += 2;
            return val;
        }

        #endregion

        #region I/O Registers (0x0000-0x03FF)

        // Ranges delegated to sysbus peripherals instead of internal I/O register handler
        private static bool IsDelegatedIO(ushort address)
        {
            // MSCAN: 0x0140-0x017F → S12MSCAN peripheral on sysbus
            return address >= 0x0140 && address <= 0x017F;
        }

        // Minimal I/O register emulation for boot.
        // Registers not explicitly handled fall through to sysbus (ArrayMemory at 0x0000).

        protected virtual byte ReadIORegister(ushort address)
        {
            switch(address)
            {
            // ---- MMC ----
            case 0x000B: return ioRegs[0x0B]; // MODE
            case 0x0011: return ioRegs[0x11]; // DIRECT
            case 0x0030: return ppage;         // PPAGE

            // ---- S12CPMU (Clock, Reset, Power Management) ----
            case 0x0034: return ioRegs[0x34]; // CPMUSYNR
            case 0x0035: return ioRegs[0x35]; // CPMUREFDIV
            case 0x0036: return ioRegs[0x36]; // CPMUPOSTDIV
            case 0x0037: // CPMUFLG - flags register
                // Always report: LOCK=1 (bit 3), UPOSC=1 (bit 0), PORF=1 (bit 6)
                return (byte)(ioRegs[0x37] | 0x49); // PORF|LOCK|UPOSC
            case 0x0038: return ioRegs[0x38]; // CPMUINT
            case 0x0039: return ioRegs[0x39]; // CPMUCLKS
            case 0x003A: return ioRegs[0x3A]; // CPMUPLL
            case 0x003B: return ioRegs[0x3B]; // CPMURTI
            case 0x003C: return ioRegs[0x3C]; // CPMUCOP (RTICTL)
            case 0x003F: return 0x00;          // CPMUARMCOP (write-only)

            // ---- Flash (S12FTMRC, S12P register map) ----
            case 0x0100: return ioRegs[0x100]; // FCLKDIV
            case 0x0101: return ioRegs[0x101]; // FSEC
            case 0x0105: return (byte)(ioRegs[0x105] | 0x80); // FERCNFG/legacy: CCIF=1
            case 0x0106: return (byte)(ioRegs[0x106] | 0x80); // FSTAT: CCIF=1 (flash ready)

            // MSCAN (0x0140-0x017F) delegated to S12MSCAN peripheral via sysbus

            // ---- SCI (0x00C8-0x00CF) ----
            case 0x00CC: return 0xC0; // SCISR1: TDRE=1, TC=1 (TX ready)
            case 0x00CE: return 0x00; // SCISR2

            // ---- Timer (TIM, 0x0040-0x006F) ----
            case 0x0044: return (byte)(timerCounter >> 8);   // TCNT high
            case 0x0045: return (byte)(timerCounter & 0xFF); // TCNT low
            case 0x004C: return ioRegs[0x4C]; // TIE - interrupt enable
            case 0x004E: return ioRegs[0x4E]; // TFLG1 - interrupt flags
            case 0x004F: return ioRegs[0x4F]; // TFLG2

            default:
                // Fall through to backing memory
                if(address < 0x0400)
                {
                    return ioRegs[address];
                }
                return machine.SystemBus.ReadByte(address);
            }
        }

        protected virtual void WriteIORegister(ushort address, byte value)
        {
            switch(address)
            {
            // ---- MMC ----
            case 0x000B: ioRegs[0x0B] = value; break; // MODE
            case 0x0011: ioRegs[0x11] = value; break; // DIRECT
            case 0x0030: ppage = (byte)(value & 0x0F); break; // PPAGE

            // ---- S12CPMU ----
            case 0x0034: ioRegs[0x34] = value; break; // CPMUSYNR
            case 0x0035: ioRegs[0x35] = value; break; // CPMUREFDIV
            case 0x0036: ioRegs[0x36] = value; break; // CPMUPOSTDIV
            case 0x0037: // CPMUFLG - write 1 to clear flag bits
                ioRegs[0x37] &= (byte)~(value & 0xD6); // clear flagged bits (not LOCK/UPOSC which are status)
                break;
            case 0x0038: ioRegs[0x38] = value; break; // CPMUINT
            case 0x0039: ioRegs[0x39] = value; break; // CPMUCLKS
            case 0x003A: ioRegs[0x3A] = value; break; // CPMUPLL
            case 0x003B: ioRegs[0x3B] = value; break; // CPMURTI
            case 0x003C: ioRegs[0x3C] = value; break; // CPMUCOP (COP rate control)
            case 0x003F: // CPMUARMCOP - COP watchdog kick (0x55, 0xAA sequence)
                copSequence = (copSequence == 0x55 && value == 0xAA) ? (byte)0 : value;
                break;

            // ---- ATD (Analog-to-Digital, 0x0070-0x008F) ----
            case 0x0075: // ATDCTL5 - writing starts a conversion sequence
                ioRegs[0x75] = value;
                // Complete the sequence at once: fill the results from the
                // channel values (CD:CA = first channel; MULT = consecutive
                // channels, otherwise all from the first) and set SCF.
                var firstChannel = value & 0x0F;
                var multiChannel = (value & 0x10) != 0;
                for(int i = 0; i < NumberOfAnalogResults; i++)
                {
                    var channel = multiChannel ? (firstChannel + i) % NumberOfAnalogChannels : firstChannel;
                    ioRegs[0x80 + i * 2] = (byte)(analogValues[channel] >> 8); // ATDDRxH
                    ioRegs[0x80 + i * 2 + 1] = (byte)analogValues[channel];    // ATDDRxL
                }
                ioRegs[0x76] |= 0x80;
                break;

            // ---- Flash ----
            case 0x0100: ioRegs[0x100] = value; break; // FCLKDIV
            case 0x0105: ioRegs[0x105] = value; break; // FSTAT
            case 0x0106: ioRegs[0x106] = value; break; // FCMD

            // ---- Timer ----
            case 0x0046: // TSCR1 - bit 7 = TEN (timer enable)
                ioRegs[0x46] = value;
                timerEnabled = (value & 0x80) != 0;
                break;
            case 0x004C: // TIE - timer interrupt enable
                ioRegs[0x4C] = value;
                break;
            case 0x004D: // TSCR2 - prescaler (bits 2:0)
                ioRegs[0x4D] = value;
                timerPrescaler = (ushort)(1 << (value & 0x07));
                break;
            case 0x004E: // TFLG1 - write 1 to clear channel flags
                ioRegs[0x4E] &= (byte)~value;
                break;
            case 0x004F: // TFLG2 - write 1 to clear overflow flag
                ioRegs[0x4F] &= (byte)~value;
                break;

            // MSCAN (0x0140-0x017F) delegated to S12MSCAN peripheral via sysbus

            default:
                // Store in backing array for all other I/O addresses
                if(address < 0x0400)
                {
                    ioRegs[address] = value;
                }
                else
                {
                    machine.SystemBus.WriteByte(address, value);
                }
                break;
            }
        }

        #endregion

        #region Interrupt Handling

        private void HandleInterrupt(int interruptNumber)
        {
            // Stack: RTN_hi, RTN_lo, Y_hi, Y_lo, X_hi, X_lo, A, B, CCR
            PushWord((ushort)(uint)PC);
            PushWord(regY);
            PushWord(regX);
            PushByte(regA);
            PushByte(regB);
            PushByte(ccr);

            // Set I bit to mask further interrupts
            ccr |= CCR_I;

            // Vector table at 0xFF80-0xFFFF (64 vectors, 2 bytes each)
            // Vector number 0 = reset at 0xFFFE, number 1 = COP at 0xFFFC, etc.
            var vectorAddress = (ushort)(0xFFFE - interruptNumber * 2);
            var handlerAddress = ReadWord(vectorAddress);

            PC = handlerAddress;
            pendingInterrupt.Remove(interruptNumber);

            InterruptAcknowledged?.Invoke(interruptNumber);
        }

        #endregion

        #region Instruction Execution

        public bool TraceEnabled { get; set; }

        private ExecutionResult ExecuteNextInstruction()
        {
            var pc = (ushort)(uint)PC;
            var opcode = FetchByte();

            RecordTrace(pc, opcode);

            if(BeforeInstruction(pc, opcode) == ExecutionResult.Aborted)
            {
                return ExecutionResult.Aborted;
            }

            if(TraceEnabled)
            {
                this.Log(LogLevel.Info, "TRACE: PC=0x{0:X4} OP=0x{1:X2} A={2:X2} B={3:X2} X={4:X4} Y={5:X4} SP={6:X4} CCR={7:X2} PP={8:X1}",
                    pc, opcode, regA, regB, regX, regY, regSP, ccr, ppage);
            }

            // Page 2 prefix (0x18) - extended instructions
            if(opcode == 0x18)
            {
                return ExecutePage2Instruction();
            }

            // Loop primitive prefix (0x04)
            if(opcode == 0x04)
            {
                return ExecuteLoopPrimitive();
            }

            switch(opcode)
            {
            // ========== Row 0x: Misc / Branches ==========
            case 0x00: return Op_BGND();
            case 0x01: return Op_MEM();
            case 0x02: return Op_INY();
            case 0x03: return Op_DEY();
            // 0x04: loop primitives (handled above)
            case 0x05: return Op_JMP(AddrMode.IDX);
            case 0x06: return Op_JMP(AddrMode.EXT);
            case 0x07: return Op_BSR();
            case 0x08: return Op_INX();
            case 0x09: return Op_DEX();
            case 0x0A: return Op_RTC();
            case 0x0B: return Op_RTI();
            case 0x0C: return Op_BSET(AddrMode.DIR);
            case 0x0D: return Op_BCLR(AddrMode.DIR);
            case 0x0E: return Op_BRSET(AddrMode.DIR);
            case 0x0F: return Op_BRCLR(AddrMode.DIR);

            // ========== Row 1x: Misc ==========
            case 0x10: return Op_ANDCC();      // ANDCC = CLC/CLI equivalent
            case 0x11: return Op_EDIV();
            case 0x12: return Op_MUL();
            case 0x13: return Op_EMUL();
            case 0x14: return Op_ORCC();       // ORCC = SEC/SEI equivalent
            case 0x15: return Op_JSR(AddrMode.IDX);
            case 0x16: return Op_JSR(AddrMode.EXT);
            case 0x17: return Op_JSR(AddrMode.DIR);
            // 0x18: page 2 (handled above)
            case 0x19: return Op_LEAY();
            case 0x1A: return Op_LEAX();
            case 0x1B: return Op_LEAS();
            // S12CPUV2: 0x1C-0x1F are EXT (16-bit address), NOT IDX as in original CPU12
            case 0x1C: return Op_BSET(AddrMode.EXT);
            case 0x1D: return Op_BCLR(AddrMode.EXT);
            case 0x1E: return Op_BRSET(AddrMode.EXT);
            case 0x1F: return Op_BRCLR(AddrMode.EXT);

            // ========== Row 2x: Short branches ==========
            case 0x20: return Op_BRA();
            case 0x21: return Op_BRN();
            case 0x22: return Op_BHI();
            case 0x23: return Op_BLS();
            case 0x24: return Op_BCC();
            case 0x25: return Op_BCS();
            case 0x26: return Op_BNE();
            case 0x27: return Op_BEQ();
            case 0x28: return Op_BVC();
            case 0x29: return Op_BVS();
            case 0x2A: return Op_BPL();
            case 0x2B: return Op_BMI();
            case 0x2C: return Op_BGE();
            case 0x2D: return Op_BLT();
            case 0x2E: return Op_BGT();
            case 0x2F: return Op_BLE();

            // ========== Row 3x: Inherent / PSHX/PULX etc ==========
            case 0x30: return Op_PULX();
            case 0x31: return Op_PULY();
            case 0x32: return Op_PULA();
            case 0x33: return Op_PULB();
            case 0x34: return Op_PSHX();
            case 0x35: return Op_PSHY();
            case 0x36: return Op_PSHA();
            case 0x37: return Op_PSHB();
            case 0x38: return Op_PULC();
            case 0x39: return Op_PSHC();
            case 0x3A: return Op_PULD();
            case 0x3B: return Op_PSHD();
            case 0x3C: return Op_WAI();
            case 0x3D: return Op_RTS();
            case 0x3E: return Op_SWI();
            case 0x3F: return Op_STOP();

            // ========== Row 4x: Inherent A/B operations ==========
            case 0x40: return Op_NEGA();
            case 0x41: return Op_COMA();
            case 0x42: return Op_INCA();
            case 0x43: return Op_DECA();
            case 0x44: return Op_LSRA();
            case 0x45: return Op_ROLA();
            case 0x46: return Op_RORA();
            case 0x47: return Op_ASRA();
            case 0x48: return Op_ASLA();  // = LSLA
            case 0x49: return Op_LSRD();
            case 0x4A: return Op_CALL(AddrMode.EXT);
            case 0x4B: return Op_CALL(AddrMode.IDX);
            // S12CPUV2 alternate encodings: same behavior as 0x0C-0x0F (DIR)
            case 0x4C: return Op_BSET(AddrMode.DIR);
            case 0x4D: return Op_BCLR(AddrMode.DIR);
            case 0x4E: return Op_BRSET(AddrMode.DIR);
            case 0x4F: return Op_BRCLR(AddrMode.DIR);

            case 0x50: return Op_NEGB();
            case 0x51: return Op_COMB();
            case 0x52: return Op_INCB();
            case 0x53: return Op_DECB();
            case 0x54: return Op_LSRB();
            case 0x55: return Op_ROLB();
            case 0x56: return Op_RORB();
            case 0x57: return Op_ASRB();
            case 0x58: return Op_ASLB();  // = LSLB
            case 0x59: return Op_ASLD();  // = LSLD
            case 0x5A: return Op_STAA(AddrMode.DIR);
            case 0x5B: return Op_STAB(AddrMode.DIR);
            case 0x5C: return Op_STD(AddrMode.DIR);
            case 0x5D: return Op_STY(AddrMode.DIR);
            case 0x5E: return Op_STX(AddrMode.DIR);
            case 0x5F: return Op_STS(AddrMode.DIR);

            // ========== Row 6x: IDX operations ==========
            case 0x60: return Op_NEG(AddrMode.IDX);
            case 0x61: return Op_COM(AddrMode.IDX);
            case 0x62: return Op_INC(AddrMode.IDX);
            case 0x63: return Op_DEC(AddrMode.IDX);
            case 0x64: return Op_LSR(AddrMode.IDX);
            case 0x65: return Op_ROL(AddrMode.IDX);
            case 0x66: return Op_ROR(AddrMode.IDX);
            case 0x67: return Op_ASR(AddrMode.IDX);
            case 0x68: return Op_ASL(AddrMode.IDX);  // = LSL
            case 0x69: return Op_CLR(AddrMode.IDX);
            case 0x6A: return Op_STAA(AddrMode.IDX);
            case 0x6B: return Op_STAB(AddrMode.IDX);
            case 0x6C: return Op_STD(AddrMode.IDX);
            case 0x6D: return Op_STY(AddrMode.IDX);
            case 0x6E: return Op_STX(AddrMode.IDX);
            case 0x6F: return Op_STS(AddrMode.IDX);

            // ========== Row 7x: EXT operations ==========
            case 0x70: return Op_NEG(AddrMode.EXT);
            case 0x71: return Op_COM(AddrMode.EXT);
            case 0x72: return Op_INC(AddrMode.EXT);
            case 0x73: return Op_DEC(AddrMode.EXT);
            case 0x74: return Op_LSR(AddrMode.EXT);
            case 0x75: return Op_ROL(AddrMode.EXT);
            case 0x76: return Op_ROR(AddrMode.EXT);
            case 0x77: return Op_ASR(AddrMode.EXT);
            case 0x78: return Op_ASL(AddrMode.EXT);  // = LSL
            case 0x79: return Op_CLR(AddrMode.EXT);
            case 0x7A: return Op_STAA(AddrMode.EXT);
            case 0x7B: return Op_STAB(AddrMode.EXT);
            case 0x7C: return Op_STD(AddrMode.EXT);
            case 0x7D: return Op_STY(AddrMode.EXT);
            case 0x7E: return Op_STX(AddrMode.EXT);
            case 0x7F: return Op_STS(AddrMode.EXT);

            // ========== Row 8x: IMM A-group ==========
            case 0x80: return Op_SUBA(AddrMode.IMM);
            case 0x81: return Op_CMPA(AddrMode.IMM);
            case 0x82: return Op_SBCA(AddrMode.IMM);
            case 0x83: return Op_SUBD(AddrMode.IMM);
            case 0x84: return Op_ANDA(AddrMode.IMM);
            case 0x85: return Op_BITA(AddrMode.IMM);
            case 0x86: return Op_LDAA(AddrMode.IMM);
            case 0x87: return Op_CLRA(); // CLRA (repurposed STAA IMM slot)
            case 0x88: return Op_EORA(AddrMode.IMM);
            case 0x89: return Op_ADCA(AddrMode.IMM);
            case 0x8A: return Op_ORAA(AddrMode.IMM);
            case 0x8B: return Op_ADDA(AddrMode.IMM);
            case 0x8C: return Op_CPD(AddrMode.IMM);
            case 0x8D: return Op_CPY(AddrMode.IMM);
            case 0x8E: return Op_CPX(AddrMode.IMM);
            case 0x8F: return Op_CPS(AddrMode.IMM);

            // ========== Row 9x: DIR A-group ==========
            case 0x90: return Op_SUBA(AddrMode.DIR);
            case 0x91: return Op_CMPA(AddrMode.DIR);
            case 0x92: return Op_SBCA(AddrMode.DIR);
            case 0x93: return Op_SUBD(AddrMode.DIR);
            case 0x94: return Op_ANDA(AddrMode.DIR);
            case 0x95: return Op_BITA(AddrMode.DIR);
            case 0x96: return Op_LDAA(AddrMode.DIR);
            case 0x97: return Op_TSTA();
            case 0x98: return Op_EORA(AddrMode.DIR);
            case 0x99: return Op_ADCA(AddrMode.DIR);
            case 0x9A: return Op_ORAA(AddrMode.DIR);
            case 0x9B: return Op_ADDA(AddrMode.DIR);
            case 0x9C: return Op_CPD(AddrMode.DIR);
            case 0x9D: return Op_CPY(AddrMode.DIR);
            case 0x9E: return Op_CPX(AddrMode.DIR);
            case 0x9F: return Op_CPS(AddrMode.DIR);

            // ========== Row Ax: IDX A-group ==========
            case 0xA0: return Op_SUBA(AddrMode.IDX);
            case 0xA1: return Op_CMPA(AddrMode.IDX);
            case 0xA2: return Op_SBCA(AddrMode.IDX);
            case 0xA3: return Op_SUBD(AddrMode.IDX);
            case 0xA4: return Op_ANDA(AddrMode.IDX);
            case 0xA5: return Op_BITA(AddrMode.IDX);
            case 0xA6: return Op_LDAA(AddrMode.IDX);
            case 0xA7: return Op_NOP(); // 0xA7 = NOP
            case 0xA8: return Op_EORA(AddrMode.IDX);
            case 0xA9: return Op_ADCA(AddrMode.IDX);
            case 0xAA: return Op_ORAA(AddrMode.IDX);
            case 0xAB: return Op_ADDA(AddrMode.IDX);
            case 0xAC: return Op_CPD(AddrMode.IDX);
            case 0xAD: return Op_CPY(AddrMode.IDX);
            case 0xAE: return Op_CPX(AddrMode.IDX);
            case 0xAF: return Op_CPS(AddrMode.IDX);

            // ========== Row Bx: EXT A-group ==========
            case 0xB0: return Op_SUBA(AddrMode.EXT);
            case 0xB1: return Op_CMPA(AddrMode.EXT);
            case 0xB2: return Op_SBCA(AddrMode.EXT);
            case 0xB3: return Op_SUBD(AddrMode.EXT);
            case 0xB4: return Op_ANDA(AddrMode.EXT);
            case 0xB5: return Op_BITA(AddrMode.EXT);
            case 0xB6: return Op_LDAA(AddrMode.EXT);
            case 0xB7: return Op_TFR_EXG();   // TFR/EXG r1,r2 (bit 7 of postbyte: 0=TFR, 1=EXG)
            case 0xB8: return Op_EORA(AddrMode.EXT);
            case 0xB9: return Op_ADCA(AddrMode.EXT);
            case 0xBA: return Op_ORAA(AddrMode.EXT);
            case 0xBB: return Op_ADDA(AddrMode.EXT);
            case 0xBC: return Op_CPD(AddrMode.EXT);
            case 0xBD: return Op_CPY(AddrMode.EXT);
            case 0xBE: return Op_CPX(AddrMode.EXT);
            case 0xBF: return Op_CPS(AddrMode.EXT);

            // ========== Row Cx: IMM B-group ==========
            case 0xC0: return Op_SUBB(AddrMode.IMM);
            case 0xC1: return Op_CMPB(AddrMode.IMM);
            case 0xC2: return Op_SBCB(AddrMode.IMM);
            case 0xC3: return Op_ADDD(AddrMode.IMM);
            case 0xC4: return Op_ANDB(AddrMode.IMM);
            case 0xC5: return Op_BITB(AddrMode.IMM);
            case 0xC6: return Op_LDAB(AddrMode.IMM);
            case 0xC7: return Op_CLRB(); // CLRB (repurposed STAB IMM slot)
            case 0xC8: return Op_EORB(AddrMode.IMM);
            case 0xC9: return Op_ADCB(AddrMode.IMM);
            case 0xCA: return Op_ORAB(AddrMode.IMM);
            case 0xCB: return Op_ADDB(AddrMode.IMM);
            case 0xCC: return Op_LDD(AddrMode.IMM);
            case 0xCD: return Op_LDY(AddrMode.IMM);
            case 0xCE: return Op_LDX(AddrMode.IMM);
            case 0xCF: return Op_LDS(AddrMode.IMM);

            // ========== Row Dx: DIR B-group ==========
            case 0xD0: return Op_SUBB(AddrMode.DIR);
            case 0xD1: return Op_CMPB(AddrMode.DIR);
            case 0xD2: return Op_SBCB(AddrMode.DIR);
            case 0xD3: return Op_ADDD(AddrMode.DIR);
            case 0xD4: return Op_ANDB(AddrMode.DIR);
            case 0xD5: return Op_BITB(AddrMode.DIR);
            case 0xD6: return Op_LDAB(AddrMode.DIR);
            case 0xD7: return Op_TSTB();
            case 0xD8: return Op_EORB(AddrMode.DIR);
            case 0xD9: return Op_ADCB(AddrMode.DIR);
            case 0xDA: return Op_ORAB(AddrMode.DIR);
            case 0xDB: return Op_ADDB(AddrMode.DIR);
            case 0xDC: return Op_LDD(AddrMode.DIR);
            case 0xDD: return Op_LDY(AddrMode.DIR);
            case 0xDE: return Op_LDX(AddrMode.DIR);
            case 0xDF: return Op_LDS(AddrMode.DIR);

            // ========== Row Ex: IDX B-group ==========
            case 0xE0: return Op_SUBB(AddrMode.IDX);
            case 0xE1: return Op_CMPB(AddrMode.IDX);
            case 0xE2: return Op_SBCB(AddrMode.IDX);
            case 0xE3: return Op_ADDD(AddrMode.IDX);
            case 0xE4: return Op_ANDB(AddrMode.IDX);
            case 0xE5: return Op_BITB(AddrMode.IDX);
            case 0xE6: return Op_LDAB(AddrMode.IDX);
            case 0xE7: return Op_SEX();   // SEX r1,r2
            case 0xE8: return Op_EORB(AddrMode.IDX);
            case 0xE9: return Op_ADCB(AddrMode.IDX);
            case 0xEA: return Op_ORAB(AddrMode.IDX);
            case 0xEB: return Op_ADDB(AddrMode.IDX);
            case 0xEC: return Op_LDD(AddrMode.IDX);
            case 0xED: return Op_LDY(AddrMode.IDX);
            case 0xEE: return Op_LDX(AddrMode.IDX);
            case 0xEF: return Op_LDS(AddrMode.IDX);

            // ========== Row Fx: EXT B-group ==========
            case 0xF0: return Op_SUBB(AddrMode.EXT);
            case 0xF1: return Op_CMPB(AddrMode.EXT);
            case 0xF2: return Op_SBCB(AddrMode.EXT);
            case 0xF3: return Op_ADDD(AddrMode.EXT);
            case 0xF4: return Op_ANDB(AddrMode.EXT);
            case 0xF5: return Op_BITB(AddrMode.EXT);
            case 0xF6: return Op_LDAB(AddrMode.EXT);
            case 0xF7: return UnimplementedOpcode(0xF7, pc); // reserved
            case 0xF8: return Op_EORB(AddrMode.EXT);
            case 0xF9: return Op_ADCB(AddrMode.EXT);
            case 0xFA: return Op_ORAB(AddrMode.EXT);
            case 0xFB: return Op_ADDB(AddrMode.EXT);
            case 0xFC: return Op_LDD(AddrMode.EXT);
            case 0xFD: return Op_LDY(AddrMode.EXT);
            case 0xFE: return Op_LDX(AddrMode.EXT);
            case 0xFF: return Op_LDS(AddrMode.EXT);

            default:
                return UnimplementedOpcode(opcode, pc);
            }
        }

        private ExecutionResult ExecutePage2Instruction()
        {
            var pc = (ushort)((uint)PC - 1); // PC of the 0x18 prefix
            var opcode = FetchByte();

            switch(opcode)
            {
            // Page 2 instructions prefixed with 0x18
            case 0x00: return Op_MOVW_IMM_IDX();  // MOVW imm->idx
            case 0x01: return Op_MOVW_EXT_IDX();  // MOVW ext->idx
            case 0x02: return Op_MOVW_IDX_IDX();  // MOVW idx->idx
            case 0x03: return Op_MOVW_IMM_EXT();  // MOVW imm->ext
            case 0x04: return Op_MOVW_EXT_EXT();  // MOVW ext->ext
            case 0x05: return Op_MOVW_IDX_EXT();  // MOVW idx->ext

            case 0x08: return Op_MOVB_IMM_IDX();
            case 0x09: return Op_MOVB_EXT_IDX();
            case 0x0A: return Op_MOVB_IDX_IDX();
            case 0x0B: return Op_MOVB_IMM_EXT();
            case 0x0C: return Op_MOVB_EXT_EXT();
            case 0x0D: return Op_MOVB_IDX_EXT();

            case 0x06: return Op_ABA();
            case 0x07: return Op_DAA();
            case 0x10: return Op_IDIV();
            case 0x11: return Op_FDIV();
            case 0x12: return Op_EMACS();
            case 0x13: return Op_EMULS();
            case 0x14: return Op_EDIVS();
            case 0x15: return Op_IDIVS();
            case 0x16: return Op_SBA();
            case 0x17: return Op_CBA();
            case 0x1C: return Op_TAB();
            case 0x1D: return Op_TBA();

            // STOP — halt CPU until interrupt (if S bit clear)
            // In emulation: treat as NOP. Timer ISR fires naturally.
            case 0x3E: return ExecutionResult.Ok;

            // Long branches (0x18 0x2x)
            case 0x20: return Op_LBRA();
            case 0x21: return Op_LBRN();
            case 0x22: return Op_LBHI();
            case 0x23: return Op_LBLS();
            case 0x24: return Op_LBCC();
            case 0x25: return Op_LBCS();
            case 0x26: return Op_LBNE();
            case 0x27: return Op_LBEQ();
            case 0x28: return Op_LBVC();
            case 0x29: return Op_LBVS();
            case 0x2A: return Op_LBPL();
            case 0x2B: return Op_LBMI();
            case 0x2C: return Op_LBGE();
            case 0x2D: return Op_LBLT();
            case 0x2E: return Op_LBGT();
            case 0x2F: return Op_LBLE();

            default:
                return UnimplementedOpcode((ushort)(0x1800 | opcode), pc);
            }
        }

        private ExecutionResult ExecuteLoopPrimitive()
        {
            // 0x04 lb rr - DBEQ/DBNE/IBEQ/IBNE/TBEQ/TBNE
            // lb: bits 7:5 = action, bit 4 = MSB of 9-bit offset, bits 2:0 = register
            // rr: lower 8 bits of 9-bit two's complement branch offset
            var postbyte = FetchByte();
            var relByte = FetchByte();
            int relOffset = (int)relByte;
            if((postbyte & 0x10) != 0) relOffset |= 0x100; // bit 4 → bit 8
            if(relOffset >= 256) relOffset -= 512;           // sign extend 9-bit

            var counterReg = postbyte & 0x07;   // bits 2-0: counter register
            var action = postbyte & 0xE0;       // bits 7-5: loop type

            // Get counter register value (CPU12 loop primitive: 0=A, 1=B, 4=D, 5=X, 6=Y, 7=SP)
            ushort counterValue;
            switch(counterReg)
            {
            case 0: counterValue = regA; break;
            case 1: counterValue = regB; break;
            case 4: counterValue = RegD; break;
            case 5: counterValue = regX; break;
            case 6: counterValue = regY; break;
            case 7: counterValue = regSP; break;
            default:
                this.Log(LogLevel.Warning, "Invalid loop counter register {0} at PC=0x{1:X4}", counterReg, (uint)PC - 3);
                return ExecutionResult.Ok;
            }

            bool branch = false;
            switch(action >> 5)
            {
            case 0: // DBEQ
                counterValue--;
                branch = (counterValue == 0);
                break;
            case 1: // DBNE
                counterValue--;
                branch = (counterValue != 0);
                break;
            case 2: // TBEQ
                branch = (counterValue == 0);
                break;
            case 3: // TBNE
                branch = (counterValue != 0);
                break;
            case 4: // IBEQ
                counterValue++;
                branch = (counterValue == 0);
                break;
            case 5: // IBNE
                counterValue++;
                branch = (counterValue != 0);
                break;
            default:
                return UnimplementedOpcode(0x04, (ushort)((uint)PC - 3));
            }

            // Write back counter (CPU12 loop primitive: 0=A, 1=B, 4=D, 5=X, 6=Y, 7=SP)
            switch(counterReg)
            {
            case 0: regA = (byte)counterValue; break;
            case 1: regB = (byte)counterValue; break;
            case 4: RegD = counterValue; break;
            case 5: regX = counterValue; break;
            case 6: regY = counterValue; break;
            case 7: regSP = counterValue; break;
            }

            if(branch)
            {
                PC = (ushort)((uint)PC + relOffset);
            }

            return ExecutionResult.Ok;
        }

        public string LastAbortReason { get; protected set; } = "";

        private ExecutionResult UnimplementedOpcode(int opcode, ushort pc)
        {
            LastAbortReason = $"Unimplemented opcode 0x{opcode:X2} at PC=0x{pc:X4} PPAGE=0x{ppage:X2} ({DumpRegisters()})";
            this.Log(LogLevel.Error, "Unimplemented opcode 0x{0:X2} at PC=0x{1:X4}", opcode, pc);
            this.Log(LogLevel.Debug, "Register dump: {0}", DumpRegisters());
            return ExecutionResult.Aborted;
        }

        #endregion

        #region Indexed Addressing Mode

        // CPU12 indexed addressing post-byte decoding.
        // Reference: S12CPUV2 Reference Manual, Section 3
        //
        // Postbyte encoding (xb):
        //   rr 0 nnnnn      — 5-bit constant offset from rr (-16 to +15)
        //   rr 1 p nnnn     — auto pre/post inc/dec rr by nnnn
        //   111 rr mmm      — special: 9/16-bit offset, accum offset, indirect
        //
        // rr (bits 7:6): 00=X, 01=Y, 10=SP, 11=PC
        // Bit 5 discriminates constant offset (0) vs auto-inc/dec (1),
        // EXCEPT when bits 7:5 = 111 which selects special modes.
        private ushort ResolveIndexedAddress()
        {
            var pb = FetchByte();

            // Special modes: bits 7:5 = 111 (check first, takes priority over auto-inc)
            if((pb & 0xE0) == 0xE0)
            {
                var rr = (pb >> 3) & 0x03; // bits 4:3 = register
                switch(pb & 0x07) // bits 2:0 = mode
                {
                case 0x00: // 9-bit constant offset, positive (1 extra byte)
                {
                    var lo = FetchByte();
                    return (ushort)(GetIndexRegister(rr) + lo);
                }
                case 0x01: // 9-bit constant offset, negative (1 extra byte)
                {
                    var lo = FetchByte();
                    int offset9 = 0x100 | lo; // set bit 8 for negative
                    offset9 |= unchecked((int)0xFFFFFE00); // sign extend
                    return (ushort)(GetIndexRegister(rr) + offset9);
                }
                case 0x02: // 16-bit constant offset (2 extra bytes)
                {
                    var offset16 = (short)FetchWord();
                    return (ushort)(GetIndexRegister(rr) + offset16);
                }
                case 0x03: // 16-bit constant offset, indirect (2 extra bytes)
                {
                    var offset16 = (short)FetchWord();
                    var addr = (ushort)(GetIndexRegister(rr) + offset16);
                    return ReadWord(addr);
                }
                case 0x04: // A,R accumulator offset
                    return (ushort)(GetIndexRegister(rr) + (sbyte)regA);
                case 0x05: // B,R accumulator offset
                    return (ushort)(GetIndexRegister(rr) + (sbyte)regB);
                case 0x06: // D,R accumulator offset
                    return (ushort)(GetIndexRegister(rr) + (short)RegD);
                case 0x07: // [D,R] indirect accumulator D offset
                {
                    var addr = (ushort)(GetIndexRegister(rr) + (short)RegD);
                    return ReadWord(addr);
                }
                }
            }

            var reg = (pb >> 6) & 0x03; // bits 7:6 = register for non-special modes

            // Auto pre/post increment/decrement: bit 5 = 1
            if((pb & 0x20) != 0)
            {
                bool isPost = (pb & 0x10) != 0; // bit 4: 0=pre, 1=post
                int n = pb & 0x0F;               // bits 3:0
                int delta;
                if(n <= 7)
                {
                    delta = n + 1;   // +1 to +8
                }
                else
                {
                    delta = n - 16;  // -8 to -1
                }

                ushort regVal = GetIndexRegister(reg);
                ushort ea;

                if(isPost)
                {
                    ea = regVal;
                    SetIndexRegister(reg, (ushort)(regVal + delta));
                }
                else
                {
                    regVal = (ushort)(regVal + delta);
                    SetIndexRegister(reg, regVal);
                    ea = regVal;
                }

                return ea;
            }

            // 5-bit constant offset: bit 5 = 0
            {
                var offset = (int)(pb & 0x1F); // bits 4:0 signed
                if((offset & 0x10) != 0)
                {
                    offset |= unchecked((int)0xFFFFFFE0); // sign extend
                }
                return (ushort)(GetIndexRegister(reg) + offset);
            }
        }

        private ushort GetIndexRegister(int rr)
        {
            switch(rr)
            {
            case 0: return regX;
            case 1: return regY;
            case 2: return regSP;
            case 3: return (ushort)(uint)PC;
            default: throw new Exception("Invalid index register");
            }
        }

        private void SetIndexRegister(int rr, ushort value)
        {
            switch(rr)
            {
            case 0: regX = value; break;
            case 1: regY = value; break;
            case 2: regSP = value; break;
            case 3: PC = value; break;
            }
        }

        #endregion

        #region Effective Address Resolution

        // Resolve an 8-bit operand based on addressing mode
        private byte ReadOperand8(AddrMode mode)
        {
            switch(mode)
            {
            case AddrMode.IMM: return FetchByte();
            case AddrMode.DIR: return ReadByte(FetchByte());
            case AddrMode.EXT: return ReadByte(FetchWord());
            case AddrMode.IDX: return ReadByte(ResolveIndexedAddress());
            default: throw new Exception($"Invalid addressing mode for 8-bit read: {mode}");
            }
        }

        // Resolve a 16-bit operand
        private ushort ReadOperand16(AddrMode mode)
        {
            switch(mode)
            {
            case AddrMode.IMM: return FetchWord();
            case AddrMode.DIR: return ReadWord(FetchByte());
            case AddrMode.EXT: return ReadWord(FetchWord());
            case AddrMode.IDX: return ReadWord(ResolveIndexedAddress());
            default: throw new Exception($"Invalid addressing mode for 16-bit read: {mode}");
            }
        }

        // Get effective address for write/RMW operations (no IMM mode)
        private ushort GetEffectiveAddress(AddrMode mode)
        {
            switch(mode)
            {
            case AddrMode.DIR: return FetchByte();
            case AddrMode.EXT: return FetchWord();
            case AddrMode.IDX: return ResolveIndexedAddress();
            default: throw new Exception($"Invalid addressing mode for EA: {mode}");
            }
        }

        #endregion

        #region CCR Flag Helpers

        private void SetNZ8(byte result)
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z));
            if(result == 0) ccr |= CCR_Z;
            if((result & 0x80) != 0) ccr |= CCR_N;
        }

        private void SetNZ16(ushort result)
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z));
            if(result == 0) ccr |= CCR_Z;
            if((result & 0x8000) != 0) ccr |= CCR_N;
        }

        private void SetAdd8Flags(byte a, byte b, int result)
        {
            byte r = (byte)result;
            ccr = (byte)(ccr & ~(CCR_H | CCR_N | CCR_Z | CCR_V | CCR_C));
            if(r == 0) ccr |= CCR_Z;
            if((r & 0x80) != 0) ccr |= CCR_N;
            if(result > 0xFF || result < 0) ccr |= CCR_C;
            // Half carry
            if(((a & 0x0F) + (b & 0x0F)) > 0x0F) ccr |= CCR_H;
            // Overflow: signs of operands same but result differs
            if(((a ^ r) & (b ^ r) & 0x80) != 0) ccr |= CCR_V;
        }

        private void SetSub8Flags(byte a, byte b, int result)
        {
            byte r = (byte)result;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(r == 0) ccr |= CCR_Z;
            if((r & 0x80) != 0) ccr |= CCR_N;
            if(result < 0) ccr |= CCR_C; // borrow
            // Overflow: signs differ and result sign matches subtrahend
            if(((a ^ b) & (a ^ r) & 0x80) != 0) ccr |= CCR_V;
        }

        private void SetAdd16Flags(ushort a, ushort b, int result)
        {
            ushort r = (ushort)result;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(r == 0) ccr |= CCR_Z;
            if((r & 0x8000) != 0) ccr |= CCR_N;
            if(result > 0xFFFF || result < 0) ccr |= CCR_C;
            if(((a ^ r) & (b ^ r) & 0x8000) != 0) ccr |= CCR_V;
        }

        private void SetSub16Flags(ushort a, ushort b, int result)
        {
            ushort r = (ushort)result;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(r == 0) ccr |= CCR_Z;
            if((r & 0x8000) != 0) ccr |= CCR_N;
            if(result < 0) ccr |= CCR_C;
            if(((a ^ b) & (a ^ r) & 0x8000) != 0) ccr |= CCR_V;
        }

        #endregion

        #region Instruction Implementations

        // ---------- NOP ----------
        private ExecutionResult Op_NOP()
        {
            return ExecutionResult.Ok;
        }

        // ---------- Load 8-bit ----------
        private ExecutionResult Op_LDAA(AddrMode mode)
        {
            regA = ReadOperand8(mode);
            SetNZ8(regA);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LDAB(AddrMode mode)
        {
            regB = ReadOperand8(mode);
            SetNZ8(regB);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        // ---------- Load 16-bit ----------
        private ExecutionResult Op_LDD(AddrMode mode)
        {
            RegD = ReadOperand16(mode);
            SetNZ16(RegD);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LDX(AddrMode mode)
        {
            regX = ReadOperand16(mode);
            SetNZ16(regX);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LDY(AddrMode mode)
        {
            regY = ReadOperand16(mode);
            SetNZ16(regY);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LDS(AddrMode mode)
        {
            regSP = ReadOperand16(mode);
            SetNZ16(regSP);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        // ---------- Store 8-bit ----------
        private ExecutionResult Op_STAA(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteByte(ea, regA);
            SetNZ8(regA);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_STAB(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteByte(ea, regB);
            SetNZ8(regB);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        // ---------- Store 16-bit ----------
        private ExecutionResult Op_STD(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteWord(ea, RegD);
            SetNZ16(RegD);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_STX(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteWord(ea, regX);
            SetNZ16(regX);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_STY(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteWord(ea, regY);
            SetNZ16(regY);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_STS(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteWord(ea, regSP);
            SetNZ16(regSP);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        // ---------- Add 8-bit ----------
        private ExecutionResult Op_ADDA(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int result = regA + operand;
            SetAdd8Flags(regA, operand, result);
            regA = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ADDB(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int result = regB + operand;
            SetAdd8Flags(regB, operand, result);
            regB = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ADCA(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int carry = (ccr & CCR_C) != 0 ? 1 : 0;
            int result = regA + operand + carry;
            SetAdd8Flags(regA, operand, result);
            regA = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ADCB(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int carry = (ccr & CCR_C) != 0 ? 1 : 0;
            int result = regB + operand + carry;
            SetAdd8Flags(regB, operand, result);
            regB = (byte)result;
            return ExecutionResult.Ok;
        }

        // ---------- Add 16-bit ----------
        private ExecutionResult Op_ADDD(AddrMode mode)
        {
            var operand = ReadOperand16(mode);
            int result = RegD + operand;
            SetAdd16Flags(RegD, operand, result);
            RegD = (ushort)result;
            return ExecutionResult.Ok;
        }

        // ---------- Sub 8-bit ----------
        private ExecutionResult Op_SUBA(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int result = regA - operand;
            SetSub8Flags(regA, operand, result);
            regA = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_SUBB(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int result = regB - operand;
            SetSub8Flags(regB, operand, result);
            regB = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_SBCA(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int borrow = (ccr & CCR_C) != 0 ? 1 : 0;
            int result = regA - operand - borrow;
            SetSub8Flags(regA, operand, result);
            regA = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_SBCB(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int borrow = (ccr & CCR_C) != 0 ? 1 : 0;
            int result = regB - operand - borrow;
            SetSub8Flags(regB, operand, result);
            regB = (byte)result;
            return ExecutionResult.Ok;
        }

        // ---------- Sub 16-bit ----------
        private ExecutionResult Op_SUBD(AddrMode mode)
        {
            var operand = ReadOperand16(mode);
            int result = RegD - operand;
            SetSub16Flags(RegD, operand, result);
            RegD = (ushort)result;
            return ExecutionResult.Ok;
        }

        // ---------- Compare ----------
        private ExecutionResult Op_CMPA(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int result = regA - operand;
            SetSub8Flags(regA, operand, result);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CMPB(AddrMode mode)
        {
            var operand = ReadOperand8(mode);
            int result = regB - operand;
            SetSub8Flags(regB, operand, result);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CPD(AddrMode mode)
        {
            var operand = ReadOperand16(mode);
            int result = RegD - operand;
            SetSub16Flags(RegD, operand, result);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CPX(AddrMode mode)
        {
            var operand = ReadOperand16(mode);
            int result = regX - operand;
            SetSub16Flags(regX, operand, result);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CPY(AddrMode mode)
        {
            var operand = ReadOperand16(mode);
            int result = regY - operand;
            SetSub16Flags(regY, operand, result);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CPS(AddrMode mode)
        {
            var operand = ReadOperand16(mode);
            int result = regSP - operand;
            SetSub16Flags(regSP, operand, result);
            return ExecutionResult.Ok;
        }

        // ---------- Logic ----------
        private ExecutionResult Op_ANDA(AddrMode mode)
        {
            regA &= ReadOperand8(mode);
            SetNZ8(regA);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ANDB(AddrMode mode)
        {
            regB &= ReadOperand8(mode);
            SetNZ8(regB);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ORAA(AddrMode mode)
        {
            regA |= ReadOperand8(mode);
            SetNZ8(regA);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ORAB(AddrMode mode)
        {
            regB |= ReadOperand8(mode);
            SetNZ8(regB);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_EORA(AddrMode mode)
        {
            regA ^= ReadOperand8(mode);
            SetNZ8(regA);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_EORB(AddrMode mode)
        {
            regB ^= ReadOperand8(mode);
            SetNZ8(regB);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BITA(AddrMode mode)
        {
            var result = (byte)(regA & ReadOperand8(mode));
            SetNZ8(result);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BITB(AddrMode mode)
        {
            var result = (byte)(regB & ReadOperand8(mode));
            SetNZ8(result);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        // ---------- Increment/Decrement (inherent) ----------
        private ExecutionResult Op_INCA()
        {
            var old = regA;
            regA++;
            SetNZ8(regA);
            ccr = (byte)((ccr & ~CCR_V) | (old == 0x7F ? CCR_V : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_INCB()
        {
            var old = regB;
            regB++;
            SetNZ8(regB);
            ccr = (byte)((ccr & ~CCR_V) | (old == 0x7F ? CCR_V : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_DECA()
        {
            var old = regA;
            regA--;
            SetNZ8(regA);
            ccr = (byte)((ccr & ~CCR_V) | (old == 0x80 ? CCR_V : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_DECB()
        {
            var old = regB;
            regB--;
            SetNZ8(regB);
            ccr = (byte)((ccr & ~CCR_V) | (old == 0x80 ? CCR_V : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_INX()
        {
            regX++;
            ccr = (byte)((ccr & ~CCR_Z) | (regX == 0 ? CCR_Z : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_DEX()
        {
            regX--;
            ccr = (byte)((ccr & ~CCR_Z) | (regX == 0 ? CCR_Z : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_INY()
        {
            regY++;
            ccr = (byte)((ccr & ~CCR_Z) | (regY == 0 ? CCR_Z : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_DEY()
        {
            regY--;
            ccr = (byte)((ccr & ~CCR_Z) | (regY == 0 ? CCR_Z : 0));
            return ExecutionResult.Ok;
        }

        // ---------- Increment/Decrement (memory) ----------
        private ExecutionResult Op_INC(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var old = ReadByte(ea);
            var result = (byte)(old + 1);
            WriteByte(ea, result);
            SetNZ8(result);
            ccr = (byte)((ccr & ~CCR_V) | (old == 0x7F ? CCR_V : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_DEC(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var old = ReadByte(ea);
            var result = (byte)(old - 1);
            WriteByte(ea, result);
            SetNZ8(result);
            ccr = (byte)((ccr & ~CCR_V) | (old == 0x80 ? CCR_V : 0));
            return ExecutionResult.Ok;
        }

        // ---------- Clear / Negate / Complement (inherent) ----------
        private ExecutionResult Op_CLRA()
        {
            regA = 0;
            ccr = (byte)((ccr & ~(CCR_N | CCR_V | CCR_C)) | CCR_Z);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CLRB()
        {
            regB = 0;
            ccr = (byte)((ccr & ~(CCR_N | CCR_V | CCR_C)) | CCR_Z);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_NEGA()
        {
            var old = regA;
            regA = (byte)(0 - regA);
            SetNZ8(regA);
            ccr = (byte)((ccr & ~(CCR_V | CCR_C)) |
                         (old == 0x80 ? CCR_V : 0) |
                         (regA != 0 ? CCR_C : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_NEGB()
        {
            var old = regB;
            regB = (byte)(0 - regB);
            SetNZ8(regB);
            ccr = (byte)((ccr & ~(CCR_V | CCR_C)) |
                         (old == 0x80 ? CCR_V : 0) |
                         (regB != 0 ? CCR_C : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_COMA()
        {
            regA = (byte)~regA;
            SetNZ8(regA);
            ccr = (byte)((ccr & ~CCR_V) | CCR_C);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_COMB()
        {
            regB = (byte)~regB;
            SetNZ8(regB);
            ccr = (byte)((ccr & ~CCR_V) | CCR_C);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_TSTA()
        {
            SetNZ8(regA);
            ccr &= unchecked((byte)~(CCR_V | CCR_C));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_TSTB()
        {
            SetNZ8(regB);
            ccr &= unchecked((byte)~(CCR_V | CCR_C));
            return ExecutionResult.Ok;
        }

        // ---------- Clear/Negate/Complement (memory) ----------
        private ExecutionResult Op_CLR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            WriteByte(ea, 0);
            ccr = (byte)((ccr & ~(CCR_N | CCR_V | CCR_C)) | CCR_Z);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_NEG(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var old = ReadByte(ea);
            var result = (byte)(0 - old);
            WriteByte(ea, result);
            SetNZ8(result);
            ccr = (byte)((ccr & ~(CCR_V | CCR_C)) |
                         (old == 0x80 ? CCR_V : 0) |
                         (result != 0 ? CCR_C : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_COM(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var result = (byte)~ReadByte(ea);
            WriteByte(ea, result);
            SetNZ8(result);
            ccr = (byte)((ccr & ~CCR_V) | CCR_C);
            return ExecutionResult.Ok;
        }

        // ---------- Shift/Rotate (inherent) ----------
        private ExecutionResult Op_ASLA()
        {
            int result = regA << 1;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regA & 0x80) != 0) ccr |= CCR_C;
            regA = (byte)result;
            SetNZ8(regA);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ASLB()
        {
            int result = regB << 1;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regB & 0x80) != 0) ccr |= CCR_C;
            regB = (byte)result;
            SetNZ8(regB);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ASRA()
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regA & 0x01) != 0) ccr |= CCR_C;
            regA = (byte)((regA >> 1) | (regA & 0x80)); // preserve sign bit
            SetNZ8(regA);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ASRB()
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regB & 0x01) != 0) ccr |= CCR_C;
            regB = (byte)((regB >> 1) | (regB & 0x80));
            SetNZ8(regB);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LSRA()
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regA & 0x01) != 0) ccr |= CCR_C;
            regA >>= 1;
            SetNZ8(regA);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LSRB()
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regB & 0x01) != 0) ccr |= CCR_C;
            regB >>= 1;
            SetNZ8(regB);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ROLA()
        {
            var oldC = (ccr & CCR_C) != 0 ? 1 : 0;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regA & 0x80) != 0) ccr |= CCR_C;
            regA = (byte)((regA << 1) | oldC);
            SetNZ8(regA);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ROLB()
        {
            var oldC = (ccr & CCR_C) != 0 ? 1 : 0;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regB & 0x80) != 0) ccr |= CCR_C;
            regB = (byte)((regB << 1) | oldC);
            SetNZ8(regB);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_RORA()
        {
            var oldC = (ccr & CCR_C) != 0 ? 0x80 : 0;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regA & 0x01) != 0) ccr |= CCR_C;
            regA = (byte)((regA >> 1) | oldC);
            SetNZ8(regA);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_RORB()
        {
            var oldC = (ccr & CCR_C) != 0 ? 0x80 : 0;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((regB & 0x01) != 0) ccr |= CCR_C;
            regB = (byte)((regB >> 1) | oldC);
            SetNZ8(regB);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LSRD()
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((RegD & 0x01) != 0) ccr |= CCR_C;
            RegD = (ushort)(RegD >> 1);
            SetNZ16(RegD);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ASLD()
        {
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((RegD & 0x8000) != 0) ccr |= CCR_C;
            RegD = (ushort)(RegD << 1);
            SetNZ16(RegD);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        // ---------- Shift/Rotate (memory) ----------
        private ExecutionResult Op_ASL(AddrMode mode) // = LSL
        {
            var ea = GetEffectiveAddress(mode);
            var val = ReadByte(ea);
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((val & 0x80) != 0) ccr |= CCR_C;
            val = (byte)(val << 1);
            WriteByte(ea, val);
            SetNZ8(val);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ASR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var val = ReadByte(ea);
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((val & 0x01) != 0) ccr |= CCR_C;
            val = (byte)((val >> 1) | (val & 0x80));
            WriteByte(ea, val);
            SetNZ8(val);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LSR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var val = ReadByte(ea);
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((val & 0x01) != 0) ccr |= CCR_C;
            val = (byte)(val >> 1);
            WriteByte(ea, val);
            SetNZ8(val);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ROL(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var val = ReadByte(ea);
            var oldC = (ccr & CCR_C) != 0 ? 1 : 0;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((val & 0x80) != 0) ccr |= CCR_C;
            val = (byte)((val << 1) | oldC);
            WriteByte(ea, val);
            SetNZ8(val);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ROR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var val = ReadByte(ea);
            var oldC = (ccr & CCR_C) != 0 ? 0x80 : 0;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if((val & 0x01) != 0) ccr |= CCR_C;
            val = (byte)((val >> 1) | oldC);
            WriteByte(ea, val);
            SetNZ8(val);
            if(((ccr & CCR_N) != 0) ^ ((ccr & CCR_C) != 0)) ccr |= CCR_V;
            return ExecutionResult.Ok;
        }

        // ---------- Branches (8-bit relative) ----------
        private ExecutionResult DoBranch(bool condition)
        {
            var offset = (sbyte)FetchByte();
            if(condition)
            {
                PC = (ushort)((uint)PC + offset);
            }
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BRA() => DoBranch(true);
        private ExecutionResult Op_BRN() => DoBranch(false);
        private ExecutionResult Op_BEQ() => DoBranch((ccr & CCR_Z) != 0);
        private ExecutionResult Op_BNE() => DoBranch((ccr & CCR_Z) == 0);
        private ExecutionResult Op_BCS() => DoBranch((ccr & CCR_C) != 0);
        private ExecutionResult Op_BCC() => DoBranch((ccr & CCR_C) == 0);
        private ExecutionResult Op_BMI() => DoBranch((ccr & CCR_N) != 0);
        private ExecutionResult Op_BPL() => DoBranch((ccr & CCR_N) == 0);
        private ExecutionResult Op_BVS() => DoBranch((ccr & CCR_V) != 0);
        private ExecutionResult Op_BVC() => DoBranch((ccr & CCR_V) == 0);
        private ExecutionResult Op_BHI() => DoBranch((ccr & (CCR_C | CCR_Z)) == 0);
        private ExecutionResult Op_BLS() => DoBranch((ccr & (CCR_C | CCR_Z)) != 0);
        private ExecutionResult Op_BGE() => DoBranch(((ccr & CCR_N) != 0) == ((ccr & CCR_V) != 0));
        private ExecutionResult Op_BLT() => DoBranch(((ccr & CCR_N) != 0) != ((ccr & CCR_V) != 0));
        private ExecutionResult Op_BGT() => DoBranch((ccr & CCR_Z) == 0 && ((ccr & CCR_N) != 0) == ((ccr & CCR_V) != 0));
        private ExecutionResult Op_BLE() => DoBranch((ccr & CCR_Z) != 0 || ((ccr & CCR_N) != 0) != ((ccr & CCR_V) != 0));

        // ---------- Long branches (16-bit relative, page 2) ----------
        private ExecutionResult DoLBranch(bool condition)
        {
            var offset = (short)FetchWord();
            if(condition)
            {
                PC = (ushort)((uint)PC + offset);
            }
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LBRA() => DoLBranch(true);
        private ExecutionResult Op_LBRN() => DoLBranch(false);
        private ExecutionResult Op_LBEQ() => DoLBranch((ccr & CCR_Z) != 0);
        private ExecutionResult Op_LBNE() => DoLBranch((ccr & CCR_Z) == 0);
        private ExecutionResult Op_LBCS() => DoLBranch((ccr & CCR_C) != 0);
        private ExecutionResult Op_LBCC() => DoLBranch((ccr & CCR_C) == 0);
        private ExecutionResult Op_LBMI() => DoLBranch((ccr & CCR_N) != 0);
        private ExecutionResult Op_LBPL() => DoLBranch((ccr & CCR_N) == 0);
        private ExecutionResult Op_LBVS() => DoLBranch((ccr & CCR_V) != 0);
        private ExecutionResult Op_LBVC() => DoLBranch((ccr & CCR_V) == 0);
        private ExecutionResult Op_LBHI() => DoLBranch((ccr & (CCR_C | CCR_Z)) == 0);
        private ExecutionResult Op_LBLS() => DoLBranch((ccr & (CCR_C | CCR_Z)) != 0);
        private ExecutionResult Op_LBGE() => DoLBranch(((ccr & CCR_N) != 0) == ((ccr & CCR_V) != 0));
        private ExecutionResult Op_LBLT() => DoLBranch(((ccr & CCR_N) != 0) != ((ccr & CCR_V) != 0));
        private ExecutionResult Op_LBGT() => DoLBranch((ccr & CCR_Z) == 0 && ((ccr & CCR_N) != 0) == ((ccr & CCR_V) != 0));
        private ExecutionResult Op_LBLE() => DoLBranch((ccr & CCR_Z) != 0 || ((ccr & CCR_N) != 0) != ((ccr & CCR_V) != 0));

        // ---------- Jump/Subroutine ----------
        private ExecutionResult Op_JMP(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            PC = ea;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_JSR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            PushWord((ushort)(uint)PC);
            PC = ea;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BSR()
        {
            var offset = (sbyte)FetchByte();
            PushWord((ushort)(uint)PC);
            PC = (ushort)((uint)PC + offset);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_RTS()
        {
            PC = PullWord();
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CALL(AddrMode mode)
        {
            // CALL: like JSR but also saves/restores PPAGE for bank switching
            // CPU12 push order: return address FIRST (higher addr), PPAGE LAST (top of stack)
            ushort targetAddr;
            byte targetPage;

            if(mode == AddrMode.EXT)
            {
                targetAddr = FetchWord();
                targetPage = FetchByte();
            }
            else // IDX
            {
                targetAddr = ResolveIndexedAddress();
                targetPage = FetchByte();
            }

            PushWord((ushort)(uint)PC);  // push return address first
            PushByte(ppage);             // push PPAGE last (on top)
            ppage = (byte)(targetPage & 0x0F);
            PC = targetAddr;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_RTC()
        {
            // CPU12 pull order: PPAGE first (from top), then return address
            ppage = (byte)(PullByte() & 0x0F);
            PC = PullWord();
            return ExecutionResult.Ok;
        }

        // ---------- Stack ----------
        private ExecutionResult Op_PSHA() { PushByte(regA); return ExecutionResult.Ok; }
        private ExecutionResult Op_PSHB() { PushByte(regB); return ExecutionResult.Ok; }
        private ExecutionResult Op_PSHC() { PushByte(ccr); return ExecutionResult.Ok; }
        private ExecutionResult Op_PSHD() { PushWord(RegD); return ExecutionResult.Ok; }
        private ExecutionResult Op_PSHX() { PushWord(regX); return ExecutionResult.Ok; }
        private ExecutionResult Op_PSHY() { PushWord(regY); return ExecutionResult.Ok; }

        private ExecutionResult Op_PULA() { regA = PullByte(); return ExecutionResult.Ok; }
        private ExecutionResult Op_PULB() { regB = PullByte(); return ExecutionResult.Ok; }
        private ExecutionResult Op_PULC() { ccr = PullByte(); return ExecutionResult.Ok; }
        private ExecutionResult Op_PULD() { RegD = PullWord(); return ExecutionResult.Ok; }
        private ExecutionResult Op_PULX() { regX = PullWord(); return ExecutionResult.Ok; }
        private ExecutionResult Op_PULY() { regY = PullWord(); return ExecutionResult.Ok; }

        // ---------- Transfer / Exchange ----------
        private ExecutionResult Op_TAB()
        {
            regB = regA;
            SetNZ8(regB);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_TBA()
        {
            regA = regB;
            SetNZ8(regA);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_TFR_EXG()
        {
            // 0xB7 eb: bit 7 of eb selects TFR (0) or EXG (1)
            // Source register: bits 6:4 (3 bits), destination: bits 3:0
            var postbyte = FetchByte();
            bool isExchange = (postbyte & 0x80) != 0;
            var srcIdx = (postbyte >> 4) & 0x07; // bits 6:4 only
            var dstIdx = postbyte & 0x0F;        // bits 3:0

            var srcVal = GetTfrExgRegister(srcIdx);

            if(isExchange)
            {
                var dstVal = GetTfrExgRegister(dstIdx);
                SetTfrExgRegister(srcIdx, dstVal);
            }

            SetTfrExgRegister(dstIdx, srcVal);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_SEX()
        {
            // SEX r1,r2: sign-extend 8-bit r1 into 16-bit r2
            var postbyte = FetchByte();
            var srcIdx = (postbyte >> 4) & 0x07; // bits 6:4
            var dstIdx = postbyte & 0x0F;
            var srcVal = GetTfrExgRegister(srcIdx);
            var signExtended = (ushort)(short)(sbyte)(byte)srcVal;
            SetTfrExgRegister(dstIdx, signExtended);
            return ExecutionResult.Ok;
        }

        private ushort GetTfrExgRegister(int code)
        {
            switch(code)
            {
            case 0: return regA;
            case 1: return regB;
            case 2: return ccr;
            case 3: // TMP3 - not user accessible, treat as 0
            case 4: return RegD;
            case 5: return regX;
            case 6: return regY;
            case 7: return regSP;
            default: return 0;
            }
        }

        private void SetTfrExgRegister(int code, ushort value)
        {
            switch(code)
            {
            case 0: regA = (byte)value; break;
            case 1: regB = (byte)value; break;
            case 2: ccr = (byte)value; break;
            case 3: break; // TMP3
            case 4: RegD = value; break;
            case 5: regX = value; break;
            case 6: regY = value; break;
            case 7: regSP = value; break;
            }
        }

        // ---------- CCR manipulation ----------
        private ExecutionResult Op_ANDCC()
        {
            var mask = FetchByte();
            ccr &= mask;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_ORCC()
        {
            var mask = FetchByte();
            ccr |= mask;
            return ExecutionResult.Ok;
        }

        // ---------- Interrupt ----------
        private ExecutionResult Op_RTI()
        {
            ccr = PullByte();
            regB = PullByte();
            regA = PullByte();
            regX = PullWord();
            regY = PullWord();
            PC = PullWord();
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_SWI()
        {
            PushWord((ushort)(uint)PC);
            PushWord(regY);
            PushWord(regX);
            PushByte(regA);
            PushByte(regB);
            PushByte(ccr);
            ccr |= CCR_I;
            PC = ReadWord(0xFFF6); // SWI vector
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_WAI()
        {
            // Wait for interrupt - push all regs, then halt until interrupt
            PushWord((ushort)(uint)PC);
            PushWord(regY);
            PushWord(regX);
            PushByte(regA);
            PushByte(regB);
            PushByte(ccr);
            // In emulation, just return and let the interrupt handler pick up on next cycle
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_STOP()
        {
            // STOP instruction - halts clocks if S bit clear
            if((ccr & CCR_S) == 0)
            {
                this.Log(LogLevel.Warning, "STOP instruction executed at PC=0x{0:X4}", (uint)PC);
            }
            return ExecutionResult.Ok;
        }

        // ---------- LEA instructions ----------
        private ExecutionResult Op_LEAX()
        {
            regX = ResolveIndexedAddress();
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LEAY()
        {
            regY = ResolveIndexedAddress();
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_LEAS()
        {
            regSP = ResolveIndexedAddress();
            return ExecutionResult.Ok;
        }

        // ---------- Bit manipulation ----------
        private ExecutionResult Op_BSET(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var mask = FetchByte();
            var val = ReadByte(ea);
            val |= mask;
            WriteByte(ea, val);
            SetNZ8(val);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BCLR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var mask = FetchByte();
            var val = ReadByte(ea);
            val &= (byte)~mask;
            WriteByte(ea, val);
            SetNZ8(val);
            ccr &= unchecked((byte)~CCR_V);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BRSET(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var mask = FetchByte();
            var rel = (sbyte)FetchByte();
            var val = ReadByte(ea);
            if((val & mask) == mask)
            {
                PC = (ushort)((uint)PC + rel);
            }
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_BRCLR(AddrMode mode)
        {
            var ea = GetEffectiveAddress(mode);
            var mask = FetchByte();
            var rel = (sbyte)FetchByte();
            var val = ReadByte(ea);
            if((val & mask) == 0)
            {
                PC = (ushort)((uint)PC + rel);
            }
            return ExecutionResult.Ok;
        }

        // ---------- Page 2 arithmetic ----------
        private ExecutionResult Op_ABA()
        {
            int result = regA + regB;
            SetAdd8Flags(regA, regB, result);
            regA = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_SBA()
        {
            int result = regA - regB;
            SetSub8Flags(regA, regB, result);
            regA = (byte)result;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_CBA()
        {
            int result = regA - regB;
            SetSub8Flags(regA, regB, result);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MUL()
        {
            // D = A * B (unsigned)
            RegD = (ushort)(regA * regB);
            ccr = (byte)((ccr & ~CCR_C) | ((regB & 0x80) != 0 ? CCR_C : 0));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_EMUL()
        {
            // Y:D = D * Y (unsigned)
            uint result = (uint)RegD * (uint)regY;
            RegD = (ushort)(result & 0xFFFF);
            regY = (ushort)(result >> 16);
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_C));
            if(result == 0) ccr |= CCR_Z;
            if((result & 0x80000000) != 0) ccr |= CCR_N;
            if((result & 0x8000) != 0) ccr |= CCR_C;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_EDIV()
        {
            // Y:D / X -> Y remainder D (unsigned)
            if(regX == 0)
            {
                ccr |= CCR_C;
                return ExecutionResult.Ok;
            }
            uint dividend = ((uint)regY << 16) | RegD;
            uint quotient = dividend / regX;
            uint remainder = dividend % regX;
            regY = (ushort)quotient;
            RegD = (ushort)remainder;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(quotient > 0xFFFF) ccr |= CCR_V;
            if(quotient == 0) ccr |= CCR_Z;
            if((quotient & 0x8000) != 0) ccr |= CCR_N;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_DAA()
        {
            // Decimal Adjust A (BCD)
            int correction = 0;
            if((ccr & CCR_H) != 0 || (regA & 0x0F) > 9) correction += 6;
            if((ccr & CCR_C) != 0 || regA > 0x99) { correction += 0x60; ccr |= CCR_C; }
            regA = (byte)(regA + correction);
            SetNZ8(regA);
            return ExecutionResult.Ok;
        }

        // ---------- Move instructions (page 2) ----------
        private ExecutionResult Op_MOVB_IMM_IDX()
        {
            var data = FetchByte();
            var ea = ResolveIndexedAddress();
            WriteByte(ea, data);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVB_EXT_IDX()
        {
            var src = FetchWord();
            var ea = ResolveIndexedAddress();
            WriteByte(ea, ReadByte(src));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVB_IDX_IDX()
        {
            var srcEa = ResolveIndexedAddress();
            var dstEa = ResolveIndexedAddress();
            WriteByte(dstEa, ReadByte(srcEa));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVB_IMM_EXT()
        {
            var data = FetchByte();
            var ea = FetchWord();
            WriteByte(ea, data);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVB_EXT_EXT()
        {
            var src = FetchWord();
            var dst = FetchWord();
            WriteByte(dst, ReadByte(src));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVB_IDX_EXT()
        {
            var srcEa = ResolveIndexedAddress();
            var dst = FetchWord();
            WriteByte(dst, ReadByte(srcEa));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVW_IMM_IDX()
        {
            var data = FetchWord();
            var ea = ResolveIndexedAddress();
            WriteWord(ea, data);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVW_EXT_IDX()
        {
            var src = FetchWord();
            var ea = ResolveIndexedAddress();
            WriteWord(ea, ReadWord(src));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVW_IDX_IDX()
        {
            var srcEa = ResolveIndexedAddress();
            var dstEa = ResolveIndexedAddress();
            WriteWord(dstEa, ReadWord(srcEa));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVW_IMM_EXT()
        {
            var data = FetchWord();
            var ea = FetchWord();
            WriteWord(ea, data);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVW_EXT_EXT()
        {
            var src = FetchWord();
            var dst = FetchWord();
            WriteWord(dst, ReadWord(src));
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MOVW_IDX_EXT()
        {
            var srcEa = ResolveIndexedAddress();
            var dst = FetchWord();
            WriteWord(dst, ReadWord(srcEa));
            return ExecutionResult.Ok;
        }

        // ---------- Stubs for less common page 2 instructions ----------
        private ExecutionResult Op_EMULS()
        {
            // Y:D = D * Y (signed)
            int result = (short)RegD * (short)regY;
            RegD = (ushort)(result & 0xFFFF);
            regY = (ushort)((uint)result >> 16);
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_C));
            if(result == 0) ccr |= CCR_Z;
            if(result < 0) ccr |= CCR_N;
            if((result & 0x8000) != 0) ccr |= CCR_C;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_EDIVS()
        {
            // Y:D / X -> Y remainder D (signed)
            if(regX == 0)
            {
                ccr |= CCR_C;
                return ExecutionResult.Ok;
            }
            int dividend = ((int)(short)regY << 16) | RegD;
            int quotient = dividend / (short)regX;
            int remainder = dividend % (short)regX;
            regY = (ushort)quotient;
            RegD = (ushort)remainder;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(quotient == 0) ccr |= CCR_Z;
            if(quotient < 0) ccr |= CCR_N;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_IDIV()
        {
            // D / X -> X remainder D (unsigned)
            if(regX == 0)
            {
                ccr |= CCR_C;
                regX = 0xFFFF;
                RegD = 0xFFFF;
                return ExecutionResult.Ok;
            }
            ushort quotient = (ushort)(RegD / regX);
            ushort remainder = (ushort)(RegD % regX);
            regX = quotient;
            RegD = remainder;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(quotient == 0) ccr |= CCR_Z;
            ccr &= unchecked((byte)~CCR_V); // V always cleared
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_FDIV()
        {
            // D / X -> X remainder D (fractional unsigned)
            if(regX == 0)
            {
                ccr |= CCR_C;
                return ExecutionResult.Ok;
            }
            uint dividend = (uint)RegD << 16;
            ushort quotient = (ushort)(dividend / regX);
            ushort remainder = (ushort)(dividend % regX);
            regX = quotient;
            RegD = remainder;
            ccr = (byte)(ccr & ~(CCR_Z | CCR_V | CCR_C));
            if(quotient == 0) ccr |= CCR_Z;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_IDIVS()
        {
            // D / X -> X remainder D (signed)
            if(regX == 0)
            {
                ccr |= CCR_C;
                return ExecutionResult.Ok;
            }
            short quotient = (short)((short)RegD / (short)regX);
            short remainder = (short)((short)RegD % (short)regX);
            regX = (ushort)quotient;
            RegD = (ushort)remainder;
            ccr = (byte)(ccr & ~(CCR_N | CCR_Z | CCR_V | CCR_C));
            if(quotient == 0) ccr |= CCR_Z;
            if(quotient < 0) ccr |= CCR_N;
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_EMACS()
        {
            // Multiply-accumulate signed: stub
            this.Log(LogLevel.Warning, "EMACS instruction not fully implemented");
            FetchWord(); // skip operand
            return ExecutionResult.Ok;
        }

        // ---------- Background debug / fuzzy logic stubs ----------
        private ExecutionResult Op_BGND()
        {
            this.Log(LogLevel.Warning, "BGND at PC=0x{0:X4}", (uint)PC);
            return ExecutionResult.Ok;
        }

        private ExecutionResult Op_MEM()
        {
            // S12CPUV2 removed MEM (fuzzy logic) — 0x01 is NOP on MC9S12P-family chips.
            // Firmware uses 0x01 as a timing NOP between I/O register bit manipulations.
            return ExecutionResult.Ok;
        }

        #endregion

        #region Constants

        // CCR bit masks
        private const byte CCR_C = 0x01; // Carry
        private const byte CCR_V = 0x02; // Overflow
        private const byte CCR_Z = 0x04; // Zero
        private const byte CCR_N = 0x08; // Negative
        protected const byte CCR_I = 0x10; // Interrupt mask
        private const byte CCR_H = 0x20; // Half carry
        private const byte CCR_X = 0x40; // X interrupt mask
        private const byte CCR_S = 0x80; // Stop disable

        // Memory map constants
        private const ushort PPAGEAddress = 0x0030;
        private const byte PPAGEResetValue = 0x0E;
        private const ushort PagedWindowStart = 0x8000;
        private const ushort PagedWindowEnd = 0xBFFF;
        private const ushort ResetVectorAddress = 0xFFFE;

        // P-Flash global addressing
        private const uint PFlashGlobalBase = 0x020000; // MC9S12P128 flash starts here
        private const uint MaxPFlashSize = 0x020000;    // 128KB max

        // ATD: 16 input channels, 8 result registers (ATDDR0-ATDDR7)
        private const int NumberOfAnalogChannels = 16;
        private const int NumberOfAnalogResults = 8;

        #endregion

        #region Fields

        private byte regA;
        private byte regB;
        private ushort regX;
        private ushort regY;
        private ushort regSP;
        private byte ccr;
        private byte ppage;
        private ulong executedInstructions;
        private byte copSequence;

        // Timer state. TCNT advances once per instruction divided by the
        // TSCR2 prescaler (one instruction stands in for one bus cycle).
        private ushort timerCounter;
        private bool timerEnabled;
        private ushort timerPrescaler = 1;
        private ushort timerPrescaleCount;
        private const ushort MinPrescaler = 1;

        private readonly ushort[] analogValues = new ushort[NumberOfAnalogChannels];

        // Timer interrupt numbers (vector table offsets)
        private static readonly int[] TimerChannelIRQ = { 8, 9, 10, 11, 12, 13, 14, 15 };

        private void TickTimer()
        {
            if(!timerEnabled)
            {
                return;
            }

            timerPrescaleCount++;
            if(timerPrescaleCount < Math.Max(timerPrescaler, MinPrescaler))
            {
                return;
            }
            timerPrescaleCount = 0;
            timerCounter++;

            // Check output compare matches
            byte tios = ioRegs[0x40];  // which channels are output compare
            byte tie = ioRegs[0x4C];   // which channels have interrupts enabled
            byte newFlags = 0;

            for(int ch = 0; ch < 8; ch++)
            {
                if((tios & (1 << ch)) == 0)
                {
                    continue; // input capture, skip
                }

                // Read TCx (16-bit compare value at 0x0050 + ch*2)
                ushort tcx = (ushort)((ioRegs[0x50 + ch * 2] << 8) | ioRegs[0x51 + ch * 2]);
                if(timerCounter == tcx)
                {
                    newFlags |= (byte)(1 << ch);
                }
            }

            if(newFlags != 0)
            {
                ioRegs[0x4E] |= newFlags; // set TFLG1 flags

                // Fire interrupts for channels that have TIE set
                byte irqFlags = (byte)(newFlags & tie);
                for(int ch = 0; ch < 8; ch++)
                {
                    if((irqFlags & (1 << ch)) != 0)
                    {
                        pendingInterrupt.Add(TimerChannelIRQ[ch]);
                    }
                }
            }
        }

        // Trace ring buffer for post-mortem debugging
        private const int TraceRingSize = 256;
        private readonly string[] traceRing = new string[TraceRingSize];
        private int traceRingIndex;

        private void RecordTrace(ushort pc, byte opcode)
        {
            traceRing[traceRingIndex] = $"#{executedInstructions} PC=0x{pc:X4} OP=0x{opcode:X2} A={regA:X2} B={regB:X2} X={regX:X4} Y={regY:X4} SP={regSP:X4} CCR={ccr:X2} PP={ppage:X1}";
            traceRingIndex = (traceRingIndex + 1) % TraceRingSize;
        }

        protected void DumpTraceRing()
        {
            this.Log(LogLevel.Warning, "=== Last {0} instructions ===", TraceRingSize);
            for(int i = 0; i < TraceRingSize; i++)
            {
                var idx = (traceRingIndex + i) % TraceRingSize;
                if(traceRing[idx] != null)
                {
                    this.Log(LogLevel.Warning, "  {0}", traceRing[idx]);
                }
            }
        }

        // I/O register backing store (0x0000-0x03FF)
        private readonly byte[] ioRegs = new byte[0x0400];

        private readonly Dictionary<ulong, HashSet<CpuAddressHook>> hooks = new Dictionary<ulong, HashSet<CpuAddressHook>>();
        private readonly SortedSet<int> pendingInterrupt = new SortedSet<int>();

        #endregion

        private enum AddrMode
        {
            IMM,  // Immediate
            DIR,  // Direct (8-bit address)
            EXT,  // Extended (16-bit address)
            IDX,  // Indexed (postbyte)
            INH,  // Inherent
        }

        private enum Registers : int
        {
            D = 0,
            X = 1,
            Y = 2,
            SP = 3,
            PC = 4,
            CCR = 5,
        }
    }
}
