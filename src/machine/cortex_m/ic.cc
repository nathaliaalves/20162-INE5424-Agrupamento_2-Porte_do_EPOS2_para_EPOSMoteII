// EPOS Cortex_M IC Mediator Implementation

#include <machine/cortex_m/ic.h>
#include <machine.h>

extern "C" { void _int_exit() __attribute__ ((alias("_ZN4EPOS1S11Cortex_M_IC4exitEv"))); }
extern "C" { void _int_dispatch(unsigned int id) __attribute__ ((alias("_ZN4EPOS1S11Cortex_M_IC8dispatchEj"))); }
extern "C" { void _int_entry() __attribute__ ((alias("_ZN4EPOS1S11Cortex_M_IC5entryEv"))); }
extern "C" { void _svc_handler() __attribute__ ((alias("_ZN4EPOS1S11Cortex_M_IC11svc_handlerEv"))); }

__BEGIN_SYS

// Class attributes
Cortex_M_IC::Interrupt_Handler Cortex_M_IC::_int_vector[Cortex_M_IC::INTS];

/*
We need to get around Cortex M3's interrupt handling to be able to make it re-entrant
The problem is that interrupts are handled in Handler mode, and in this mode the processor
is only preempted by interrupts of higher (not equal!) priority.
Moreover, the processor automatically pushes information into the stack when an interrupt happens,
and it only pops this information and gets out of Handler mode when a specific value (called EXC_RETURN)
is loaded to PC, representing the final return from the interrupt handler.
When an interrupt happens, the processor pushes this information into the current stack:

   (1) Stack pushed by processor
         +-----------+
SP + 32  |<alignment>| (one word of padding if necessary, to make the stack 8-byte-aligned)
         +-----------+
SP + 28  |xPSR       | (with bit 9 set if there is alignment)
         +-----------+
SP + 24  |PC         | (return address)
         +-----------+
SP + 20  |LR         | (link register before being overwritten with EXC_RETURN)
         +-----------+
SP + 16  |R12        | (general purpose register 12)
         +-----------+
SP + 12  |R3         | (general purpose register 3)
         +-----------+
SP +  8  |R2         | (general purpose register 2)
         +-----------+
SP +  4  |R1         | (general purpose register 1)
         +-----------+
SP       |R0         | (general purpose register 0)
         +-----------+

Also, it enters Handler mode and the value of LR is overwritten with EXC_RETURN
(in our case, it is always 0xFFFFFFF9).
To execute dispatch() in Thread mode, which is preemptable, we extend this stack with the following:

   (2) Stack built to make the processor execute dispatch() outside of Handler mode
         +-----------+
SP + 32  |EXC_RETURN | (value used later on)
         +-----------+
SP + 28  |1 << 24    | (xPSR with Thumb bit set (the only mandatory bit for Cortex-M3))
         +-----------+
SP + 24  |dispatch   | (address of the actual dispatch method)
         +-----------+
SP + 20  |exit       | (address of the interrupt epilogue)
         +-----------+
SP + 16  |Don't Care | (general purpose register 12)
         +-----------+
SP + 12  |Don't Care | (general purpose register 3)
         +-----------+
SP +  8  |Don't Care | (general purpose register 2)
         +-----------+
SP +  4  |Don't Care | (general purpose register 1)
         +-----------+
SP       |int_id     | (to be passed as argument to dispatch())
         +-----------+

And then load EXC_RETURN into pc. This will cause stack (2) to the popped up until the EXC_RETURN value pushed.
The stack will return to state (1) with the addition of EXC_RETURN, the processor will be in Thread mode, and
the followingregisters of interest will be updated:
    r0 = int_id
    pc = dispatch
    lr = exit

Then dispatch(int_id) will be executed and return to exit(), which simply issues a supervisor call (SVC).
The processor then enters handler mode and pushes a new stack like (1) to execute the SVC. The svc handler
simply ignores this stack, sets the stack back to (1) and returns from the interrupt, making the processor
restore the context it saved in (1).

We use SVC to return because the processor does things when returning from an interrupt that are hard to be
replicated in software. For instance, it might consistently return to the middle (not the beginning) of
an stm (Store Multiple) instruction.

Known issues:
- If the handler executed disables interrupts, the svc instruction in exit() will cause a hard fault.
This can be detected and revert if necessary. One would need to make the hard fault handler detect that the
fault was generated in exit(), and in this case simply call svc_handler.

More information can be found at:
[1] ARMv7-M Architecture Reference Manual:
        Section B1.5.6 (Exception entry behavior)
        Section B1.5.7 (Stack alignment on exception entry)
        Section B1.5.8 (Exception return behavior)
[2] https://sites.google.com/site/sippeyfunlabs/embedded-system/how-to-run-re-entrant-task-scheduler-on-arm-cortex-m4
[3] https://community.arm.com/thread/4919
*/

// Class methods
void Cortex_M_IC::entry() // __attribute__((naked));
{
    // Building the fake stack (2)
    ASM("   mrs     r0, xpsr           \n"
        "   and     r0, #0x3f          \n"); // Store int_id in r0 (which will be passed as argument to dispatch())

    if(Traits<Cortex_M_USB>::enabled) {
        // This is a workaround for the USB interrupt (60). It is level-enabled, so we need to process it in handler mode, otherwise the handler will never exit
        ASM("   cmp     r0, #60            \n" // Check if this is the USB IRQ
            "   bne     NOT_USB            \n"
            "   b       _int_dispatch      \n" // Execute USB interrupt in handler mode
            "   bx      lr                 \n" // Return from handler mode directly to pre-interrupt code
            "NOT_USB:                      \n");
    }

    ASM("   mov     r3, #1             \n"
        "   lsl     r3, #24            \n" // xPSR with Thumb bit only. Other bits are Don't Care
        "   ldr     r1, =_int_exit     \n" // Fake LR (will cause exit() to execute after dispatch())
        "   ldr     r2, =_int_dispatch \n" // Fake PC (will cause dispatch() to execute after entry())
        "   sub     r2, #1             \n" // This one instruction is necessary for old versions of qemu (confirmed on 2.0.0). It is inocuous and can be removed on newer versions of qemu (confirmed on 2.6.0) and on the actual EPOSMoteIII.
        "   push    {lr}               \n" // Push EXC_RETURN code, which will be popped by svc_handler
        "   push    {r1-r3}            \n" // Fake stack (2): xPSR, PC, LR
        "   push    {r0-r3, r12}       \n" // Push rest of fake stack (2)
        "   bx      lr                 \n"); // Return from handler mode. Will proceed to dispatch()
}

void Cortex_M_IC::dispatch(unsigned int id)
{
    if((id != INT_TIMER) || Traits<IC>::hysterically_debugged)
        db<IC>(TRC) << "IC::dispatch(i=" << id << ")" << endl;

    _int_vector[id](id);
    // Will proceed to exit()
}

void Cortex_M_IC::exit() // __attribute__((naked));
{
    ASM("svc #7 \n"); // 7 is an arbitrary number
    // Will enter handler mode and proceed to svc_handler()
}

void Cortex_M_IC::svc_handler()
{
    // Set the stack back to state (1) and tell the processor to recover the pre-interrupt context
    ASM("   ldr r0, [sp, #28]\n" // Read stacked xPSR
        "   and r0, #0x200   \n" // Bit 9 indicating alignment existence
        "   lsr r0, #7       \n" // if bit9==1 then r0=4 else r0=0
        "   add r0, sp       \n"
        "   add r0, #32      \n" // r0 now points to were EXC_RETURN was pushed
        "   mov sp, r0       \n" // Set stack pointer to that address
        "   isb              \n"
        "   pop {pc}         \n"); // Pops EXC_RETURN, so that stack is in state (1)
                                   // Load EXC_RETURN code to pc
                                   // Processor unrolls stack (1)
                                   // And we're back to pre-interrupt code
}

void Cortex_M_IC::int_not(const Interrupt_Id & i)
{
    db<IC>(WRN) << "IC::int_not(i=" << i << ")" << endl;
}

void Cortex_M_IC::hard_fault(const Interrupt_Id & i)
{
    db<IC>(ERR) << "IC::hard_fault(i=" << i << ")" << endl;
    Machine::panic();
}

__END_SYS
