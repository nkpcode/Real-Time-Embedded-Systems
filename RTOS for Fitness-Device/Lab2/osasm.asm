;/*****************************************************************************/
; OSasm.asm: low-level OS commands, written in assembly                       */
; Runs on LM4F120/TM4C123/MSP432
; Lab 2 starter file
; February 10, 2016
;

        .thumb
        .text
        .align 2
        .global  RunPt            ; currently running thread
        .global  StartOS
        .global  SysTick_Handler
        .global  Scheduler

RunPtAddr .field RunPt,32
SysTick_Handler:  .asmfunc     ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                  ; 2) Prevent interrupt during switch
    PUSH    {R4-R11}           ; 3) Save remaining regs r4-11
    LDR     R0, =RunPt
    LDR     R1, [R0]
    STR     SP, [R1]           ; save current thread SP into tcbs SP
    ADD     R1, R1, #4         ; go to address of next pointer
    LDR     R1, [R1]           ; get the next pointer
    STR     R1, [R0]           ; set RunPt = RunPt->next
    LDR     SP, [R1]           ; SP = new thread's SP
    POP     {R4-R11}
    CPSIE   I                  ; 9) tasks run with interrupts enabled
    BX      LR                 ; 10) restore R0-R3,R12,LR,PC,PSR

       .endasmfunc
StartOS: .asmfunc

    LDR    R0,=RunPt
    LDR    R1,[R0]
    LDR    SP,[R1]     ;get the SP for the first thread from tcbs[0]
    POP     {R4-R11}    ;Populate the register with that of the thread stack
    POP     {R0-R3}
    POP     {R12}
    ADD     SP, SP, #4  ;discard first stack LR
    POP     {LR}        ;put the addresss in PC into LR
    ADD     SP, SP, #4  ;discard PSR
    CPSIE   I                  ; Enable interrupts at processor level
    BX      LR                 ; start first thread

       .endasmfunc
      .end

