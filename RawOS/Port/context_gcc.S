/*
 [] RTT,scontext_gcc.S
*/
 
    .cpu    cortex-m3
    .fpu    softvfp
    .syntax unified
    .thumb
    .text

	.equ	NVIC_INT_CTRL,0xE000ED04
	.equ	NVIC_SYSPRI14,0xE000ED22
	.equ	NVIC_PENDSV_PRI,0xFF
	.equ	NVIC_PENDSVSET,0x10000000


/*unsigned int OS_CPU_SR_Save(void)*/
    .global OS_CPU_SR_Save
    .type OS_CPU_SR_Save, %function
OS_CPU_SR_Save:
    MRS     R0, PRIMASK
    CPSID   I
    BX      LR

/*void OS_CPU_SR_Restore(unsigned int sr)*/
    .global OS_CPU_SR_Restore
    .type OS_CPU_SR_Restore, %function
OS_CPU_SR_Restore:
    MSR     PRIMASK, R0
    BX      LR

/*void        CPU_IntDis       (void)*/
    .global CPU_IntDis
    .type CPU_IntDis, %function
CPU_IntDis:
        CPSID   I
        BX      LR

/*void        CPU_IntEn       (void)*/
    .global CPU_IntEn
    .type CPU_IntEn, %function
CPU_IntEn:
        CPSIE   I
        BX      LR


	.global raw_start_first_task
    .type raw_start_first_task, %function
raw_start_first_task:
	LDR     R0, =NVIC_SYSPRI14
    LDR     R1, =NVIC_PENDSV_PRI
    STRB    R1, [R0]

    MOVS    R0, #0
    MSR     PSP, R0

	MRS 	R0, MSP
	LSRS    R0, R0, #3
	LSLS    R0, R0, #3
	MSR     MSP, R0

    LDR     R0, =NVIC_INT_CTRL
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]

    CPSIE   I

OSStartHang:
    B       OSStartHang

	.global port_task_switch
 	.type port_task_switch, %function
port_task_switch:
    LDR     R0, =NVIC_INT_CTRL
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR

	.global raw_int_switch
 	.type raw_int_switch, %function
raw_int_switch:
    LDR     R0, =NVIC_INT_CTRL
    LDR     R1, =NVIC_PENDSVSET
    STR     R1, [R0]
    BX      LR

	.global PendSV_Handler
 	.type PendSV_Handler, %function
PendSV_Handler:
    CPSID   I
    MRS     R0, PSP
    CBZ     R0, OS_CPU_PendSVHandler_nosave

	//IF		{FPU} != "SoftVFP"
	//VSTMFD	r0!, {d8 - d15} 		; push FPU register s16~s31
	//ENDIF

    SUBS    R0, R0, #0x20
    STM     R0, {R4-R11}

    LDR     R1, =raw_task_active
    LDR     R1, [R1]
    STR     R0, [R1]

	PUSH    {R14}
	bl      raw_stack_check
	POP     {R14}

OS_CPU_PendSVHandler_nosave:

    LDR     R0, =raw_task_active
    LDR     R1, =high_ready_obj
    LDR     R2, [R1]
    STR     R2, [R0]

    LDR     R0, =raw_task_active
    LDR     R1, =high_ready_obj
    LDR     R2, [R1]
    STR     R2, [R0]

	LDR     R0, [R2]
    LDM     R0, {R4-R11}
    ADDS    R0, R0, #0x20

	//IF		{FPU} != "SoftVFP"
	//VSTMDB  R0!, {d8 - d15}		//; pop FPU register s16~s31
	//ENDIF

    MSR     PSP, R0
    ORR     LR, LR, #0x04
    CPSIE   I
    BX      LR

