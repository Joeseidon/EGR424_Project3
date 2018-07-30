/*
 * Implement the thread creation task:
 *
 *   - initialize the jump buffer with appropriate values for
 *     R13 (stack) and R14 (first address to jump to)
 *   - all other registers are irrelevant upon thread creation
 *
 *   In the jump buffer, the R13 slot is set to the second parameter of this
 *   function (the top-of-stack address, passed in R1). The R14 slot is set to
 *   the address of the threadStarter() function.
 *
 *   The C prototype for this function call is:
 *       createThread(threads[i].state, threads[i].stack)
 *   thus:
 *       R0 <-- state (a setjmp()-style jump buffer)
 *       R1 <-- stack (address of top-of-stack)
 */
    .syntax unified
    .text
    .align 2
    .thumb
    .thumb_func
    .type createThread,function
    .global createThread
createThread:  

	/* Get the address to return to */
	mov r6, r2
	    
				/*Load data on stack*/
	/* This will set the value to be loaded into xPSR setting the T bit (bit24) to indicate thumb state*/
	
	movw r3, 0x0000
	movt r3, 0x0100				/*0x80 sets the 8th bit of the top 16 bits*/
	MSR PSR, r3
	
	stmdb r1!, {r3} 			/*This value will now be loaded for xPSR */
	
	
	stmdb r1!, {r6}				/*Save thread starter as address2returnto*/
	
	/*Save default values*/
	mov r2, #0
	mov r3, #0
	mov r12, #0
	stmdb r1!, {r0-r3,r12,r14}	

	/*Load data into buffer*/
	mov r12, r1	/*move stack pointer into r12 so that it will be at the top of the stack*/
	stmia r0, {r4-r12}	

    bx lr
    
.L0:
    .word   threadStarter
