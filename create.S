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
	ldr r6, .L0
	    
				/*Load data on stack*/
	/* This will set the value to be loaded into xPSR setting the T bit (bit24) to indicate thumb state*/
	
	movt r3, 0x80 				/*0x80 sets the 8th bit of the top 16 bits*/
	MSR PSR, r3
	
	
	stmdb r1!, {r3} 			/*This value will now be loaded for xPSR */
	
	
	stmdb r1!, {r6}				/*Save thread starter as address2returnto*/
	
	
	stmdb r1!, {r0-r3,r12,r14}	/*Save default values*/

	/*Load data into buffer*/
	stmea r0, {r4-r11, r1}
  
  /* Now we save SP and LR in that order. SP is the R1 parameter, and we have
   * to get the address of threadStarter() into a higher register (so they are
   * placed in the jump buffer in the right order). */

    bx lr
    
.L0:
    .word   threadStarter
