#include <stdio.h>
#include <setjmp.h>
#include <stdlib.h>
#include "driverlib/debug.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "rit128x96x4.h"
#include "scheduler.h"
#include "driverlib/interrupt.h"
#include "inc/lm3s6965.h"

#define STACK_SIZE 1604   // Amount of stack space for each thread

// This is the lock variable used by all threads. Interface functions
// for it are:
//      void lock_init(unsigned *threadlockptr);        // You write this
//      unsigned lock_acquire(unsigned *threadlockptr); // Shown in class
//      void lock_release(unsigned *threadlockptr);     // You write this

void handleSVC(int code)
{
	int priv = 3;
  // NOTE: iprintf() is a bad idea inside an exception
  // handler (exception handlers should be small and short).
  // But this is the easiest way to show we got it right.
  switch (code & 0xFF) {
    case 76:
	  iprintf("Force a Systick Interrupt\r\n");
	  NVIC_ST_CURRENT_R = 1;
	  break;
	 
	case 24:
	  
	  asm volatile(
				  "MRS %0,CONTROL\n": "=r" (priv) : 
				  );
	  if((priv&0x1) == 1)
	  {
		  //Change to privileged
		  asm volatile(
			  "MRS R0, CONTROL\n"
			  "AND R0, R0, #0xFE\n"
			  "MSR CONTROL, R0\n"
			  "ISB"
			  );
			  
		  //Confirm Change Occur
		  asm volatile(
			  "MRS %0,CONTROL\n": "=r" (priv) : 
			  );
		  if((priv&0x1) == 0){
			  iprintf("Unprivileged. Setting to Privileged\r\n");
		  }else{
			  iprintf("No Change\r\n");
		  }
	  }
	  else
	  {
		  //Change to unprivileged
		  asm volatile(
				  "MRS R0, CONTROL\n"
				  "ORR R0, R0, #1\n"
				  "MSR CONTROL, R0\n"
				  "ISB"
				  );
		  //Confirm change occurred
		  asm volatile(
			  "MRS %0,CONTROL\n": "=r" (priv) : 
			  );
		  if((priv&0x1) == 1){
			  iprintf("Privileged. Setting to Unprivileged\r\n");
		  }else{
			  iprintf("No Change\r\n");
		  }
	  }
	  
	  break;

    default:
      iprintf("UNKNOWN SVC CALL\r\n");
      break;
  }
}

void SVCHandler(void){
	int code;
	asm volatile(
				"LDR R1, [R13, #24]\n"
				"SUB R1, R1, #2\n"
				"LDRH R3, [R1]\n"
				"MOV R0, R3" : "=r" (code) :
				);
				
	handleSVC(code);
}
unsigned threadlock;
int lock_count = 0;

// These are functions you have to write. Right now they are do-nothing stubs.
void lock_init(unsigned *lock)
{
	//set to default unused state
  *lock=1;
}

void UART_Lock(void){
	lock_count++;
}

void UART_Unlock(void){
	lock_count--;
}



extern unsigned lock_acquire(unsigned *lock) __attribute__((naked));
unsigned lock_acquire(unsigned *lock)
{
  asm volatile(
				"MOV r1, #0 \n"
				"LDREX r2, [r0]\n"			//Load in the lock value
				"CMP r2, r1\n"				//Check to see if th lock == 0
				"ITT NE\n"
				"STREXNE r2, r1, [r0]\n"	//If the lock is != 0, available, write a 0 to claim
				"CMPNE r2, #1\n"			//Check if the lock is still 1
				"BEQ 1f\n"					//If so, jump to failed return path 
				"MOV R0, #1\n"				//If successful return a 1
				"BX LR\n"
				"1:\n"						//Failed return path
				"CLREX\n"					//Clear exclusive access
				"MOV R0, #0\n"				//return a 0
				"BX LR\n");
}

void lock_release(unsigned *lock)
{
	if(lock_count==0)
		*lock=1;
}
typedef struct {
  int active;       // non-zero means thread is allowed to run
  char *stack;      // pointer to TOP of stack (highest memory location)
  int state[10];    // saved state for longjmp()
} threadStruct_t;

// thread_t is a pointer to function with no parameters and
// no return value...i.e., a user-space thread.
typedef void (*thread_t)(void);

// These are the external user-space threads. In this program, we create
// the threads statically by placing their function addresses in
// threadTable[]. A more realistic kernel will allow dynamic creation
// and termination of threads.
extern void UART_thread1(void);
extern void UART_thread2(void);
extern void OLED_thread(void);
extern void LED_thread(void);
extern void Buzzer_thread(void);

/* BUZZER REMOVED */
static thread_t threadTable[] = {
  UART_thread1,
  UART_thread2,
  OLED_thread,
  LED_thread
};
#define NUM_THREADS (sizeof(threadTable)/sizeof(threadTable[0]))

// These static global variables are used in scheduler(), in
// the yield() function, and in threadStarter()
static jmp_buf scheduler_buf;   // saves the state of the scheduler
static threadStruct_t threads[NUM_THREADS]; // the thread table
unsigned currThread;    // The currently active thread

extern int register_save(int *buffer, void *initial_stack) __attribute__((naked));
int register_save(int *buffer, void *initial_stack)
{
	asm volatile (
				"mov r12, %[initial_stack]" : [initial_stack] "=r" (initial_stack));
	asm volatile (
				"stmea r0!, {r4-r11, r12, lr }\n"
				"mov r0, #0\n"
				"bx lr");
}

extern int register_load(int *buffer, void *initial_stack) __attribute__((naked));
int register_load(int *buffer, void *initial_stack)
{
	asm volatile (
				"ldmea r0!,{r4-r11, r12, lr }\n"
				"mov r13, r12\n"
				"mov r0, #0\n"
				"bx lr");
}

// This function is called from within user thread context. It executes
// a jump back to the scheduler. When the scheduler returns here, it acts
// like a standard function return back to the caller of yield().
void yield(void)
{
  if (register_save(threads[currThread].state,threads[currThread].stack) == 0) {
    // yield() called from the thread, jump to scheduler context
    //register_load(scheduler_buf);
	asm volatile ("svc #76"); //fire interrupt
  } else {
    // longjmp called from scheduler, return to thread context
    return;
  }
}
void systick_setup(void){
	//disable timer
	NVIC_ST_RELOAD_R = 0;
	
	//Configure and enable
	//bit0 = 1 counter operate in multi-shot manner
	//bit1 = 1 no exception 
	//bit2 = 1 core clock
	NVIC_ST_CTRL_R = 0x7;
	
	//reload register (1 ms)
	NVIC_ST_RELOAD_R = (1000 * 8000);
}
void systick_isr(void){
	//save state of current thread on the array of 10 elements (buf) for that thread
	register_save(threads[currThread].state, threads[currThread].stack);
	//Identify the next active thread
	currThread=(currThread+1)%NUM_THREADS;
	//restore the state of the next thread from the array of 10 elements 
	register_load(threads[currThread].state, threads[currThread].stack);
	//fake a return from the handler to use thread mode and process stack (i.e use the LR register)
	asm volatile(
				"movw r0, 0xFFFd\n" 
				"AND lr, r0\n"
				"bx lr");
}

// This is the starting point for all threads. It runs in user thread
// context using the thread-specific stack. The address of this function
// is saved by createThread() in the LR field of the jump buffer so that
// the first time the scheduler() does a longjmp() to the thread, we
// start here.
void threadStarter(void)
{
	
	int j;
	//Fill stack with sentinel values
	for(j = 0; j < STACK_SIZE; j++){
		*(threads[currThread].stack - j)=0xFF;
	}
  // Call the entry point for this thread. The next line returns
  // only when the thread exits.
  (*(threadTable[currThread]))();
  
		for(j=STACK_SIZE-1; j>=0; j--){
			if(*(threads[currThread].stack-j)!=0xFF)
				break;
		}
		iprintf("Free Space (bytes)= %d\r\n",(STACK_SIZE-j));

  // Do thread-specific cleanup tasks. Currently, this just means marking
  // the thread as inactive. Do NOT free the stack here because we're
  // still using it! Remember, this function runs in user thread context.
  threads[currThread].active = 0;

  // This yield returns to the scheduler and never returns back since
  // the scheduler identifies the thread as inactive.
  yield();
}

// This function is implemented in assembly language. It sets up the
// initial jump-buffer (as would setjmp()) but with our own values
// for the stack (passed to createThread()) and LR (always set to
// threadStarter() for each thread).
extern void createThread(jmp_buf buf, char *stack);

// This is the "main loop" of the program.
/*void scheduler(void)
{
  unsigned i;

  currThread = -1;
  
  do {
    // It's kinda inefficient to call setjmp() every time through this
    // loop, huh? I'm sure your code will be better.
    if (register_save(scheduler_buf)==0) {

      // We saved the state of the scheduler, now find the next
      // runnable thread in round-robin fashion. The 'i' variable
      // keeps track of how many runnable threads there are. If we
      // make a pass through threads[] and all threads are inactive,
      // then 'i' will become 0 and we can exit the entire program.
      i = NUM_THREADS;
      do {
        // Round-robin scheduler
        if (++currThread == NUM_THREADS) {
          currThread = 0;
        }

        if (threads[currThread].active) {
          register_load(threads[currThread].state);
        } else {
          i--;
        }
      } while (i > 0);

      // No active threads left. Leave the scheduler, hence the program.
      return;

    } else {
      // yield() returns here. Did the thread that just yielded to us exit? If
      // so, clean up its entry in the thread table.
      if (! threads[currThread].active) {
        free(threads[currThread].stack - STACK_SIZE);
      }
    }
  } while (1);
}*/

void main(void)
{
  unsigned i;
  unsigned long ulPeriod;

  // Set the clocking to run directly from the crystal.
  SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN |
                 SYSCTL_XTAL_8MHZ);

  // Initialize the OLED display and write status.
  RIT128x96x4Init(1000000);
  RIT128x96x4StringDraw("Scheduler Demo",       20,  0, 15);

  // Enable the peripherals used by this example.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

  // Set GPIO A0 and A1 as UART pins.
  GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

  // Configure the UART for 115,200, 8-N-1 operation.
  UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 115200,
                      (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                       UART_CONFIG_PAR_NONE));
					   
	// Enable the GPIO port that is used for the on-board LED.
    //
    SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOF;
	//
	// Enable the GPIO pin for the LED (PF0).  Set the direction as output, and
	// enable the GPIO pin for digital function.
	//
	GPIO_PORTF_DIR_R = 0x01;
	GPIO_PORTF_DEN_R = 0x01;
	
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

	GPIOPinTypePWM(GPIO_PORTD_BASE, GPIO_PIN_1);
	
	//
    // Compute the PWM period based on the system clock.
    //
    ulPeriod = SysCtlClockGet() / 220;
	
	PWMGenConfigure(PWM_BASE, PWM_GEN_0,
                    PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    PWMGenPeriodSet(PWM_BASE, PWM_GEN_0, ulPeriod);
	
	PWMPulseWidthSet(PWM_BASE, PWM_OUT_1, ulPeriod * 3 / 4);
	
	//
    // Enable the PWM0 and PWM1 output signals.
    //
    PWMOutputState(PWM_BASE,PWM_OUT_1_BIT, false);

    //
    // Enable the PWM generator.
    //
    PWMGenEnable(PWM_BASE, PWM_GEN_0);
	
	
	
  // Create all the threads and allocate a stack for each one
  for (i=0; i < NUM_THREADS; i++) {
    // Mark thread as runnable
    threads[i].active = 1;

    // Allocate stack
    threads[i].stack = (char *)malloc(STACK_SIZE) + STACK_SIZE;
    if (threads[i].stack == 0) {
      iprintf("Out of memory\r\n");
      exit(1);
    }

    // After createThread() executes, we can execute a longjmp()
    // to threads[i].state and the thread will begin execution
    // at threadStarter() with its own stack.
    createThread(threads[i].state, threads[i].stack);
  }

  // Initialize the global thread lock
  lock_init(&threadlock);

  iprintf("Setup Systick and Enable Interrupts\r\n");
	  //setup systick
	  systick_setup();
	  //Enable all interrupts
      IntMasterEnable();
	  
  // Start running coroutines
  //scheduler();
	asm volatile ("svc #76");
	while(1){
		;
	}
  // If scheduler() returns, all coroutines are inactive and we return
  // from main() hence exit() should be called implicitly (according to
  // ANSI C). However, TI's startup_gcc.c code (ResetISR) does not
  // call exit() so we do it manually.
  exit(0);
}

/*
 * Compile with:
 * ${CC} -o lockdemo.elf -I${STELLARISWARE} -L${STELLARISWARE}/driverlib/gcc 
 *     -Tlinkscript.x -Wl,-Map,lockdemo.map -Wl,--entry,ResetISR 
 *     lockdemo.c create.S threads.c startup_gcc.c syscalls.c rit128x96x4.c 
 *     -ldriver
 */
// vim: expandtab ts=2 sw=2 cindent
