#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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


// This method attempt to grad a lock for the uart terminal.
// If successful, the function prints a message then relases
// the lock.
void UART_thread1(void)
{
  while (1) {
    if (lock_acquire(&threadlock)) {
		UART_Lock();
		iprintf("Thread 1 printing\r\n");
		UART_Unlock();

      lock_release(&threadlock);
    }
    yield();
  }
}

// This method attempt to grad a lock for the uart terminal.
// If successful, the function prints a message then relases
// the lock.
void UART_thread2(void)
{
  while (1) {
    if (lock_acquire(&threadlock)) {
		UART_Lock();
      iprintf("Thread 2 printing\r\n");
	  UART_Unlock();
      lock_release(&threadlock);
    }
    yield();
  }
}

// This method prints and clears a message on the OLED as 
// many times as it can before the scheduler takes control
void OLED_thread(void){
	volatile unsigned long count = 0;
	while(1){
		for(count = 0; count < 200000; count++);
		RIT128x96x4StringDraw("            ", 30, 24, 15);
		for(count = 0; count < 200000; count++);
		RIT128x96x4StringDraw("Hello World!", 30, 24, 15);
	}
}

// This method blinks an LED as many times as it can before
// its prempted by the scheduler.
void LED_thread(void){
	volatile unsigned long count = 0;
	while(1){
		for(count = 0; count < 200000; count++);
		GPIO_PORTF_DATA_R ^= 0x01;
	}
}

// This method beeps the onboard buzzer
void Buzzer_thread(void){
	volatile unsigned long count = 0;
	while(1){
		for(count = 0; count < 100000; count++);
		PWMOutputState(PWM_BASE,PWM_OUT_1_BIT, true);
		for(count = 0; count < 100000; count++);
		PWMOutputState(PWM_BASE,PWM_OUT_1_BIT, false);
		yield();
	}
}