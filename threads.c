#include <stdio.h>
#include "scheduler.h"

void UART_thread1(void)
{
  while (1) {
    if (lock_acquire(&threadlock)) {
      // Simulate code that is occasionally interrupted
      iprintf("THIS IS T");
          yield(); // context switch "interrupt"
      iprintf("HREAD NU");
          yield(); // context switch "interrupt"
      iprintf("MBER 1\r\n");

      lock_release(&threadlock);
    }
    yield();
  }
}

void UART_thread2(void)
{
  while (1) {
    if (lock_acquire(&threadlock)) {
      // Simulate code that is occasionally interrupted
      iprintf("this is t");
          yield(); // context switch "interrupt"
      iprintf("hread number 2\r\n");

      lock_release(&threadlock);
    }
    yield();
  }
}

void OLED_thread(void){
	while(1){
		;
	}
	yield();
}


void LED_thread(void){
	while(1){
		;
	}
	yield();
}

void Buzzer_thread(void){
	while(1){
		;
	}
	yield();
}