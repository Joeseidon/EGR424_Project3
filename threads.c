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

void OLED_thread(void){
	static int count = 0;
	static int onOff = 1;
	while(1){
		count++;
		if(count == 150){
			RIT128x96x4Init(1000000);
			if(onOff){
				RIT128x96x4Clear();
				onOff = 0;
			}else{
				RIT128x96x4StringDraw("Hello World!", 30, 24, 15);
				onOff = 1;
			}
			count = 0;
		}else{
			yield();
		}
	}
	yield();
}


void LED_thread(void){
	static int count = 0;
	while(1){
		count++;
		if(count==50){
			GPIO_PORTF_DATA_R ^= 0x01;
			count = 0;
		}else{
			yield();
		}
	}
	yield();
}

void Buzzer_thread(void){
	static int count = 0;
	static int onOff = 1;
	while(1){
		count++;
		if(count==50){
			if(onOff){
				PWMOutputState(PWM_BASE,PWM_OUT_1_BIT, false);
			}else{
				PWMOutputState(PWM_BASE,PWM_OUT_1_BIT, true);
			}
			onOff= (onOff==0)? 1:0;
			count=0;
		}else{
			yield();
		}
	}
	yield();
}