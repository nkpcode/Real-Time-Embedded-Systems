#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void){
  msTicks++;
}

// initialize the system tick
void init_systick(void){
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}

// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n) {
  uint32_t msTicks2 = msTicks + n;
  while(msTicks < msTicks2) ;
}


void init_LED_pins()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD

  //GPIOD->MODER &= ~(0x3 << (2*12)); // clear the 2 bits corresponding to pin 12
 GPIOD->MODER &= ~(0xff << (2*12)); // clear the 8 bits corresponding to pins 12,13,14,15
  
 GPIOD->MODER |= (0x55 << (2*12)); // set pins 12, 13, 14, 15, as Gerneral purpose output
//	GPIOD->MODER |= (1 << (2*12));    // set pin 12 to be general purpose output
  // to enable the other pins (13,14,15), you will need to write to the appropriate bits of the ~MODER~ register.
}


void LED_On(uint32_t i);// Fucntion Prototype
void LED_On(uint32_t i)
{
	/* Take precaution of strange entries */
	if(i > 3)
	{
		i = 3;
	}
	else if(i < 0)
	{
		i = 0;
	}

	/* i = 0, 1, 2, 3 are connected to pins 12, 13, 14, and 15
	   Setting the respective BSRRL bit to 1 if */

	GPIOD->BSRRL |= (0x1 << (12 + i));
} 

void LED_Off(uint32_t i);// Fucntion Prototype
void LED_Off(uint32_t i)
{
	/* Take precaution of strange entries */
	if(i > 3)
	{
		i = 3;
	}
	else if(i < 0)
	{
		i = 0;
	}

	/* i = 0, 1, 2, 3 are connected to pins 12, 13, 14, and 15
	   Setting the respective BSRRL bit to 1 if */

	GPIOD->BSRRH |= (0x1 << (12 + i));
} 

uint32_t read_button(void);
uint32_t read_button()
{
	return (0x0001 & GPIOA->IDR);
}

void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA

  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
}


int main(void)
{
  // initialize
  SystemInit();
  init_systick();
  init_LED_pins();
  init_button();


  while (1)
    {

    }
}




