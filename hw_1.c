#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>


static int a = 0;

int f(int a1, int a2)
{
  int b = (a1+a2);
  a ++;
  return b;
}

int g(int a1);

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

void blink_LED()
{
  unsigned int pin = 13;		  // pin 13 is the orange LED
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD
  GPIOD->MODER = (1 << 2*pin);          // set pin 13 to be general purpose output

  while (1) {
    delay_ms(1000);
    GPIOD->ODR ^= (1 << pin);           // Toggle pin 
    setbuf(stdout, NULL);
    printf("Toggling LED state\r\n");
  }
}

int main(void)
{
  // initialize
  SystemInit();
  init_systick();

  int k1 = f(5,6);
  int k2 = g(5);

  blink_LED();
}




