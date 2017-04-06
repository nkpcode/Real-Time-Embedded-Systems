#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks
static volatile int LED_0_State;
static volatile int LED_1_State;
static volatile int LED_2_State;
static volatile int LED_3_State;

static volatile int global_state_pin;/* Assuming the button is released when powered up */
static volatile int global_counter_1;
static volatile int global_counter_2;
static volatile int global_CurrentState;
static volatile int global_NextState;
static volatile int global_TimerEvent;

static volatile int global_CurrentState_2;
static volatile int global_NextState_2;

//float NUMBERTOBECONVERTED = 23.45;

#define ButtonIsPressed  1
#define ButtonIsReleased  0
#define ButtonPressed 1
#define ButtonReleased 0


//int floor(float x);
//typedef int (*func)(float);
//int floor(float x);
int floor(float x);

typedef void* (*LED_toggle_pointer)(void);

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


void toggle_LED0();
void toggle_LED0()
{
	/* Toggle LED 0 (pin 12) using LED_on and LED_off */
		
#if 0
	/*+++ Try this later +++*/
	uint8_t state_pin = 0;
	state_pin = GPIO_ReadInputDataBit(GPIOD, GPIO_Pin_12);
	
	if (state_pin == 0)
	{
		/* Turn on LED_0 */		
		LED_On(12);
	}

	else if(state_pin == 1)
	{
		/* Turn off LED_0 */
		LED_Off(12);
	}

#endif


	if (LED_0_State == 0)
	{
		/* Turn on LED_0 */		
		LED_On(0);
		LED_0_State = 1;
	}

	else if(LED_0_State == 1)
	{
		/* Turn off LED_0 */
		LED_Off(0);
		LED_0_State = 0;
	}
}

void toggle_LED1();
void toggle_LED1()
{
	/* Toggle LED 1 (pin 13) using LED_On and LED_Off */

	if (LED_1_State == 0)
	{
		/* Turn on LED_1 */		
		LED_On(1);
		LED_1_State = 1;
	}

	else if(LED_1_State == 1)
	{
		/* Turn off LED_1 */
		LED_Off(1);
		LED_1_State = 0;
	}
}

void toggle_LED2();
void toggle_LED2()
{
	/* Toggle LED 2 (pin 14) using LED_On and LED_Off */

	if (LED_2_State == 0)
	{
		/* Turn on LED_2 */		
		LED_On(2);
		LED_2_State = 1;
	}

	else if(LED_2_State == 1)
	{
		/* Turn off LED_2 */
		LED_Off(2);
		LED_2_State = 0;
	}
}

void toggle_LED3();
void toggle_LED3()
{
	/* Toggle LED 3 (pin 15) using LED_On and LED_Off */

	if (LED_3_State == 0)
	{
		/* Turn on LED_3 */		
		LED_On(3);
		LED_3_State = 1;
	}

	else if(LED_3_State == 1)
	{
		/* Turn off LED_3 */
		LED_Off(3);
		LED_3_State = 0;
	}
}


struct TimedTask
{
	LED_toggle_pointer function;
	float rep_interval;
	uint32_t last_call;
};

struct TimedTask timed_tasks[4] = 
{
	{
	  toggle_LED0,
	  0.5,/* 0.5 * 1000 */
	  0
	},

	{
	  toggle_LED1,
	  0.25,/* 0.25 * 1000 */
	  0
	},

	{
	  toggle_LED2,
	  1.0,/* 1.0 * 1000 */
	  0
	},

	{
	  toggle_LED3,
	  2.0,/* 2.0 * 2000 */
	  0
	},

};

void print_times();

LED_toggle_pointer print_t = print_times;
void print_times()
{
  	int i = 0;
	while(i <= 3)
	{
	  
	  printf("toggle_LED%d has been called %d\n",i,timed_tasks[i].last_call);

	  i++;
	}

}


struct TimedTask print_values =
{
  print_times,
  5.0,/* 2.0 * 2000 */
  0
};


void print_2(void);
void print_2()
{
	printf("Button pressed \n");
}


void print_3(void);
void print_3()
{
	printf("Button released \n");
}


/* timed_tasks[0].function()  */

void add_timed_task(LED_toggle_pointer function, float interval);
void add_timed_task(LED_toggle_pointer function, float interval)
{
	float converted_interval = interval * 1000;
	//float converted_interval = interval;	
	int i = 0;
	int get_time_factor = 0;
	int time_factor = 0;

#if 1   
	if(function == print_values.function)
	{/* Execute the print function */
		get_time_factor = print_values.last_call;
		time_factor = get_time_factor + 1;
		if(msTicks == (converted_interval * time_factor))
		{ /* execute the function call only at the respective time */ 
		  function();/* for print_times */
		  print_values.last_call++;	  
		}
	}

#endif	

	else
	{
		while(i <= 3)
		{

		  if (timed_tasks[i].function == function)
		    {
		      get_time_factor =  timed_tasks[i].last_call;
		      
		      break;
		    }
		 i++;
		}
	
		time_factor = get_time_factor + 1;
		 

		if(msTicks == (converted_interval * time_factor))
		{ /* execute the function call only at the respective time */ 
		  function();/* Invoke the function */
		  i = 0;
		  while(i <= 3)
		  {
		  
		    if (timed_tasks[i].function == function)
		    {
		      timed_tasks[i].last_call++ ;
		      break;
		    }

		    i++;
	  
		  }	  
		}
	}
	
}

uint32_t read_button(void);
uint32_t read_button()
{
	return (0x0001 & GPIOA->IDR);
}


// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void){
  msTicks++;
  
  

	/* To monitor if the button is pressed or release and to generate a timer event accordingly */
  global_CurrentState = global_NextState;
  switch (global_CurrentState) 
  {	
		case ButtonIsPressed:
			global_state_pin = read_button();
			if(global_state_pin == 0)
			{/* To indicate button is released */
				global_counter_1 = global_counter_1 + 1;
			}			
			if(global_counter_1 == 250)
			{/* Ready to transition into the ButtonIsReleased state */
				global_TimerEvent = ButtonReleased;
				global_NextState = ButtonIsReleased;
				global_counter_1 = 0;/* Reset counter */
			} 
			break;
	
		
		case ButtonIsReleased:
			global_state_pin = read_button();
			if(global_state_pin == 1)
			{/* To indicate button is pressed */
				global_counter_1 = global_counter_1 + 1;
			}	
			if(global_counter_1 == 250)
			{/* Ready to transition into the ButtonIsPressed state */
				global_TimerEvent = ButtonPressed;
				global_NextState = ButtonIsPressed;
				global_counter_1 = 0;/* Reset counter */
			} 
			break;
  }


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




void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA

  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
}

void init_LED_states();
void init_LED_states()
{
	LED_0_State = 0;/* Assuming the pin will be low voltage at the start */
	LED_1_State = 0;/* Assuming the pin will be low voltage at the start */
	LED_2_State = 0;/* Assuming the pin will be low voltage at the start */
	LED_3_State = 0;/* Assuming the pin will be low voltage at the start */
}

init_counter_states();
init_counter_states()
{
	global_state_pin = 0;/* Assuming the button is released when powered up */
	global_counter_1 = 0;
	global_counter_2 = 0;
	global_CurrentState = ButtonIsReleased;/* Assuming the button is released when powered up */
	global_NextState = ButtonIsReleased;/* Assuming the button is released when powered up */
	global_TimerEvent = ButtonReleased;

	global_CurrentState_2	 = 	ButtonReleased;
	global_NextState_2 = ButtonReleased;
}


void question_2(void);
void question_2(void)
{
	global_CurrentState_2 = global_NextState_2;
	switch (global_CurrentState_2) 
	{	
		case ButtonPressed:
			if(global_TimerEvent == ButtonReleased)
			{
				global_NextState_2 = global_TimerEvent;
				print_3();
			}
/*					else
			{
				//printf("Button released %d  %d\n",global_counter_1,msTicks);
				print_2();
				
			}*/
			break;


		case ButtonReleased:
			if(global_TimerEvent == ButtonPressed)
			{
				global_NextState_2 = global_TimerEvent;
				print_2();
			}
/*					else
			{
				//printf("Button pressed %d  %d\n",global_counter_1,msTicks);
				print_3();
			}*/
			break;
	}

}

void question_1(void);
void question_1(void)
{
	delay_ms(10);
	add_timed_task(timed_tasks[0].function, timed_tasks[0].rep_interval);
	//add_timed_task(timed_tasks[0].function, 0.5);	
	add_timed_task(timed_tasks[1].function, timed_tasks[1].rep_interval);
	add_timed_task(timed_tasks[2].function, timed_tasks[2].rep_interval);
	add_timed_task(timed_tasks[3].function, timed_tasks[3].rep_interval);
	add_timed_task(print_t, 5);
	//printf("Button pressed %d  %d\n",global_counter_1,msTicks);
	//puts("B");
}

int y = 0;

int main(void)
{
  // initialize
  SystemInit();
  init_systick();
  init_LED_pins();
  init_button();
  init_LED_states();
  init_counter_states();
	float z = 1.5;	
	y = floor(z);
	

  while (1)
  {

		question_2();
		question_1();

	}

}




