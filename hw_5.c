#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <math.h>

#include "accelerometers/accelerometers.h"
#include "temperature/temperature.h"
#include "RNG/random_number_generator.h"

#define ButtonIsPressed  1
#define ButtonIsReleased  0

#define ButtonReleased 0
#define ButtonPressed 1
#define LongPause_begin 2
#define LongPause 3
#define WaitState 4

#if 0

static uint32_t LongPress;
static uint32_t ShortPress;
static uint32_t Pause;
static uint32_t NoChar;

#endif

#define LongPress 1
#define ShortPress 2
#define Pause 3
#define NoChar 4

#define COUNTERPEAK 250/*250*/
#define COUNTERPEAK2 1500/*4*/
#define COUNTERPEAK3 5000/*5000*/

#define MAXLENGTHMORSE 5 /* four is the max length */
#define ALPHABETSTOT 26

#define NUMBEROFLETTERFUNCT 4

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

/* global varibles being used */
static volatile uint32_t global_counter;
static volatile uint32_t global_CurrentState;
static volatile uint32_t global_NextState;
static volatile uint32_t global_TimerEvent;
static volatile uint32_t keep_msTicks_BReleased;
static volatile uint32_t keep_msTicks_BPressed_start;
static volatile uint32_t keep_msTicks_BPressed;
static volatile uint32_t pause_msTicks_BReleased;
static volatile int32_t global_difference_msTicks;
static volatile uint32_t global_state_pin;
static volatile uint32_t global_counter_1;
static volatile uint32_t global_CurrentState_2;
static volatile uint32_t global_NextState_2;
static volatile uint32_t global_PrevState_2;
int global_flag;
int global_flag_full;
char update_character;

static int global_signal[MAXLENGTHMORSE];
static int global_signal_index;

struct JUMP 
{
	int signal[MAXLENGTHMORSE];
	char letter;
};

//jumptable alphabet[ALPHABETSTOT];
struct JUMP alphabet[ALPHABETSTOT];

typedef struct Z 
{
	int signal[MAXLENGTHMORSE];
	char letter;
}z;

typedef void (*functionPointer)(void);

char array_letter[NUMBEROFLETTERFUNCT];

void (*array_functions[NUMBEROFLETTERFUNCT])(void);


uint32_t read_button(void);
uint32_t read_button()
{
	return (0x0001 & GPIOA->IDR);
}

void accelerometer_print(void);
void accelerometer_print(void)
{
	float a[3]; // array of 3 floats into which accelerometer data will be read
	read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
	printf("%f %f %f\n", a[0], a[1], a[2]);
	float pitch, roll;
	calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
	printf("Pitch = %f, Roll = %f\n", pitch, roll);
}

void get_msTicks(void)
{
  uint32_t r = msTicks;
  printf("msTicks = %lu\n",r);

}

void read_temperature_sensor_function(void)
{/* Print temperature */
  float temp = read_temperature_sensor();
  printf("Temp = %f\n",temp);
}

void get_random_number_function(void)
{/* Print random number */
  uint32_t r = get_random_number();
  printf("Random = %lu\n",r); // print unsigned int
}



void initialize_alphaMorse(void);
void initialize_alphaMorse(void)
{/* Initialize alphabets and its Morse equivalent */

#if 1
	struct JUMP x[ALPHABETSTOT] =

		{
			{{ShortPress, LongPress, Pause, NoChar, NoChar},'A'},/* A */
			{{LongPress, ShortPress, ShortPress, ShortPress, Pause},'B'},/* B */
			{{LongPress, ShortPress, LongPress, ShortPress, Pause},'C'},/* C */
			{{LongPress, ShortPress, ShortPress, Pause, NoChar},'D'},/* D */
			{{ShortPress, Pause, NoChar, NoChar, NoChar},'E'},/* E */

			{{ShortPress, ShortPress, LongPress, ShortPress, Pause},'F'},/* F */
			{{LongPress, LongPress, ShortPress, Pause, NoChar},'G'},/* G */
			{{ShortPress, ShortPress, ShortPress, ShortPress, Pause},'H'},/* H */
			{{ShortPress, ShortPress, Pause, NoChar, NoChar},'I'},/* I */
			{{ShortPress, LongPress, LongPress, LongPress, Pause},'J'},/* J */

			{{LongPress, ShortPress, LongPress, Pause, NoChar},'K'},/* K */
			{{ShortPress, LongPress, ShortPress, ShortPress, Pause},'L'},/* L */
			{{LongPress, LongPress, Pause, NoChar, NoChar},'M'},/* M */
			{{LongPress, ShortPress, Pause, NoChar, NoChar},'N'},/* N */
			{{LongPress, LongPress, LongPress, Pause, NoChar},'O'},/* O */

			{{ShortPress, LongPress, LongPress, ShortPress, Pause},'P'},/* P */
			{{LongPress, LongPress, ShortPress, LongPress, Pause},'Q'},/* Q */
			{{ShortPress, LongPress, ShortPress, Pause, NoChar},'R'},/* R */
			{{ShortPress, ShortPress, ShortPress, Pause, NoChar},'S'},/* S */
			{{LongPress, Pause, NoChar, NoChar, NoChar},'T'},/* T */

			{{ShortPress, ShortPress, LongPress, Pause, NoChar},'U'},/* U */
			{{ShortPress, ShortPress, ShortPress, LongPress, Pause},'V'},/* V */
			{{ShortPress, LongPress, LongPress, Pause, NoChar},'W'},/* W */
			{{LongPress, ShortPress, ShortPress, LongPress, Pause},'X'},/* X */
			{{LongPress, ShortPress, LongPress, LongPress, Pause},'Y'},/* Y */

			{{LongPress, LongPress, ShortPress, ShortPress, Pause},'Z'}/* Z */

		};

		int i; 
		for(i = 0; i < ALPHABETSTOT; i++)
		{
			alphabet[i] = x[i];
		}
		
	
#endif

#if 0
		int alphabets[] = {1,2,3,4,5};
		z x[] = {{0,1,2,3,4},'S'};
		x[0].letter = 'y';
	
#endif
}

void intialize_arrays(void);
void intialize_arrays(void)
{
		/* letter array */
		char y[NUMBEROFLETTERFUNCT] = {'A','M','T','R'};
		
		int i; 
		for(i = 0; i < NUMBEROFLETTERFUNCT; i++)
		{
			array_letter[i] = y[i];
		}
		
		/* Function array */

		functionPointer z[NUMBEROFLETTERFUNCT] = 
		{
			accelerometer_print, /* A */
			get_msTicks,/* M */
			read_temperature_sensor_function, /* T */
			get_random_number_function /* R */
		};
																					
		for(i = 0; i < NUMBEROFLETTERFUNCT; i++)
		{
			array_functions[i] = z[i];
		}

}



void fillWithNoChar(int *arr);
void fillWithNoChar(int *arr)
{/* Flush out old signal */
	int i; 
	for(i = 0; i < MAXLENGTHMORSE; i++)
	{
		*(arr + i) = NoChar;
	}
	global_signal_index = 0; 
	global_flag_full = 0;/* To indicate space is available */
}

void init_globals(void);
void init_globals(void)
{

	global_counter = 0;
	global_counter_1 = 0;
	global_CurrentState = ButtonIsReleased;
	global_NextState = ButtonIsReleased;
	global_TimerEvent = ButtonReleased;
	keep_msTicks_BReleased = 0;
	keep_msTicks_BPressed_start = 0;
	keep_msTicks_BPressed = 0;
	pause_msTicks_BReleased = 0;

	global_difference_msTicks = 0;
	global_CurrentState_2 = ButtonReleased;
	global_NextState_2 = ButtonReleased;
	global_PrevState_2 = ButtonReleased;
	global_flag = 1;

	fillWithNoChar(global_signal);
	global_signal_index = 0;

	global_flag_full = 0;

	update_character = ' ';

	initialize_alphaMorse();
} 


void ButtonFSM(void);
void ButtonFSM(void)
{
	/* To monitor if the button is pressed or release and to generate a timer event accordingly */
  global_CurrentState = global_NextState;
  switch (global_CurrentState) 
  {	
		case ButtonIsPressed:
			global_state_pin = read_button();
			keep_msTicks_BPressed = msTicks;/* Keep track of msTicks */
			if(global_state_pin == 0)
			{/* To indicate button is released */
				global_counter_1 = global_counter_1 + 1;
			}			
			if(global_counter_1 == COUNTERPEAK)
			{ 
				/* Ready to transition into the ButtonIsReleased state */
				/* Keep record of the time ButtonIsReleased event is created */

				global_NextState_2 = ButtonReleased;
				global_PrevState_2 = ButtonPressed;
				global_NextState = ButtonIsReleased;
				global_counter_1 = 0;/* Reset counter */

				keep_msTicks_BReleased = msTicks;/* Keep track of msTicks */
			}
			global_flag = 1; 
			break;
	
		
		case ButtonIsReleased:
			global_state_pin = read_button();
			keep_msTicks_BReleased = msTicks;/* Keep updating this as long as it is in this state */
			if(global_state_pin == 1)
			{/* To indicate button is pressed */
				global_counter_1 = global_counter_1 + 1;
			}	
			if(global_counter_1 == COUNTERPEAK)
			{/* Ready to transition into the ButtonIsPressed state */
				/* Keep record of the time ButtonIsPressed event is created */
				global_NextState_2 = ButtonPressed;
				global_PrevState_2 = ButtonReleased;
				global_NextState = ButtonIsPressed;
				global_counter_1 = 0;/* Reset counter */

				keep_msTicks_BPressed_start = msTicks;/* Keep track of msTicks */
			} 
			break;
  }

}

// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void)
{
  msTicks++;
	ButtonFSM();
}



// initialize the system tick
void init_systick(void)
{
	SystemCoreClockUpdate();                      /* Get Core Clock Frequency   */
  if (SysTick_Config(SystemCoreClock / 1000)) { /* SysTick 1 msec interrupts  */
    while (1);                                  /* Capture error              */
  }
}

// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n)
{
  uint32_t msTicks2 = msTicks + n;
  while(msTicks < msTicks2) ;
}



void init_LED_pins()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;  // enable clock to GPIOD
	int i = 0;

  for (i =12; i<=15; i++)
  {
    GPIOD->MODER &= ~(0x3 << (2*i)); // clear the 2 bits corresponding to pin i
    GPIOD->MODER |= (1 << (2*i));    // set pin i to be general purpose output
  }
}

void LED_On(uint32_t i)
{
  GPIOD->BSRRL = 1 << (i+12);
}

void LED_Off(uint32_t i)
{
  GPIOD->BSRRH = 1 << (i+12);
}


void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA

  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
}


void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
	*roll = (180.0/M_PI)*atan2(acc_y, acc_z);
	*pitch = (180.0/M_PI)*atan2(-acc_x, sqrt(acc_y*acc_y+acc_z*acc_z));
}




int checkifsignal_full(void);
int checkifsignal_full(void)
{
	return global_flag_full;
}


void addelement_Signal(int sig);
void addelement_Signal(int sig)
{/* Add element to create signal */
	
	if (sig == NoChar)
	{/* Clear signal */
		fillWithNoChar(global_signal);
	}
	global_signal[global_signal_index] = sig;
	global_signal_index = global_signal_index + 1;
	if (global_signal_index == MAXLENGTHMORSE)
	{
		global_flag_full = 1;
	}
	
}


char compareSignal(int *ch);
char compareSignal(int *ch)
{
	/* Function to compare the signal received and return the required
		 alphabets */
	int keepindex;	
	int j;
	int k;
	int found = 0;
	char letter = ' ';	

	for(k = 0; k < ALPHABETSTOT; k++)
	{	
		for (j = 0; j < (MAXLENGTHMORSE - 1); j++)
		{/* compare signal with alphabet's signal */
	
			if (alphabet[k].signal[j] != *(ch + j))
			{
				found = 0;
				break;/* break when not similar */			
			}	
			else
			{
				found = 1;
			}

			if (alphabet[k].signal[j] == Pause)
			{/* Break when Pause is reached */
					break;
			}
	
		}
		if(found == 1)
		{
			/* Found alphabet */
			letter = alphabet[k].letter;
			break; 	
		}
	}
	return letter;
}



void compareLetter(char Letter_Function);
void compareLetter(char Letter_Function)
{/* Find if 'T', 'R' or 'M' */
	
	int i;
	for(i = 0; i < NUMBEROFLETTERFUNCT; i++)
	{	
		if (array_letter[i] == Letter_Function)
		{
			array_functions[i]();/* execute function */
		}		
	}
	

}

void initialise_monitor_handles();

void question_2(void);
void question_2(void)
{
	global_CurrentState_2 = global_NextState_2;
	/*switch (global_TimerEvent) */
	switch (global_CurrentState_2)
	{	
		case ButtonPressed:
			//keep_msTicks_BPressed = msTicks;
			LED_On(0);/* Turn on LED 12 */
			LED_Off(1);
			LED_Off(2);
			LED_Off(3);
			break;


		case ButtonReleased:
			//keep_msTicks_BReleased = msTicks;
			LED_On(1);/* Turn on LED 12 */
			LED_Off(0);
			LED_Off(2);
			LED_Off(3);
			/* For detecting long pause */
			if((keep_msTicks_BReleased > keep_msTicks_BPressed_start) && (global_PrevState_2 == ButtonPressed))
			{
				global_difference_msTicks = keep_msTicks_BReleased - keep_msTicks_BPressed_start;

				/* Detect press duration */
				if((global_difference_msTicks >= COUNTERPEAK2))/* 1500 */
				{	/* Detect long duration press */

					printf("global_difference_msTicks = %d is more than 1.5 secs\n ",global_difference_msTicks);
					global_difference_msTicks = 0;
					addelement_Signal(LongPress);
					if (checkifsignal_full())
					{/* If end of signal space is reached change to state to LongPause to begin comparison */
						global_NextState_2 = LongPause;
					}
					else
					{
						global_NextState_2 = LongPause_begin;/* Stay in longpause state until next button press */
					}
				}
				else
				{/* Detecting short press */
					printf("global_difference_msTicks = %d is less than 1.5 secs\n ",global_difference_msTicks);
					global_difference_msTicks = 0;
					addelement_Signal(ShortPress);
					if (checkifsignal_full())
					{/* If end of signal space is reached change to state to LongPause to begin comparison */
						global_NextState_2 = LongPause;
					}
					else
					{
						global_NextState_2 = LongPause_begin;/* Stay in longpause state until next button press */
					}
				}	

			}
			break;

		case LongPause_begin:

			LED_On(2);/* Turn on LED 12 */
			LED_Off(1);
			LED_Off(0);
			LED_Off(3);	

			if(keep_msTicks_BReleased > keep_msTicks_BPressed)						
			{
				if(((keep_msTicks_BReleased - keep_msTicks_BPressed) > COUNTERPEAK3)) /*&& (global_flag == 1))*/
				{
					printf("long pause detected\n");
					addelement_Signal(Pause);

					global_NextState_2 = LongPause;

				}			
			}

			break;

		case LongPause:
			
			LED_On(3);/* Turn on LED 12 */
			LED_Off(1);
			LED_Off(0);
			LED_Off(2);		
			
			/* compare and get character from signal */
			update_character = compareSignal(global_signal);
			if(update_character != ' ')
			{/* print character */
				printf("character recognized: %c\n", update_character);
				compareLetter(update_character);
			}
			else
			{
				printf("Character not recognized\n");
			}
			global_NextState_2 = WaitState; 

			break;
		
		case WaitState:
			/* Do nothing */
			LED_On(3);/* Turn on LED 12 */
			LED_On(1);
			LED_On(0);
			LED_On(2);	
			/* Flush out global_signal */
			fillWithNoChar(global_signal);
			
			break;

	}

}

int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  init_systick();
  init_LED_pins();
  init_button();
  init_accelerometers(); // initialize accelerometers
  init_rng(); // initialize random number generator
  init_temperature_sensor();
	intialize_arrays();

	init_globals();
	char x = 'a';


  uint32_t t_prev = 0;
  while (1)
	{
		/* State machine to keep track of global difference */	
		question_2();
		//printf("x= %c\n", x); 		



#if 0
    if ( (msTicks - t_prev) >= 1000) // 1 second has elapsed
    {
      t_prev = msTicks;
      float a[3]; // array of 3 floats into which accelerometer data will be read
      read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
      printf("%f %f %f\n", a[0], a[1], a[2]);
      float pitch, roll;
      calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
      printf("Pitch = %f, Roll = %f\n", pitch, roll);

      float temp = read_temperature_sensor();
      printf("Temp = %f\n",temp);

      uint32_t r = get_random_number();
      printf("Random = %lu\n",r); // print unsigned int
    }

#endif

  }


}




