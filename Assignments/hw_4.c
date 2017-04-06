#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"

#include <stdio.h>
#include <math.h>

#include "accelerometers/accelerometers.h"


#define DetectingStage1 0
#define DetectingStage2 1
#define DetectingStage3 2
#define CounterPeak 50

//#define CircularBufferLength 5
int CircularBufferLength = 5;
#define CircularBufferLength 5
#define Numberofstages 3

#define RAW /* define this for Question 1, remove it for Question 2 */

//#define C_Code_calc_average /* remove Define for Question 3 */

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks
static volatile uint32_t global_counter;
static volatile uint32_t global_currentState;
static volatile uint32_t global_nextState;
static volatile uint32_t global_last_call;
static volatile uint32_t global_last_call2;
static volatile uint32_t global_last_call3;
static volatile uint32_t nw;/* Circular buffer index keeper */

float global_avg[3];

float global_pitch;
float	global_roll;	


//typedef int*(*Update_with_int_ret)(float pitch, float roll);
typedef void*(*Update_with_int_ret)(void);
typedef void*(*CALC_AVG)(float *avg_x, float *avg_y, float *avg_z);

uint32_t printFloor(float x);
/* For function definition look in funcs1.s */

#if 0
printFloor:

	PUSH {LR}

	/* Convert 32 bit single precision floating point to unsigned 32 bit
	integer */

	VCVT.f32.s32 S0,S0
	VMOV R0, S0 /* This value is returned */

	POP {LR}
	BX LR /* Return from function */	
#endif
/*
uint32_t printFloor(float x)
{
	asm(
		"VCVT.s32.f32 S0,S0 \n\t"
		"VMOV R0, S0 \n\t"
		"BX LR"
	);	
}
*/

void init_globalvariables();
void init_globalvariables()
{
	/* Initialize all variables before using them */
	global_counter = 0;
	global_currentState = DetectingStage1;
	global_nextState = DetectingStage1;
	global_pitch = 0;
	global_roll = 0;	
	global_last_call = 0;
	global_last_call2 = 0;
	global_last_call3 = 0;
	init_circularbuffer();
	nw = 0;
}


/* Setting the tolerence level for each stage of the gesture */
float flat_pitch_tol_h = 5.0;
float flat_roll_tol_h = 5.0;
float flat_pitch_tol_l = 0.0;
float flat_roll_tol_l = 0.0;

#ifdef PITCHCALCULATIONS
/* Setting the tolerence level for each stage of the gesture */
float slant_pitch_tol_h = 50.0;
float slant_roll_tol_h = 5.0;
float slant_pitch_tol_l = 40.0;
float slant_roll_tol_l = 0.0;
#else
/* Setting the tolerence level for each stage of the gesture */
float slant_pitch_tol_h = 5.0;
float slant_roll_tol_h = 50.0;
float slant_pitch_tol_l = 0.0;
float slant_roll_tol_l = 40.0;
#endif



void update_state1(void);
void update_state1(void)
{
	/* Called to check if the counter has reached 50 to move into DetectingStage2 */
	/* Takes in to arguement */
	/* Returns the state to go into */

	if((global_pitch >= flat_pitch_tol_l && global_pitch <= flat_pitch_tol_h)&&
		 (global_roll >= flat_roll_tol_l && global_roll <= flat_roll_tol_h))
	{
		if (global_counter < CounterPeak)
		{/* Upto 50 */
			global_counter = global_counter + 1;/* Incrementing counter */
			//return DetectingStage1;
			global_nextState = DetectingStage1;
			
		}

		else /*if (global_counter == CounterPeak)*/
		{/* When it reaches 50 */
			global_counter = 0;/* Resetting counter */
			//printf ("//////////////Stage 1 detected\n");
			LED_On(1);
			LED_Off(0);
			//return DetectingStage2;
			global_nextState = DetectingStage2;
		}
	}
	else
	{/* Remain in current state */
		//return DetectingStage1;
			global_nextState = DetectingStage1;
	}	

}

void update_state2(void);
void update_state2(void)
{
	/* Called to check if the counter has reached 50 to move into DetectingStage3 */
	/* Takes in to arguement */
	/* Returns the state to go into */
	
	if((global_pitch >= slant_pitch_tol_l && global_pitch <= slant_pitch_tol_h)&&
		 (global_roll >= slant_roll_tol_l && global_roll <= slant_roll_tol_h))
	{
		if (global_counter < CounterPeak)
		{/* Upto 50 */
			global_counter = global_counter + 1;/* Incrementing counter */
			//return DetectingStage2;
			global_nextState = DetectingStage2;		
		}

		else/* if (global_counter == CounterPeak)*/
		{/* When it reaches 50 */
			global_counter = 0;/* Resetting counter */
			//printf ("//////////////Stage 2 detected\n");
			LED_On(2);
			LED_Off(1);
			//return DetectingStage3;
			global_nextState = DetectingStage3;
		}
	}
	else
	{/* Remain in current state */
		//return DetectingStage2;
			global_nextState = DetectingStage2;
	}	

}

void update_state3(void);
void update_state3(void)
{
	/* Called to check if the counter has reached 50 to move into DetectingStage1 */
	/* Takes in to arguement */
	/* Returns the state to go into */

	if((global_pitch >= flat_pitch_tol_l && global_pitch <= flat_pitch_tol_h)&&
		 (global_roll >= flat_roll_tol_l && global_roll <= flat_roll_tol_h))
	{
		if (global_counter < CounterPeak)
		{/* Upto 50 */
			global_counter = global_counter + 1;/* Incrementing counter */
			//return DetectingStage3;		
			global_nextState = DetectingStage3;
		}

		else/* if (global_counter == CounterPeak)*/
		{/* When it reaches 50 */
			global_counter = 0;/* Resetting counter */
			printf ("/////////////////Stage 3 detected, Gesture detected\n");
			LED_On(0);
			LED_Off(2);
			//delay_ms(2000);
			//return DetectingStage1;
			global_nextState = DetectingStage1;
		}
	}
	else
	{/* Remain in current state */
		//return DetectingStage3;
			global_nextState = DetectingStage3;
	}	

}

/* Create an array of function pointers which return the states to be or to go in to */

/* Need to use array of function pointers */
/*+++Figure out how to use function pointers with argue ents in array form */
Update_with_int_ret update_funcions[3] = 
{
	update_state1,/* DetectingStage1 */
	update_state2,/* DetectingStage2 */
	update_state3,/* DetectingStage3 */
};

//typedef (void*)(*Update_pointer)(void);
struct TimedTask
{
	Update_with_int_ret function;
	float rep_interval;
//	uint32_t last_call;
};

struct TimedTask timed_tasks[Numberofstages] = 
{
	{/* DetectingStage1 */
	  update_state1,//update_funcions[0],
	  1,//0.020,/* 0.5 * 1000 */
//	  0
	},

	{/* DetectingStage2 */
	  update_state2,//update_funcions[1],
	  1,//0.020,/* 0.25 * 1000 */
//	  0
	},

	{/* DetectingStage3 */
	  update_state3,//update_funcions[2],
	  1,//0.020,/* 1.0 * 1000 */
//	  0
	},

};


/* Using timed task to call the update_functions */
void add_timed_task(Update_with_int_ret function, float interval);
void add_timed_task(Update_with_int_ret function, float interval)
{

	uint32_t x = msTicks - global_last_call;
	if(x >= interval)
	{ /* execute the function call only at the respective time */

	  //global_nextState = function(global_pitch, global_roll);/* Invoke the function */
		global_last_call = msTicks;
		function();/* Invoke the function */
	}
}

void add_timed_task2(CALC_AVG function, float interval);
void add_timed_task2(CALC_AVG function, float interval)
{

	uint32_t x = msTicks - global_last_call2;
	if(x >= interval)
	{ /* execute the function call only at the respective time */

	  //global_nextState = function(global_pitch, global_roll);/* Invoke the function */
		global_last_call2 = msTicks;
		function((global_avg), (global_avg+1), (global_avg+2));/* Invoke the function */
	}
}

void add_timed_task3(Update_with_int_ret function, float interval);
void add_timed_task3(Update_with_int_ret function, float interval)
{

	uint32_t x = msTicks - global_last_call3;
	if(x >= interval)
	{ /* execute the function call only at the respective time */

	  //global_nextState = function(global_pitch, global_roll);/* Invoke the function */
		global_last_call3 = msTicks;
		function();/* Invoke the function */
	}
}

/*+++ Not to be used now +++*/




// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void)
{
  msTicks++;
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
	int i=12;

  for (i=12; i<=15; i++)
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

void initialise_monitor_handles();

/* To store the three measurements from accelerometer */
struct AccelData
{
	float x;
	float y;
	float z;
}accel_data_window[CircularBufferLength];

#define CircularBufferLength 5

void updateAccelDataBuffer(float x, float y, float z);
void updateAccelDataBuffer(float x, float y, float z)
{	/* Updating circular buffer with raw values of accelerometer */
	accel_data_window[nw].x = x;
	accel_data_window[nw].y = y;
	accel_data_window[nw].z = z;	
	if (nw <	CircularBufferLength)
	{
		nw++;			
	}
	else
	{
		nw = 0;/* Reset to indicate start of buffer */			
	}
}

void calc_average(float *avg_x, float *avg_y, float *avg_z);

#ifdef C_Code_calc_average
void calc_average(float *avg_x, float *avg_y, float *avg_z)
{
	int i = 0;
	float x = 0;
	float y = 0;
	float z = 0;

	for (i = 0; i < CircularBufferLength; i++)
	{
		x = x + accel_data_window[i].x;
		y = y + accel_data_window[i].y;
		z = z + accel_data_window[i].z; 	
	}

	/* Updating the averages of accelerometer values */
	*avg_x = x/CircularBufferLength;
	*avg_y = y/CircularBufferLength;
	*avg_z = z/CircularBufferLength;
}
#endif

void print_accelerometer(void);
void print_accelerometer(void)
{
	//calc_pitch_roll(global_avg[0], global_avg[1], global_avg[2], &global_pitch, &global_roll);
	//if (global_pitch < 0) {global_pitch = global_pitch * (-1);}
	//if (global_roll < 0) {global_roll = global_roll * (-1);}		 
	printf("Pitch = %f, Roll = %f\n", global_pitch, global_roll);	

}

void init_circularbuffer(void);
void init_circularbuffer(void)
{
	int i = 0;
	for(i = 0; i < CircularBufferLength; i++)
	{
		accel_data_window[i].x = 0;		
		accel_data_window[i].y = 0;
		accel_data_window[i].z = 0;
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
  init_accelerometers();
	init_globalvariables();
	uint32_t x = printFloor(2.5);

  uint32_t t_prev = 0;
  while (1)
	{
#if 0
    if ( (msTicks - t_prev) >= 1000) // 1 second has elapsed
    {
      t_prev = msTicks;
      float a[3]; // array of 3 floats into which accelerometer data will be read
      read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
//      printf("%f %f %f\n", a[0], a[1], a[2]);
			calc_pitch_roll(a[0], a[1], a[2], &global_pitch, &global_roll);
			if (global_pitch < 0) {global_pitch = global_pitch * (-1);}
			if (global_roll < 0) {global_roll = global_roll * (-1);}		 
			printf("Pitch = %f, Roll = %f\n", global_pitch, global_roll);

			global_currentState = global_nextState;
			add_timed_task( update_funcions[global_currentState], 1);

    }
#else

      float a[3]; // array of 3 floats into which accelerometer data will be read
      read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
      //printf("%f %f %f\n", a[0], a[1], a[2]);

#endif

#ifdef RAW
/* For getting only RAW data */
			
			calc_pitch_roll(a[0], a[1], a[2], &global_pitch, &global_roll);
			if (global_pitch < 0) {global_pitch = global_pitch * (-1);}
			if (global_roll < 0) {global_roll = global_roll * (-1);}		 
			//printf("Pitch = %f, Roll = %f\n", global_pitch, global_roll);

			global_currentState = global_nextState;

			/* Call every 20 milli second 50 times */
			add_timed_task( update_funcions[global_currentState], 20);
#else
/* For getting only filtered data */
			
			/* Update raw values of acclerometer into circular buffer */
			updateAccelDataBuffer(a[0], a[1], a[2]);

			/* Calculate average average after every collection of accelerometer data */
			calc_average((global_avg), (global_avg+1), (global_avg+2));
			
			calc_pitch_roll(a[0], a[1], a[2], &global_pitch, &global_roll);
			if (global_pitch < 0) {global_pitch = global_pitch * (-1);}
			if (global_roll < 0) {global_roll = global_roll * (-1);}

			/* global_avg has the averaged value of collected accelerometer data */
			/* Print every one second */
			add_timed_task3(print_accelerometer, 1000);

			global_currentState = global_nextState;
			add_timed_task( update_funcions[global_currentState], 20);

#endif
  }
}





