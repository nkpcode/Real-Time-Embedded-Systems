#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_tim.h"

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


#define STEPPERPINS 4

#define STEP0 0
#define STEP1 1
#define STEP2 2
#define STEP3 3

#define calSteps(angle) angle*5.68

/* Stepper use */
#define PitchHighLimit -90
#define PitchLowLimit 90
#define StepperAngleDetect 5
#define StepperAngleDetectNeg -5
#define MinStepperMovement 20

/* Servo use */
#define RollHighLimit -90
#define RollLowLimit 90
#define ServoAngleMin 20
#define ServoMaxLimit 900
#define ServoMinLimit 200
#define ServoAngleDetect 5
#define ServoAngleDetectNeg -5


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

// Function prototypes 

void LDR_read(void);

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

//int global_pin[STEPPERPINS] = {1,2,3,4};
int global_pin[STEPPERPINS];

static int Step_State;
static int Step_next;

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

void config_PWM(void) {

    // Structures for configuration
    GPIO_InitTypeDef            GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    TIM_OCInitTypeDef           TIM_OCInitStructure;
    
    // TIM4 Clock Enable
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    
    // GPIOB Clock Enable
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    
    // Initalize PB6 (TIM4 Ch1) and PB7 (TIM4 Ch2)
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_6;// | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_100MHz;    // GPIO_High_Speed
    GPIO_InitStructure.GPIO_OType   = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd    = GPIO_PuPd_UP;         // Weak Pull-up for safety during startup
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    
    // Assign Alternate Functions to pins
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_TIM4);
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);
    
    /* Setup TIM / PWM values
     Servo Requirements:  (May be different for your servo)
        - 50Hz (== 20ms) PWM signal
        - 0.6 - 2.1 ms Duty Cycle
     
     1. Determine Required Timer_Freq.
            TIM_Period = (Timer_Freq. / PWM_Freq) - 1
     
            - We need a period of 20ms (or 20000µs) and our PWM_Freq = 50Hz (i.e. 1/20ms)
            - See NOTES, for why we use µs
            TIM_Period = 20000 - 1 = 19999  (since its 0 offset)
     
            Timer_Freq = (TIM_Period + 1) * PWM_Freq.
            Timer_Freq = (19999 + 1) * 50
            Timer_Freq = 1000000 = 1MHz
     
     2. Determine Pre-Scaler
        APB1 clock frequency:
            - SYS_CLK/4 when prescaler == 1 (i.e. 168MHz / 4 = 42MHz)
            - SYS_CLK/2 when prescaler != 1 (i.e. 168MHz / 2 = 84MHz)
     
        Prescaler = APB1_Freq / Timer_Freq
        Prescaler = 84 MHz / 1 MHz
        Prescaler = 84
     
        For our example, we can prescale the TIM clock by 84, which gives us a Timer_Freq of 1MHz
            Timer_Freq = 84 MHz / 84 = 1 MHz
        So the TIMx_CNT register will increase by 1 000 000 ticks every second. When TIMx_CNT is increased by 1 that is 1 µs. So if we want a duty cycle of 1.5ms (1500 µs) then we can set our CCRx register to 1500.
     
     NOTES:
        - TIMx_CNT Register is 16 bits, i.e. we can count from 0 to (2^16)-1 = 65535
        - If the period, TIMx_ARR, is greater than the max TIMx_CNT value (65535), then we need to choose a larger prescaler value in order to slow down the count.
        - We use the µs for a more precise adjustment of the duty cycle
     
     */
    uint16_t PrescalerValue = (uint16_t) 84;

    // Time Base Configuration
    TIM_TimeBaseStructure.TIM_Period        = 19999;
    TIM_TimeBaseStructure.TIM_Prescaler     = PrescalerValue;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;
    
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    
    // Common TIM Settings
    TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse       = 0;                        // Initial duty cycle
    TIM_OCInitStructure.TIM_OCPolarity  = TIM_OCPolarity_High;
    
    // Channel 1
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  
    // Channel 2
//    TIM_OC2Init(TIM4, &TIM_OCInitStructure);
//    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);
    
    TIM_ARRPreloadConfig(TIM4, ENABLE);
    
    // Start timer
    TIM_Cmd(TIM4, ENABLE);
}

void sendPWMSignal(int x);
void sendPWMSignal(int x)
{
	TIM4->CCR1 = x;//0	
	delay_ms(700);
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

	Step_State = STEP0;
	Step_next = STEP0;

} 


// SysTick Handler (Interrupt Service Routine for the System Tick interrupt)
void SysTick_Handler(void)
{
  msTicks++;
	//ButtonFSM();
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

void init_LDR_set()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA
  // These pin are input from the LDR for the horizontal axis rotation 
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  GPIOA->MODER &= ~(0x3 << (2*1)); // clearing the PIN 1 for GPIO A 
  GPIOA->MODER &= ~(0x3 << (2*2)); // clearing the PIN 2 for GPIO A 
  GPIOA->MODER &= ~(0x3 << (2*3)); // clearing the PIN 3 for GPIO A 
  // LDR pins for the vertical axis movement to be added 
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

void init_GPIO_ADC(void);
void init_GPIO_ADC(void)
{

	GPIO_InitTypeDef GPIO_initStructre;
	GPIO_initStructre.GPIO_Pin = GPIO_Pin_2; //The ADC channel 2 is connected to PA2
  GPIO_initStructre.GPIO_Mode = GPIO_Mode_AN;
  GPIO_Init(GPIOA,&GPIO_initStructre); //Using GPIO A

	//global_pin[] = {1,2,3,4};
	int x[] = {1,2,3,4};
	int i = 0;

	for(i = 0; i < STEPPERPINS; i++)
	{
		global_pin[i] = x[i];
		
	}

}

void turnon_pin_stepper(int x);
void turnon_pin_stepper(int x)
{
	GPIOD->BSRRL = 1 << (x);
}

void turnoff_pin_stepper(int x);
void turnoff_pin_stepper(int x)
{
	GPIOD->BSRRH = 1 << (x);
}


void stepper_coil_on();
void stepper_coil_on()
{/* Turining on coils in the sequence - 1, 2, 3, 4 */

	static int step_count = 0;
	static int reverse = 0x00;
	Step_State = Step_next;		
	switch(Step_State)
	{		
		case STEP0: 		
		turnon_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);

		if (reverse != 0)
		{Step_next = STEP1;}
		else
		{Step_next = STEP3;}
		break;
	
		case STEP1:
		turnon_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);

		if (reverse != 0)
		{Step_next = STEP2;}
		else
		{Step_next = STEP0;}
		break;

		case STEP2:
		turnon_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);

		if (reverse != 0)
		{Step_next = STEP3;}
		else
		{Step_next = STEP1;}
		break;

		case STEP3:
		turnon_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);

		if (reverse != 0)
		{Step_next = STEP0;}
		else
		{Step_next = STEP2;}
		break;

	}
	step_count++;/* Increment count after each step */

	if(step_count >= calSteps(45))
	{
			reverse = reverse ^ 1;/* toggle reverse */
			step_count = 0;
	}

}

/* Do the necessary calulations for forward and reverse motion */
void stepper_forward();
void stepper_forward()
{/* Turining on coils in the sequence - 1, 2, 3, 4 */
		
		turnon_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		delay_ms(10);

		turnon_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		delay_ms(10);

		turnon_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		delay_ms(10);

		turnon_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		delay_ms(10);

}

void stepper_reverse();
void stepper_reverse()
{/* Turining on coils in the sequence - 4, 3, 2, 1 */
		
		turnon_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		delay_ms(10);

		turnon_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		delay_ms(10);

		turnon_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		turnoff_pin_stepper(global_pin[0]);
		delay_ms(10);

		turnon_pin_stepper(global_pin[0]);
		turnoff_pin_stepper(global_pin[1]);
		turnoff_pin_stepper(global_pin[2]);
		turnoff_pin_stepper(global_pin[3]);
		delay_ms(10);

}



void init_GPIO_stepper();
void init_GPIO_stepper()
{/* setting 4 pins from port d as output for stepper motor usage */
	//int pin[4] = {1,2,3,4};
	int i = 0;

  for (i = 0 ; i< 4; i++)
  {
    GPIOD->MODER &= ~(0x3 << (2*global_pin[i])); // clear the 2 bits corresponding to pin i
    GPIOD->MODER |= (1 << (2*global_pin[i]));    // set pin i to be general purpose output
  }

}


void init_button()
{
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  // enable clock to GPIOA
  // These pin are input from the LDR for the horizontal axis rotation 
  // if the 2 bits corresponding to pin 0 are 00, then it is in input mode
  GPIOA->MODER &= ~(0x3 << (2*0)); // clear the 2 bits corresponding to pin 0
  GPIOA->MODER &= ~(0x3 << (2*1)); // clearing the PIN 1 for GPIO A 
  GPIOA->MODER &= ~(0x3 << (2*4)); // clearing the PIN 4 for GPIO A 
  GPIOA->MODER &= ~(0x3 << (2*3)); // clearing the PIN 3 for GPIO A 

	GPIOA->MODER &= ~(0x3 << (2*8)); // clearing the PIN 5 for GPIO A 
  GPIOA->MODER &= ~(0x3 << (2*6)); // clearing the PIN 6 for GPIO A 
  GPIOA->MODER &= ~(0x3 << (2*10)); // clearing the PIN 7 for GPIO A 
  // LDR pins for the vertical axis movement to be added 
}


void calc_pitch_roll(float acc_x, float acc_y, float acc_z, float *pitch, float *roll)
{
	*roll = (180.0/M_PI)*atan2(acc_y, acc_z);
	*pitch = (180.0/M_PI)*atan2(-acc_x, sqrt(acc_y*acc_y+acc_z*acc_z));
}

void initialise_monitor_handles();

void LDRStepperSet(void);
void LDRStepperSet(void)
{
	//printf("GPIO IDR %d\n", 0xffff&GPIOA->IDR);
	int LDR_left = 0x0002 & GPIOA->IDR;
	int LDR_center = 0x0010 & GPIOA->IDR;
	int LDR_right = 0x0008 & GPIOA->IDR;
	
	
	static int LDR_l =0, LDR_r = 0, LDR_c = 0; 	

	if(LDR_left == 2){ LDR_l =1;} else LDR_l = 0;
	if(LDR_center == 16){LDR_c =1;} else LDR_c = 0;
  if(LDR_right == 8){LDR_r =1;} else LDR_r = 0;
	
	//printf("LDR_l = %d\n", LDR_l);
	//printf("LDR_c = %d\n", LDR_c);
	//printf("LDR_r = %d\n", LDR_r);
	
	if(LDR_c == 1) // Center LED is not receving light, have to move more towards light.
		{
		
		if(LDR_l == 1 && LDR_c == 1 && LDR_r == 0 ) // move towards left, light is at left
		{
			int i = 0;
			while(i<10)
			{			
				stepper_reverse();
				//printf("rev\n");	
				i++;			
			}
			i = 0;		
		}
		if(LDR_l == 0 && LDR_c == 1 && LDR_r == 1) // move towards right, light at right
		{
			int i = 0;
			while(i<10)
			{			
				stepper_forward();
				//printf("for\n");	
				i++;			
			}
			i = 0;	
		}
	}
	
}


void LDRServoSet(void);
int stepper = 350;
void LDRServoSet(void)
{
	int LDR_top = 0x0100 & GPIOA->IDR;
	int LDR_center = 0x0040 & GPIOA->IDR;
	int LDR_bottom = 0x0400 & GPIOA->IDR;
	
	//printf("LDR_left = %d\n", LDR_top);
	//printf("LDR_center = %d\n", LDR_center);
	//printf("LDR_right = %d\n\n", LDR_bottom);
	
	static int LDR_l =0, LDR_r = 0, LDR_c = 0; 
	

	if(LDR_top == 256){ LDR_l =1;} else LDR_l = 0;
	if(LDR_center == 64){LDR_c =1;} else LDR_c = 0;
  if(LDR_bottom == 1024){LDR_r =1;} else LDR_r = 0;
	
	//printf("LDR_l = %d\n", LDR_l);
	//printf("LDR_c = %d\n", LDR_c);
	//printf("LDR_r = %d\n", LDR_r);

	if(LDR_c == 1)
	{
		if(LDR_l == 1 && LDR_c == 1 && LDR_r == 0 )
		{
			stepper -= 20;
			sendPWMSignal(stepper);
		}
		if(LDR_l == 0 && LDR_c == 1 && LDR_r == 1)
		{
			stepper += 20;	
			if(stepper >= 550 ) { stepper = 550 ;}	
			sendPWMSignal(stepper);
		}
	}
}


void StepperControlGesture(float pitch);			
void StepperControlGesture(float pitch)
{	/* Depends on the pitch of the accelerometer */

	static float pitch_prev = 0;
	/* Stepper Control */
	if(pitch > PitchHighLimit || pitch < PitchLowLimit)
	{
		if((pitch - pitch_prev) > StepperAngleDetect)
		{
			int i = 0;
			while(i < MinStepperMovement)
			{
				stepper_forward();
				i++;
			}
			i = 0 ;
		} 

		if((pitch - pitch_prev) < StepperAngleDetectNeg)
		{
			int j = 0;
			while(j < MinStepperMovement)
			{
				stepper_reverse();
				j++;
			}
			j = 0 ; 	
		}

	}// if pitch condition ends 

	pitch_prev = pitch;
}

void ServoControlGesture(float roll);
void ServoControlGesture(float roll)
{/* Servo Control */
	
	static float roll_prev = 0;	
	static int step = 200;

	if(roll > RollHighLimit || roll < RollLowLimit)
	{
		if(roll - roll_prev > ServoAngleDetect)
		{
			step+= ServoAngleMin;
			/* Setting the higher and lower limits for stepper */		
			if (step > ServoMaxLimit){ step = ServoMaxLimit; }			
		} 
	
		if(roll - roll_prev < ServoAngleDetectNeg)
		{
			step-= ServoAngleMin;
			/* Setting the higher and lower limits for stepper */				
			if (step < ServoMinLimit){ step = ServoMinLimit; }
		}
		/* Call Servo function */
		sendPWMSignal(step);

	}// if roll condition ends 

	roll_prev = roll;
}


void ManualControl(void);		
void ManualControl(void)
{

	float a[3]; // array of 3 floats into which accelerometer data will be read
	read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
	//printf("%f %f %f\n", a[0], a[1], a[2]);
	float pitch, roll;
	calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
	//printf("Pitch = %f, Roll = %f\n", pitch, roll);

	StepperControlGesture(pitch);			
	ServoControlGesture(roll);

}

void Automatic(void);
void Automatic(void)
{
			LDRStepperSet();
			LDRServoSet();		
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
	init_GPIO_ADC();
	init_GPIO_stepper();
	init_port_adc();/* Initialize ADC port A, pin 2 */
	config_PWM();/* Configure the stepper motor */
	sendPWMSignal(350);
	intialize_arrays();

	init_globals();
	//char x = 'a';
	//float pitch_prev = 0 , roll_prev = 0 ;
  uint32_t t_prev = 0;
  while (1)
	{
		//ManualControl();
#if 1
		if ( (msTicks - t_prev) >= 2) // 100 milisecond has elapsed
		{
			t_prev = msTicks;
			//ManualControl();
			Automatic();
		}
#endif
	}


}







