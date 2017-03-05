#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "accelerometers/accelerometers.h"
#include "temperature/temperature.h"
#include "RNG/random_number_generator.h"
#include "serial_port_usb/serial_port_usb.h"

static void init_systick();
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

#define NumberOfCommands 5

typedef void (*FUNC_PTR)(arm_matrix_instance_f32 x, arm_matrix_instance_f32 y, arm_matrix_instance_f32 z);
struct funct_struct
{
	char letter;
	FUNC_PTR function; 
};
typedef struct funct_struct F_struct_type; 

/* Has all the functions to be executed based on the input given */
//F_struct_type F_Struct[NumberOfCommands];
F_struct_type F_Struct[];


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

  for (int i=12; i<=15; i++)
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

// void initialise_monitor_handles();

// read a line (a sequence of characters until end of a line) from the USB serial port
void read_line_from_serial_usb(char *s)
{
  while (1)
  {
    uint8_t c;
    uint8_t read_byte = read_serial_usb_byte(&c); // read one character
    if (read_byte == 1) // if a character was read
    {
      write_serial_usb_bytes(&c,1); // echo the character back to the USB serial port
      *s = c; s++;
      if ( (c == '\n') || (c == '\r') ) break; // line feed or carriage return ASCII codes
    }
  }
}

// wait for a character from the serial port
void wait_for_byte_from_serial_usb()
{
  while(1)
  {
    uint8_t c;
    uint8_t read_byte = read_serial_usb_byte(&c); // read one character
    if ((read_byte == 1) && (c == 'a')) // if a character was read
      return;
  }
}

// read 4 floats from a char array
void interpret_line_as_2x2_matrix(char *s, arm_matrix_instance_f32 *m)
{
  float32_t a11, a12, a21, a22;
  sscanf(s,"%f %f %f %f",&a11, &a12, &a21, &a22); // read 4 floats from string
  m->pData[0] = a11;
  m->pData[1] = a12;
  m->pData[2] = a21;
  m->pData[3] = a22;
}

void test_matrix()
{
  float32_t a1[4];
  float32_t a2[4];
  float32_t r[4];
  arm_matrix_instance_f32 a1_m;
  arm_matrix_instance_f32 a2_m;
  arm_matrix_instance_f32 r_m;
  arm_mat_init_f32(&a1_m, 2,2, (float32_t *)a1);  
  arm_mat_init_f32(&a2_m, 2,2, (float32_t *)a2);
  arm_mat_init_f32(&r_m, 2,2, (float32_t *)r);

	/* Look for command from keyboard */
	 
	wait_for_byte_from_serial_usb(); // wait for user to type some character to start
	char x[128];
	{
	  char prompt[] = "\r\nEnter a command a,s,m,t,i:";
	  write_serial_usb_bytes(prompt, strlen(prompt));
	}
	read_line_from_serial_usb(x);
	
  char ch = x[0];
	int i = 0;

	/* Get the first matrix */
  char s[128];
  {
    char prompt[] = "\r\nEnter matrix 1: ";
    write_serial_usb_bytes(prompt, strlen(prompt));
  }
  read_line_from_serial_usb(s);
  interpret_line_as_2x2_matrix(s,&a1_m);

	/* Perform the opeartion based on input command */
  switch (ch)
	{
		case 'a':
			{
			/* Get the second matrix */				
			{
				char prompt[] = "\r\nEnter matrix 2: ";
				write_serial_usb_bytes(prompt, strlen(prompt));
			}
			read_line_from_serial_usb(s);
			interpret_line_as_2x2_matrix(s,&a2_m);

			/* Do the calculation */
			arm_mat_add_f32(&a1_m, &a2_m, &r_m);
			char result_string[] = "\r\nSum of the two matrices: ";
			write_serial_usb_bytes(result_string, strlen(result_string));
			}
			break;		
		case 's':
			/* Get the second matrix */				
			{
				char prompt[] = "\r\nEnter matrix 2: ";
				write_serial_usb_bytes(prompt, strlen(prompt));
			}
			read_line_from_serial_usb(s);
			interpret_line_as_2x2_matrix(s,&a2_m);

			/* Do the calculation */
			{			
			arm_mat_sub_f32(&a1_m, &a2_m, &r_m);
			char result_string[] = "\r\nSub of the two matrices: ";
			write_serial_usb_bytes(result_string, strlen(result_string));
			}
			break;
		case 'm':
			/* Get the second matrix */				
			{
				char prompt[] = "\r\nEnter matrix 2: ";
				write_serial_usb_bytes(prompt, strlen(prompt));
			}
			read_line_from_serial_usb(s);
			interpret_line_as_2x2_matrix(s,&a2_m);

			/* Do the calculation */
			{			
			arm_mat_mult_f32(&a1_m, &a2_m, &r_m);
			char result_string[] = "\r\nMul of the two matrices: ";
			write_serial_usb_bytes(result_string, strlen(result_string));
			}
			break;
		case 't':
			/* Do the operation */
			{
			arm_mat_trans_f32(&a1_m, &r_m);
			char result_string[] = "\r\nTranspose of the matrice: ";
			write_serial_usb_bytes(result_string, strlen(result_string));
			}
			break;
		case 'i':
			/* Do the operation */
			{
			arm_mat_inverse_f32(&a1_m, &r_m);
			char result_string[] = "\r\nInverse of the matrice: ";
			write_serial_usb_bytes(result_string, strlen(result_string));
			}
			break;
	}
	

//  arm_mat_add_f32(&a1_m, &a2_m, &r_m);

	/* Print the output */
  for (int i=0; i<4; i++)
  {
    sprintf(s," %.4f", r_m.pData[i]); // print up to 4 digits after the decimal point
    write_serial_usb_bytes(s, strlen(s));
  }
  sprintf(s," \r\n"); // print new line
  write_serial_usb_bytes(s, strlen(s));


}


int main(void)
{
  // initialize
  SystemInit();
  // initialise_monitor_handles(); -- not required; we are printing over USB-connected serial port
  init_systick();
  init_LED_pins();
  init_button();
  init_accelerometers(); // initialize accelerometers
  init_rng(); // initialize random number generator
  init_temperature_sensor();

  init_serial_port_usb();

  delay_ms(3000); 

  int n_input = 0; // count of number of characters that have come IN over serial port -- i.e., number of characters that user has typed into serial terminal on computer
  while (1)
  {
		test_matrix();/* Perform the operation */

    char s[128]; // char array into which we will print a string and then send out over USB-connected serial port

    float a[3]; // array of 3 floats into which accelerometer data will be read
    read_accelerometers(a); // read data from accelerometers (X, Y, and Z axes)
    sprintf(s,"%f %f %f\r\n", a[0], a[1], a[2]); // print to string
    write_serial_usb_bytes(s,strlen(s)); // strlen finds length of a string


    float pitch, roll;
    calc_pitch_roll(a[0], a[1], a[2], &pitch, &roll);
    sprintf(s,"Pitch = %f, Roll = %f\r\n", pitch, roll); // print to string
    write_serial_usb_bytes(s,strlen(s));

    float temp = read_temperature_sensor();
    sprintf(s,"Temp = %f\r\n",temp);
    write_serial_usb_bytes(s,strlen(s));

    uint32_t r = get_random_number();
    sprintf(s,"Random = %lu\r\n",r); // print unsigned int
    write_serial_usb_bytes(s,strlen(s));

    sprintf(s, "n_input = %d\r\n", n_input); // number of characters typed by user
    write_serial_usb_bytes(s,strlen(s));

    // read characters coming in over serial port
    while (1)
    {
      uint8_t c;
      uint8_t read_byte = read_serial_usb_byte(&c); // read one character
      if (read_byte == 1) // if a character was read
      {
        write_serial_usb_bytes(&c,1); // echo the character back to the USB serial port
        n_input ++;
      }
      else break; // if no more characters at the current time
    }

    delay_ms(1000); // pause for 1 second
  }

}




