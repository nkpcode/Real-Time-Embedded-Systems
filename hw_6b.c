#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "arm_math.h"
#include "sim_ground_vehicle.h"

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

  SimGroundVehicle vehicle;
  float F = 0; // force applied to ground vehicle
  init_sim_ground_vehicle(&vehicle);

  delay_ms(3000);

  int n_delay = 10; // 10 milliseconds delay per iteration of "while" loop (i.e., 0.01 s sampling period)

  while (1)
  {
    char s[128]; // char array into which we will print a string and then send out over USB-connected serial port

    float pos = update_sim_ground_vehicle(&vehicle, F, n_delay*0.01);
    float vel = get_vel_sim_ground_vehicle(&vehicle);

    sprintf(s, "F: %.2f, pos: %.2f, vel: %.2f\r\n", F, pos, vel);
    write_serial_usb_bytes(s, strlen(s));


    // read characters coming in over serial port
    while (1)
    {
      uint8_t c;
      uint8_t read_byte = read_serial_usb_byte(&c); // read one character
      if (read_byte == 1) // if a character was read
      {
        if (c == '+') F += 0.1;
        if (c == '-') F -= 0.1;
      }
      else break; // if no more characters at the current time
    }

    delay_ms(n_delay); // pause for n_delay milliseconds
  }

}




