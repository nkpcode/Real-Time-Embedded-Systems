#include "stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "arm_math.h"

#include <stdio.h>
#include <math.h>
#include <string.h>

#include "accelerometers/accelerometers.h"
#include "temperature/temperature.h"
#include "RNG/random_number_generator.h"

/* static void init_systick(); */
static void delay_ms(uint32_t n);
static volatile uint32_t msTicks; // counts 1 ms timeTicks

enum ButtonStates
{
  ButtonIsPressed,
  ButtonIsReleased
};
static enum ButtonStates button_state = ButtonIsReleased;
static uint32_t button_counter;

uint32_t read_button(void);

QueueHandle_t queue_button; // queue managed by the RTOS
SemaphoreHandle_t mutex_printf; // in FreeRTOS, mutexes use the same struct definition as semaphores, but are created using the xSemaphoreCreateMutex function


// SysTick_Handler is registered by the RTOS -- currently configured at 1 kHz -- in FreeRTOSConfig.h -- hence, each tick is 1 ms
// the SysTick_Handler from the RTOS will call the function vApplicationTickHook every time it runs; hence, any code that needs to be run every system tick (i.e., every 1 ms) can be placed in this function
void vApplicationTickHook(void) {
  msTicks ++;
  uint32_t b = read_button();
  if (button_state == ButtonIsReleased)  // button_state is ButtonIsReleased
  {
    if (b == 0) button_counter = 0;
    else button_counter ++;
    if (button_counter >= 250)
    {
      button_counter = 0;
      button_state = ButtonIsPressed;
      int a = 1;
      xQueueSendFromISR(queue_button,&a,NULL); // send 1 to queue to signify a ButtonPress event
      // xQueueSendFromISR is used to add an item to a queue from within an interrupt handler
    }
  }
  else // button_state is ButtonIsPressed
  {
    if (b == 1) button_counter = 0;
    else button_counter ++;
    if (button_counter >= 250)
    {
      button_counter = 0;
      button_state = ButtonIsReleased;
      int a = 2;
      xQueueSendFromISR(queue_button,&a,NULL); // send 1 to queue to signify a ButtonRelease event
    }
  }
}

// pause for a specified number (n) of milliseconds
void delay_ms(uint32_t n)
{
  vTaskDelay(n);
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

uint32_t read_button(void)
{
  uint32_t button_bit = GPIOA->IDR & 0x1; // button is connected to pin A0, hence least significant bit of GPIOA->IDR corresponds to the button

  return button_bit;
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

void thread_button_messages(void *p)
{
  while (1)
  {
    int a;
    // xQueueReceive is used to get an item from a queue; the first argument specifies the queue from which to receive; the second argument is the pointer to where data should be written from the queue; the third argument specifies how long to wait (in ticks) -- setting this to 0 says to wait indefinitely to get an element from the queue
    if  ( xQueueReceive(queue_button, (void *)(&a), 0) == pdTRUE)
    {
      // xSemaphoreTake is used to lock the mutex/semaphore; second argument is amount of time (in ticks) to wait to lock; if set to 10, for example, this function will wait for 10 ticks and if it could not lock successfully, it will return pdFALSE; if set to  portMAX_DELAY, it will wait indefinitely to lock; this function returns pdTRUE if it successfully locked
      if ( xSemaphoreTake( mutex_printf, portMAX_DELAY) == pdTRUE)
      {
        float t = msTicks * 0.001; // alternatively, xTaskGetTickCount gives time in ticks since the RTOS scheduler was started
        if (a == 1) printf("Button pressed at time %f\n",t);
        if (a == 2) printf("Button released at time %f\n",t);
        xSemaphoreGive(mutex_printf); // release the mutex
      }
    }
    delay_ms(10); // wait for 10 ms before checking the queue again
  }
}

void thread_func2(void *p)
{
  while (1)
  {
    LED_On(1);
    delay_ms(1000);
    LED_Off(1);
    delay_ms(1000);
  }
}

void thread_func3(void *p)
{
  while (1)
  {
    // lock mutex_printf -- wait indefinitely to lock
    if ( xSemaphoreTake( mutex_printf, portMAX_DELAY) == pdTRUE)
    {
      printf("Hello\n");
      xSemaphoreGive(mutex_printf); // release the mutex
    }
    delay_ms(2000);
  }
}


int main(void)
{
  // initialize
  SystemInit();
  initialise_monitor_handles();
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
  init_LED_pins();
  init_button();
  init_accelerometers(); // initialize accelerometers
  init_rng(); // initialize random number generator
  init_temperature_sensor();

  // xQueueCreate is used to create a queue
  // first argument to xQueueCreate is length of the queue (max number of items it can hold); second argument is size of each item (in bytes)
  queue_button = xQueueCreate( 10, sizeof(int));

  // xSemaphoreCreateMutex is used to create a mutex
  mutex_printf = xSemaphoreCreateMutex();

  printf("time = %f\n",msTicks*0.001);

  // xTaskCreate is used to create a thread
  // first argument to xTaskCreate is function pointer; second argument is an arbitrary name for the function; third argument is stack size for the thread; fourth argument is a pointer to any arguments (e.g., pointer to a struct or can be set to NULL); fifth argument is priority; sixth argument is pointer to a handle to the thread (can be set to NULL)
  // configMINIMAL_STACK_SIZE is minimum stack size for a thread configured as 256 bytes in FreeRTOSConfig.h
  uint8_t ret1 = xTaskCreate(thread_button_messages, "button_messages", 2048, NULL, 1, NULL);
  uint8_t ret2 = xTaskCreate(thread_func2, "LED1", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
  uint8_t ret3 = xTaskCreate(thread_func3, "print", 512, NULL, 1, NULL);

  if ( (ret1 == pdTRUE) && (ret2 == pdTRUE) && (ret3 == pdTRUE) ) {
    vTaskStartScheduler();  // start the RTOS scheduler -- this function will never return
  } else {
    // error
    while (1) {}
  }
}


