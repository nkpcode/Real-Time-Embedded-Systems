// os.c
// Runs on LM4F120/TM4C123/MSP432
// Lab 3 starter file.
// Daniel Valvano
// March 24, 2016

#include <stdint.h>
#include "os.h"
#include "CortexM.h"
#include "BSP.h"

// function definitions in osasm.s
void StartOS(void);

#define NUMTHREADS  6        // maximum number of threads
#define NUMPERIODIC 2        // maximum number of periodic threads
#define STACKSIZE   100      // number of 32-bit words in stack per thread
#define FREQOFTASK0 1000 
#define PRIORITYOFEVENT 5
struct tcb{
  int32_t *sp;       // pointer to stack (valid for threads not running
  struct tcb *next;  // linked-list pointer
   // nonzero if blocked on this semaphore
  int32_t *blocked;  
   // nonzero if this thread is sleeping
  int32_t Sleep;
//*FILL THIS IN****
};
typedef struct tcb tcbType;
tcbType tcbs[NUMTHREADS];
tcbType *RunPt;
int32_t Stacks[NUMTHREADS][STACKSIZE];


/* For Periodic tasks */
int32_t periodoftasks;
(void)(*period_tasks)();


// ******** OS_Init ************
// Initialize operating system, disable interrupts
// Initialize OS controlled I/O: periodic interrupt, bus clock as fast as possible
// Initialize OS global variables
// Inputs:  none
// Outputs: none
void OS_Init(void){
  DisableInterrupts();
  BSP_Clock_InitFastest();// set processor clock to fastest speed
// perform any initializations needed, 

// like setting up periodic timer to run runperiodicevents
  BSP_PeriodicTask_InitB(runperiodicevents, FREQOFTASK0, PRIORITYOFEVENT);
}

void SetInitialStack(int i){

  // **Same as Lab 2 and Lab 3****
  tcbs[i].sp = &Stacks[i][STACKSIZE-16];
  Stacks[i][STACKSIZE-1] = 0x01000000; /* PSW - setting to thumb mode */
  /* R13 = Program counter; set later in OS_AddThreads() */ 
  Stacks[i][STACKSIZE-3] = 0x14141414; /* R14 */
  Stacks[i][STACKSIZE-4] = 0x12121212; /* R12 */

  Stacks[i][STACKSIZE-5] = 0x03030303; /* R3 */
  Stacks[i][STACKSIZE-6] = 0x02020202; /* R2 */
  Stacks[i][STACKSIZE-7] = 0x01010101; /* R1 */
  Stacks[i][STACKSIZE-8] = 0x00000000; /* R0 */
  
  Stacks[i][STACKSIZE-9] = 0x11111111; /* R11 */
  Stacks[i][STACKSIZE-10] = 0x10101010; /* R10 */
  Stacks[i][STACKSIZE-11] = 0x09090909; /* R9 */
  Stacks[i][STACKSIZE-12] = 0x08080808; /* R8 */
  Stacks[i][STACKSIZE-13] = 0x07070707; /* R7 */
  Stacks[i][STACKSIZE-14] = 0x06060606; /* R6 */
  Stacks[i][STACKSIZE-15] = 0x05050505; /* R5 */
  Stacks[i][STACKSIZE-16] = 0x04040404; /* R4 */  
  
  
}

//******** OS_AddThreads ***************
// Add six main threads to the scheduler
// Inputs: function pointers to six void/void main threads
// Outputs: 1 if successful, 0 if this thread can not be added
// This function will only be called once, after OS_Init and before OS_Launch
int OS_AddThreads(void(*thread0)(void),
                  void(*thread1)(void),
                  void(*thread2)(void),
                  void(*thread3)(void),
                  void(*thread4)(void),
                  void(*thread5)(void)){
  // **similar to Lab 2. initialize as not blocked, not sleeping****
  int32_t status;
  status = StartCritical();
  /* Initializing the states of the tasks in the tcb */
  tcbs[0].next = &tcbs[1];
  tcbs[1].next = &tcbs[2];
  tcbs[2].next = &tcbs[3];
  tcbs[3].next = &tcbs[4];
  tcbs[4].next = &tcbs[5];
  tcbs[5].next = &tcbs[0];

  int i;
  for(i = 0; i < NUMTHREADS; i++)
  {
    tcbs[i].blocked = NULL;
    tcbs[i].Sleep = 0;
  }
// initializing RunPt
  RunPt = &tcbs[0];
 
 // initialize four stacks, including initial PC
  SetInitialStack(0); 
  Stacks[0][STACKSIZE - 2] = (int32_t)(thread0);

  SetInitialStack(1); 
  Stacks[1][STACKSIZE - 2] = (int32_t)(thread1);

  SetInitialStack(2); 
  Stacks[2][STACKSIZE - 2] = (int32_t)(thread2);

  SetInitialStack(3); 
  Stacks[3][STACKSIZE - 2] = (int32_t)(thread3);

  SetInitialStack(4); 
  Stacks[4][STACKSIZE - 2] = (int32_t)(thread4);

  SetInitialStack(5); 
  Stacks[5][STACKSIZE - 2] = (int32_t)(thread5);

  EndCritical(status);  
  return 1;               // successful
}

//******** OS_AddPeriodicEventThread ***************
// Add one background periodic event thread
// Typically this function receives the highest priority
// Inputs: pointer to a void/void event thread function
//         period given in units of OS_Launch (Lab 3 this will be msec)
// Outputs: 1 if successful, 0 if this thread cannot be added
// It is assumed that the event threads will run to completion and return
// It is assumed the time to run these event threads is short compared to 1 msec
// These threads cannot spin, block, loop, sleep, or kill
// These threads can call OS_Signal
// In Lab 3 this will be called exactly twice
int OS_AddPeriodicEventThread(void(*thread)(void), uint32_t period){

  int32_t status;
  status = StartCritical();
// ****IMPLEMENT THIS****

  periodoftasks = period;
  period_tasks = thread;

  EndCritical(status);
  return 1;

}

void static runperiodicevents(void){
// ****IMPLEMENT THIS****
// **RUN PERIODIC THREADS, DECREMENT SLEEP COUNTERS
  static int32_t timerforperiod0 = 0;
  timerforperiod0 = timerforperiod0 + 1;
  static int32_t i = 0;/* For loop */
  
  /* running task0 1000 times in a second = once every mili-second */
  /* runperiodicevents is called by the timer every mili-seconds */
  Task0();

  /* running task1 10 times in a second = once every 100 mili-seconds */
  if(timerforperiod0 == 100)
  {
    Task1();
    timerforperiod0 = 0;
  }

  /* Decrement Sleep Counters for all the threads if they are sleeping */
  
  for(i = 0; i < NUMTHREADS; i++)
  {
    if(tcbs[i].Sleep > 0)
    {
      tcbs[i].Sleep = tcbs[i].Sleep - 1;
    }
  }

}

//******** OS_Launch ***************
// Start the scheduler, enable interrupts
// Inputs: number of clock cycles for each time slice
// Outputs: none (does not return)
// Errors: theTimeSlice must be less than 16,777,216
void OS_Launch(uint32_t theTimeSlice){
  STCTRL = 0;                  // disable SysTick during setup
  STCURRENT = 0;               // any write to current clears it
  SYSPRI3 =(SYSPRI3&0x00FFFFFF)|0xE0000000; // priority 7
  STRELOAD = theTimeSlice - 1; // reload value
  STCTRL = 0x00000007;         // enable, core clock and interrupt arm
  StartOS();                   // start on the first task
}
// runs every ms
void Scheduler(void){ // every time slice
// ****IMPLEMENT THIS****
// ROUND ROBIN, skip blocked and sleeping threads
  while(true)
  {
    /* go through all the threads available */
    /* Assuming here that atleast one thread is always active */
    RunPt = RunPt->next;
    if(RunPt-> Sleep == 0 || RunPt-> blocked == NULL)
    {
      break;
    }

  } 

}

//******** OS_Suspend ***************
// Called by main thread to cooperatively suspend operation
// Inputs: none
// Outputs: none
// Will be run again depending on sleep/block status
void OS_Suspend(void){
  STCURRENT = 0;        // any write to current clears it
  INTCTRL = 0x04000000; // trigger SysTick
// next thread gets a full time slice
}

// ******** OS_Sleep ************
// place this thread into a dormant state
// input:  number of msec to sleep
// output: none
// OS_Sleep(0) implements cooperative multitasking
void OS_Sleep(uint32_t sleepTime){
// ****IMPLEMENT THIS****
// set sleep parameter in TCB
  RunPt -> Sleep = sleepTime;
// suspend, stops running
  OS_Suspend();
}

// ******** OS_InitSemaphore ************
// Initialize counting semaphore
// Inputs:  pointer to a semaphore
//          initial value of semaphore
// Outputs: none
void OS_InitSemaphore(int32_t *semaPt, int32_t value){
//***IMPLEMENT THIS***
  *semaPt = value;
}

// ******** OS_Wait ************
// Decrement semaphore and block if less than zero
// Lab2 spinlock (does not suspend while spinning)
// Lab3 block if less than zero
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Wait(int32_t *semaPt){
//***IMPLEMENT THIS***
  DisableInterrupts();    
  *semaPt = *semaPt - 1;   
  if(*semaPt < 0)
  {
    RunPt->blocked = semaPt;/* address of semaphore that is blocking the thread */
    EnableInterrupts();
    /* Restart Systick timer */
    OS_Suspend();
  }  
  EnableInterrupts();

}

// ******** OS_Signal ************
// Increment semaphore
// Lab2 spinlock
// Lab3 wakeup blocked thread if appropriate
// Inputs:  pointer to a counting semaphore
// Outputs: none
void OS_Signal(int32_t *semaPt){
//***IMPLEMENT THIS***
  tcbType *tempPt =  RunPt;
  DisableInterrupts();

  *semaPt = *semaPt + 1;
  if(*semaPt <= 0)
  {/* If value of semaphore is non-negative then one thread can be un-blocked */
    while(true)
    {
      tempPt = tempPt->next;
      if(tempPt->blocked == semaPt)
      {/* If blocking on this semaphore */
        tempPt->blocked = NULL;
        break;
      }      
    }
  }  

  EnableInterrupts();

}

#define FSIZE 10    // can be any size
uint32_t PutI;      // index of where to put next
uint32_t GetI;      // index of where to get next
uint32_t Fifo[FSIZE];
int32_t CurrentSize;// 0 means FIFO empty, FSIZE means full
uint32_t LostData;  // number of lost pieces of data

// ******** OS_FIFO_Init ************
// Initialize FIFO.  
// One event thread producer, one main thread consumer
// Inputs:  none
// Outputs: none
void OS_FIFO_Init(void){
//***IMPLEMENT THIS***
  PutI = 0;
  GetI = 0;
  OS_InitSemaphore(*CurrentSize,0);
  LostData = 0;
}

// ******** OS_FIFO_Put ************
// Put an entry in the FIFO.  
// Exactly one event thread puts,
// do not block or spin if full
// Inputs:  data to be stored
// Outputs: 0 if successful, -1 if the FIFO is full
int OS_FIFO_Put(uint32_t data){
//***IMPLEMENT THIS***
  if(CurrentSize >= FSIZE)
  {
    LostData = LostData + 1;
    return -1;
  }  
  else
  {
    Fifo[PutI] = data;
    PutI = (PutI + 1)%FSIZE;
    OS_Signal(&CurrentSize);
    return 0;   // success
  }
}

// ******** OS_FIFO_Get ************
// Get an entry from the FIFO.   
// Exactly one main thread get,
// do block if empty
// Inputs:  none
// Outputs: data retrieved
uint32_t OS_FIFO_Get(void){uint32_t data;
//***IMPLEMENT THIS***
  OS_Wait(&CurrentSize);/* blocks thread if no data available */
  data = Fifo[GetI];
  GetI = (GetI + 1)%FSIZE;
  return data;
}



