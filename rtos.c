// RTOS Framework - Spring 2016
// J Losh

// Student Name: Nagendra Babu Bagam
// Mav Id: 1001243831
// E-mail: nagendrababu.bagam@mavs.uta.edu

//-----------------------------------------------------------------------------
// Hardware Target
//-----------------------------------------------------------------------------

// Target Platform: EK-TM4C123GXL Evaluation Board
// Target uC:       TM4C123GH6PM
// System Clock:    40 MHz

// Hardware configuration:
// 4 Pushbuttons and 4 LEDs, UART

//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// REQUIRED: correct these bitbanding references for green and yellow LEDs (temporary to guarantee compilation)
#define RED_LED      (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 2*4))) //PD2
#define ORANGE_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 1*4))) //PE1
#define GREEN_LED    (*((volatile uint32_t *)(0x42000000 + (0x400073FC-0x40000000)*32 + 3*4))) //PD3
#define YELLOW_LED   (*((volatile uint32_t *)(0x42000000 + (0x400243FC-0x40000000)*32 + 2*4))) //PE2
#define PB1			 (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 1*4))) //PB1
#define PB2			 (*((volatile uint32_t *)(0x42000000 + (0x400053FC-0x40000000)*32 + 0*4))) //PB0
#define PB3			 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 6*4))) //PA6
#define PB4			 (*((volatile uint32_t *)(0x42000000 + (0x400043FC-0x40000000)*32 + 7*4))) //PA7
#define PI_PB		 (*((volatile uint32_t *)(0x42000000 + (0x400253FC-0x40000000)*32 + 4*4))) //PF4

//-----------------------------------------------------------------------------
// RTOS Defines and Kernel Variables
//-----------------------------------------------------------------------------

// function pointer
typedef void (*_fn)();
_fn get_functionName(char *str);
char tolower(char c);
void update_SP(void* sp);
void putsUart0(char* str);
int atoi(const char *str);
void intTostring(int value,char *str);

// task States
#define STATE_INVALID    0 // no task/Invalid
#define STATE_READY      1 // ready to run
#define STATE_BLOCKED    2 // has run, but now blocked by semaphore
#define STATE_DELAYED    3 // has run, but now awaiting timer

// semaphore structure Details
#define MAX_QUEUE_SIZE 10
struct semaphore
{
	unsigned int count;
	unsigned int queueSize;
	unsigned int processQueue[MAX_QUEUE_SIZE]; // store task index here
	char name[15];
	unsigned int readIndex;
	unsigned int writeIndex;
	uint8_t currentUser;
} *s, keyPressed, keyReleased, flashReq, pInheritance;

#define MAX_TASKS 10       // maximum number of valid tasks
uint8_t taskCurrent = 0xFF;// index of last dispatched task
uint8_t taskCount = 0;     // total number of valid tasks
uint32_t stTime = 39999;
uint32_t endTime = 0;
int cpuTimeout = 1000;
bool PI = false;			// PrirotyInheritance Selector
int zeros=0;
uint8_t newPriority = 0xFF;
// TaskBlock Structure
struct _tcb
{
	uint8_t state;                 // see STATE values above
	void *pid;                     // used to uniquely identify process
	void *sp;                      // location of stack pointer for process
	uint8_t priority;              // 0=highest, 7=lowest
	uint8_t currentPriority;       // used for priority inheritance
	uint8_t skipcount;			   // Skipcount for scheduling task
	uint32_t ticks;                // ticks until sleep complete
	char name[15];				   // Name of the task
	int cpuUsage;				   // CPU Usage by This task
	uint32_t cputime;			   // CPU time used for this task
	struct semaphore *current;	   // Semaphore used currently by this task.
} tcb[MAX_TASKS];

uint32_t stack[MAX_TASKS][256];

//-----------------------------------------------------------------------------
// RTOS Kernel
//-----------------------------------------------------------------------------
// Semaphore Initialization
void init(void* p, int count,char *name)
{
	s = p;
	s->count = count;
	s->queueSize = 0;
	s->readIndex = 0;
	s->writeIndex = 0;
	s->currentUser = 0xFF;
	strcpy(s->name,name);
}
// Initializing RTOS tasks
void rtosInit()
{
	uint8_t i;
	taskCount = 0;				// No tasks Running
	// clear out all tcb records
	for (i = 0; i < MAX_TASKS; i++)
	{
		tcb[i].state = STATE_INVALID;
		tcb[i].pid = 0;
	}
	// systick for 1ms system timer
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_CLK_SRC | NVIC_ST_CTRL_INTEN;
	NVIC_ST_RELOAD_R = 39999;
	NVIC_ST_CURRENT_R = 1;
}
// RTOS schedular to dispatch next task.
int rtosScheduler()
{
	bool ok;
	uint8_t prevTask = taskCurrent;
	static uint8_t task = 0xFF;
	uint32_t tDiff = 0;
	ok = false;
	while (!ok)
	{
		task++;
		if (task >= MAX_TASKS)
			task = 0;
		if(tcb[task].state == STATE_READY)
		{
			if(tcb[task].skipcount == tcb[task].currentPriority)
			{
				tcb[task].skipcount = 0;
				ok = true;
			}
			else
			{
				if(tcb[task].skipcount < tcb[task].currentPriority)
					tcb[task].skipcount++ ;
			}
		}
	}
  	endTime = (int)NVIC_ST_CURRENT_R;
  	if(zeros == 0)
		tDiff =  stTime - endTime;
  	else
  		tDiff = (39999-endTime) + stTime + (zeros-1)*39999;
    tcb[prevTask].cputime += tDiff;
    zeros = 0;
  	stTime = (int)NVIC_ST_CURRENT_R;
	return task;
}
// Start the RTOS by strating first task.
void rtosStart()
{
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_ENABLE;
	// call the first task to be run, restoring the preloaded context
	taskCurrent = rtosScheduler();
	// Restore the stack to run the first process
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" pop {R15} ");
}
// To create The process
bool createProcess(_fn fn, int priority,char *name)
{
	NVIC_ST_CTRL_R&= ~NVIC_ST_CTRL_INTEN;
	bool ok = false;
	uint8_t i = 0;
	bool found = false;
	// REQUIRED: take steps to ensure a task switch cannot occur
	if (taskCount < MAX_TASKS)
	{
		// make sure fn not already in list (prevent reentrancy)
		while (!found && (i < MAX_TASKS))
		{
			found = (tcb[i++].pid ==  fn);
		}
		if (!found)
		{
			// find first available tcb record
			i = 0;
			while (tcb[i].state != STATE_INVALID) {i++;}
			// Task Initialization
			tcb[i].pid = fn;
			tcb[i].sp = &stack[i][243];
			stack[i][251] = (int)fn;
			tcb[i].priority = priority;
			tcb[i].currentPriority = priority;
			tcb[i].skipcount = priority;
			strcpy(tcb[i].name,name);
			tcb[i].cputime = 0;
			tcb[i].cpuUsage = 0;
			tcb[i].current = NULL;
			tcb[i].state = STATE_READY;
			taskCount++;
			ok = true;
		}
	}
	NVIC_ST_CTRL_R |= NVIC_ST_CTRL_INTEN;
	return ok;
}
// To update the stackpointer to given value
void update_SP(void* sp)
{
	__asm(" ADD R13,#8");	// To eliminate SP-8 Effect
	__asm(" MOV R13,R0");
	__asm(" SUB R13,#8");	// To eliminate SP+8 Effect
}
// To get current SP value
void* get_SP()
{
	__asm(" MOV R0,R13");
	__asm(" SUB R13,#8");
}
// Yield proceesor for other Tasks
void yield()
{
	__asm(" ADD R13,#8 ");
	__asm(" PUSH {R14} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp =get_SP();
	taskCurrent = rtosScheduler();
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" POP {R15} ");
}
// To set Sleep time to a task and execute other
void sleep(uint32_t tick)
{
	tcb[taskCurrent].ticks = tick;
	__asm(" ADD R13,#8 ");
	__asm(" PUSH {R14} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp =get_SP();
	tcb[taskCurrent].state = STATE_DELAYED;
	taskCurrent = rtosScheduler();
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" POP {R15} ");
}
// Waiting for a semaphore
void wait(void* pSemaphore)
{
	s = pSemaphore;
	__asm(" ADD R13,#16 ");
	__asm(" PUSH {R14} ");
	__asm(" PUSH {R4-R11} ");
	tcb[taskCurrent].sp = get_SP();
	if(s->count == 0)
	{	// If semaphore not available
		// If priority Inheritance is selected, Inherit hiher priority to lower priority task.
		if(PI && (tcb[s->currentUser].priority > tcb[taskCurrent].priority))
		{
			tcb[s->currentUser].currentPriority = tcb[taskCurrent].priority;
			tcb[s->currentUser].skipcount = tcb[taskCurrent].priority;
		}
		// If task requested for semaphore is already assigned previously
		if(s->currentUser == taskCurrent)
			s->currentUser = 0xFF;
		tcb[taskCurrent].state = STATE_BLOCKED;
		// Write to current writeindex in process queue ringbuffer and increment writeindex
		s->processQueue[s->writeIndex++ % 10] = taskCurrent;
		s->queueSize++;
		// Call schedular for next task as current process is blocked
		taskCurrent = rtosScheduler();
	}
	else
	{
		// If semaphore is already available
		tcb[taskCurrent].current = s;
		s->currentUser = taskCurrent;
		s->count --;

	}
	update_SP(tcb[taskCurrent].sp);
	__asm(" POP { R4-R11 } ");
	__asm(" POP {R15} ");
}
// Posting a Semaphore to its pool
void post(void* pSemaphore)
{
	s = pSemaphore;
	tcb[taskCurrent].current = NULL;
	s->currentUser = 0xFF;
	// Reverting task priority to its Normal priority if changed due to priority inheritance
	if(PI && (tcb[taskCurrent].currentPriority != tcb[taskCurrent].priority))
	{
		tcb[taskCurrent].currentPriority = tcb[taskCurrent].priority;
	}
	int task=0xFF;
	s->count++;
	if(s->count >= 1)
	{
		if(s->queueSize > 0)
		{	// If process exits in queue
			task = s->processQueue[s->readIndex++ %10];
			s->queueSize--;
			// Assign semaphore only if task state is Blocked
			// This is to prevent if task is destroyed after adding to process queue.
			if(tcb[task].state == STATE_BLOCKED)
			{
				tcb[task].state = STATE_READY;
				tcb[task].current = s;
				s->currentUser = task;
				s->count--;
			}
		}
	}
}
// Function to destroy a process
bool destroyProcess(_fn fn)
{
	uint8_t i=0;
	bool ok=false;
	while(i <= MAX_TASKS)
	{
		if(tcb[i].pid == fn)
		{
			ok= true;
			break;
		}
		else
			i++;
	}
	if(ok==false)
		return ok;
	// destroying readkeys,debounce and shell will have unpredicted results.So should be careful while destroying.
	// shell process destruction is not allowed.
	if(strcmp(tcb[i].name,"shell") == 0)
	{
		putsUart0("You can not delete this Process!\r\n");
		return false;
	}
	else
	{
		//If this task is delayed
		if(tcb[i].state == STATE_DELAYED)
			tcb[i].ticks = 0;
		//If task is blocked by a semaphore,the semaphore dispatcher logic will take care of that!
		//If this task is using semaphore,take back before deleting semaphore
		if(tcb[i].current != NULL)
		{
			post(tcb[i].current);
		}
		tcb[i].pid = 0;
		tcb[i].priority = 0;
		tcb[i].currentPriority = 0;
		tcb[i].skipcount = 0;
		tcb[i].cputime = 0;
		tcb[i].cpuUsage = 0;
		tcb[i].current = NULL;
		tcb[i].state = STATE_INVALID;
		taskCount--;
		return true;
	}
}

// system timer Interrupt service routine
void systickIsr()
{
	cpuTimeout -- ;
	zeros ++ ;
	bool ok = false;
	if(cpuTimeout == 0)
	{
		ok=true;
		cpuTimeout = 1000;
	}
	int i=0;
	// Decrement tick count for all delayed Tasks
	for(i=0;i<MAX_TASKS;i++)
	{
		if(tcb[i].state == STATE_DELAYED )
		{
			tcb[i].ticks -- ;
			if(tcb[i].ticks == 0)
				tcb[i].state = STATE_READY;
		}
		// If set count Elapses for CPU usage,Caluculate CPU usage
		if(ok)
		{
			float val=((float)tcb[i].cputime)/40000000;
			tcb[i].cpuUsage = val*100;
			tcb[i].cputime = 0;
		}
	}
}

//-----------------------------------------------------------------------------
// Subroutines / Supported Functions
//-----------------------------------------------------------------------------

// Initialize Hardware
void initHw()
{
    // REQUIRED: Add initialization for orange, red, green, and yellow LEDs
	//           4 pushbuttons, and uart
	// Configure HW to work with 16 MHz XTAL, PLL enabled, system clock of 40 MHz
	SYSCTL_RCC_R = SYSCTL_RCC_XTAL_16MHZ | SYSCTL_RCC_OSCSRC_MAIN | SYSCTL_RCC_USESYSDIV | (4 << SYSCTL_RCC_SYSDIV_S);

	// Set GPIO ports to use AP (not needed since default configuration -- for clarity)
	SYSCTL_GPIOHBCTL_R = 0;

	// Enable GPIO port F peripherals
	SYSCTL_RCGC2_R = SYSCTL_RCGC2_GPIOA | SYSCTL_RCGC2_GPIOB | SYSCTL_RCGC2_GPIOD |  SYSCTL_RCGC2_GPIOE | SYSCTL_RCGC2_GPIOF ;

	// Configure pushbutton pins
	GPIO_PORTA_DIR_R = 0x00;  // bits 6 and 7 are inputs
	GPIO_PORTA_DR2R_R = 0x00; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTA_DEN_R = 0xC0;  // enable bits 6 and 7
	GPIO_PORTA_PUR_R = 0xC0;  // enable internal pull-up for bits 6 and 7 (push buttons)
	GPIO_PORTB_DIR_R = 0x00;  // bits 0 and 1 are inputs
	GPIO_PORTB_DR2R_R = 0x00; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTB_DEN_R = 0x03;  // enable bits 0 and 1
	GPIO_PORTB_PUR_R = 0x03;  // enable internal pull-up for bits 6 and 7 (push buttons)
	// Configure led pins
	GPIO_PORTD_DIR_R = 0x0C;  // bits 3 and 2 are outputs
	GPIO_PORTD_DR2R_R = 0x0C; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTD_DEN_R = 0x0C;  // enable bits 3 and 2
	GPIO_PORTD_PUR_R = 0x00;  // enable internal pull-up as required
	GPIO_PORTE_DIR_R = 0x06;  // bits 2 and 1 are outputs
	GPIO_PORTE_DR2R_R = 0x06; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTE_DEN_R = 0x06;  // enable bits 2 and 1
	GPIO_PORTE_PUR_R = 0x00;  // enable internal pull-up
	// Priority inheritance selector push button
	GPIO_PORTF_DIR_R = 0x00;  // bits 1 and 3 are outputs, other pins are inputs
	GPIO_PORTF_DR2R_R = 0x00; // set drive strength to 2mA (not needed since default configuration -- for clarity)
	GPIO_PORTF_DEN_R = 0x10;  // enable LEDs and pushbuttons
	GPIO_PORTF_PUR_R = 0x10;  // enable internal pull-up for push button
	//URAT0 configuration
	SYSCTL_RCGCUART_R |= SYSCTL_RCGCUART_R0;         // turn-on UART0, leave other uarts in same status
	__asm(" NOP");                                  // wait 3 clocks
	__asm(" NOP");
	__asm(" NOP");
	GPIO_PORTA_DEN_R |= 3;                           // default, added for clarity
	GPIO_PORTA_AFSEL_R |= 3;                         // default, added for clarity
	GPIO_PORTA_PCTL_R = GPIO_PCTL_PA1_U0TX | GPIO_PCTL_PA0_U0RX;

	// Configure UART0 to 115200 baud, 8N1 format (must be 3 clocks from clock enable and config writes)
	UART0_CTL_R = 0;                                 // turn-off UART0 to allow safe programming
	UART0_CC_R = UART_CC_CS_SYSCLK;                  // use system clock (40 MHz)
	UART0_IBRD_R = 21;                               // r = 40 MHz / (Nx115.2kHz), set floor(r)=21, where N=16
	UART0_FBRD_R = 45;                               // round(fract(r)*64)=45
	UART0_LCRH_R = UART_LCRH_WLEN_8 | UART_LCRH_FEN; // configure for 8N1 w/ 16-level FIFO
	UART0_CTL_R = UART_CTL_TXE | UART_CTL_RXE | UART_CTL_UARTEN; // enable TX, RX, and module
}
// Approximate busy waiting (in units of microseconds), given a 40 MHz system clock
void waitMicrosecond(uint32_t us)
{
	                                            // Approx clocks per us
	__asm("WMS_LOOP0:   MOV  R1, #6");          // 1
    __asm("WMS_LOOP1:   SUB  R1, #1");          // 6
    __asm("             CBZ  R1, WMS_DONE1");   // 5+1*3
    __asm("             NOP");                  // 5
    __asm("             B    WMS_LOOP1");       // 5*3
    __asm("WMS_DONE1:   SUB  R0, #1");          // 1
    __asm("             CBZ  R0, WMS_DONE0");   // 1
    __asm("             B    WMS_LOOP0");       // 1*3
    __asm("WMS_DONE0:");                        // 40 clocks/us + error
}
// To Read the pushbuttons Value
uint8_t readPbs()
{
	uint8_t value=0;
	value = (!PB1 << 0) + (!PB2 << 1) + (!PB3 << 2)+ (!PB4 << 3);
	return value;
}
// Print character to UART terminal
void putcUart0(char c)
{
	while (UART0_FR_R & UART_FR_TXFF);
	UART0_DR_R = c;
}
// To print string to UART terminal
void putsUart0(char* str)
{
	int i;
	int slen = strlen(str);
    for (i = 0; i < slen; i++)
	  putcUart0(str[i]);
}
// To convert integer to string
void intTostring(int value,char *str)
{
	int num = value;
	int i=0;
	bool isNegative=false;
	char res[5];
    int len = 0;
    if(num==0)
    {
    	*str='0';
    	str++;
    	*str='\0';
    	return;
    }
    if(num<0)
    {
    	isNegative=true;
    	num=0-num;
    }
    for(; num > 0; ++len)
    {
       res[len] = num%10+'0';
       num/=10;
    }
    if(isNegative)
    {
    	res[len]='-';
    	len=len+1;
    }
    res[len] = 0; //null-terminating
    //now we need to reverse res
    for(i = 0; i < len/2; ++i)
    {
        char c = res[i]; res[i] = res[len-i-1]; res[len-i-1] = c;
    }
    for(i=0;i<len;i++)
    {
    	*str=res[i];
    	str++;
    }
    *str='\0';
}

// ------------------------------------------------------------------------------
//  Task functions
// ------------------------------------------------------------------------------

// Idle task must be ready at all times  to prevent scheduler fail
void idle()
{
  while(true)
  {
    ORANGE_LED = 1;
    waitMicrosecond(1000);
    ORANGE_LED = 0;
    yield();
  }
}
// Flash4Hz process to Flash a Led(green) at 4Hz rate
void flash4Hz()
{
  while(true)
  {
    GREEN_LED ^= 1;
    sleep(125);
  }
}
// Oneshot to switch on yellow LED for 1sec
void oneshot()
{
  while(true)
  {
    wait(&flashReq);
    YELLOW_LED = 1;
    sleep(1000);
    YELLOW_LED = 0;
  }
}
// Part of some lengthy function
void partOfLengthyFn()
{
  waitMicrosecond(1000);
  yield();
}
// Lengthy Function
void lengthyFn()
{
  uint16_t i;
  while(true)
  {
    for (i = 0; i < 4000; i++)
    {
      partOfLengthyFn();
    }
    RED_LED ^= 1;
  }
}
// To read pushButtons when pressed
void readKeys()
{
  uint8_t buttons;
  while(true)
  {
    wait(&keyReleased);
    buttons = 0;
    while (buttons == 0)
    {
      buttons = readPbs();
      yield();
    }
    post(&keyPressed);
    if ((buttons & 1) != 0)
    {
      YELLOW_LED ^= 1;
      RED_LED = 1;
    }
    if ((buttons & 2) != 0)
    {
      post(&flashReq);
      RED_LED = 0;
    }
    if ((buttons & 4) != 0)
    {
      char pname[10] = "flash4hz";
      createProcess(flash4Hz, 0,pname);
    }
    if ((buttons & 8) != 0)
    {
      destroyProcess(flash4Hz);
	}
    yield();
  }
}
// Debounce process for sensing pushbutton debounce.
void debounce()
{
  uint8_t count;
  while(true)
  {
    wait(&keyPressed);
    count = 10;
    while (count != 0)
    {
      sleep(10);
      if (readPbs() == 0)
        count--;
      else
        count = 10;
    }
    post(&keyReleased);
  }
}
// Uncooperative process
void uncooperative()
{
  while(true)
  {
    while (readPbs() == 8)
    {
    }
    yield();
  }
}
//PI High prioriy task
void PI_highPri()
{
	while(true)
	{
		wait(&pInheritance);
		RED_LED=1;
		waitMicrosecond(1000);
		RED_LED=0;
		post(&pInheritance);
		yield();
	}
}
//PI low priority task
void PI_lowPri()
{
	while(true)
	{
		wait(&pInheritance);
		int i=0;
		for(i=0;i<3000;i++)
		{
			GREEN_LED^=1;
			waitMicrosecond(1000);
			yield();
		}
		post(&pInheritance);
		sleep(1000);
	}
}
// Shell process to accept commands from user and execute them.
void shell()
{
	char command[20] = {0};
	static int pos = 0;
	int maxsize = 20;
	while(true)
	{
		putsUart0("Enter your Command: ");
		while(1)
		{
			char c= '\0';
			while(UART0_FR_R & UART_FR_RXFE)
				yield();
			c = UART0_DR_R & 0xFF;
			if(c=='\b')
			{
				if(pos>0)
				{
					pos=pos-1;
					putcUart0(' ');	putcUart0('\b');
				}
				else
					putcUart0(' ');
			}
			else if(c=='\r')
			{
				command[pos]='\0';
				break;
			}
			else
			{
				if(pos==maxsize)
				{
					command[++pos]='\0';
					break;
				}
				else
					command[pos++]=tolower(c);
			}
		}
		putsUart0("\n\r");
		char *cmd1 = NULL;
		char *cmd2 = NULL;
		cmd1 = strtok(command," ");
		cmd2 = strtok(NULL," ");
		int i=0;
		bool found=false;
		if(strcmp(cmd1,"ps")==0)				// Process Status Info
		{
			putsUart0("ProcessName\t PID\t Priority\t Current_Pri\t CPU_Usage\n\r");
			for(i=0;i<MAX_TASKS;i++)
			{
				if(tcb[i].state != STATE_INVALID)
				{
					char pid[10] ={0};
					char cpu[5]={0};
					char pri[2]={0};
					char pipri[2]={0};
					intTostring((int)tcb[i].pid,pid);
					intTostring(tcb[i].cpuUsage,cpu);
					intTostring(tcb[i].priority,pri);
					intTostring(tcb[i].currentPriority,pipri);
					putsUart0(tcb[i].name);
					if(strlen(tcb[i].name) < 8)
						putsUart0("\t\t");
					else
						putsUart0("\t");
					putsUart0(pid);
					putsUart0("\t\t");
					putsUart0(pri);
					putsUart0("\t\t");
					putsUart0(pipri);
					putsUart0("\t\t");
					putsUart0(cpu);
					putsUart0("\r\n");
				}
			}
		}
		else if(strcmp(cmd1,"ipcs")==0)				// Process Semaphore statuses
		{
			putsUart0("Semaphore_name\tCount\tcurrent_User\tQueue_size\r\n");
			char count[3] = {'\0'};
			char qSize[3] = {'\0'};
			struct semaphore *t = NULL;
			struct semaphore a[4] = {keyPressed,keyReleased,flashReq,pInheritance};
			for(i=0;i<4;i++)
			{
				t=&a[i];
				intTostring((int)t->count,count);
				intTostring((int)t->queueSize,qSize);
				putsUart0(t->name);
				putsUart0("\t");
				putsUart0(count);
				putsUart0("\t");
				if(t->currentUser != 0xFF)
				{
					putsUart0(tcb[t->currentUser].name);
					if(strlen(tcb[t->currentUser].name) < 8)
						putsUart0("\t\t\t");
					else
						putsUart0("\t\t");
				}
				else
					putsUart0("_ _ _ _\t\t\t");
				putsUart0(qSize);
				putsUart0("\r\n");
			}
		}
		else if(strcmp(cmd1,"kill")==0)				// To kill a Process
		{
			bool ok = false;
			if(cmd2 == NULL)
				putsUart0("Process Id not provided!\n\r");
			else
			{
				int pId = atoi(cmd2);
				if(pId == 0)
					putsUart0("Process Id Invalid!\n\r");
				else
				{
					ok = destroyProcess((_fn)pId);
					if(!ok)
						putsUart0("Destroying Process Failed!! \r\n");
					else
						putsUart0("Process destroyed successfully!! \r\n");
				}
			}
		}
		else if(strcmp(cmd1,"pidof")==0)			// To know Pidof a process
		{
			if(cmd2 != NULL)
			{
				for(i=0;i<MAX_TASKS;i++)
				{
					if(strcmp(tcb[i].name,cmd2) == 0)
					{
						found = true;
						break;
					}
				}
				if(found)
				{
					char outpid[10]={0};
					intTostring((int)tcb[i].pid,outpid);
					putsUart0(outpid);
					putsUart0("\r\n");
				}
				else
					putsUart0("Process with given name not found!\r\n");
			}
			else
				putsUart0("Process name not provided!\r\n");
		}
		else if(strcmp(cmd1,"reboot")==0)			// To reboot the controller
		{
			NVIC_APINT_R = NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ;  //opt2: NVIC_APINT_VECT_RESET;
		}
		else
		{
			if(strcmp(cmd2,"&")==0 && cmd1!= NULL)	// To create a Process
			{
				bool ok =false;
				_fn fn = get_functionName(cmd1);
				if(fn!=0 && newPriority != 0xFF)
				{
					ok=createProcess(fn, newPriority,cmd1);
					if(ok == false)
						putsUart0("Process already Running!! \n\r");
					else
						putsUart0("Process Created Successfully!!\n\r");
				}
				else
					putsUart0("Invalid Process to create!\n\r");
			}
			else
				putsUart0("Invalid command Provided!\n\r");
		}
		pos = 0;
		memset(command,sizeof(command),'\0');
	}
}
// Getting Fn pointer based on function name
_fn get_functionName(char *str)
{
	_fn fn;
	newPriority = 0xFF;
	if(strcmp(str,"idle")==0)
	{
		fn = idle;
		newPriority = 7;
	}
	else if(strcmp(str,"flash4hz")==0)
	{
		fn = flash4Hz;
		newPriority = 0;
	}
	else if(strcmp(str,"lengthyfn")==0)
	{
		fn = lengthyFn;
		newPriority = 6;
	}
	else if(strcmp(str,"oneshot")==0)
	{
		fn = oneshot;
		newPriority = 3;
	}
	else if(strcmp(str,"readkeys")==0)
	{
		fn = readKeys;
		newPriority = 1;
	}
	else if(strcmp(str,"debounce")==0)
	{
		fn = debounce;
		newPriority = 3;
	}
	else if(strcmp(str,"uncooperative")==0)
	{
		fn = uncooperative;
		newPriority = 5;
	}
	else if(strcmp(str,"shell")==0)
	{
		fn = shell;
		newPriority = 2;
	}
	else if(strcmp(str,"pi_highpri")==0)
	{
		fn = PI_highPri;
		newPriority = 0;
	}
	else if(strcmp(str,"pi_lowpri")==0)
	{
		fn = PI_lowPri;
		newPriority = 4;
	}
	else
		fn = 0;
	return fn;
}
//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

int main(void)
{
    bool ok;
	// Initialize hardware
	initHw();
	if(!PI_PB)
		PI = true;
	rtosInit();
	// Power-up flash
	RED_LED = 1;
	waitMicrosecond(250000);
	RED_LED = 0;
	waitMicrosecond(250000);
	// Initialize semaphores
	char name[15] = "keyPressed";
	init(&keyPressed, 0,name);
	strcpy(name,"keyReleased");
	init(&keyReleased, 1,name);
	strcpy(name,"flashReq");
	init(&flashReq, 5,name);
	strcpy(name,"pInheritance");
	init(&pInheritance,1,name);
	strcpy(name,"idle");
	ok =  createProcess(idle,7,name);
	if(PI)
	{
		strcpy(name,"pi_highpri");
		ok &= createProcess(PI_highPri,0,name);
		strcpy(name,"pi_lowpri");
		ok &= createProcess(PI_lowPri,4,name);
	}
	else
	{
		strcpy(name,"flash4hz");
		ok &= createProcess(flash4Hz, 0,name);
		strcpy(name,"lengthyfn");
		ok &= createProcess(lengthyFn, 6,name);
		strcpy(name,"oneshot");
		ok &= createProcess(oneshot, 3,name);
		strcpy(name,"readkeys");
		ok &= createProcess(readKeys, 1,name);
		strcpy(name,"debounce");
		ok &= createProcess(debounce, 3,name);
		strcpy(name,"uncooperative");
		ok &= createProcess(uncooperative, 5,name);
	}
	strcpy(name,"shell");
	ok &= createProcess(shell, 2,name);
	putsUart0("\r\n");
	// Start the RTOS
	if (ok)
	  rtosStart();
	else
	  RED_LED = 1;

    return 0;
    // don't delete this unreachable code
    // if a function is only called once in your code, it will be
    // accessed with two goto instructions instead of call-return,
    // so any stack-based code will not function correctly
    yield(); sleep(0); wait(0); post(0);
}


