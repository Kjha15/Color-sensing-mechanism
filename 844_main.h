#ifndef __Embedded_Project
#define __Embedded_Project

#include <stdbool.h>
#include <stdint.h>
#include <math.h>

#include "inc/tm4c123gh6pm.h"

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_uart.h"

#include "driverlib/pin_map.h"
#include "driverlib/pwm.h"
#include "driverlib/sysctl.h"
#include "driverlib/sysctl.c"
#include "driverlib/i2c.h"
#include "driverlib/i2c.c"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/gpio.c"
#include "driverlib/interrupt.h"
#include "driverlib/interrupt.c"
#include "driverlib/uart.h"
#include "driverlib/uart.c"
#include "driverlib/rom.h"
#include "driverlib/udma.h"
#include "driverlib/udma.c"
#include "driverlib/debug.h"
#include "driverlib/systick.h"
#include "driverlib/systick.c"
#include "driverlib/cpu.h"
#include "utils/uartstdio.h"
#include "utils/uartstdio.c"
#include "utils/ustdlib.h"

volatile uint8_t receive;


// Defining the size of UART transfer buffers and they need not to be of same value
#define UART_txbufA_SIZE        	1
#define UART_rxbufA_SIZE         4

static uint32_t data;
// The transmit and receive buffers used for the UART transfers.  There is one
// transmit buffer and a pair of recieve ping-pong buffers.
static uint8_t txbufA[UART_txbufA_SIZE];
static uint8_t rxbufA[UART_rxbufA_SIZE];
// The count of UART buffers filled, one for each ping-pong buffer.

static uint32_t g_ui32rxbufAACount = 0;
static bool bufA;
// The circular buffer for the DMA Ping-Pong buffer configuration
static unsigned long ping_buf = 0;
static unsigned long pong_buf = 0;


// PWM Frequency for SG90 Servo Motor (50 Hz/20 ms) 
#define PWM_FREQUENCY 50
//defining values for PWM
volatile uint32_t ui32period;
volatile uint32_t ui32clockpwm;
volatile uint8_t ui8freqadjust;

// Below is control used by DMA controller. 

#if defined(dmacontrol)
#pragma data_alignment=1024
uint8_t ui8ControlTable[1024];
#elif defined(ccs)
#pragma DATA_ALIGN(ui8ControlTable, 1024)
uint8_t ui8ControlTable[1024];
#else
uint8_t ui8ControlTable[1024] __attribute__ ((aligned(1024)));
#endif



#endif __Embedded_Project
