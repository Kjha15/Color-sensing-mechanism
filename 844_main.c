#include "844_main.h"
#define I2C_SLAVE 0x3C
// initialize I2C module 0
void Initialize_I2C(){
	
	//enable I2C module 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

	//reset module
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);
	 
	//enable GPIO peripheral that contains I2C 0
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

	// Configure the pin muxing for I2C0 functions on port B2 and B3. Hex values of the pins are written as Keil was shoing error in the address of pins
	GPIOPinConfigure(0x00010803);
	GPIOPinConfigure(0x00010C03);
	 
	// Select the I2C function for these pins.
	GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);
	GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);

  I2CSlaveInit(I2C0_BASE, 0x3C);
	I2CSlaveEnable(I2C0_BASE);
}

// Configure the UART and its pins
void Initialize_UART(void){
	// Enable the GPIO Peripheral used by the UART
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	
	// Enable UART0
	GPIOPinConfigure(0x00000001);
	GPIOPinConfigure(0x00000401);

	// Configure GPIO Pins for UART mode
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
		
	// Use the internal 16Mh oscillator as the UART clock source
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	
	// Initialize the UART for console I/O
	UARTStdioConfig(0, 115200, 16000000); //115200 is baud rate, 16 Mhz is internal oscillator clock
}




//
// The interrupt handler for UART1.  This interrupt will occur when a DMA
// transfer is complete using the UART1 uDMA channel.  UART is kept in loopback operation TX to RX and back. 


void
UART1IntHandler(void)
{
    uint32_t ui32Status;
    uint32_t ui32Mode;

    //
    // Read the interrupt status of the UART.
    //
    ui32Status = UARTIntStatus(UART1_BASE, 1);

    //
    // Clear any pending status, even though there should be none since no UART
    // interrupts were enabled.  Still, as uDMA uses both Tx and Rx there should be no interrupt at the starting 
    UARTIntClear(UART1_BASE, ui32Status);


    //
    // Check the DMA control table to see if the ping-pong "B" transfer is
    // complete.  The "B" transfer uses receive buffer "B", and the alternate
    // control structure.
    //
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT);
		
		//
    // If the primary control structure indicates stop, that means the ping
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the pong buffer.
    //
    if(ui32Mode == UDMA_MODE_STOP)
    {
			switch(ping_buf%3)
			{
				
				case 1:
					// process data in A+B
					//
					// Increment a counter to indicate data was received into buffer A.  In
					// a real application this would be used to signal the main thread that
					// data was received so the main thread can process the data.
					//
					
				
					// PC6
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x40);	
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~0x40);
				
					//
					// Set up the next transfer for the "A" buffer, using the primary
					// control structure.  When the ongoing receive into the  buffer is
					// done, the uDMA controller will switch back to this one.  
					//
					uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
																		 UDMA_MODE_PINGPONG,
																		 (void *)(UART1_BASE + UART_O_DR),
																		 rxbufA, sizeof(rxbufA));	
																		 
					break;
			}
			ping_buf++;
    }

    //
    // Check the DMA control table to see if the pong transfer is
    // complete.  The pong buffer transfer uses the alternate
    // control structure.
    //
    ui32Mode = uDMAChannelModeGet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT);

    //
    // If the alternate control structure indicates stop, that means the pong
    // receive buffer is done.  The uDMA controller should still be receiving
    // data into the ping buffer.
    //
    if(ui32Mode == UDMA_MODE_STOP)
    {			
			switch(pong_buf%3)
			{
				case 1:
					// process data in A
					//
					// Increment a counter to indicate data was received into buffer A.  In
					// a real application this would be used to signal the main thread that
					// data was received so the main thread can process the data.
					//
				
					
					// PC6
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, 0x40);	
					GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_6, ~0x40);
								
					//
					// Set up the next transfer for the "A" buffer, using the alternate
					// control structure.  When the ongoing receive into the "A" buffer is
					// done, the uDMA controller will switch back to this one.  This
					// example re-uses buffer A, but a more sophisticated application could
					// use a rotating set of buffers to increase the amount of time that
					// the main thread has to process the data in the buffer before it is
					// reused.
					//
						uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT, UDMA_MODE_PINGPONG,(void *)(UART1_BASE + UART_O_DR), rxbufA, sizeof(rxbufA));		
					break;
				
				
			}
			pong_buf++;
    }				
		
    //
    // If the UART1 DMA TX channel is disabled, that means the TX DMA transfer
    // is done.
    //
    if(!uDMAChannelIsEnabled(UDMA_CHANNEL_UART1TX))
    {
			//
			// Start another DMA transfer to UART1 TX.
			//
			uDMAChannelTransferSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,UDMA_MODE_BASIC, txbufA,(void *)(UART1_BASE + UART_O_DR), sizeof(txbufA));

			//
			// The uDMA transmit channel should be enabled
			//
			uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
			// pin 4 of port C is connected to logic analyzer for checking 												 
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x10);	
			GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~0x10);																														
    }
}


//
// Initializes the UART1 peripheral and sets up the TX and RX uDMA channels.
// The UART is configured for loopback mode so that any data sent on TX will be
// received on RX.  The uDMA channels are configured so that the TX channel
// will copy data from a buffer to the UART TX output and the uDMA RX channel
// will receive any incoming data into a pair of buffers in ping-pong mode.
//

void
InitUART1Transfer(void)
{
	unsigned int ux = 0;

	//
	// Fill the TX buffer with a simple data pattern.
	//
	while(ux < UART_txbufA_SIZE)
	{
		receive = I2CSlaveDataGet(I2C0_BASE);
		data = (char)receive;
		
		if((data == 'cl') | (data == 'red') | (data == 'green') | (data == 'blue')){
			txbufA[ux] = data;
			ux++;
		}
	}

	//  UART peripheral enabling 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);

	//
	//Configuring UART parameters	//
	UARTConfigSetExpClk(UART1_BASE, SysCtlClockGet(), 115200,
													UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
													UART_CONFIG_PAR_NONE);

	//
	// Set both the TX and RX trigger thresholds to 4.  This will be used by
	// the uDMA controller to signal when more data should be transferred.  The
	// uDMA TX and RX channels will be configured so that it can transfer 4
	// bytes in a burst when the UART is ready to transfer more data.
	//FIFOs of size 8 bits are used for UART 
	UARTFIFOLevelSet(UART1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

	//
	// Enable the UART for operation, and enable the uDMA interface for both TX
	// and RX channels.
	//
	UARTEnable(UART1_BASE);
	UARTDMAEnable(UART1_BASE, UART_DMA_RX | UART_DMA_TX);

	//
	// This register write will set the UART to operate in loopback mode.  Any
	// data sent on the TX output will be received on the RX input.
	//
	HWREG(UART1_BASE + UART_O_CTL) |= UART_CTL_LBE;
	// Put the attributes in a known state for the uDMA UART1RX channel.  These
	// should already be disabled by default.
	//
	uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1RX,
															UDMA_ATTR_ALTSELECT | UDMA_ATTR_USEBURST |
															UDMA_ATTR_HIGH_PRIORITY |
															UDMA_ATTR_REQMASK);

	//
	// Configure the control parameters for the primary control structure for
	// the UART RX channel.  The primary contol structure is used for the "A"
	// part of the ping-pong receive.  The transfer data size is 8 bits, the
	// source address does not increment since it will be reading from a
	// register.  The destination address increment is byte 8-bit bytes.  The
	// arbitration size is set to 4 to match the RX FIFO trigger threshold.
	// The uDMA controller will use a 4 byte burst transfer if possible.  This
	// will be somewhat more effecient that single byte transfers.
	//
	uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
														UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
														UDMA_ARB_1);

	//
	// Configure the control parameters for the alternate control structure for
	// the UART RX channel.  The alternate contol structure is used for the "B"
	// part of the ping-pong receive.  The configuration is identical to the
	// primary/A control structure.
	//
	uDMAChannelControlSet(UDMA_CHANNEL_UART1RX | UDMA_ALT_SELECT,
														UDMA_SIZE_32 | UDMA_SRC_INC_NONE | UDMA_DST_INC_32 |
														UDMA_ARB_1);

	//
	// Set up the transfer parameters for the UART RX primary control
	// structure.  The mode is set to ping-pong, the transfer source is the
	// UART data register, and the destination is the receive "A" buffer.  The
	// transfer size is set to match the size of the buffer.
	//
	uDMAChannelTransferSet(UDMA_CHANNEL_UART1RX | UDMA_PRI_SELECT,
														 UDMA_MODE_PINGPONG,
														 (void *)(UART1_BASE + UART_O_DR),
														 rxbufA, sizeof(rxbufA));

	

	//
	// Put the attributes in a known state for the uDMA UART1TX channel.  These
	// should already be disabled by default.
	//
	uDMAChannelAttributeDisable(UDMA_CHANNEL_UART1TX,
																	UDMA_ATTR_ALTSELECT |
																	UDMA_ATTR_HIGH_PRIORITY |
																	UDMA_ATTR_REQMASK);

	//
	// Set the USEBURST attribute for the uDMA UART TX channel.  This will
	// force the controller to always use a burst when transferring data from
	// the TX buffer to the UART.  This is somewhat more effecient bus usage
	// than the default which allows single or burst transfers.
	//
	uDMAChannelAttributeEnable(UDMA_CHANNEL_UART1TX, UDMA_ATTR_USEBURST);

	//
	// Configure the control parameters for the UART TX.  The uDMA UART TX
	// channel is used to transfer a block of data from a buffer to the UART.
	// The data size is 8 bits.  The source address increment is 8-bit bytes
	// since the data is coming from a buffer.  The destination increment is
	// none since the data is to be written to the UART data register.  The
	// arbitration size is set to 4, which matches the UART TX FIFO trigger
	// threshold.
	//
	uDMAChannelControlSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
												UDMA_SIZE_32 | UDMA_SRC_INC_8 | UDMA_DST_INC_NONE |
												UDMA_ARB_1);

	//
	// Set up the transfer parameters for the uDMA UART TX channel.  This will
	// configure the transfer source and destination and the transfer size.
	// Basic mode is used because the peripheral is making the uDMA transfer
	// request.  The source is the TX buffer and the destination is the UART
	// data register.
	//
	uDMAChannelTransferSet(UDMA_CHANNEL_UART1TX | UDMA_PRI_SELECT,
												 UDMA_MODE_BASIC, txbufA,
												 (void *)(UART1_BASE + UART_O_DR),
												 sizeof(txbufA));

	//
	// Now both the uDMA UART TX and RX channels are primed to start a
	// transfer.  As soon as the channels are enabled, the peripheral will
	// issue a transfer request and the data transfers will begin.
	//
	uDMAChannelEnable(UDMA_CHANNEL_UART1RX);
	uDMAChannelEnable(UDMA_CHANNEL_UART1TX);
			
	// PC4											 
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, 0x10);	
	GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_4, ~0x10);										
}

// Initialize GPIO ports
void PortFunctionInit(void){
	// Set clock for GPIO port A 
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	
	// Set clock for GPIO port C for the Logic Analyzer
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
	
	// Set clock for GPIO ports B, E, and F for the servos
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	
	// Set PA5, PA7 as output
	GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_7);
	
	// Set outputs for logic analyzer
	GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7);	

}

// Initilaze PWM for SG90 micro servo
void Init_PWM_module(void){
	// Set values for servo
	ui8freqadjust = 0;
	ui32clockpwm = SysCtlClockGet() / 64;
	ui32period = (ui32clockpwm / PWM_FREQUENCY) - 1;

	// Set the clock for PWM. 
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);
	
	// Set clock for PWM0 as all three servos are connected to PWM0 module
	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
	// Configure PB4, PB6, and PE4 as PWM outputs
	GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6);
	GPIOPinTypePWM(GPIO_PORTE_BASE, GPIO_PIN_4);

	
	// Configure PWM signals
	// M0PWM0 - Module 0 - PWM Generator 0 - PWM Out 0 - PB6
	// M0PWM2 - Module 0 - PWM Generator 1 - PWM Out 2 - PB4
	// M0PWM4 - Module 0 - PWM Generator 2 - PWM Out 4 - PE4
	// I have to write hex values because it was showing errors if I write GPIO_PB6_M0PWM0
	GPIOPinConfigure(0x00011804); // PB6 address
	GPIOPinConfigure(0x00011004); //PB4 address
	GPIOPinConfigure(0x00041004); //PE4 address
	// Configure PWM0 generator 0, 1, and 2 for count down mode
	// Counts from a value down to zero and resets to preset value
	PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_1, PWM_GEN_MODE_DOWN);
	PWMGenConfigure(PWM0_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
	
	

	// Set the period for generator 0, 1, and 2 as ui32period
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, ui32period);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_1, ui32period);
	PWMGenPeriodSet(PWM0_BASE, PWM_GEN_2, ui32period);
	
	

	// Set the output for PWM
	PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_2_BIT, true);
	PWMOutputState(PWM0_BASE, PWM_OUT_4_BIT, true);
	
	

	// Enable PWM on PWM0 generator 0, 1, and 2;
	PWMGenEnable(PWM0_BASE, PWM_GEN_0);	
	PWMGenEnable(PWM0_BASE, PWM_GEN_1);
	PWMGenEnable(PWM0_BASE, PWM_GEN_2);
	
	
}

//

// Give delay of 2 seconds to servo arms 
//
void redcolour(void){		
	// Set initial freq adjust values
	ui8freqadjust = 111;
	
	// make the servo motor rotate
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, ui8freqadjust * ui32period / 1000);
	
	
	
	//give delay of 2 seconds 
	SysCtlDelay(2 * SysCtlClockGet()/3);
	
	
}

void bluecolour(void){
	// Set initial adjustment value
	ui8freqadjust = 111;

	// make the servo motor rotate		
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, ui8freqadjust * ui32period / 1000);

	

	//give delay of 2 seconds 		
	SysCtlDelay(2*SysCtlClockGet()/3);
	
	
}



void greencolour(void){	
	// Set initial adjustment value
	ui8freqadjust = 111;

	// make the servo motor rotate		
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, ui8freqadjust * ui32period / 1000);

	

	//give delay of 2 seconds 
	SysCtlDelay(2 * SysCtlClockGet()/3);
		
	
}

// Reset servos to original position
void resetPWM(void){
	//Bring servo back to original position	
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, 56 * ui32period / 1000);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_2, 56 * ui32period / 1000);
	PWMPulseWidthSet(PWM0_BASE, PWM_OUT_4, 56 * ui32period / 1000);	
}


int main(void){
	// Set clock to 40 MHz
	SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);
	
	// Counter for how many correct transactions passed (in this case only one)
	int counter;
	
	// Initialize
	Initialize_UART();
	Initialize_I2C();
	PortFunctionInit();
	Init_PWM_module();	
	
	// Reset servos to start position
	resetPWM();
  
  // Enable the uDMA controller at the system level. 
  SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
  
  // Enable the uDMA controller.
	Enable_uDMA();

  // Point at the control table to use for channel control structures
  uDMAControlBaseSet(ui8ControlTable);

  // Initialize the uDMA UART transfers
	InitUART1Transfer();

	// Print to program terminal
	UARTprintf("Program Start \n");
	
	while(1){	
		// Receive value of color sensor
		receive = I2CSlaveDataGet(I2C0_BASE);
		// Print value
		UARTprintf("Slave recieves data: %d \n", receive);
		// Check if value is correct. If value is correct write the data as a char
		// and store as data. Increment counter and store data in the transmit buffer.
			if(((char)receive == 'c') | ((char)receive == 'r') | ((char)receive == 'g') | ((char)receive == 'b')){
			data = (char)receive;
			UARTprintf("Data receive: %c \n", data);
			counter++;
			txbufA[0] = data;
		}		
		
		// Check the values in the buffers 
		UARTprintf("TX: %c\n", txbufA[0]);
		UARTprintf("BufA: %c\n", rxbufA[3]);
		UARTprintf("Counter: %d\n\n", counter);
		
		// Increment the color count 
		if((rxbufA[3] == 'c') | (rxbufA[3] == 'r') | (rxbufA[3] == 'g') | (rxbufA[3] == 'b')){
			if(rxbufA[3] == 'c'){
				UARTprintf("clear");				
			}else if(rxbufA[3] == 'r'){
				UARTprintf("Red colour");
				UARTprintf("Red!\n");
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, ui32period + 8000);
				redcolour();
				SysCtlDelay(SysCtlClockGet()/3);
				resetPWM();
			}else if(rxbufA[3] == 'g'){
				UARTprintf("Green colour");
				UARTprintf("Green!\n");
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_0, ui32period + 8000);
				greencolour();
				SysCtlDelay(SysCtlClockGet()/3);
				resetPWM();
			}else if(rxbufA[3] == 'b'){
				UARTprintf("Blue colour");
				UARTprintf("Blue!\n");
				GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_5, 0x20);
				PWMPulseWidthSet(PWM1_BASE, PWM_OUT_4, ui32period + 8000);
				bluecolour();
				SysCtlDelay(SysCtlClockGet()/3);
				resetPWM();
			}
		}
	}
	}	


