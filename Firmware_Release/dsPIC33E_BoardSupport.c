///////////////////////////////////////////////////////////////////////////////////////////////////
//
//  APPLICATION PROGRAM INTERFACE ROUTINES FOR dsPIC33EXXXX MICROCONTROLLER
//   
///////////////////////////////////////////////////////////////////////////////////////////////////
//
// Filename         : os_dsPIC33E_BoardSupport.c
// Author           : Fabian Kung
// Last modified	: 28 June 2019
//
// Toolsuites		: Microchip MPLAB X IDE v5.10 or above
//                	  MPLAB XC-16 C-Compiler v1.33 or above
// Microcontroller	: dsPIC33E families.

// Include common header to all drivers and sources.  Here absolute path is used.
// To edit if one changes folder
#include "dsPIC33E_BoardSupport.h"

// --- GLOBAL AND EXTERNAL VARIABLES DECLARATION ---

// --- FUNCTIONS' PROTOTYPES ---
void ClearWatchDog(void);
void dsPIC33E_PORInit(int);

// --- REGISTER FOR STORING PROCESSOR'S INTERRUPT CONTEXT ---
unsigned char bytIEC0bak;
unsigned char bytIEC1bak;
unsigned char bytIEC2bak;


// --- FUNCTIONS' BODY ---

//////////////////////////////////////////////////////////////////////////////////////////////
//  BEGINNING OF CODES SPECIFIC TO dsPIC33EXXXX MICROCONTROLLER	//////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////


// Configuration bits setting for the microcontroller in the header file "p33EP256MU806.h".

#pragma config FCKSM = CSDCMD   // Disable Fail-safe Clock Monitor and Clock-switching.
#pragma config IOL1WAY = OFF    // Allow multiple peripheral pin select configuration.
#pragma config OSCIOFNC = OFF   // OSC2 pin as oscillator output.
#pragma config POSCMD = XT     // Primary oscillator mode is XT (crylstal oscillator).
#pragma config FNOSC = PRIPLL   // Primary oscillator, XT mode with phase-locked loop.
							   // Note: the actual clock frequency is determined by setting the 
							   // feedback divider M, pre-divider N1 and post-divider N2 of the 
							   // PLL system in the Power-On Reset initialization routines.
							   // On power-up the processor will runs at the oscillator frequency,
							   // which is 4MHz in this case.

#pragma config FWDTEN = ON                          // Enable Watch-dog Timer.
//#pragma config FWDTEN = OFF                          // Disable Watch-dog Timer.
#pragma config BOREN = ON //& BOREN_ON              // Enable brown-out reset.
#pragma config FPWRT = PWR32                        // Set power-on reset timer to 32 msec.
//#pragma config GSS = ON                           // User program memory is code-protected.
                                                    // Note: whenever either GSS or GWRP is ON, the GSSK must be ON to
                                                    // ensure proper operation.
#pragma config GSS = ON                             // User program memory is code-protected but still writable.
#pragma config GWRP = OFF                           // Here we enable general section (GS) code protection,
#pragma config GSSK = ON                            // but still allows the user program to modify the general section,
                                                    // e.g. write protection OFF.

// Interrupt Service Routine 
// Author           : Fabian Kung
// Last modified	: 28 June 2019
// Purpose          : 
//
// Arguments		: None
// Return           : None
void __attribute__((__interrupt__)) _T1Interrupt( void )
{
    if (IFS0bits.T1IF)                                                          // If it is Timer 1 overflow interrupt
    { 
		OSEnterCritical();
        gnRunTask = 1;                          
		PR1 = __TIMER1COUNT;				// Load Period Register.
 		IFS0bits.T1IF = 0;					// reset Timer 1 interrupt flag.
		OSExitCritical(); 	
    }
}     

void __attribute__((__interrupt__)) _MathError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _AddressError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _HardTrapError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _StackError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _SoftTrapError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

void __attribute__((__interrupt__)) _DMACError( void )
{
    PIN_ILED2 = 0;						// Turn off indicator LED2.
    PIN_PSW = 0;                        // Turn off analog power supply.
    while (1)
    {
        ClearWatchDog();			    // Clear the Watch Dog Timer.
        PIN_OSPROCE1 = 1; 				// Turn on indicator LED1.
        PIN_ILED2 = 1;					// Turn on indicator LED2.
    }
}

// Function name	: ClearWatchDog 
// Author           : Fabian Kung
// Last modified	: 28 June 2019
// Purpose          : Reset the Watch Dog Timer
// Arguments		: None
// Return           : None
void ClearWatchDog(void)
{
	asm ("clrwdt");	// Inline assembly instruction to clear the Watch Dog Timer.
}

// Function Name	: dsPIC33E_PORInit
// Author           : Fabian Kung
// Last modified	: 28 June 2019
// Description		: Perform power-on reset (POR) initialization on the microcontroller.
//                    Upon completion of this routine, all the microcontroller peripherals and
//                    I/O ports will be set to known state. For I/O ports, all pins will
//                    be set to
//                    (a) Digital mode,
//                    (b) Output and
//                    (c) A logic '0'.
// Arguments		: nProcessorClockMHz - Processor clock frequency, valid values are 120, 130 or 140 MHz.
//                    If not valid input is provided, the processor clock defaults to 120 MHz.
// Return           : None
// 
void dsPIC33E_PORInit(int nProcessorClockMHz)
{

	// Sets the internal phase-locked loop (PLL) for clock generation.
	// Oscillator frequency = Fin = 4MHz.
	// Fclk = Fin (M/(N1XN2).  
	// M = PLL feedback divider = PLLDIV + 2
	// N1 = Pre-divider into PLL = PLLPRE + 2
	// N2 = Post-divider from voltage-controlled oscillator (VCO) of PLL = 2(PLLPOST + 1)
	// If we are setting Fclk to 120MHz for 60MIPS operation (2 clock cycle per instruction).
    // The following selections are used:
    // Option 1: M = 120, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 120 MHz.
    // or PLLDIV = 118 = b'01110110', PLLPRE = 0, PLLPOST = 0.
	// Option 2,3 and 4: For slightly higher clock speed (with slightly higher power dissipation)
    // M = 124, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 124 MHz.
    // or PLLDIV = 122, PLLPRE = 0, PLLPOST = 0.
    // M = 128, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 128 MHz.
    // or PLLDIV = 126, PLLPRE = 0, PLLPOST = 0.
    // M = 132, N1 = 2, N2 = 2 and Fin = 4 MHz, Fclk = 132 MHz.
    // or PLLDIV = 130, PLLPRE = 0, PLLPOST = 0.
    
    if (nProcessorClockMHz == 140)
    {
        PLLFBD = 138; // Clock option 3, sets PLLDIV = 140, Option 3, Fclk = 140 MHz
    }
    else if (nProcessorClockMHz == 130)
    {
        PLLFBD = 128; // Clock option 2, sets PLLDIV = 130, Option 2, Fclk = 130 MHz
    }
    else
    {
        PLLFBD = 118;
        //PLLFBD = 0x0076; // Clock option 1, sets PLLDIV = 120. Option 1, Fclk = 120 MHz.
    }    
    
    CLKDIV = 0x0000; // Sets PLLPRE = 0 and PLLPOST = 0.
                     // Peripheral clock = processor clock.
                     // Internal fast RC oscillator post scaler = 1.
                     // Fcy divided by 1, e.g. processor clock = fosc/2.

	// I/O port setting:
	// Port B setting, all Port B pins are set to outputs by default to prevent floating inputs.
	LATB = 0x0000;
	TRISB = 0x0000;
    ANSELB = 0x0000;        // Set all pins of Port B to digital by default.  This is important as dsPIC33E
                            // sets all analog capable pins to analog mode on power up.
                            // Port B pins are mapped to ADC inputs.

    TRISBbits.TRISB0 = 1;   // Set RB0 to input.  This is the input from voltage reference IC.

	// Port C setting, all Port C pins are set to outputs by default to prevent floating inputs.
	LATC = 0x0000;	
	TRISC = 0x0000;

    // Port D setting, all Port D pins are set to outputs by default to prevent floating inputs.
	LATD = 0x0000;
	TRISD = 0x0000;

    // Port E setting, all Port E pins are set to digital mode, and outputs by
    // default to prevent floating inputs.
	LATE = 0x0000;
	TRISE = 0x0000;
    ANSELE = 0x0000;        // Set all pins to digital be default.  This is important as dsPIC33E
                            // sets all analog capable pins to analog mode on power up.

    // Port F setting, all Port F pins are set to outputs by default to prevent floating inputs.
	LATF = 0x0000;
	TRISF = 0x0000;

    // Port G setting, all Port g pins are set to outputs by default to prevent floating inputs.
	LATG = 0x0000;
	TRISG = 0x0000;
    ANSELG = 0x0000;        // Set all pins of Port G to digital by default.  Port G pins are
                                // mapped to Analog Comparator inputs.
	// Setup 16-bit Timer1:
	// Note - Timer1 will be driven by internal clock cycle.  It will increase up 
	// to the value set in PR1, then reset back to 0. An interrupt will be triggered
	// during this event, known as Timer1 overflow.  
	// Set Timer1 to default state first.
	T1CON = 0;
	// Prescaler = 1:1, valid value 0-3 (2 bits).
	T1CONbits.TCKPS = 0;    // Clock source from internal oscillator (peripheral clock).
	// Load Period Register.
	PR1 = __TIMER1COUNT;	
	// Reset Timer1 register.		
	TMR1 = 0; 
	// reset Timer 1 interrupt flag.
 	IFS0bits.T1IF = 0;
 	// set Timer1 interrupt priority level to 1 (highest priority).
	IPC0bits.T1IP = 1;
	// enable Timer 1 interrupt.
 	IEC0bits.T1IE = 1;
	// Turn on Timer1 and start counting.
	T1CONbits.TON = 1; // Turn on Timer1.
}

// Function name	: OSEnterCritical
// Author           : Fabian Kung
// Last modified	: 28 June 2019
// Description		: Disable all processor interrupts for important tasks
//                    involving Stacks, Program Counter other critical
//	                  processor registers.
void OSEnterCritical(void)
{
	bytIEC0bak = IEC0;	// Store current processor interrupt settings.
	bytIEC1bak = IEC1;
	bytIEC2bak = IEC2;
	IEC0 = 0x00;		// Disable all processor interrupts.
	IEC1 = 0x00;
	IEC2 = 0x00;
}											

// Function name	: OSExitCritical
// Author           : Fabian Kung
// Last modified	: 28 June 2019
// Description		: Enable all processor interrupts for important tasks.
void OSExitCritical(void)
{
    IEC0 = bytIEC0bak;	// Restore processor interrupt settings.
    IEC1 = bytIEC1bak;
    IEC2 = bytIEC2bak;
}											

// Function name	: BlinkLED
// Author           : Fabian Kung
// Last modified	: 28 June 2019
// Description		: Blink an indicator LED1 to show that the microcontroller is 'alive'.
#define _LED1_ON_MS	500		// LED1 on period in msec, i.e. 500 msec.

void BlinkLED(void)
{
    static int  nState = 0;
    static int  nTimer = 1;
    
    if ((nTimer--) == 0)
    {
        switch (nState)
        {
            case 0: // State 0 - On Indicator LED1
                PIN_OSPROCE1 = 1;                           // Turn on indicator LED1.
                nState = 1;
                nTimer = _LED1_ON_MS*__NUM_SYSTEMTICK_MSEC;
            break;

            case 1: // State 1 - Off Indicator LED1
                PIN_OSPROCE1 = 0;                            // Turn off indicator LED1.
                nState = 0;
                nTimer = _LED1_ON_MS*__NUM_SYSTEMTICK_MSEC; 
            break;

            default:
                nState = 0;
                nTimer = 1; 
        }
    }
}

// NOTE: Public function prototypes are declared in the corresponding *.h file.
SCI_STATUS gSCIstatus;				// Status for UART and RF serial communication interface.

//
// --- PUBLIC VARIABLES ---
//
// Data buffer and address pointers for wired serial communications (UART).
BYTE gbytTXbuffer[__SCI_TXBUF_LENGTH-1];       // Transmit buffer.
BYTE gbytTXbufptr;                             // Transmit buffer pointer.
BYTE gbytTXbuflen;                             // Transmit buffer length.
BYTE gbytRXbuffer[__SCI_RXBUF_LENGTH-1];       // Receive buffer length.
BYTE gbytRXbufptr;                             // Receive buffer length pointer.

//
// --- PRIVATE VARIABLES ---
//


//
// --- Process Level Constants Definition --- 
//

#define	_UART_BAUDRATE_kBPS	115.2	// Set datarate in kilobits-per-second.

///
/// Process name	: UART1Driver
///
/// Author          : Fabian Kung
///
/// Last modified	: 25 Oct 2019
///
/// Code version	: 0.99
///
/// Processor		: dsPIC33EP256GMU8XX family.
///                       
///
/// Processor/System Resource 
/// PINS		: 1. Pin RG6 = U1TX (Remappable I/O RP118), output.
///  			  2. Pin RG7 = U1RX (Remappable I/O RP119), input.
///               3. PIN_ILED2 = indicator LED2.
///
/// MODULES		: 1. UART1 (Internal).
///
/// SCHEDULER 	: Round-robin equal-priority scheduling.
///
/// Global variable	: gbytRXbuffer[]
///                   gbytRXbufptr
///                   gbytTXbuffer[]
///                   gbytTXbufptr
///                   gbytTXbuflen
///                   gSCIstatus
///

///
/// Description		: 1. Driver for built-in UART1 Module.
///                   2. Serial Communication Interface (UART) transmit buffer manager.
///                      Data will be taken from the SCI transmit buffer gbytTXbuffer in FIFO basis
///                      and transmitted via USART module.  Maximum data length is determined by the
///			             constant _SCI_TXBUF_LENGTH in file "TWSB_header.h".
///                   3. Serial Communication Interface (UART) receive buffer manager.
///			     Data received from the UART module of the microcontroller will be
///			     transferred from the UART registers to the RAM of the microcontroller
///			     called SCI receive buffer (gbytRXbuffer[]).
///			     The flag bRXRDY will be set to indicate to the user modules that valid
///			     data is present.
///			     Maximum data length is determined by the constant _SCI_RXBUF_LENGTH in
///			     file "TWSB_header.h".
///
///
/// Example of usage : The codes example below illustrates how to send 2 bytes of character,
///			'a' and 'b' via UART1.
///         if (gSCIstatus.bTXRDY == 0)	// Check if any data to send via UART.
///         {
///             gbytTXbuffer[0] = 'a';	// Load data.
///		   	    gbytTXbuffer[1] = 'b';
///		   	    gbytTXbuflen = 2;		// Set TX frame length.
///		  	    gSCIstatus.bTXRDY = 1;	// Initiate TX.
///         }
///
/// Example of usage : The codes example below illustrates how to retrieve 1 byte of data from
///                    the UART1 receive buffer.
///         if (gSCIstatus.bRXRDY == 1)	// Check if UART receive any data.
///		    {
///             if (gSCIstatus.bRXOVF == 0) // Make sure no overflow error.
///			    {
///                 bytData = gbytRXbuffer[0];	// Get 1 byte and ignore all others.
///             }
///             else
///            {
///                 gSCIstatus.bRXOVF = 0; 	// Reset overflow error flag.
///            }
///            gSCIstatus.bRXRDY = 0;	// Reset valid data flag.
///            gbytRXbufptr = 0; 		// Reset pointer.
///			}
///
/// Note: Another way to check for received data is to monitor the received buffer pointer
/// gbytRXbufptr.  If no data this pointer is 0, a value greater than zero indicates the
/// number of bytes contain in the receive buffer.


void UART1Driver(void)
{
    static int  nState = 0;
    static int  nTimer = 1;
    
	if ((nTimer--) == 0)
	{
		switch (nState)
		{
			case 0: // State 0 - UART1 Initialization.
				// Setup IO pins mode and configure the remappable peripheral pins:
                TRISGbits.TRISG7 = 1;                   // Set RG7 (RPI119) to input.
                RPINR18bits.U1RXR = 0b1110111;          // RPI119 tied to U1RX.
                RPOR13bits.RP118R = 0b000001;           // RP118 tied to U1TX.
			    // Note: Once this is done, the controller will 
				// override the I/O mode in the original TRISx 
				// register.
                
				// Setup baud rate generator register, this requires BRGH=1:
				// NOTE: This formula is specified in the manner shown to prevent overflow of the 
				// Integer (16-bits) during the arithmetic operation.
			
				U1BRG = (__FOSC_MHz/8)*(1000/_UART_BAUDRATE_kBPS)-1;
                                
				// Setup UART1 operation mode part 1:
				// 1. Enable UART1 module.
				// 2. Continue module operation in idle, no wake-up enabled.
				// 3. Loopback mode disabled.
				// 4. No auto-baud rate detect.	
				// 5. Only U1TX and U1RX pins are used, no flow control.
				// 6. 8 bits data, even parity, 1 stop bit.
				// 7. Sync break transmission disabled.
				// 8. Interrupt when any character is received (the U1RSR can take in 4 characters).
				// 9. Address detect is disabled.
				U1MODEbits.PDSEL = 0b01;        // 8 bits, even parity.
				U1MODEbits.BRGH = 1;            // High-speed clock generation mode (better).
				U1MODEbits.UARTEN = 1;          // Enable UART.
				U1MODEbits.UEN = 0b00;          // Only U1TX and U1RX pins are used. U1CTS, U1RTS and BCLK
                                                // pins are controlled by latches.
                U1MODEbits.PDSEL = 0b00;        // 8 bits data, no parity.
				U1MODEbits.STSEL = 0;           // 1 stop bit.
                U1MODEbits.UARTEN = 1;          // Enable UART1 module.
				U1STAbits.UTXEN = 1;            // Enable Transmit module in UART.
                //U1STAbits.URXEN = 1;          // NOTE: This bit is not available in all dsPIC33E devices.
				gbytTXbuflen = 0;               // Initialize all relevant variables and flags.
				gbytTXbufptr = 0;
				gSCIstatus.bRXRDY = 0;	
				gSCIstatus.bTXRDY = 0;
				gSCIstatus.bRXOVF = 0;
                //U1STAbits.OERR = 0;
				gbytRXbufptr = 0;
                PIN_ILED2 = 0;                      // Off indicator LED2.
                nState = 1;
                nTimer = 1000; 
			break;
			
			case 1: // State 1 - Transmit and receive buffer manager.
				// Check for data to send via UART.
				// Note that the transmit FIFO buffer is 4-level deep in dsPIC33 and PIC24H micro-controllers.
				if (gSCIstatus.bTXRDY == 1)                         // Check if valid data in SCI buffer.
				{
                    while (U1STAbits.UTXBF == 0)                    // Check if UART transmit buffer is not full.
                                                                    //conditions to exit the load transmit buffer
                                                                    // routines, (1) When transmit butter is full or
                                                                    // (2) when transmit buffer pointer equals
                                                                    // transmit data length.
                    {
                        PIN_ILED2 = 1;                              // On indicator LED2.
                        if (gbytTXbufptr < gbytTXbuflen)
                        {
                            U1TXREG = gbytTXbuffer[gbytTXbufptr];  // Load 1 byte data to UART transmit buffer.
                            gbytTXbufptr++;                         // Pointer to next byte in TX buffer.
                        }
                        else
                        {
                            gbytTXbufptr = 0;                       // Reset TX buffer pointer.
                            gbytTXbuflen = 0;                       // Reset TX buffer length.
                            gSCIstatus.bTXRDY = 0;                  // Reset transmit flag.
                            PIN_ILED2 = 0;                          // Off indicator LED2.
                            break;
                        }
                    }
				}

				// Check for data to receive via UART.
				// Note that the receive FIFO buffer is 4-level deep in dsPIC33 and PIC24H micro-controllers.
                // Here we ignore Framing and Parity error.  If overflow error is detected, the dsPIC33
                // UART receive circuitry will hangs, we need to reset it by clearing the OERR bits in
                // UxSTA register.
                                
                if (U1STAbits.OERR == 0)                            // Make sure no overflow error.
                {
                    while (U1STAbits.URXDA == 1)                    // If at least 1 data byte is
                                                                    // available at UART
                    {
                        PIN_ILED2 = 1;                              // On indicator LED2.
                        if (gbytRXbufptr < (__SCI_RXBUF_LENGTH-1))	// check for data overflow.
                        {                                           // Read a character from USART.
                            gbytRXbuffer[gbytRXbufptr] = U1RXREG;   // Get received data byte.
                            gbytRXbufptr++;                         // Pointer to next byte in RX buffer.
                            gSCIstatus.bRXRDY = 1;                  // Set valid data flag.
                        }
                        else                                        // data overflow.
                        {
                            gbytRXbufptr = 0;                       // Reset buffer pointer.
                            gSCIstatus.bRXOVF = 1;                  // Set receive data overflow flag.
                        }
                        //PIN_ILED2 = 0;
                    }
                }
                else                                            // Overflow/overrun error.
                {
                    U1STAbits.OERR = 0;                         // Clear overrun flag, this will also
                                                                // clear and reset the receive FIFO in
                                                                // the UART module.
                    gbytRXbufptr = 0;                           // Reset buffer pointer.
                    gSCIstatus.bRXOVF = 1;                      // Set receive data overflow flag.
                } 
                nState = 1;
                nTimer = 1; 
			break;

            case 10: // State 10 - Keep on reading the receive buffer to clear the receive status flag.
                     // Note: 1 May 2015, I noticed that upon power up, the UART module may contains invalid
                     // data, and the URXDA flag will be set to 1.  Thus we should clear this before
                     // proceeding. 
                if (U1STAbits.URXDA == 1)
                {
                    gbytRXbuffer[gbytRXbufptr] = U1RXREG;               // Read the receive buffer.
                    nState = 10;
                    nTimer = 1; 
                }
                else
                {
                    nState = 1;
                    nTimer = 1; 
                }
            break;

			default:
                nState = 0;
                nTimer = 1; 
			break;
		}
	}
}


// Data buffer and address pointers for wired I2C serial communications.
#define     __MAX_I2C_DATA_BYTE               16     // Number of bytes for I2C receive and transmit buffer.
#define     __I2C_TIMEOUT_COUNT               25    // No. of system ticks before the I2C routine timeout during
                                                    // read data stage.
//#define     __I2C_BAUD_RATE_MHZ               0.2   // 200 kHz
#define     __I2C_BAUD_RATE_MHZ               0.4   // 400 kHz

I2C_STATUS  gI2CStat;                   // I2C status.
BYTE        gbytI2CSlaveAdd;            // Slave address (7 bit, from bit0-bit6).
BYTE        gbytI2CRegAdd;              // Slave register address.
BYTE        gbytI2CByteCount;           // No. of bytes to read or write to Slave.
BYTE        gbytI2CRXbuf[__MAX_I2C_DATA_BYTE];                // Data read from Slave register.
BYTE        gbytI2CTXbuf[__MAX_I2C_DATA_BYTE];               // Data to write to Slave register.

///
/// Function name	: I2CDriver
///
/// Author          : Fabian Kung
///
/// Last modified	: 25 Oct 2019
///
/// Code Version	: 0.95
///
/// Processor/System Resource
/// PINS            : 1. Pin RF5 = I2C2 SCL, output.
///                   2. Pin RF4 = I2C2 SDA, input/output.
///
/// MODULES         : 1. I2C2 (Internal).
///
/// RTOS            : Ver 1 or above, round-robin scheduling.
///
/// Global Variables    :

///
/// Description	:
/// This is a driver routines for I2C2 module of the dsPIC33E processor.  Due to the pin
/// assignment of the dsPIC33E core, I2C1 module cannot be used as the pins are already
/// in use for other application.  
/// The operation mode is the dsPIC33E processor assume the role of I2C Master, with
/// multiple slave devices.
/// This driver handles the low-level transmit and receive operations. 
/// Ver 0.91, 15 Nov 2015 - Basic version.
/// Ver 0.95, 17 Aug 2016 - Improved efficiency of multi-bytes read.
/// Ver 0.96, 3 April 2018 - Further improvement in efficiency of multi-bytes read.
/// Ver 0.97, 13 July 2018 - Further improve the efficiency of multi-bytes read by 1 clock tick! 

/// I2C bus properties:
/// Baud rate = 200 kHz.
/// Mode: Single Master.
///
/// --- Example of usage: Transmit operation ---
/// Suppose we want to update the registers of a Slave device:
/// Register address 0x20: data = 0xFA
/// Register address 0x21: data = 0xCD
/// The Slave device has an address 0x1E.
///
/// The codes as follows:
/// if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
/// {
///     gbytI2CByteCount = 2;       // Indicate no. of bytes to transmit.
///     gbytI2CRegAdd = 0x20;       // Start address of register.
///     gbytI2CTXbuf[0] = 0xFA;
///     gbytI2CTXbuf[1] = 0xCD;
///     gbytI2CSlaveAdd =  0x1E;
///     gI2CStat.bSend = 1;
/// }
/// The user routine can monitor the flag gI2CStat.bI2CBusy or gI2CStat.bSend.  Once the
/// transmission is completed, both flags will be cleared by the driver.
///
/// --- Example of usage: Receive operation ---
/// Suppose we want to receive 1 byte from the Slave device:
/// Register address 0x30.
/// Slave device address: 0x1E.
///
/// if (gI2CStat.bI2CBusy == 0)     // Make sure I2C module is not being used.
/// {
///     gbytI2CByteCount = 1;       // Indicate no. of bytes to read.
///     gbytI2CRegAdd = 0x30;       // Start address of register.
///     gbytI2CSlaveAdd =  0x1E;
///     gI2CStat.bRead = 1;
/// }
/// The user routine can monitor the flag gI2CStat.bI2CBusy or gI2CStat.bRead.  Once the
/// reception is completed, both flags will be cleared by the driver.  The received
/// data will be stored in gbytI2CRXbuf[0].
/// if (gI2CStat.bRead == 0)       // Check if Read operation is completed.
/// {                              // Read operation complete, check received data.
///     User codes here
/// }
/// else if (gI2CStat.bCommError == 1)  // Check for I2C bus error.
/// {
/// }

void I2CDriver(void)
{
    static int  nState = 0;
    static int  nTimer = 1;    
    static int nIndex = 0;
    static int nTimeOut = 0;
    static int nCount = 0;

    if ((nTimer--) == 0)
    {
        switch (nState)
        {
            case 0: // State 0 - Initialization of I2C2 module and set as Master mode.
                gI2CStat.bCommError = 0;                // Clear error flag.
                gI2CStat.bI2CBusy = 1;                  // Initially indicates I2C module is busy.
                gbytI2CRXbuf[0] = 0;                    // Once it is ready we will clear the busy flag.
                gbytI2CRegAdd = 0;
                gI2CStat.bSend = 0;
                gI2CStat.bRead = 0;

                // Note: 23 July 2015.  According to the datasheet for dsPIC33EP256GP806, the pulse
                // gobbler delay ranges from 65-390 nsec, with 130 nsec being the typical value.
                I2C2BRG = ((1/__I2C_BAUD_RATE_MHZ)-0.13)*(__FCYC_MHz) - 1;
                I2C2CONbits.I2CEN = 1;                  // Enable I2C2 module.
                nIndex = 0;                             // Reset the index.
                nState = 53;
                nTimer = 1000;                
            break;

            case 1: // State 1 - Dispatcher.
                if (gI2CStat.bRead == 1)                  // Reading data from Slave.
                {
                    gI2CStat.bI2CBusy = 1;                  // Indicate I2C module is occupied.
                    nState = 30;
                    nTimer = 1;
                }
                else if (gI2CStat.bSend == 1)             // Transmission of data to Slave.
                {
                    gI2CStat.bI2CBusy = 1;                // Indicate I2C module is occupied.
                    nState = 45;
                    nTimer = 1;                    
                }
                else
                {
                    nState = 1;
                    nTimer = 1;                    
                }
                break;

            // --- Multi-byte read ---
            case 30: // State 30 - Assert Start condition.
                if (I2C2STATbits.P == 1)                    // Make sure the I2C bus is in idle condition.
                {
                    I2C2CONbits.SEN = 1;                    // Assert Start condition on I2C bus.
                    nCount = 0;                             // Reset counter.
                    nState = 31;
                    nTimer = 1;                    
                }
                else
                {
                    nState = 30;
                    nTimer = 1;                    
                }             
            break;

           case 31: // State 31 - Tx slave device address, write mode.
                I2C2TRN = gbytI2CSlaveAdd<<1;            // Data = Slave address with R/W bit = 0 (Master is writing to the Slave).
                nState = 32;
                nTimer = 1;                
           break;

           case 32: // State 32 - Check status of device address transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    nState = 32;
                    nTimer = 1;                      
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, continue.
                    {
                        gI2CStat.bCommError = 0;            // Clear communication error flag.
                        I2C2TRN = gbytI2CRegAdd;            // Send register address to read.
                        nState = 34;
                        nTimer = 1;                          
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        nState = 41;
                        nTimer = 1;                          
                    }    
                }
                break;

           case 34: // State 34 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    nState = 34;
                    nTimer = 1;                             
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, continue.
                    {
                        gI2CStat.bCommError = 0;        // Clear communication error flag.
                        I2C2CONbits.RSEN = 1;                // Assert repeat start condition on I2C bus.
                        nState = 36;
                        nTimer = 1;                                 
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        nState = 41;
                        nTimer = 1;                                 
                    }                   
                }
                break;

           case 36: // State 36 - Tx slave device address, read mode.
                I2C2TRN = (gbytI2CSlaveAdd<<1) | 0x01;     // Data = Slave address with R/W bit = 1 (Master is reading from the Slave).
                nState = 37;
                nTimer = 1;                         
                break;

           case 37: // State 37 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    nState = 37;
                    nTimer = 1;                           
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, continue.
                    {
                        gI2CStat.bCommError = 0;            // Clear communication error flag.
                        I2C2CONbits.RCEN = 1;               // Enable receive operation.
                        nTimeOut = 0;                       // Reset timeout timer.      
                        nState = 39;
                        nTimer = 1;                           
                                                            // Note: we can actually jump to state 38, but this
                                                            // will cause one tick delay.
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        nState = 41;
                        nTimer = 1;                         
                    }
                    
                }
                break;

           case 38: // State 38 - Enable receive.
                I2C2CONbits.RCEN = 1;                    // Enable receive operation.
                nTimeOut = 0;                            // Reset timeout timer.
                nState = 39;
                nTimer = 1;                 
               break;

           case 39: // State 39 - Check for receive buffer full status.
                if (I2C2STATbits.RBF == 1)              // Check if all bits are received.
                {
                    gbytI2CRXbuf[nCount] = I2C2RCV;      // Get received data.
                    gbytI2CByteCount--;
                    nCount++;
                    // Check for end of data to read, or the I2C receive buffer is full.
                    if ((gbytI2CByteCount == 0) || (nCount == __MAX_I2C_DATA_BYTE))
                    {
                        I2C2CONbits.ACKDT = 1;              // Set NAK when receive data.
                        I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.
                        gI2CStat.bI2CBusy = 0;              // I2C module is idle.
                        gI2CStat.bRead = 0;                 // Clear read flag.    
                        nState = 41;
                        nTimer = 1;                             
                    }
                    else                                    // Still in reading mode.
                    {
                        I2C2CONbits.ACKDT = 0;              // Set ACK when receive data.
                        I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.  
                        nState = 38;
                        nTimer = 1;                             
                    }                    
                }
                else
                {
                    nTimeOut++;
                    if (nTimeOut > __I2C_TIMEOUT_COUNT)  // Check for timeout.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        gbytI2CByteCount--;
                        nCount++;
                        // Check of end of data to read, or the I2C receive buffer is full.
                        if ((gbytI2CByteCount == 0) || (nCount == __MAX_I2C_DATA_BYTE))
                        {
                            I2C2CONbits.ACKDT = 1;              // Set NAK when receive data.
                            I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.
                            gI2CStat.bI2CBusy = 0;              // I2C module is idle.
                            gI2CStat.bRead = 0;                 // Clear read flag.      
                            nState = 41;
                            nTimer = 1;                                 
                        }
                        else                                    // Still in reading mode.
                        {
                            I2C2CONbits.ACKDT = 0;              // Set ACK when receive data.
                            I2C2CONbits.ACKEN = 1;              // Enable acknowledge sequence.
                            nState = 38;
                            nTimer = 1;                                 
                        }
                    }
                    else
                    {
                        nState = 39;
                        nTimer = 1;                             
                        //OSSetTaskContext(ptrTask, 39, 1);   // Next state = 39, timer = 1.
                    }
                }
                break;

            case 41: // State 41 - End, initiate Stop condition on I2C bus and tidy up.
                     // Also to hasten the response of this driver, we run the dispatcher in this state.
                     // If there no data to be read from I2C bus, then the state-machine will revert
                     // to state 1 and wait for new commands.  By running the dispatcher here,
                     // the driver can immediately execute another read operation right after the 
                     // stop condition is generated.
                I2C2CONbits.PEN = 1;                    // Initiate Stop condition.
                if (gI2CStat.bRead == 1)                  // Reading data from Slave.
                {
                    gI2CStat.bI2CBusy = 1;                  // Indicate I2C module is occupied.
                    nState = 30;
                    nTimer = 1;                     
                }
                else if (gI2CStat.bSend == 1)             // Transmission of data to Slave.
                {
                    gI2CStat.bI2CBusy = 1;                // Indicate I2C module is occupied.
                    nState = 45;
                    nTimer = 1;                        
                }
                else
                {
                    nState = 1;
                    nTimer = 1;                     
                }                
                break;


            // --- Multi-byte write ---
            case 45: // State 45 - Assert Start condition.
                //gI2CStat.bI2CBusy = 1;                  // Indicate I2C module is occupied.
                nCount = 0;                             // Reset pointer.
                I2C2CONbits.SEN = 1;                    // Assert Start condition on I2C bus.
                nState = 46;
                nTimer = 1;                 
                break;

           case 46: // State 46 - Tx slave device address, write mode.
                I2C2TRN = gbytI2CSlaveAdd<<1;          // Data = Slave address with R/W bit = 0 (Master is writing to the Slave).
                nState = 47;
                nTimer = 1;                   
           break;

           case 47: // State 47 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    nState = 47;
                    nTimer = 1;                       
                }
                else                                    // Transmission end, check acknowledg from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, proceed.
                    {
                        gI2CStat.bCommError = 0;
                        nState = 48;
                        nTimer = 1;                              
                    }
                    else                            // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        nState = 52;
                        nTimer = 1;                              
                    }                
                }
                break;
                
           case 48: // State 48 - TX start register to write to.
                I2C2TRN = gbytI2CRegAdd;                 // Send register address to update.
                nState = 49;
                nTimer = 1;                      
                break;

           case 49: // State 49 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    nState = 49;
                    nTimer = 1;                        
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave, proceed.
                    {
                        gI2CStat.bCommError = 0;
                        nState = 50;
                        nTimer = 1;                            
                    }
                    else                                // NAK received from Slave, end.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        nState = 52;
                        nTimer = 1;                           
                    }                   
                }
                break;

            case 50: // State 50 - TX data to Slave.
                I2C2TRN = gbytI2CTXbuf[nCount];          // Upload data for Slave.
                nState = 51;
                nTimer = 1;                   
                break;

            case 51: // State 51 - Check status of transmission.
                if (I2C2STATbits.TRSTAT == 1)           // Check if Master transmission is still in progress.
                {
                    nState = 51;
                    nTimer = 1;                        
                }
                else                                    // Transmission end, check acknowledge from Slave.
                {
                    if (I2C2STATbits.ACKSTAT == 0)      // ACK received from Slave.
                    {
                        gbytI2CByteCount--;
                        nCount++;
                        gI2CStat.bCommError = 0;
                        // Check for end of byte or reach end of I2C transmit buffer.
                        if ((gbytI2CByteCount == 0) || (nCount == __MAX_I2C_DATA_BYTE))
                        {
                            nState = 52;
                            nTimer = 1;                                
                        }
                        else
                        {
                            nState = 50;
                            nTimer = 1;                                 
                        }
                    }
                    else                                // NAK received from Slave.
                    {
                        gI2CStat.bCommError = 1;        // Set communication error flag.
                        nState = 52;
                        nTimer = 1;                             
                    }
                }
                break;

            case 52: // State 52 - End, initiate Stop condition on I2C bus.
                I2C2CONbits.PEN = 1;                    // Initiate Stop condition.
                nState = 53;
                nTimer = 1;                     
                break;

            case 53: // State 53 - Tidy up.
                gI2CStat.bI2CBusy = 0;                  // I2C module is idle.
                gI2CStat.bSend = 0;
                nState = 1;
                nTimer = 1;                    
                break;

            default:
                nState = 0;
                nTimer = 1;                    
            break;
        }
    }
}


DSPIC33E_A4988_DRIVER    gobjDriverA4988;
long        gnDistanceMoveLW = 0;   // Distance traveled by left wheel in no. of steps, 32 bits integer.
long        gnDistanceMoveRW = 0;   // Distance traveled by right wheel in no. of steps, 32 bits integer.
long        gnDistanceMoveW = 0;    // Average distance traveled by both wheels in no. of steps, 32 bits integer.
int         gnHeading = 0;          // This indicates the direction, essentially the difference between right and left 
                                    // wheel distance.  Facing right if > 0, and facing left if < 0.			
unsigned int    gunDeadBandThres = 1 ; // Deadband threshold, speed setting less than this magnitude will be ignored.
#define _DISABLE_A4988                   1
#define _ENABLE_A4988                    0

///
/// Process name	: A4988StepperMotorDriver
///
/// Author			: Fabian Kung
///
/// Last modified   : 25 Oct 2019
///
/// Code Version	: 0.95
///
/// Processor		: dsPIC33EP256MU80X family.
///                   dsPIC33EP512MC80X family.
///
/// Processor/System Resources 
/// PINS			: 1. RD4 - Step output to A4988 chip 1.
///                   2. RB6 - Direction output to A4988 chip 1. 
///                   3. RD5 - Step output to A4988 chip 2.
///                   4. RB7 - Direction output to A4988 chip 2. 
///                   5. RB9 - Enable pin control for both A4988 chips.
///                            Low = disable FET outputs of both chips.
///                            High = enable FET outputs of both chips.
/// 
/// MODULES			: 1. OC7 (Internal) for Motor1.
///                   2. OC8 (Internal) for Motor2.
///                   3. TIMER3, with 70 MHz processor clock.
///
/// RTOS			: Ver 1 or above, round-robin scheduling.
///
/// Global variables	: gobjDriverA4988
///                       gnDistanceMoveLW
///                       gnDistanceMoveRW
///                       gnDistanceMoveW
///                       gnHeading


/// Description	: Subroutine to drive the A4988 or compatible (such as DRV8825) bi-phase stepper motor driver chips.
///               This driver supports two A4988 chips, e.g. two stepper motors, driven in quarter-step (0.45 
///               degree) mode.
///               The codes make uses of linearization algorithm between the speed setting and 
///               pulse duration adapted from J. Brokking's codes for arduino,
///               in: http://www.brokking.net/yabr_main.html
///               The process uses the internal Output Compare (OC) module of the dsPIC33 processors in 
///               edge-aligned PWM mode to generate the periodic step pulses (OCM<2:0> = 110).  The corresponding
///               OCxTMR counter will increment from 0 to a value equal to OCxRS, then reset and repeat. Thus the
///               periodic frequency of the pulses is determined by value stored in OCxRS. The frequency of the
///               PWM pulses is proportional to the rotational speed needed.  Another I/O pin is used 
///               to control the DIR (direction).  In addition to driving the stepper motor, this process also 
///               keep track of the instantaneous distance traveled by the left and right wheels (assuming no 
///               slippage) by sampling the STEP pins to the stepper motor driver.
///                    
/// Example of usage: 
/// 1) To start the sampling and conversion, we first set gobjDriverA4988.unEn4988 to 1 or larger. 
/// 2) Then we set nSpeed1 or nSpeed2 to between -800 and +800.  Values between -gunDeadBandThres to +gunDeadBandThres
///    will stop the motor.  The driver codes has a linearization routines, which will
///    generate almost linear angular velocity output.  See the Excel file for more detail of the algorithm. 
///
/// 3) The global variables gnDistanceMoveLW, gnDistanceMoveRW, gnDistanceMoveW and gnHeading
///    stores the instantaneous left wheel distance, right wheel distance, average wheel 
///    distance and the heading.  User routines can read this value as frequently as needed.
///    These global variables will be cleared to 0 whenever the driver is disabled.
///    Disabling the driver by setting gobjDriverA4988.unEn4988 to 0 or smaller will reset
///    all these registers to zero.

#define PIN_STEPPER_DIR1                 _RB6 
#define PIN_STEPPER_DIR2                 _RB7 
#define PIN_A4988_ENABLE                 _RB9
#define PIN_STEPPER_STEP1                _RD4
#define PIN_STEPPER_STEP2                _RD5

#if __FOSC_MHz < 116    // Check processor clock frequency, this driver may not work
                        // properly if clock frequency is below 116 MHz.
    #error "Proce_A4988_Driver: Processor frequency is less than 59 MHz, this driver may not work properly"
#endif

#define  T3_PS	64                                                           // TIMER3 Prescalar.

#define	_STEPPER_MOTOR_PULSE_WIDTH      8       // The pulse width for the STEP pulse, in multiples of the period resolution.
                                                // For instance if the period resolution is 20 usec, and this value is 9, then
                                                // each pulse will be 180 usec.  Thus we need to make sure that 
                                                // _STEPPER_MOTOR_MIN_PERIOD_US is greater than this value.  Setting this value
                                                // smaller will allow higher maximum frequency of the STEP control pulse, but 
                                                // beware on the implication on EMC and signal integrity issue that might affect
                                                // the shape of the pulse.
#define _STEPPER_MOTOR_PERIOD_RESOLUTION_US    20      // This is the smallest increment/decrement.
#define _STEPPER_MOTOR_COUNT            _STEPPER_MOTOR_PERIOD_RESOLUTION_US/(__TCLK_US*T3_PS)  // No. of count needed in 
                                                                                               // timer T3 for 1 period
                                                                                               // resolution.
//#define _STEPPER_MOTOR_MIN_SPEED        0
//#define _STEPPER_MOTOR_MAX_SPEED        800


void A4988StepperMotorDriver(void)
{
    static int  nState = 0;
    static int  nTimer = 1;
    int nTemp, nTemp2;
    long nlTemp;
    static int  nLeftOutHigh = 0;
    static int  nRightOutHigh = 0;
    
    if ((nTimer--) == 0)
    {
        switch (nState)
        {
            case 0: // State 0 - Initialization OC7 and OC8 to PWM mode, with clock source 
                    // provided by Timer3.
                
                TRISBbits.TRISB6 = 0;                   // Set RB6 as output.
                TRISBbits.TRISB7 = 0;                   // Set RB7 as output.
                TRISBbits.TRISB9 = 0;                   // Set RB9 as output.
                TRISFbits.TRISF4 = 0;                   // Set RF4 as output.
                TRISFbits.TRISF5 = 0;                   // Set RF5 as output.
                
                PIN_A4988_ENABLE = _DISABLE_A4988;      // First disable all the FET outputs of the A4988 chips.
                
                // Settings of remappable output pin. The cluster of bits RPnR
                // control the mapping of the pin internal peripheral output.
                // Note n = integer.  If we refer to the datasheet, pin RF4
                // is also called RP100, and RP100R bits are contained in
                // the special function register RPOR9 in the controller.
                RPOR2bits.RP68R = 0b010110;             // RP68 or RD4 connected to OC7's output.
                RPOR2bits.RP69R = 0b010111;             // RP69 or RD5 connected to OC8's output.
                //RPOR9bits.RP100R = 0b010110;             // RP100 or RF4 connected to OC7's output.
                //RPOR9bits.RP101R = 0b010111;             // RP101 or RF5 connected to OC8's output.
                
                OC7CON1 = 0;                            // Initialize both control registers for OC7.     
                OC7CON2 = 0;
                OC8CON1 = 0;                            // Initialize both control registers for OC8.     
                OC8CON2 = 0;                
                
                OC8CON1bits.OCSIDL = 1;                 // Output Compare 1 halts when in idle mode.
                OC8CON2bits.SYNCSEL = 0b11111;          // 0x1F, No trigger or sync source is selected.  
                                                        // The internal timer OC8TMR resets when it reaches 
                                                        // the value of OC8RS.
                OC8CON1bits.OCTSEL = 0x01;              // Timer3 provides the clock source. 
                //OC8CON1bits.OCM = 0b110;                // Set OC8 to PWM mode.                 
                                                        // Note: 4/7/2017 F.Kung
                                                        // I discovered that this frequency is given by:
                                                        // fclock = fsource / pre-scalar
                                                        // fsource is the input clock source for Timer 3,
                                                        // which is the peripheral clock in this case.
                OC7CON1bits.OCSIDL = 1;                 // Same settings as OC8.
                OC7CON2bits.SYNCSEL = 0b11111;
                OC7CON1bits.OCTSEL = 0x01;
                //OC7CON1bits.OCM = 0b110;
                OC8RS = 0;  
                OC7RS = 0;
                OC8R = 0;
                OC7R = 0;
                OC8TMR = 0x0000;
                OC7TMR = 0x0000;       
                
                T3CONbits.TSIDL = 1;                    // Stop TIMER3 when in idle mode.
                T3CONbits.TCS = 0;                      // Clock source for TIMER3 is peripheral clock (Tclk/2).
                T3CONbits.TCKPS = 0b10;                 // TIMER3 prescalar = 1:64.
                T3CONbits.TON = 1;                      // Turn on TIMER3.
                TMR3 = 0x0000;                          // Reset TIMER.
                
                gobjDriverA4988.nSpeed1 = 0;
                gobjDriverA4988.nSpeed2 = 0;
                nState = 1;
                nTimer = 100*__NUM_SYSTEMTICK_MSEC;			 
            break;

            case 1: // State 1 - Wait for module to be enabled.
                if (gobjDriverA4988.unEn4988 > 0)       // Check if module is enabled.
                {
                    nState = 2;
                    nTimer = 1;                    
                    if (OC8CON1bits.OCM != 0b110)       // If the OC8 is not in PWM mode,
                    {                                   // set it to PWM mode.
                        OC8CON1bits.OCM = 0b110;        
                    }
                    if (OC7CON1bits.OCM != 0b110)       // If the OC7 is not in PWM mode,
                    {                                   // set OC7 to PWM mode.
                        OC7CON1bits.OCM = 0b110;        
                    }
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;  // Cut-off the power to stepper motor.
                    OC7CON1bits.OCM = 0b000;            // Turn off OC7 and OC8.
                    OC8CON1bits.OCM = 0b000;
                    gobjDriverA4988.nSpeed1 = 0;        // Reset motor 1 and motor 2 speed settings.
                    gobjDriverA4988.nSpeed2 = 0;
                    gnDistanceMoveLW = 0;               // Clear all distance counters.
                    gnDistanceMoveRW = 0;
                    gnDistanceMoveW = 0;
                    gnHeading = 0;                      // Reset heading register.
                    nState = 1;
                    nTimer = 1;                             
                }                
                
                // Update distance counters by polling the OC7 and OC8 output pins.  Also compute average
                // distance and heading.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1)) // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1)              // Check for forward direction.
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                } 
                nlTemp = gnDistanceMoveLW + gnDistanceMoveRW;       // Compute the average distance traveled.  Here a long integer
                gnDistanceMoveW = nlTemp >> 1;                      // is used to prevent overflow. Divide by 2.
                gnHeading = gnDistanceMoveRW - gnDistanceMoveLW;    // Compute the direction or heading.
            break;

            case 2: // State 2 - Scan unSpeed1 and set the OC7 module accordingly.  
                nTemp = gobjDriverA4988.nSpeed1;                
                if (nTemp < 0)      
                {
                    PIN_STEPPER_DIR1 = 1;
                    nTemp = -nTemp;
                }
                else
                {
                    PIN_STEPPER_DIR1 = 0; 
                }
                
                if (nTemp > gunDeadBandThres)      // There is a deadband.  For two-wheels robot 
                {                   // a small deadband can reduce unwanted oscillation.    
                   // NOTE: 18 May 2019, by F. Kung.
                   // The wheel angular velocity is generated by a series of pulses to the STEP input of A4988
                   // chip.  The higher the frequency the faster is the rotation speed.  However the frequency
                   // is inversely proportional to the period of 1 pulse.  This inverse relation between the
                   // set velocity and period is non-linear, thus a compensation routine is needed.
                   // The compensation routine for non-linear relationship between angular 
                   // velocity and pulse period is based J. Brokking codes for 
                   // Arduino. Where for positive speed setting:        
                   // NewSpeedSetting = 805 - 5500/(SpeedSettings + 9)
                   // For negative speed setting:
                   // NewSpeedSetting = -805 - 5500/(SpeedSettings - 9)
                   // Since this is inversely proportional to the step interval, 
                   // we subtract from the maximum time interval.
                   // Here the first coefficient 5500 can be increase or decrease to change the slope of the mapping 
                   // between set velocity and actual motor shaft rotation speed.  See the corresponding Excel file
                   // for further information.
                   PIN_A4988_ENABLE = _ENABLE_A4988;
                   nTemp2 = 11000/(nTemp+9);
                   OC7R = _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT;      // Set OC7 output pin to high. The width of the 
                                                                                // the pulse is determined by 
                   OC7RS = nTemp2*_STEPPER_MOTOR_COUNT;                         // _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT
                                                                                // while the period is set by OC7RS value.  We must
                                                                                // make sure OC7RS >= OC7R at all times! 
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;                          // Cut-off the power to stepper motor when
                                                                                // not turning.  This depends on application, 
                                                                                // it means there will be no holding torque
                                                                                // when the motor is not turning. Turning the motor
                                                                                // off when not moving reduces power consumption
                                                                                // and makes the stepper motor cooler.       
                    OC7R = 0;
                    OC7RS = 0;                    
                }                                                               
                                                                                               
                // Note: 30 Aug 2017, the above codes to set the Output Compare unit needs to execute first
                // before we update the distance ticks.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }                
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1))    // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1)              // Check for forward direction.
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                }                    
                nState = 3;
                nTimer = 1;                          
            break;

            case 3: // State 3 - Scan unSpeed2 and set the OC8 module accordingly. 
                nTemp = gobjDriverA4988.nSpeed2;                
                if (nTemp < 0)
                {
                    PIN_STEPPER_DIR2 = 0;
                    nTemp = -nTemp;
                }
                else
                {
                    PIN_STEPPER_DIR2 = 1; 
                }
                if (nTemp > gunDeadBandThres)      // There is a deadband.
                {                    
                   PIN_A4988_ENABLE = _ENABLE_A4988;
                   nTemp2 = 11000/(nTemp+9);               
                   OC8R = _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT;      // Set OC8 output pin to high.  The width of the 
                                                                                // pulse is determined by the value
                   OC8RS = nTemp2*_STEPPER_MOTOR_COUNT;                         // _STEPPER_MOTOR_PULSE_WIDTH*_STEPPER_MOTOR_COUNT
                                                                                // while the value in OC8RS sets the period. We must
                                                                                // make sure OC8RS >= OC8R at all times!    
                }
                else
                {
                    PIN_A4988_ENABLE = _DISABLE_A4988;                          // Cut-off the power to stepper motor when
                                                                                // not turning.  This depends on application,
                                                                                // it means there will be no holding torque
                                                                                // when the motor is not turning. Turning the motor
                                                                                // off when not moving reduces power consumption
                                                                                // and makes the stepper motor cooler.
                    OC8R = 0;                                                   // By setting OC8R = OC8RS, output pin of OC8
                    OC8RS = 0;                                                  // will be driven to low when not active.
                }                                                                
                                                                                
                // Note: 30 Aug 2017, the above codes to set the Output Compare unit needs to execute first
                // before we update the distance ticks.
                if ((nLeftOutHigh == 0) && (PIN_STEPPER_STEP1 == 1)) // Check for low-to-high transition on left motor output.
                {
                    if (PIN_STEPPER_DIR1 == 0)          // Check for forward direction.
                    {
                        gnDistanceMoveLW++;
                    }
                    else
                    {
                        gnDistanceMoveLW--;
                    }
                    nLeftOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nLeftOutHigh == 1) && (PIN_STEPPER_STEP1 == 0))
                {
                    nLeftOutHigh = 0;
                }                
                if ((nRightOutHigh == 0) && (PIN_STEPPER_STEP2 == 1)) // Check for low-to-high transition on right motor output.
                {
                    if (PIN_STEPPER_DIR2 == 1) 
                    {
                        gnDistanceMoveRW++;
                    }
                    else
                    {
                        gnDistanceMoveRW--;
                    }
                    nRightOutHigh = 1;                   // Indicate that low-to-high transition has already occur.
                }
                else if ((nRightOutHigh == 1) && (PIN_STEPPER_STEP2 == 0))
                {
                    nRightOutHigh = 0;
                }      
                nState = 1;
                nTimer = 1;                                    
                break;
                
            case 10: // State 10 - Stop the motor.
                OC7CON1bits.OCM = 0b000;
                OC8CON1bits.OCM = 0b000;
                nState = 1;
                nTimer = 1;                  
                break;
                
            default:
                nState = 0;
                nTimer = 1;                  
            break;
        }
    }
}


//////////////////////////////////////////////////////////////////////////////////////////////
//  END OF CODES SPECIFIC TO dsPIC33EXXXX MICROCONTROLLER	   ///////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////
