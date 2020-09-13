// Author			: Fabian Kung
// Date				: 13 Sep 2020
// Filename			: dsPIC33E_BoardSupport.h

///////////////////////////////////////////////////////////////////////////////////////////////////
//  BEGINNING OF CODES SPECIFIC TO dsPIC33E MICROCONTROLLER        ////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////


// --- Microcontroller specific header file (provided by compiler manufacturer) ---

#include <xc.h>     // This header file will allow access to all include files.
                    // Recommended by Microchip, and also to conform to the practice
                    // of code portability, via the concept of Common C Interface (CCI).
                    // The header file for the microcontroller will be automatically
                    // included with this header file.

#define DSPIC33E_PAGELENGTH 1024    // Page length for dsPIC33E family,
                                    // 1024 24-bits instruction words in one page.
#define DSPIC33E_ROWLENGTH 128      // Row length for dsPIC33E family,
                                    // 1024 24-bits instruction words in one row.

// --- Mapping of variable datatype to match the controller's architecture ---
#define	INT16 	int                 // 16-bits signed integer.
#define INT32   long                // 32-bits signed integer.
#define	UINT16	unsigned int		// 16-bits unsigned integer.
#define UINT32  unsigned long		// 32-bits unsigned integer.
#define	BYTE	unsigned char		// 8-bits unsigned integer.

// --- Microcontroller I/O Pin definitions ---
#define	PIN_OSPROCE1            _RD10 		// Indicator LED1 driver pin.
#define	PIN_ILED2               _RD11 		// Indicator LED2 driver pin.
#define PIN_PSW                 _RB15       // Analog power supply switch control.

// --- Processor Clock and Kernel Cycle in microseconds ---
// Note: Uncomment the required value for _TIMER1COUNT, and update the corresponding definition
// for the constant _SYSTEMTICK_US, in microseconds.

// - For 140 MHz oscillator -
#define	__FOSC_MHz              140             // Oscillator clock frequency in MHz.
#define __FCYC_MHz              __FOSC_MHz/2    // Processor frequency = 70 MHz.
#define	__TCLK_US               0.014286        // Processor clock = fosc/2 = fcyc for dsPIC33E.
                                                // Tclk = 1/70000000 = 1.4286-08.
                                                // Tcyc = (_TIMER1COUNT*(Tclk))= 166.67 usec
                                                
                                                
#if     __FOSC_MHz  == 120                       // No. of Tcyc for Timer1 to expire (for approximately 166.67us cycle).
    #define __TIMER1COUNT           10000
#elif   __FOSC_MHz  == 124
    #define __TIMER1COUNT           10333
#elif   __FOSC_MHz  == 128
    #define __TIMER1COUNT           10666
#elif   __FOSC_MHz  == 132
    #define __TIMER1COUNT           11000
#elif   __FOSC_MHz  == 136
    #define __TIMER1COUNT           11333
#elif   __FOSC_MHz  == 140
    #define __TIMER1COUNT           11666
#else
    #error  "TWSB_header.h: Wrong frequency setting for oscillator"
#endif


#define     __SYSTEMTICK_US         166.67          // System_Tick = (_TIMER1COUNT*(Tclk))

#define     __NUM_SYSTEMTICK_MSEC         6     // Requires 6 system ticks to hit 1 msec period.

///////////////////////////////////////////////////////////////////////////////////////////////////
//  END OF CODES SPECIFIC TO PIC24H/dsPIC33 MICROCONTROLLER  //////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////



#define __SCI_TXBUF_LENGTH      100	// SCI transmitt  buffer length in bytes.
#define __SCI_RXBUF_LENGTH      12	// SCI receive  buffer length in bytes.
#define __RFSCI_TXBUF_LENGTH	32	// RF SCI transmitt  buffer length in bytes.
#define __RFSCI_RXBUF_LENGTH	32	// RF SCI receive  buffer length in bytes.

// Type cast for a Bit-field structure - Serial Communication Interface (SCI) status
typedef struct StructSCI
{
	unsigned bTXRDY: 	1;	// Set to indicate valid data for the wired SCI module to transmit.
	unsigned bRXRDY: 	1;	// Set if there is valid byte data in the wired SCI receive buffer.
	unsigned bRXOVF:	1;	// Set if there is data overflow in wired SCI receive buffer, i.e. old
                            // data has not been read but new data has arrived.
    unsigned bRESERVED1:    1; // Reserved.
} SCI_STATUS;

// Type cast for Bit-field structure - I2C interface status.
typedef struct StructI2CStatus
{
    unsigned bI2CBusy:      1;      // Mutex, set when I2C module is being used.
	unsigned bCommError:    1;      // Set to indicate communication error in the I2C bus.
	unsigned bRead:         1;      // Set to initiate reading of data (Slave -> Master).
    unsigned bSend:         1;      // Set to initiate sending of data (Master -> Slave).
} I2C_STATUS;

// --- RTOS FUNCTIONS' PROTOTYPES ---

// Note: The body of the followings routines is in the file "dsPIC33E_BoardSupport.c"
void ClearWatchDog(void);
void dsPIC33E_PORInit(int);
void OSEnterCritical(void);
void OSExitCritical(void);
void BlinkLED(void);
void UART1Driver(void);
void I2CDriver(void);
void A4988StepperMotorDriver(void);

// --- GLOBAL/EXTERNAL VARIABLES DECLARATION ---
extern  volatile int gnRunTask;
extern  SCI_STATUS gSCIstatus;				// Status for UART and RF serial communication interface.

// Data buffer and address pointers for wired serial communications (UART1).
extern  BYTE gbytTXbuffer[__SCI_TXBUF_LENGTH-1];       // Transmit buffer.
extern  BYTE gbytTXbufptr;                             // Transmit buffer pointer.
extern  BYTE gbytTXbuflen;                             // Transmit buffer length.
extern  BYTE gbytRXbuffer[__SCI_RXBUF_LENGTH-1];       // Receive buffer length.
extern  BYTE gbytRXbufptr;      

// Data buffer and address pointers for wired serial communication (I2C).
#define     __MAX_I2C_DATA_BYTE               16     // Number of bytes for I2C receive and transmit buffer.
#define     __I2C_TIMEOUT_COUNT               25    // No. of system ticks before the I2C routine timeout during
extern  I2C_STATUS  gI2CStat;                           // I2C status.
extern  BYTE    gbytI2CSlaveAdd;                        // Slave address (7 bit, from bit0-bit6).
extern  BYTE    gbytI2CRegAdd;                          // Slave register address.
extern  BYTE    gbytI2CByteCount;                       // No. of bytes to read or write to Slave.
extern  BYTE    gbytI2CRXbuf[__MAX_I2C_DATA_BYTE];      // Data read from Slave register.
extern  BYTE    gbytI2CTXbuf[__MAX_I2C_DATA_BYTE];      // Data to write to Slave register.

// Data structure and global variables for dual A4988 stepper motor driver.
typedef struct StructProceA4988Driver
{
    UINT16  unEn4988;           // Set greater than 0 to enable A4988 module.
                                // Note: User process can reset the driver by setting this
                                // global variable to 0.  Reseting the driver also initialize
                                // all the velocity and distance variables.
    INT16   nSpeed1;            // Speed settings, from 0 (stop the motor) to
    INT16   nSpeed2;            // 800 (maximum speed).  Positive rotate clockwise,
                                // negative rotate anti-clockwise.
} DSPIC33E_A4988_DRIVER;

extern  DSPIC33E_A4988_DRIVER    gobjDriverA4988;
extern  long        gnDistanceMoveLW;   // Distance traveled by left wheel in no. of steps, 32 bits integer.
extern  long        gnDistanceMoveRW;   // Distance traveled by right wheel in no. of steps, 32 bits integer.
extern  long        gnDistanceMoveW;    // Average distance traveled by both wheels in no. of steps, 32 bits integer.
extern  int         gnHeading;          // This indicates the direction, essentially the difference between right and left 
                                    // wheel distance.  Facing right if > 0, and facing left if < 0.			
extern  unsigned int    gunDeadBandThres; // Deadband threshold, speed setting less than this magnitude will be ignored.
#define _DISABLE_A4988                   1
#define _ENABLE_A4988                    0