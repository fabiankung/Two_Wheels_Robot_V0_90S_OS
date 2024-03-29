
//
// Filename         : Main.c
// Author(s)		: Fabian Kung
// Last modified	: 29 Nov 2020
// Release Version	: 0.901 (This refers to OS version or release)
// Description		: Main C file for the RTOS.
//                    Uses round-robin scheduling algorithm to schedule tasks.
// Toolsuites		: Microchip MPLAB X IDE v5.35 or above
//                	  MPLAB XC16 C-Compiler v1.50 or above
//                    MPLAB PICkit 4, MPLAB Snap.
// Microcontroller	: dsPIC33EP256MU806 16-bits, 64 pins, 28kbyte RAM, 256kbyte Flash ROM, 70 MIPS.
// Oscillator/Clock	: 4 MHz crystal (XT mode) with Phase-Locked Loop multiplier to 140 MHz (70 MIPS)
//					  
// Main Features:
// 1) MCU running at 70 MIPS.
// 2) Equal priority multi-tasking.
// 3) Hardware watchdog timer.
// 4) Systick timer.
// 5) Ability to assign a timer for all processes, resolution of 1 Systick.

///////////////////////////////////////////////////////////////////////////////
// --- INCLUDE ALL DRIVER AND USER HEADER FILES --- ///////////////////////////
///////////////////////////////////////////////////////////////////////////////

// Include common header to all drivers and sources.  Here absolute path is used for library source.
// To edit if one change folder.

// Main header file.  All source codes should include this.
///////////////////////////////////////////////////////////////////////////////
#include "dsPIC33E_BoardSupport.h"
#include "RobotTasks.h"

volatile int gnRunTask = 0;

// --- MAIN LOOP ---

int main(void)
{
	
	OSEnterCritical();			// Disable and store all processor's interrupt setting.
	OSEnterCritical();			// Do this twice to prevent interrupt happen midway, and the 
                                // processor interrupt is re-enabled upon return from the 
                                // interrupt service routine.
	dsPIC33E_PORInit(140);		// Power-on Reset initialization for the controller, set processor
                                // clock speed to 140 MHz.  Set all I/O pins to default state and
                                // enable Timer 1.

    while (1)                   // Infinite loop.
    {
        ClearWatchDog();		// Clear the Watch Dog Timer.
        if (gnRunTask == 1)
        {
            BlinkLED();
            UART1Driver();
            I2CDriver();            
            A4988StepperMotorDriver(); 
            Robot_Sensor_MPU6050();            
            Robot_Balance();
            Robot_MoveLinear();
            Robot_Turn();
            Robot_HighLevelProcess();
            gnRunTask = 0;                 
        }
    }
    return 0;
}

