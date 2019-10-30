
#include "dsPIC33E_BoardSupport.h"

// Include MPLAB XC16 standard libraries.
#include <math.h>

// --- Tilt Angle and Orientation Parameters as measured by IMU ---
int     gnStatus_IMU = 0;               // IMU status.  It is recommended to check this status
                                        // upon power up to make sure the IMU is READY, otherwise
                                        // the outputs from the IMU are not valid.
#define     _READY          1
#define     _NOT_READY      0
#define     _ERROR          -1

float   gfWg_MPU6050;                   // Instantaneous tilt angular velocity from MPU6050 gyroscope in radian/sec.
float   gfTheta_MPU6050;                // Robot's tilt angle in radian, from MPU6050, in radian.
int     gnTheta_Deg_MPU6050;            // Robot's tilt angle in degree, from MPU6050, in degree.

int     gnTiltOrient_IMU = -5;          // Robot tilt orientation
#define     _ROR_UPRIGHT        0       // Upright (within +- 3 deg of normal, i.e. 0 degree).
#define     _ROR_LEAN_FRONT     2       // Lean to the front (+4 to +29 degrees).
#define     _ROR_LEAN_BACK      -2      // Lean to the back (-4 to -29 degrees).
#define     _ROR_TOPPLE_FRONT   5       // Toppled to the front (> 29 degrees).
#define     _ROR_TOPPLE_BACK    -5      // Toppled to the back (< -29 degrees).

// --- Primary Control Loop Coefficients: Balancing Feedback Control Parameters ---
float 	gfE;                            // Error variable.
int   	gnC;                            // Control variable for motor driver, general.
int     gnCL;                           // Control variable for motor driver, left wheel (after adding offset).
int     gnCR;                           // Control variable for motor driver, right wheel (after adding offset).
                                        // Coefficients for state-space feedback control.
float 	gfR = 0.00;                     // Reference tilt angle in radian.
float	gfP = 0.00;                     // Coefficient for proportional term.
float	gfI = 0.00;                     // Coefficient for integral term.
float 	gfCwg = 0.00;                   // Coefficient for difference term.
int     gnControlCoefficient;           // This coefficient takes into account the stepper motor mode, whether it is
                                        // half step, quarter step or eight step.
// --- Secondary Control Loop Coefficients: Turning, PD.
int     gnKp_Turn = 0;                  // Coefficient for turning PD control loop.
int     gnKd_Turn = 0;

// --- Secondary Control Loop: Linear Movement Speed Regulation Feedback Control Parameters ---
float	gfRoffset = 0.00;               // Offset for reference angle in radian.
int   	gnCLoffset = 0;                 // Offset for control voltage (in mV) to left wheel motor driver.
int     gnCRoffset = 0;                 // Offset for control voltage (in mV) to right wheel motor driver.
INT32   gnlDistanceSet = 0;             // Robot linear traveled distance setting in no. of ticks from quadrature encoder.

INT16   gnHeadingSet = 0;               // Robot heading (direction) setting.
                                        // 0 = Look straight ahead
                                        // > 0 turn left.
                                        // < 0 turn right.
INT16   gnContinuousTurn = 0;           // To enable continuous turning or not.
                                        // 0 = Disable continuous turning.
                                        // > 0 continuous turn left.  The magnitude indicates the rate of turning.
                                        // Currently magnitude of 1 and 2 are supported only.
                                        // < 0 continuous turn right.  The magnitude indicates the rate of turning.
int     gnOmegaWSet = 0;                // Robot wheel rotation velocity setting in rotation/sec.
                                        // > 0.0 move forward.
                                        // < 0.0 move backward. 
int     gnOmegaW = 0;
int     gnKp_Move = 0;
int     gnKd_Move = 0;
int     gnKi_Move = 0;
 
#define PIN_HC_05_RESET                 _RF0                    // Pin RF0 = Reset pin for HC-05 bluetooth module.

#define PIN_MS1                        _RB8
#define _STEPPER_MOTOR_QUARTER_STEP     0
#define _STEPPER_MOTOR_EIGHTH_STEP      1

#define     _ON_ANALOG_POWER    1   // We follow the C-language convention of true (not zero) and false (zero)
#define     _OFF_ANALOG_POWER   0
#define     _ENABLE             1   
#define     _DISABLE            0

int     gnRobotBalance = _DISABLE;                  // 0 = Disable/off balance module.
                                                    // 1 or other = Power up balancing module. 


///
/// Process name	: Robot_Sensor_MPU6050
///
/// Author          : Fabian Kung
///
/// Last modified	: 30 Oct 2019
///
/// Code Version	: 0.80
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  I2C pins
///
/// MODULE          :  MPU6050 external module.
///
/// DRIVER          :  I2CDriver
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variable : 
///

/// Description     :  This is a driver for Invensense 6-axis IMU chip, the MPU6050.
///                    If there is no delay in each states, the sampling rate is 4.0 msec, 
///                    e.g. the accelerometer and gyroscope outputs are sample at 4.0 msec interval.  
///                    Complementary Filter techniques are used to fuse the outputs of accelerometer 
///                    and gyroscope to estimate the actual inclination angle.
///                     
/// Example of usage: 
///                   gfTheta_MPU6050 gives the angle measured along Y axis in radian.
///                   gnTheta_Deg_MPU6050 gives the same angle in degree.
///                   gnTiltOrient_IMU gives the orientation (in enumerated form).
///                   These parameters can be read as frequent as required, but their values will only
///                   be updated every 4.0 msec.


#define     __MPU6050_I2C_ADDRESS       0x68    // I2C slave address of MPU6050 (Assume pin AD0 is shorted to GND).

#define	_MPU6050_SMPLRT_DIV     0x19	//125Hz
#define	_MPU6050_CONFIG			0x1A	//0x06(5Hz)
#define	_MPU6050_GYRO_CONFIG	0x1B	//2000deg/s
#define	_MPU6050_ACCEL_CONFIG	0x1C	//5Hz
#define	_MPU6050_ACCEL_XOUT_H	0x3B
#define	_MPU6050_ACCEL_XOUT_L	0x3C
#define	_MPU6050_ACCEL_YOUT_H	0x3D
#define	_MPU6050_ACCEL_YOUT_L	0x3E
#define	_MPU6050_ACCEL_ZOUT_H	0x3F
#define	_MPU6050_ACCEL_ZOUT_L	0x40
#define	_MPU6050_TEMP_OUT_H		0x41
#define	_MPU6050_TEMP_OUT_L		0x42
#define	_MPU6050_GYRO_XOUT_H	0x43
#define	_MPU6050_GYRO_XOUT_L	0x44	
#define	_MPU6050_GYRO_YOUT_H	0x45
#define	_MPU6050_GYRO_YOUT_L	0x46
#define	_MPU6050_GYRO_ZOUT_H	0x47
#define	_MPU6050_GYRO_ZOUT_L	0x48
#define	_MPU6050_PWR_MGMT_1		0x6B	//
#define	_MPU6050_WHO_AM_I		0x75	//
#define	_MPU6050_SlaveAddress	0xD0	


 void Robot_Sensor_MPU6050(void)
{
    static int  nState = 0;
    static int  nTimer = 1;
    
    unsigned int unTemp;
    static  int     nAccelRaw, nGyroRaw;
    static  float   fTemp1, fTemp2, fTemp3;
    static  float   fAngleAccel;
    static  float   fTheta, fTheta1;
    
    nTimer--;
    if (nTimer == 0)
    {
        switch (nState)
        {
            case 0: // State 0 - Initialization.
                gbytI2CSlaveAdd =  __MPU6050_I2C_ADDRESS;               // Initialize slave device I2C address. 
                fTheta = 0.0;
                fTheta1 = 0.0;
                nAccelRaw = 0.0;
                nGyroRaw = 0.0;
                gfWg_MPU6050 = 0.0;
                gfTheta_MPU6050 = 0.0;
                gnTheta_Deg_MPU6050 = 0;
                nState = 1;
                nTimer = 100*__NUM_SYSTEMTICK_MSEC;
                break;
            
            case 1: // State 1 - Initialize MPU6050.
                    // Set clock source to internal PLL (from X-gyroscope).
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_PWR_MGMT_1;                // Start address of register.
                    gbytI2CTXbuf[0] = 0x01;                             // Data.  Wakes up device and set internal clock to X gyro PLL.
                    //gbytI2CTXbuf[0] = 0x00;                             // Data.  Wakes up device and set internal clock to 8 MHz RC oscillator.
                    gI2CStat.bSend = 1;
                    nState = 2;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                    
                }                
                else
                {
                    nState = 1;
                    nTimer = 1;                    
                }                
                break;

            case 2: // State 2 - Initialize MPU6050.
                    // Set sample rate for both accelerometer and gyroscope.
                    // Sample rate = (Actual output rate)/(1 + _MPU6050_SMPLRT_DIV)
                    // According to Jeff Rowland, the sample rate for accelerometer is 1 kHz.
                    // For gyroscope it is 8 kHz if the digital LPF (dLPF) is disabled, and 1 kHz
                    // if dLPF is enable.  I suppose this is an 8 tabs IIR filter.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_SMPLRT_DIV;                // Start address of register.
                    //gbytI2CTXbuf[0] = 0x02;                             // Data - sample rate = 333.33 sample/sec.
                    //gbytI2CTXbuf[0] = 0x03;                             // Data - sample rate = 250 sample/sec.
                    //gbytI2CTXbuf[0] = 0x04;                             // Data - sample rate = 200 sample/sec.
                    gbytI2CTXbuf[0] = 0x00;                             // Data - sample rate = 1000 sample/sec.
                    //gbytI2CTXbuf[0] = 0x06;                             // Data - sample rate = 142.857 sample/sec.
                    gI2CStat.bSend = 1;
                    nState = 3;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                    
                }                
                else
                {
                    nState = 2;
                    nTimer = 1;
                }                
                break;
                
            case 3: // State 3 - Initialize MPU6050 - Set external synchronization and digital LPF bandwidth.
                
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_CONFIG;                    // Start address of register.
                    //gbytI2CTXbuf[0] = 0x01;                           // Data - disable external synchronization, BW around 194 Hz.
                    //gbytI2CTXbuf[0] = 0x02;                             // Data - disable external synchronization, BW around 94 Hz.
                    //gbytI2CTXbuf[0] = 0x03;                             // Data - disable external synchronization, BW around 44 Hz.
                    gbytI2CTXbuf[0] = 0x04;                             // Data - disable external synchronization, BW around 20 Hz.
                    //gbytI2CTXbuf[0] = 0x05;                             // Data - disable external synchronization, BW around 10 Hz.
                    //gbytI2CTXbuf[0] = 0x06;                             // Data - disable external synchronization, BW around 5 Hz.
                    gI2CStat.bSend = 1;
                    nState = 4;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                    
                }                
                else
                {
                    nState = 3;
                    nTimer = 1;                    
                }                
                break;                

            case 4: // State 4 - Initialize MPU6050 - Set full-scale range of gyroscope.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_GYRO_CONFIG;               // Start address of register.
                    gbytI2CTXbuf[0] = 0x00;                             // Data - +-250 deg/sec.
                    //gbytI2CTXbuf[0] = 0x08;                             // Data - +-500 deg/sec.
                    //gbytI2CTXbuf[0] = 0x10;                             // Data - +-1000 deg/sec.
                    //gbytI2CTXbuf[0] = 0x18;                             // Data - +-2000 deg/sec.
                    gI2CStat.bSend = 1;
                    nState = 5;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                       
                }                
                else
                {
                    nState = 4;
                    nTimer = 1;                     
                }                
                break;   

            case 5: // State 5 - Initialize MPU6050 - Set full-scale range of accelerometer.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 1;                               // Indicate no. of bytes to transmit.
                    gbytI2CRegAdd = _MPU6050_ACCEL_CONFIG;              // Start address of register.
                    //gbytI2CTXbuf[0] = 0x00;                             // +-2g.
                    gbytI2CTXbuf[0] = 0x08;                             // +-4g.
                    gI2CStat.bSend = 1;
                    nState = 6;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                           
                }                
                else
                {
                    nState = 5;
                    nTimer = 1;                       
                }                
                break;
                
            case 6: // State 6 - Read accelerometer data.
                    // Also set the robot orientation parameter gnTiltOrient_IMU.  
                    // Note:  Actually this can be done at any stages in the tilt angle computation.  
                    // Most other states (from state 7 to 11) has some floating points computations, 
                    // so the loading is high.  Thus we slot this routine in State 6 as it has no 
                    // floating point computation if this routine is not present.           
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 2;                               // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _MPU6050_ACCEL_YOUT_H;              // Start address of register.
                    gI2CStat.bRead = 1;                                 // Start read.
                    nState = 7;
                    nTimer = 10;                         
                }                
                else
                {
                    nState = 6;
                    nTimer = 1;                         
                }
                
                gnTheta_Deg_MPU6050 = fTheta*57.2958;                   // Tilt angle in degree.
                gfTheta_MPU6050 = fTheta;                               // Tilt angle in radian.
                
                if (gnTheta_Deg_MPU6050 > 30)                       // Check tilt angle for threshold of leaning forward.
                {
                    gnTiltOrient_IMU = _ROR_TOPPLE_FRONT;           // Robot topples to the front.
                }
                else if (gnTheta_Deg_MPU6050 > 4)
                {
                    gnTiltOrient_IMU = _ROR_LEAN_FRONT;             // Robot leans to the front.
                }
                else if (gnTheta_Deg_MPU6050 >= -4)
                {
                    gnTiltOrient_IMU = _ROR_UPRIGHT;                // Robot is upright.
                }
                else if (gnTheta_Deg_MPU6050 > -30)                 // Check tilt angle for threshold of leaning back.
                {
                    gnTiltOrient_IMU = _ROR_LEAN_BACK;              // Robot leans to the back.
                }
                else
                {
                    gnTiltOrient_IMU = _ROR_TOPPLE_BACK;           // Robot topples to the back.               
                }                     
                break;
                                
            case 7: // State 7 - Read accelerometer data.
                if (gI2CStat.bRead == 0)                                // Check if Read operation is completed.
                {                                                       // Read operation complete, check received data.
                    unTemp = gbytI2CRXbuf[0]<<8;                        // Get upper 8 bits.                                //
                    nAccelRaw = unTemp + gbytI2CRXbuf[1];               // Form 16 bits unsigned integer by adding lower 8 bits. 
                                                                        // Note that the data is 16-bits 2's complement format.
                    if (nAccelRaw > 8192)                               // Limit the raw accelerator output, for +-4g full scale.
                    {
                        nAccelRaw = 8192;
                    }
                    if (nAccelRaw < -8192)
                    {
                        nAccelRaw = -8192;
                    }    
                    nState = 8;
                    nTimer = 1;                         
                    fTemp1 = 0.00012207*nAccelRaw;                      // Convert to g, where 1/8192 = 0.00012207
                }
                else
                {
                    nState = 7;
                    nTimer = 1;                         
                }
                break;
                
            case 8: // State 8 - Read gyroscope data.
                if (gI2CStat.bI2CBusy == 0)                             // Make sure I2C module is not being used.
                {
                    gbytI2CByteCount = 2;                               // Indicate no. of bytes to read.
                    gbytI2CRegAdd = _MPU6050_GYRO_XOUT_H;               // Start address of register.
                    gI2CStat.bRead = 1;                                 // Start read. 
                    nState = 9;
                    nTimer = 10;                         
                    fAngleAccel = asinf(fTemp1);                        // Convert to radian.
                }                
                else
                {
                    nState = 8;
                    nTimer = 1;                         
                }                  
                break;

            case 9: // State 9 - Read gyroscope data.
                if (gI2CStat.bRead == 0)                                // Check if Read operation is completed.
                {                                                       // Read operation complete, check received data.
                    unTemp = gbytI2CRXbuf[0]<<8;                        // Get upper 8 bits.                                //
                    nGyroRaw = unTemp + gbytI2CRXbuf[1];                // Form 16 bits unsigned integer by adding lower 8 bits. 
                                                                        // Note that the data is 16-bits 2's complement format.
                    nState = 10;
                    nTimer = 1;                            
                    gfWg_MPU6050 = 0.0001332*nGyroRaw;                  // Sensitivity = 1/131 or 0.0076336 for +-250 dps full scale.
                                                                        // Since our unit is radian not degree. We must multiply
                                                                        // by (pi/180) or 0.017453.  Thus the total coefficient is
                                                                        // 0.017453*0.0076336 = 0.0001332.
                }
                else
                {
                    nState = 9;
                    nTimer = 1;                         
                }
                break;
                
            case 10: // State 10 - Compute tilt angle using Complementary Filter approach.
                fTheta1 = fTheta;                               // Store previous tilt angle sample.
                fTemp1 = gfWg_MPU6050*0.004;                    // Theta(n) = a(Theta(n-1) + W*Delta_t) + (1-a)(ThetaAcc(n))
                                                                // Deltat = 4 ms.
                fTemp2 = fTheta1 + fTemp1;                      // The time-constant for the low-pass and high-pass filters      
                nState = 11;
                nTimer = 1;                     
                break;

            case 11: // State 11 - Combine outputs from accelerometer and gyroscope using Complementary Filters
                    // approach.    
                fTemp2 = 0.994*fTemp2;			
                fTemp3 = 0.006*fAngleAccel;		 
                fTheta = fTemp2 + fTemp3;                       // Work out final tilt angle.
                
                gnStatus_IMU = _READY;                          // Indicate IMU module has valid output.
                nState = 6;
                nTimer = 1;                           
            break;
            
            default:
                nState = 0;
                nTimer = 1;                     
            break;                
        }
    }
}

// --- Primary feedback control loop coefficients ---
// These constants need to be tuned to fit the physical characteristics of the robot.
    #define     _FR_DEFAULT         -0.000    // Default set tilt angle, in radian.
    
// Gain of 10, quarter-step mode.
    #define     _FP_DEFAULT_4        -64.00   // Proportional gain, for delta1 = 1 msec.
    #define     _FI_DEFAULT_4        -0.500   // Integral gain, for deltat = 1 msec
    #define     _FCW_DEFAULT_4       -0.500    // Differential gain, for deltat = 1 msec

// --- Secondary feedback control loop coefficients ---
    
    #define     _FP2_MOVE_DEFAULT_4   4
    #define     _FD2_MOVE_DEFAULT_4   0
    #define     _FI2_MOVE_DEFAULT_4   1
 

    #define     _STEPPER_MOTOR_COEFFICIENT_2    1  // For half-step mode.
    #define     _STEPPER_MOTOR_COEFFICIENT_4    2  // For quarter-step mode.

    #define     _MOTOR_DRIVER_MAX_SPEED    800            // Max and minimum speed settings
    #define     _MOTOR_DRIVER_MIN_SPEED    -800           // for stepper motor driver.
    #define     _MOTOR_DRIVER_TURN_OFFSET_MAX  85
    #define     _MOTOR_DRIVER_TURN_OFFSET_MIN  -85

     
///
/// Process name	: Robot_Balance
///
/// Author          : Fabian Kung
///
/// Last modified	: 29 Oct 2019
///
/// Code version	: 0.75
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  None
///
/// MODULE          :  Stepper Motor Driver (External)
///                    IMU (External)
///
/// DRIVER          :  Proce_A4988_Driver
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variables	:  gnRobotBalance
///                        gnMotComp_mV
///                        gnCLoffset
///                        gnCRoffset
///                        gfTheta_IMU or gfTheta_MPU6050
///                        gfWg_IMU (optional)
///                        gfRoffset
///                        gfR
///                        And the feedback control coefficients
///
/// Description	: User process - Self-balancing algorithm using PID discrete-time
///               feedback control is implemented here.  The global variable gnRobotBalance
///               determines whether to activate or deactivate the motor driver/wheels.
///               The interval to execute the control tasks is 1.0 msec.
///
///               The PID control equation is as follows:
///               1. Thetaset = User set tilt angle.
///               2. deltat = time step.
///               3. Theta = Actual robot tilt angle.
///               4. Error = Thetaset - Theta
///               5. d(Error)/dt = d(Thetaset)/dt - d(Theta)/dt
///               Since Thetaset is almost constant (the higher order routines needs to be much slower
///               than the balancing routines), d(Error)/dt = -d(Theta)/dt
///               6. Therefore  
///                  Output = Kp*Error + Kd*d(Error)/dt + Ki*(Error*deltat)
///               => Output = Kp*Error - Kd*d(Theta)/dt + (Ki*deltat)*Error
/// 
///               In our implementation we add a scale factor:
///                 Output = Kscale*(Kp*Error - Kd*d(Theta)/dt + (Ki*deltat)*Error)
///
///               Here Kscale = 10S, gfE = Error, gfP = Kp, gfCwg = Kd, gfI = Ki*deltat
///               in the codes.  S is an integer, depending on the step size of the
///               stepper motor.  S = 1 for half step, 2 for quater step and 4 for eighth
///               step.  The purpose of having Kscale is so that the PID
///               coefficients can be within the valid range, especially when we are using fix-point
///               approach to represent the control coefficients globally (this is for ease 
///               of communication with other routines).  
///               
///               For this version we can use either analog IMU or the digital IMU, MPU6050,
///               Thus in State 2 there is option to use either angles for error computation.
///               You should uncomment the relevant code.


void Robot_Balance(void)
{
    static int  nState = 0;
    static int  nTimer = 1;
    
    static float fC = 0.00;     // The output in floating point.
    static long nC;             // The output in integer.  A long integer format is used to prevent
                                // overflow during calculation.    
    //static int nC;              // The output in integer.
    static int nCR, nCL;		// The control voltage output for right and left motors,
                                // in mV.
    static float   fE1;		
    static float   fE2;
    static float   fEOld;
    static float   dfE;
 

    nTimer--;                   // Decrement timer.
    if (nTimer == 0)
    {
        switch (nState)
        {
            case 0: // State 0 - Initialization.
                gfR = _FR_DEFAULT;                  // Reference in radian.
                gfP = _FP_DEFAULT_4;                // Proportional coefficient.
                gfI = _FI_DEFAULT_4;                // Integral coefficient.
                gfCwg = _FCW_DEFAULT_4;             // Coefficient for measured angular velocity.
                gnCLoffset = 0;                     // Initialized left and right wheels offset voltage
                gnCRoffset = 0;                     // for turning.
                gnRobotBalance = _DISABLE;
                gfRoffset = 0.0;
                gfE = 0.0;                            // Reset all error terms.
                fE1 = 0.0;
                fE2 = 0.0;
                fEOld = 0.0;
                
                gobjDriverA4988.unEn4988 = 1;       // Enable the stepper motor driver (if not enabled).
                gobjDriverA4988.nSpeed1 = 0;        // Turn off both left and right stepper motors.
                gobjDriverA4988.nSpeed2 = 0; 
                nState = 1;
                nTimer = 200*__NUM_SYSTEMTICK_MSEC;
                break;

            case 1: // State 1 - Check if machine is ready before proceeding.
                       
                if (gnRobotBalance == _DISABLE)         // Check if the balancing module is enable or not.
                {                                       // Do not run balancing module if it is disabled.
                    gobjDriverA4988.nSpeed1 = 0;        // Turn off both left and right stepper motors.
                    gobjDriverA4988.nSpeed2 = 0; 
                    gobjDriverA4988.unEn4988 = 0;       // Disable the stepper motor driver.
                    nState = 10;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                    
                }
                else
                {
                //--- Test codes ---
                if (_RB2 == 0)
                {
                    _RB2 = 1;
                }
                else
                {
                    _RB2 = 0;
                }     
                // --- End of test codes ---                     
                    nState = 2;
                    nTimer = 1;                    
                }
                break;

            case 2: // State 2 - Check for robot's orientation and calculate output for tilt angle error.  
                gfE = (gfR + gfRoffset) - gfTheta_MPU6050;
                fC = gfP*gfE;                           // Proportional term. 
                nState = 3;
                nTimer = 1;                        
                break;

            case 3: // State 3 - Compute the difference term.  
                // --- Using gyroscope output ---
                //fC = fC - (gfCwg*gfWg_MPU6050);             // Add the effect of instantaneous angular velocity
                                                        // measured from gyroscope.
                // --- Using finite difference ---
                dfE = gfE - fEOld;
                fC = fC - (gfCwg*dfE*0.001);            // Here the sampling interval is assumed 1 msec.
                
                fE1 = fE1 + gfE;                        // Update the integral of the error.                               
                nState = 4;
                nTimer = 1;                            
                break;

            case 4: // State 4 - Compute the integration term.     
                fC = fC + (gfI*fE1);                    // Add the effect of integral error.
                                                        // Note: The I term should be Ki x Delta_t x (Cumulative sum of error)
                                                        // Here we absorbed Delta_t into Ki, i.e. gfI = Ki x Delta_t
                                                        // For instance if Delta_t = 0.001 (1 msec sampling interval)
                                                        // and gfI = -0.3125, then actual Ki = -312.5
                fEOld = gfE;                            // Update the old error value.
                nState = 5;
                nTimer = 1;                            
                break;

            case 5: // State 5 - Compute control voltage for left and right wheels.         
                nC = gnControlCoefficient*fC;
                nCL = nC + gnCLoffset;                  // Add left and right offsets to control movement
                nCR = nC + gnCRoffset;                  // behavior.                 
                if (nCL > _MOTOR_DRIVER_MAX_SPEED)      // Saturate the maximum speed setting.
                {
                    nCL = _MOTOR_DRIVER_MAX_SPEED;      // Limit the input to motor driver, else this
                }                                       // may overload the stepper motor.
                else if (nCL < _MOTOR_DRIVER_MIN_SPEED)
                {
                    nCL = _MOTOR_DRIVER_MIN_SPEED;
                }
                if (nCR > _MOTOR_DRIVER_MAX_SPEED)     // Saturate the maximum speed setting.
                {
                    nCR = _MOTOR_DRIVER_MAX_SPEED;     // Limit the input to motor driver, else this
                                                       // may overload the stepper motor.
                }
                else if (nCR < _MOTOR_DRIVER_MIN_SPEED)
                {
                    nCR = _MOTOR_DRIVER_MIN_SPEED;
                }
                gnC = nC;                               // Assign control variable to global integer.  gnC will
                gnCL = nCL;                             // transferred to external monitor.
                gnCR = nCR;
                nState = 6;
                nTimer = 1;                            
            break;

            case 6: // State 6 - Drive stepper motors and wheels.               
                // Drive right wheel.
                gobjDriverA4988.nSpeed1 = nCR;        
                // Drive left wheel.
                gobjDriverA4988.nSpeed2 = nCL;
                nState = 1;
                nTimer = 1;                            
            break;
                
            case 10: // State 10 - Shut down the wheel motor and monitor the gnRobotBalance variable.
                // Check robot orientation, if it is still toppled over.
                if (gnRobotBalance == _DISABLE)
                {
                    nState = 10;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                     
                }
                else
                {                                       // Set the robot to regain balance again.
                    gfE = 0.0;                          // Reset all error terms and coefficients.
                    fE1 = 0.0;
                    fE2 = 0.0;
                    fEOld = 0.0;
                    fC = 0.0;      
                    nC = 0;
                    gfR = _FR_DEFAULT;                  // Reference in radian.
                    gfRoffset = 0.0;
                    gobjDriverA4988.unEn4988 = 1;       // Enable the stepper motor driver.
                    nState = 2;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                     
                }
            break;

            default:
                nState = 0;
                nTimer = 1;            
            break;
        }
    }
}

 
///
/// Process name	: Robot_MoveLinear
///
/// Author          : Fabian Kung
///
/// Last modified	: 30 Oct 2019
///
/// Code version	: 0.82
///
/// Processor		: dsPIC33EP256MU80X family.
///
/// Processor/System Resource
/// PIN             :  None
///
/// MODULE          :  Proce_Balance(), internal.
///
/// DRIVER          :  None
///
/// RTOS            :  Ver 1 or above, round-robin scheduling.
///
/// Global variables	: gnKp_Move, gnKi_Move
///                       gnC 
///                       gnOmegaWSet
///                       gnlDistanceSet
///                       gnDistanceMoveW
///
/// Description	:  This is a higher level control routine that sits on top of the main
///                balancing routine.  This routine makes the robot move forward or backward
///                according to the linear velocity set by the user processes.
///                Here we fix the rotational speed to between 0.25 rotations/second,
///                Or 100 steps per second (half step mode),
///                to 0.50 rotation/sec, or 200 steps per second (half step mode).  
///                This means the total steps recorded in gnDistanceMoveW
///                must change by 1-2 units every 10 msec.  For this we set the sampling interval
///                to around 10 msec.  The speed setting is +-22 for forward and backward movement.
///                in slow moving mode and +-35 for normal moving mode.   
/// Example of usage:
///                1. To move forward, we just set gnOmegaWSet to a value between 1 to 100.
///                2. To move backward, we set gnOmegaWSet to between -1 and -100.
///                3. To stop the robot, set gnOmegaWSet to 0.
///

#define _MAX_DISTANCE_ERROR_MAGNITUDE   _STEPPER_MOTOR_COEFFICIENT_4*300     
                                        // This is equivalent to 0.75 
                                        //rotation (for half-step and quarter-step drive).
#define _MAX_TILT_ANGLE_OFFSET_RAD      0.15    // Maximum offset angle in radian, about 9.3 degrees. 

void Robot_MoveLinear(void)
{
    static int  nState = 0;
    static int  nTimer = 1;    
    
    static int  nSpeedError = 0;
    static int  nSpeedErrorOld = 0;
    static int  nDeltaSpeedError = 0;
    static int  nDistanceError = 0;
    static float fTiltOffsetLimitU = 0.01;
    static float fTiltOffsetLimitL = -0.01;
    static int  nSpeed1 = 0, nSpeed2 = 0, nSpeed3 = 0;
    static int  nCount = 0;
    int         nTemp;
    
    nTimer--;                                       // Decrement timer.
    if (nTimer == 0)            
    {
        switch (nState)
        {
            case 0: // State 0 - Initialization.
                nSpeedError = 0;                                            // Reset speed and distance errors.
                nDistanceError = 0;
                nSpeedErrorOld = 0;
                nSpeed1 = 0;
                nSpeed2 = 0;
                nSpeed3 = 0;                
                gnOmegaWSet = 0;
                gnKp_Move = _FP2_MOVE_DEFAULT_4;
                gnKd_Move = _FD2_MOVE_DEFAULT_4;
                gnKi_Move = _FI2_MOVE_DEFAULT_4;
                nState = 1;
                nTimer = 200*__NUM_SYSTEMTICK_MSEC;
                //OSSetTaskContext(ptrTask, 1, 200*__NUM_SYSTEMTICK_MSEC);    // Next state = 1, timer = 200 msec.
            break;
                
            case 1: // State 1 - Check if machine is ready before proceeding.  If the balancing routine
                    // is not running.
                if (gnRobotBalance == _DISABLE)  // Check if the balancing module is enable or not.
                {                                                       // Do not run balancing module.
                    nState = 10;
                    nTimer = 1;
                    //OSSetTaskContext(ptrTask, 10, 1);                   // Next state = 10, timer = 1.
                }
                else
                {                                                       // Balancing is enabled. 
                    nState = 2;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                    
                    //OSSetTaskContext(ptrTask, 2, 1*__NUM_SYSTEMTICK_MSEC); // Next state = 2, timer = 1 msec.
                }
            break;
                
            case 2: // State 2 - PID slow speed: Compute the speed and distance error
                if (gnOmegaWSet > 0)            // Move forward.
                {
                    if (gnOmegaWSet > (24*_STEPPER_MOTOR_COEFFICIENT_4))          // Distinguish between medium and low speed.
                    {                                                           // Threshold is 25.
                        gnlDistanceSet = gnlDistanceSet + (4*_STEPPER_MOTOR_COEFFICIENT_4);    // Medium speed forward.
                    }
                    else
                    {
                        gnlDistanceSet = gnlDistanceSet + (2*_STEPPER_MOTOR_COEFFICIENT_4);    // Low speed forward.
                    }
                }
                else if (gnOmegaWSet < 0)       // Move backward.
                {
                    if (gnOmegaWSet < (-24*_STEPPER_MOTOR_COEFFICIENT_4))
                    {
                        gnlDistanceSet = gnlDistanceSet - (4*_STEPPER_MOTOR_COEFFICIENT_4);    // Medium speed reverse.
                    }
                    else
                    {
                        gnlDistanceSet = gnlDistanceSet - (2*_STEPPER_MOTOR_COEFFICIENT_4);    // Low speed reverse.
                    }
                }
                
                nTemp = gnC + nSpeed1 + nSpeed2 + nSpeed3;              // Perform a 4 samples moving average.
                nTemp = nTemp >> 2;  
                    
                nSpeedError = gnOmegaWSet - nTemp;                      // Speed error from averaged speed samples.
                //nSpeedError = gnOmegaWSet - gnC;                      // Speed error from current speed.
                nDeltaSpeedError = nSpeedError - nSpeedErrorOld;
                nDistanceError = gnlDistanceSet - gnDistanceMoveW;      // Distance error.
                if (nDistanceError > _MAX_DISTANCE_ERROR_MAGNITUDE)     // Limit the magnitude of the distance
                {                                                       // error.
                    nDistanceError = _MAX_DISTANCE_ERROR_MAGNITUDE;
                }
                if (nDistanceError < -_MAX_DISTANCE_ERROR_MAGNITUDE)
                {
                    nDistanceError = -_MAX_DISTANCE_ERROR_MAGNITUDE;
                }
                
                nSpeed3 = nSpeed2;                                      // Update samples for moving average.
                nSpeed2 = nSpeed1;
                nSpeed1 = gnC;
                nState = 3;
                nTimer = 7*__NUM_SYSTEMTICK_MSEC;                 
                //OSSetTaskContext(ptrTask, 3, 7*__NUM_SYSTEMTICK_MSEC); // Next state = 3, timer = 7 msec.
                break;
                
            case 3: // State 3 - PID slow speed: Compute the offset tilt angle based on speed and distance error, using
                    // proportional approach.
                nTemp = gnKp_Move*nSpeedError;
                nTemp = nTemp + gnKd_Move*nDeltaSpeedError;
                nTemp = nTemp + (gnKi_Move*nDistanceError);                                                                       
                gfRoffset = nTemp*0.0001;                               // Convert integer into floating point,
                                                                        // and multiply by 0.0001.
                if (gfRoffset > fTiltOffsetLimitU)                      // Limit the offset tilt angle.
                {                                                       // 
                    gfRoffset = fTiltOffsetLimitU;
                }
                else if (gfRoffset < fTiltOffsetLimitL)
                {
                    gfRoffset = fTiltOffsetLimitL;
                }
                nSpeedErrorOld = nSpeedError;                           // Store current error in speed.
                nState = 4;
                nTimer = 1*__NUM_SYSTEMTICK_MSEC;                 
                //OSSetTaskContext(ptrTask, 4, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 4, timer = 1 msec.
                break;
                
            case 4: // State 4 - Adjust the upper and lower limits of the offset tilt angle gradually.  
                    // Note: 28 Aug 2019 F. Kung
                    // I observed that if we do not limit the maximum offset tilt angle magnitude computed 
                    // by this process, it will cause the robot to accelerate and decelerate violently when
                    // the robot go from topple state to self-balancing state. Basically there is a possibility 
                    // of excessively offset tilt angle magnitude being computed by the algorithm during transition
                    // from lying down to upright, and when we add to the set tilt angle value in the 
                    // Balancing Process, this can result in high initial velocity setting for the motor
                    // angular velocity.  Thus we slowing increase the limits by 0.01 every 100 msec or so
                    // until a maximum magnitude is reached.
                nCount++;
                if (nCount > 10)                                        // Only execute every 10 cycles.
                {
                    nCount = 0; 
                    if (fTiltOffsetLimitU < _MAX_TILT_ANGLE_OFFSET_RAD)
                    {
                        fTiltOffsetLimitU += 0.01;                      // Increment offset tilt angle upper limit. 
                    }
                    if (fTiltOffsetLimitL > -_MAX_TILT_ANGLE_OFFSET_RAD)
                    {
                        fTiltOffsetLimitL -= 0.01;                      // Decrement offset tilt angle lower limit. 
                    }
                }
                nState = 1;
                nTimer = 1*__NUM_SYSTEMTICK_MSEC;                 
                //OSSetTaskContext(ptrTask, 1, 1*__NUM_SYSTEMTICK_MSEC);  // Next state = 1, timer = 1 msec.
                break;
                
            case 10: // State 10 - Balancing operation is disabled, continue to monitor the global variable 
                     // gnRobotBalance until balancing routine is enabled again.
                if (gnRobotBalance == _DISABLE)
                {
                    nState = 10;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC; 
                    //OSSetTaskContext(ptrTask, 10, 1*__NUM_SYSTEMTICK_MSEC);     // Next state = 10, timer = 1 msec.
                }
                else    // back to balancing mode, initialize all internal variables.
                {	                   
                    nSpeedError = 0;                                            // Reset speed and distance errors.
                    nDistanceError = 0;
                    nSpeedErrorOld = 0;
                    nSpeed1 = 0;
                    nSpeed2 = 0;
                    nSpeed3 = 0;
                    gnOmegaWSet = 0;
                    gnlDistanceSet = gnDistanceMoveW;
                    fTiltOffsetLimitU = 0.01;
                    fTiltOffsetLimitL = -0.01;
                    nState = 2;
                    nTimer = 1*__NUM_SYSTEMTICK_MSEC;                     
                    //OSSetTaskContext(ptrTask, 2, 1*__NUM_SYSTEMTICK_MSEC);      // Next state = 2, timer = 1 msec.
                }
                break;
                
            default:
                nState = 0;
                nTimer = 1;                 
            break;
        }
    }
}


// High level process, coordinate all other routines in the robot.
void Robot_HighLevelProcess(void)
{
    static int  nState = 0;
    static int  nTimer = 1;

    static  int  nTens = 0;
    static  int  nSign = 0;
    static  int  nAngle;
    
    nTimer--;               // Decrement timer.
    if (nTimer == 0)
    {
        switch (nState)
        {
            case 0: // State 0 - Initialization.
                PIN_HC_05_RESET = 1;                    // Deasert reset pin for Bluetooth module. 
                                                        // Note: Here we assume the HC-05 Bluetooth module EN pin 
                                                        // is active low. Certain modules has active high EN pin,
                                                        // so you need to find out the correct polarity.
                //PIN_PSW = _OFF_ANALOG_POWER;            // Turn off Analog Power Switch.
                gobjDriverA4988.unEn4988 = 1;           // Enable the stepper motor driver.
                //PIN_MS1 = _STEPPER_MOTOR_QUARTER_STEP;  // Set stepper motor driver IC to quarter-step mode.
                gnControlCoefficient = _STEPPER_MOTOR_COEFFICIENT_4*10;
                gnRobotBalance = _DISABLE;              // Disable balancing routines.                
                nState = 1;
                //nState = 105;
                nTimer = 1000*__NUM_SYSTEMTICK_MSEC;
            break;

            case 1: // State 1 - Optional step, reset HC-05 Bluetooth module if it is attached.  Sometimes the HC-05
                    // Bluetooth module cannot boot up properly due to rise time of the supply voltage too slow.
                PIN_HC_05_RESET = 0;                     // Reset Bluetooth module.
                nState = 2;
                //nState = 20;
                nTimer = 10*__NUM_SYSTEMTICK_MSEC;
                break;
            
            case 2: // State 2 - Optional step, deaasert HC-05 reset.
                PIN_HC_05_RESET = 1;
                nState = 3;
                nTimer = 100*__NUM_SYSTEMTICK_MSEC;
                break;
                
            case 3: // State 3 - Wait until robot is upright.
                
                if ((gnTiltOrient_IMU == _ROR_UPRIGHT)&&(gnStatus_IMU == _READY))
                // Check if robot is upright, with IMU and wheel encoder outputs are valid.
                {
                    gfRoffset = 0.0;                                            // Clear all motion settings.
                    gnOmegaWSet = 0;                                            // Present linear velocity and
                    gnlDistanceSet = 0;                                         // distance settings.
                    gnRobotBalance = _ENABLE;                                   // Enable balancing routines. 
                    //PIN_PSW = _ON_ANALOG_POWER;  
                    nState = 4;
                    nTimer = 1;                     
                }
                else
                {
                    nState = 3;
                    nTimer = 1; 
                }                                   
                break;
            
            case 4: // State 4 - Check for topple condition.
                if ((gnTiltOrient_IMU ==  _ROR_TOPPLE_FRONT) || (gnTiltOrient_IMU ==  _ROR_TOPPLE_BACK))
                {
                    nState = 10;
                    nTimer = 1; 
                }
                else    // Robot is upright, proceed to other operating mode tasks.
                {                   
                    nState = 5;
                    nTimer = 1; 
                }                             
                break;
                
            case 5: // State 5 - Send back tilt angle reading to external monitor software. 
                nAngle = gnTheta_Deg_MPU6050;           // Read tilt angle.
                if (nAngle < 0)                         // Steps to convert signed integer value
                {                                       // to sign BCD (Binary coded decimal).
                    nAngle = -nAngle;
                    nSign = -1;
                }
                else
                {
                    nSign = 0;
                }
                nTens = 0;
                nState = 6;
                nTimer = 1;                
                break;               
                
            case 6: // State 6 - Continue with signed integer to signed BCD conversion.
                while (nAngle > 9)
                {
                    nAngle = nAngle - 10;
                    nTens++;
                }
                nState = 7;
                nTimer = 1;  
                break;
                
            case 7: // State 7 - Transmit signed BCD value of tilt angle via UART1 port.
                if (nSign < 0)
                {
                    gbytTXbuffer[0] = '-';
                }
                else
                {
                    gbytTXbuffer[0] = '+';
                }
                
                gbytTXbuffer[1] = nTens + 48;	// Convert tenth digit to ASCII and load data.
		   	    gbytTXbuffer[2] = nAngle + 48;  // Convert unit digit to ASCII and load data.
                gbytTXbuffer[3] = '\n';         // Add newline character.
		   	    gbytTXbuflen = 4;               // Set TX frame length.
		  	    gSCIstatus.bTXRDY = 1;          // Initiate TX.                
                nState = 4;
                nTimer = 50*__NUM_SYSTEMTICK_MSEC;  // Repeat this 20 times per second.                   
                break;                
                
            case 10: // State 10 - Disable balancing routine, robot toppled.
                     // When the robot topples, we need to shut down all motors to avoid damaging the motor driver.
                gnRobotBalance = _DISABLE;                          // Set the robot to a known state:
                nState = 2;
                nTimer = 1;                 
            break;
            
            case 100: // State 100 - Test mode.
                //PIN_PSW = _ON_ANALOG_POWER; 
                gobjDriverA4988.unEn4988 = 1;       // Enable the stepper motor driver.
                nState = 101;
                nTimer = 1000*__NUM_SYSTEMTICK_MSEC;
            break;     

            case 101: // State 101 - Test mode.
                gobjDriverA4988.nSpeed1 = 90;
                gobjDriverA4988.nSpeed2 = 90;
                nState = 101;
                //nState = 102;
                nTimer = 1500*__NUM_SYSTEMTICK_MSEC;                             
            break;
                        
            case 102: // State 102 - Test mode.
                gobjDriverA4988.nSpeed1 = 0;
                gobjDriverA4988.nSpeed2 = 0;
                nState = 103;
                nTimer = 100*__NUM_SYSTEMTICK_MSEC;                                  
            break;

            case 103: // State 103 - Test mode.   
                gobjDriverA4988.nSpeed1 = -50;
                gobjDriverA4988.nSpeed2 = -50;       
                nState = 104;
                nTimer = 1500*__NUM_SYSTEMTICK_MSEC;                    
            break;

            case 104: // State 104 - Test mode.
                gobjDriverA4988.nSpeed1 = 0;
                gobjDriverA4988.nSpeed2 = 0;
                nState = 101;
                nTimer = 100*__NUM_SYSTEMTICK_MSEC;                       
            break;            
            
            case 105: // State 105 - Test mode, test UART1.
                gbytTXbuffer[0] = '1';	// Load data.
		   	    gbytTXbuflen = 1;		// Set TX frame length.
		  	    gSCIstatus.bTXRDY = 1;	// Initiate TX.    
                nState = 105;
                nTimer = 100*__NUM_SYSTEMTICK_MSEC;  
                break;
                
            default:
                nState = 0;
                nTimer = 1; 
        }
    }
}

