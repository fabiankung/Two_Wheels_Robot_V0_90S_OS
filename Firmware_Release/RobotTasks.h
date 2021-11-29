/* 
 * File:   RobotTasks.h
 * Author: User
 *
 * Created on October 26, 2019, 3:13 PM
 * Last modified: 29 Nov 2021
 */

#ifndef ROBOTTASKS_H
#define	ROBOTTASKS_H

#ifdef	__cplusplus
extern "C" {
#endif

    void Robot_Balance(void);
    void Robot_HighLevelProcess(void);
    void Robot_Sensor_MPU6050(void);
    void Robot_MoveLinear(void);
    void Robot_Turn(void);

#ifdef	__cplusplus
}
#endif

#endif	/* ROBOTTASKS_H */

