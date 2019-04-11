/*
 * main.h
 *
 *  Created on: Mar 28, 2015
 *      Author: akobyljanec
 */

#ifndef MAIN_H_
#define MAIN_H_

#include "portmacro.h"
// System clock rate, 120 MHz
#define SYSTEM_CLOCK    120000000U

// TaskID enum
typedef enum
{
    ID_LEDTask = 0,
    ID_TemperatureTask = 1,
    ID_LoggerTask = 2,
    ID_AlertTask = 3,

} TaskID_t;

typedef enum
{
    LogLevel_Data  = 0,
    LogLevel_Alert = 1,
    LogLevel_Error = 2,

} LogLevel_t;

// Packet Structure
typedef struct
{
    TaskID_t taskID;
    uint32_t value;
    char * data;
    TickType_t timestamp;
    LogLevel_t loglevel;

} QueuePacket;


#endif /* MAIN_H_ */
