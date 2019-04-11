/**
 * @file logger_t.c
 * @author Madhumitha Tolakanahalli
 * @brief
 * @version 0.1
 * @date 2019-03-26
 *
 * @citation https://github.com/akobyl/TM4C129_FreeRTOS_Demo
 *           https://github.com/rheidebr/ECEN5013-LUX_Temp_demo
 *           Simply Blinky Sample Project
 *
 * @copyright Copyright (c) 2019
 *
 */

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// TivaWare includes
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/pin_map.h"
#include "drivers/pinout.h"
#include "utils/uartstdio.h"
#include "driverlib/inc/hw_i2c.h"
#include "driverlib/inc/hw_memmap.h"

// FreeRTOS includes
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "portmacro.h"

// User Header Files
#include "main.h"

// Task declarations
void vUserTaskCreate();
void vLEDTask(void *pvParameters);
void vAlertTask(void *pvParameters);
void vTemperatureTask(void *pvParameters);
void vLoggerTask(void *pvParameters);
void vTimer_Handler(TimerHandle_t handler);
void vI2CInit(uint32_t output_clock_rate_hz);
uint16_t uiI2CReadTMP102(uint8_t slaveAddr, uint8_t regAddr);
uint32_t uiRegValToTempC(uint16_t regval);

#define TIMERS                  2
#define TASKS                   3
#define LOGLEVELS               3

// Port and Pin Macros for LED1 and LED2
#define LED_D1_PORT             GPIO_PORTN_BASE
#define LED_D1_PIN              GPIO_PIN_1

#define LED_D2_PORT             GPIO_PORTN_BASE
#define LED_D2_PIN              GPIO_PIN_0

// Task Priorities
#define PRIO_ALTER              ( tskIDLE_PRIORITY + 4 )
#define PRIO_LOGGER             ( tskIDLE_PRIORITY + 3 )
#define PRIO_LED                ( tskIDLE_PRIORITY + 2 )
#define PRIO_TEMPERATURE        ( tskIDLE_PRIORITY + 1 )

// Queue Macros
#define QUEUE_LENGTH            10

// TMP102 Macros
#define RESOLUTION_C            (0.0625)
#define TMP102_SLAVE_ADDR 		(0x48)
#define UART_CLOCK_PIOSC        (0x00000005)
#define THRESHOLD_TEMPRERATURE  (250000)         // (thresholdtemperature * 100000)

// Global Variables
static QueueHandle_t xQueue = NULL;
TimerHandle_t arr_TimerHandler[TIMERS];
TaskHandle_t arr_TaskHandler[4];

// Array for Log Levels
char * arr_loglevels[LOGLEVELS] = {"Data", "Alert", "Error"};

// Main function
int main(void)
{
    // Initialize system clock to 120 MHz
    uint32_t output_clock_rate_hz;
    output_clock_rate_hz = ROM_SysCtlClockFreqSet(
                                   (SYSCTL_XTAL_25MHZ | SYSCTL_OSC_MAIN |
                                    SYSCTL_USE_PLL | SYSCTL_CFG_VCO_480),
                                   SYSTEM_CLOCK);

    // Initialize I2C2 Bus
    vI2CInit(output_clock_rate_hz);

    // Initialize the GPIO pins
    PinoutSet(false, false);

    // Initialize UART
    UARTStdioConfig(0, 115200, SYSTEM_CLOCK);

    // Create a Queue
    xQueue = xQueueCreate(QUEUE_LENGTH, sizeof(QueuePacket));

    // Create tasks
    vUserTaskCreate();

    vTaskStartScheduler();

    return 0;
}

void vUserTaskCreate()
{
    xTaskCreate(vLEDTask, (const portCHAR *)"LEDTask",
                configMINIMAL_STACK_SIZE, NULL, PRIO_LED, &arr_TaskHandler[0]);

    xTaskCreate(vTemperatureTask, (const portCHAR *)"Temperature",
                configMINIMAL_STACK_SIZE, NULL, PRIO_TEMPERATURE, &arr_TaskHandler[1]);

    xTaskCreate(vLoggerTask, (const portCHAR *)"Logger",
                    configMINIMAL_STACK_SIZE, NULL, PRIO_LOGGER, &arr_TaskHandler[2]);

    xTaskCreate(vAlertTask, (const portCHAR *)"Alert",
                    configMINIMAL_STACK_SIZE, NULL, PRIO_LOGGER, &arr_TaskHandler[3]);

    return;
}
// Toggle LEDs at the Frequency of 10Hz
void vLEDTask(void *pvParameters)
{
    UARTprintf("# LED Task Entered \n");
    arr_TimerHandler[0] = xTimerCreate("LEDTimer10Hz", pdMS_TO_TICKS(100) ,
                                        pdTRUE,  (void*)0, vTimer_Handler);

    if((xTimerStart(arr_TimerHandler[0], 0)) != pdTRUE)
    {
        while(1);
    }

    // Suspend Task
    vTaskSuspend(NULL);
}


// Send Temperature at 1Hz to Logger Task
void vTemperatureTask(void *pvParameters)
{
    UARTprintf("# Temperature Task Entered \n");
    // Set up the UART which is connected to the virtual COM port
    arr_TimerHandler[1] = xTimerCreate("TemperatureTimer1Hz", pdMS_TO_TICKS(1000) ,
                                        pdTRUE,  (void*)0, vTimer_Handler);

    if((xTimerStart(arr_TimerHandler[1], 0)) != pdTRUE)
    {
        while(1);
    }

    // Suspend Task
    vTaskSuspend(NULL);
}

// Logger Task
void vLoggerTask(void *pvParameters)
{
    UARTprintf("# Logger Task Entered \n");
    static QueuePacket RxPacket;

    //Initialize Packet
     memset(&RxPacket, 0, sizeof(QueuePacket));

     while(xQueueReceive(xQueue, &RxPacket, portMAX_DELAY) == pdTRUE)
     {
         switch (RxPacket.taskID)
         {
             case ID_TemperatureTask :
                 UARTprintf("%u      -|Temperature Packet|-      (%s)       Temperature : %d.%dC \n", RxPacket.timestamp, arr_loglevels[RxPacket.loglevel],RxPacket.value / 10000, RxPacket.value % 10000); break;
             case ID_LEDTask :
                 UARTprintf("%u      -----<LED Packet>-----      (%s)      Toggle Count : %d         <User> %s \n", RxPacket.timestamp, arr_loglevels[RxPacket.loglevel], RxPacket.value,RxPacket.data); break;
             case ID_AlertTask :
                 UARTprintf("%u      ----[Alert Packet]----      (%s)       %s\n", RxPacket.timestamp, arr_loglevels[RxPacket.loglevel], RxPacket.data); break;
             default :
                 UARTprintf("%u      ----Invalid Packet----      (%s)         \n",arr_loglevels[LogLevel_Error]); break;
         }
     }

     // Suspend Task
     vTaskSuspend(NULL);

}

//Alert Task
void vAlertTask(void *pvParameters)
{
    UARTprintf("# Alert Task Entered \n");
    static QueuePacket TxPacket;

    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        //Initialize Packet
        memset(&TxPacket, 0, sizeof(QueuePacket));

        TxPacket.taskID    = ID_AlertTask;
        TxPacket.data      = "Temperature out of range! \n";
        TxPacket.timestamp = xTaskGetTickCount();
        TxPacket.loglevel  = LogLevel_Alert;

        // Send Packet
        if(xQueueSendToBack(xQueue, &TxPacket, 0) != pdTRUE)
        {
           UARTprintf("ERROR : Alert Queue Send \n");
        }
    }

}
void vTimer_Handler(TimerHandle_t handler)
{
    int16_t regval = 0;
    uint32_t temperature = 0;
    static uint32_t u32valLED1 = 0x0;
    static uint32_t u32valLED2 = LED_D2_PIN;
    static uint32_t u32toggleCount;

    static QueuePacket TxPacket;
    if(handler == arr_TimerHandler[0])
    {
        u32valLED1 = u32valLED1 ^ LED_D1_PIN;
        GPIOPinWrite(LED_D1_PORT, LED_D1_PIN, u32valLED1);

        u32valLED2 = u32valLED2 ^ LED_D2_PIN;
        GPIOPinWrite(LED_D2_PORT, LED_D2_PIN, u32valLED2);

        //Reinitialize Packet
        memset(&TxPacket, 0, sizeof(QueuePacket));

        // Populate Packet
        TxPacket.taskID    = ID_LEDTask;
        TxPacket.value     = u32toggleCount++;
        TxPacket.data      = "Madhumitha";
        TxPacket.timestamp = xTaskGetTickCount();
        TxPacket.loglevel  = LogLevel_Data;

        // Send Packet
        if(xQueueSendToBack(xQueue, &TxPacket, 0) != pdTRUE)
        {
            UARTprintf("ERROR : LED Queue Send \n");
        }
    }

    if(handler == arr_TimerHandler[1])
    {
        //Reinitialize Packet
         memset(&TxPacket, 0, sizeof(QueuePacket));

        regval      = uiI2CReadTMP102(TMP102_SLAVE_ADDR, 0x00);
        temperature = uiRegValToTempC(regval);

        // Populate Packet
        TxPacket.taskID    = ID_TemperatureTask;
        TxPacket.value     = temperature;
        TxPacket.data      = '\0';
        TxPacket.timestamp = xTaskGetTickCount();
        TxPacket.loglevel  = LogLevel_Data;

        // Send Packet
        if(xQueueSendToBack(xQueue, &TxPacket, 0) != pdTRUE)
            UARTprintf("ERROR : Temperature Queue Send Error \n");

        // If Temperature goes out of range, notify the Alert Task
        if(temperature > THRESHOLD_TEMPRERATURE)
            xTaskNotifyGive(arr_TaskHandler[3]);
    }
}

uint16_t uiI2CReadTMP102(uint8_t slaveAddr, uint8_t regAddr)
{
   I2CMasterSlaveAddrSet(I2C2_BASE, slaveAddr, false);

   I2CMasterDataPut(I2C2_BASE, regAddr);

   I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_SINGLE_SEND);

   while(!I2CMasterBusy(I2C2_BASE));

   while(I2CMasterBusy(I2C2_BASE));

   I2CMasterSlaveAddrSet(I2C2_BASE, slaveAddr, true);

   I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);

   while(!I2CMasterBusy(I2C2_BASE));

   while(I2CMasterBusy(I2C2_BASE));

   uint32_t ui32byte1 = I2CMasterDataGet(I2C2_BASE);

   I2CMasterControl(I2C2_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

   while(!I2CMasterBusy(I2C2_BASE));

   while(I2CMasterBusy(I2C2_BASE));

   uint32_t ui32byte2 = I2CMasterDataGet(I2C2_BASE);

   return (ui32byte1 << 8) | ui32byte2;
}

void vI2CInit(uint32_t output_clock_rate_hz)
{
    // Enable I2C2 Peripheral
   SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C2);

   // Enable GPION and configure Pin4 and Pin5
   SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

   GPIOPinConfigure(GPIO_PN4_I2C2SDA);
   GPIOPinConfigure(GPIO_PN5_I2C2SCL);

   GPIOPinTypeI2CSCL(GPIO_PORTN_BASE, GPIO_PIN_5);
   GPIOPinTypeI2C(GPIO_PORTN_BASE, GPIO_PIN_4);
   I2CMasterInitExpClk(I2C2_BASE, output_clock_rate_hz, false);

   return;
}
// Function to convert Value read from Sensor to Celcius
uint32_t uiRegValToTempC(uint16_t regval)
{
    float tempval = 0.0;

    // Convert Value to 12 bits ignoring 4 least significant bits
    regval = regval >> 4  & 0x0FFF;

    // Check Negative or Positive Temperature
    if(regval > 0x7FF)
        tempval = (regval * RESOLUTION_C)- 256;
    else
        tempval = (regval * RESOLUTION_C);

    uint32_t ret_temp = (uint32_t) (10000.0 * tempval);

    return ret_temp;

}

/*  ASSERT() Error function
 *
 *  failed ASSERTS() from driverlib/debug.h are executed in this function
 */
void __error__(char *pcFilename, uint32_t ui32Line)
{
    // Place a breakpoint here to capture errors until logging routine is finished
    while (1)
    {
    }
}
