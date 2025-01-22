// ARM Framework - Brian Ordonez

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

TaskHandle_t TASK1    // SPI/UART Communication/ISR :    Core 0
TaskHandle_t TASK2    // Variable Read/Write and Math:   Core 0
TaskHandle_t TASK3    // Servo & Stepper Motor Cotnrol:  Core 1
TaskHandle_t TASK4    // IMU, ASS600. VS56LOX:           Core 1

void setup() 
{
  //Setting Up Tasks 1-4 
  xTaskCreatePinnedToCore(
                    Communication,   /* Task function. */
                    "TASK1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */ 

  xTaskCreatePinnedToCore(
                    Read_Write,   /* Task function. */
                    "TASK2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK2,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    Motor_Control,   /* Task function. */
                    "TASK3",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK3,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */  

  xTaskCreatePinnedToCore(
                    Sensors,   /* Task function. */
                    "TASK4",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK4,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */               
}
// =============================================================================================
void Communication(void * pvParameters)
{
  Serial.print("ARM Communication Running...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {

  } 
}

void Read_Write(void * pvParameters)
{
  Serial.print("ARM Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {

  }
}

void Motor_Control(void * pvParameters)
{
  Serial.print("ARM Motor_Control Running...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {
    
  }
}

void Sensors(void * pvParameters)
{
  Serial.print("ARM Sensors...Core: ");
  Serial.println(xPortGetCoreID());
  for(;;)
  {

  }
}

































void loop() {}