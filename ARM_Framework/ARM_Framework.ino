// ARM Framework - Brian Ordonez

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <ESP32SPISlave.h>
#include "semphr.h"

TaskHandle_t TASK1;    // SPI/UART Communication/ISR :    Core 0
TaskHandle_t TASK2;    // Variable Read/Write and Math:   Core 0
TaskHandle_t TASK3;    // Servo & Stepper Motor Cotnrol:  Core 1
TaskHandle_t TASK4;    // IMU, ASS600. VS56LOX:           Core 1

SemaphoreHandle_t motorAngleMutex;

ESP32SPISlave slave;
static constexpr uint32_t BUFFER_SIZE {5};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

QueueHandle_t queue;

typedef struct
{
  float a, b, c, d, e;
}MotorAngle;

typedef struct
{
  float a, b, c, d, e;
}AngleMeasurements;

Servo shoulderYAWServo;                                 // Shoulder Servo Motor
volatile float shoulderYawPA[3] = {0.0, 0.0, 0.0};
bool shoulderYawPAWR = false;

Servo shoulderROLLServo;
volatile float shoulderRollPA[3] = {0.0, 0.0, 0.0};
bool shoulderRollPAWR = false;

Servo elbowROLLServo;
volatile float elbowRollPA[3] = {0.0, 0.0, 0.0};
bool elbowRollPAWR = false;

Servo wristROLLServo;
volatile float wristRollPA[3] = {0.0, 0.0, 0.0};
bool wristRollPAWR = false;

Servo firstFingerServo;
volatile float firstFingerPA[3] = {0.0, 0.0, 0.0};
bool firstFingerPAWR = false;


void setup() 
{
  Serial.begin(115200);

  // SPI Slave Initialization
  slave.setDataMode(SPI_MODE0);
  slave.begin();

  // Clear Buffers - SPI
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  motorAngleMutex = xSemaphoreCreateMutex();

  // Create Queue for Data given from NVIDIA Jetsons
  queue = xQueueCreate(5, sizeof(AngleMeasurements));

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

  // Position pos;
  // unsigned char data;

  for(;;)
  {
    Serial.println("[Core 0]: Waiting for Handshake...");

    while(slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

    if(spi_slave_rx_buf[0] == 0xA1)
    {
      Serial.println("[Core 0]: Handshake received, sending ACK...");
      spi_slave_tx_buf[0] = 0xA5;
      slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE);
    }

    // xSemaphoreTake(motorAngleMutex, portMAX_DELAY);
    // // Reading Values 
    // xSemaphoreGive(motorAngleMutex);
    // Serial.println("[Core 0]: Received Motor Angles");
  }
}

void Read_Write(void * pvParameters)
{
  Serial.print("ARM Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());

  AngleMeasurements angleMeasure;

  for(;;)
  {
    if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
    {
      angleMeasure.a = spi_slave_rx_buf[0];
      angleMeasure.b = spi_slave_rx_buf[1];
      angleMeasure.c = spi_slave_rx_buf[2];
      angleMeasure.d = spi_slave_rx_buf[3];
      angleMeasure.e = spi_slave_rx_buf[4]; 
      xQueueSend(queue, &angleMeasure, portMAX_DELAY);
      //memcpy(motorAngle, &spi_slave_rx_buf[1], sizeof(motorAngle));
      xSemaphoreGive(motorAngleMutex);
    }

    xTaskNotifyGive(TASK3);

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    //FROM HERE, you need to send sensor data from TASK4 back to the Jetson using SPI

    vTaskDelay(pdMS_TO_TICKS(100));


    // if(xQueueReceive(queue, &resPosition, portMAX_DELAY))
    // {

    // }
    // vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void Motor_Control(void * pvParameters)
{
  Serial.print("ARM Motor_Control Running...Core: ");
  Serial.println(xPortGetCoreID());

  MotorAngle motorAngle;

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(motorAngleMutex, portMAX_DELAY))
    {
      // Write to Motors
      if(xQueueReceive(queue, &motorAngle, portMAX_DELAY))
      {
        shoulderYawPA[0] = motorAngle.a;
        shoulderRollPA[0] = motorAngle.b;
        elbowRollPA[0] = motorAngle.c;
        wristRollPA[0] = motorAngle.d;
        firstFingerPA[0] = motorAngle.e;
      }
      xSemaphoreGive(motorAngleMutex);
    }

    if ((shoulderYawPA[0] != shoulderYawPA[1]) && shoulderYawPAWR == false)    // Shoulder Pitch 
    {  
      shoulderYawPAWR = true;                                      // Flag enable
      shoulderYAWServo.write(shoulderYawPA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
      shoulderYawPA[2] = shoulderYawPA[1];
      shoulderYawPA[1] = shoulderYawPA[0];                              // Log
      shoulderYawPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      shoulderYawPAWR = true;                                      // Flag enable
      shoulderYAWServo.write(shoulderYawPA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      shoulderYawPAWR = false;                                     // Flag disable
    }

    if ((shoulderRollPA[0] != shoulderRollPA[1]) && shoulderRollPAWR == false)    // Shoulder Pitch 
    {  
      shoulderRollPAWR = true;                                      // Flag enable
      shoulderROLLServo.write(shoulderRollPA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
      shoulderRollPA[2] = shoulderRollPA[1];
      shoulderRollPA[1] = shoulderRollPA[0];                              // Log
      shoulderRollPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      shoulderRollPAWR = true;                                      // Flag enable
      shoulderROLLServo.write(shoulderRollPA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      shoulderRollPAWR = false;                                     // Flag disable
    }

    if ((elbowRollPA[0] != elbowRollPA[1]) && elbowRollPAWR == false)    // Shoulder Pitch 
    {  
      elbowRollPAWR = true;                                      // Flag enable
      elbowROLLServo.write(elbowRollPA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                              // Minor Wait
      elbowRollPA[2] = elbowRollPA[1];
      elbowRollPA[1] = elbowRollPA[0];                              // Log
      elbowRollPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      elbowRollPAWR = true;                                      // Flag enable
      elbowROLLServo.write(elbowRollPA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      elbowRollPAWR = false;                                     // Flag disable
    }

    if ((wristRollPA[0] != wristRollPA[1]) && wristRollPAWR == false)    // Shoulder Pitch 
    {  
      wristRollPAWR = true;                                      // Flag enable
      wristROLLServo.write(wristRollPA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
      wristRollPA[2] = wristRollPA[1];
      wristRollPA[1] = wristRollPA[0];                              // Log
      wristRollPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      wristRollPAWR = true;                                      // Flag enable
      wristROLLServo.write(wristRollPA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      wristRollPAWR = false;                                     // Flag disable
    }

    if ((firstFingerPA[0] != firstFingerPA[1]) && firstFingerPAWR == false)    // Shoulder Pitch 
    {  
      firstFingerPAWR = true;                                      // Flag enable
      firstFingerServo.write(firstFingerPA[0]);                                  //Write Angle
      vTaskDelay(pdMS_TO_TICKS(1));                                            // Minor Wait
      firstFingerPA[2] = firstFingerPA[1];
      firstFingerPA[1] = firstFingerPA[0];                              // Log
      firstFingerPAWR = false;                                     // Flag Disable
    } 
    else 
    {
      firstFingerPAWR = true;                                      // Flag enable
      firstFingerServo.write(firstFingerPA[1]);
      vTaskDelay(pdMS_TO_TICKS(1));
      firstFingerPAWR = false;                                     // Flag disable
    }
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