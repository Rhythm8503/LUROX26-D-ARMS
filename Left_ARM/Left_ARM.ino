// LEG Framework - Brian Ordonez
// This message is coming from the NVIDIA Jetson!

#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <ESP32SPISlave.h>
#include "semphr.h"

#define pin_Shoulder 20
#define pin_ShoulderRoll 41
#define pin_Elbow 21
#define pin_Wrist 47
#define pin_Thumb 38
#define pin_Index 37
#define pin_Middle 36
#define pin_Ring 35

#define pin_UpperDir 1
#define pin_UpperStep 2

#define pin_LowerDir 15
#define pin_LowerStep 16


const int stepDelayMicros = 4000;
long currentPosition_Upper = 0;
long currentPosition_Lower = 0;

long homePosition = 0;
const long finalPosition = 500;
const int StepRev = 4000;

TaskHandle_t TASK1;    // SPI/UART Communication/ISR :    Core 0
TaskHandle_t TASK2;    // Variable Read/Write and Math:   Core 0
TaskHandle_t TASK3;    // Process MotorAngles:            Core 0
TaskHandle_t TASK4;    // ShoulderMotorControl:              Core 0
TaskHandle_t TASK5;    // ShoulderRollMotorControl:              Core 1
TaskHandle_t TASK6;    // ElbowMotorControl:               Core 1
TaskHandle_t TASK7;    // WristMotorControl:         Core 1
TaskHandle_t TASK8;    // ThumbMotorControl:         Core 1
TaskHandle_t TASK9;    // IndexMotorControl             Core 1
TaskHandle_t TASK10;    // MiddleMotorControl             Core 1
TaskHandle_t TASK11;    // RingMotorControl             Core 1
TaskHandle_t TASK12;    // StepperUpperMotorControl             Core 1
TaskHandle_t TASK13;    // StepperLowerMotorControl             Core 1

SemaphoreHandle_t sema;
QueueHandle_t queue;           // A Queue designed to contain data from Jetson and transfer it to other tasks 
                               // This prevents race conditions and is thread safe 

ESP32SPISlave slave;
static constexpr uint32_t BUFFER_SIZE {12};
uint8_t spi_slave_tx_buf[BUFFER_SIZE];
uint8_t spi_slave_rx_buf[BUFFER_SIZE];

uint8_t controllerID;               // Byte 0: Controller Address
typedef struct
{
  uint8_t M1;
  uint8_t M2;
  uint8_t M3;
  uint8_t M4;
  uint8_t M5;
  uint8_t M6;
  uint8_t M7;
  uint8_t M8;
  uint8_t M9;
  uint8_t M10;

}MotorAngleReading;

// Shoulder Servo Motor
Servo shoulderServo;
volatile float ShoulderPA[3] = {0.0, 0.0, 0.0};
bool ShoulderPAWR = false;

// ShoulderRoll Servo Motor
Servo shoulderrollServo;
volatile float ShoulderRollPA[3] = {0.0, 0.0, 0.0};
bool ShoulderRollPAWR = false;

// Elbow Servo Motor
Servo elbowServo;
volatile float ElbowPA[3] = {0.0, 0.0, 0.0};
bool ElbowPAWR = false;

// Wrist Servo Motor
Servo wristServo;
volatile float WristPA[3] = {0.0, 0.0, 0.0};
bool WristPAWR = false;

// Thumb Servo Motor
Servo thumbServo;
volatile float ThumbPA[3] = {0.0, 0.0, 0.0};
bool ThumbPAWR = false;

// Index Servo Motor
Servo indexServo;
volatile float IndexPA[3] = {0.0, 0.0, 0.0};
bool IndexPAWR = false;

// Middle Servo Motor
Servo middleServo;
volatile float MiddlePA[3] = {0.0, 0.0, 0.0};
bool MiddlePAWR = false;

// Thumb Servo Motor
Servo ringServo;
volatile float RingPA[3] = {0.0, 0.0, 0.0};
bool RingPAWR = false;

// Stepper Motor Upper Variable Storage
volatile float StepperMotorUpperPA[3] = {0.0, 0.0, 0.0};
bool StepperMotorUpperPAWR = false;

// Stepper Motor Lower Variable Storage
volatile float StepperMotorLowerPA[3] = {0.0, 0.0, 0.0};
bool StepperMotorLowerPAWR = false;

EventGroupHandle_t syncEventGroup;

const int TASK4_START_BIT = (1 << 0);
const int TASK5_START_BIT = (1 << 1);
const int TASK6_START_BIT = (1 << 2);
const int TASK7_START_BIT = (1 << 3);
const int TASK8_START_BIT = (1 << 4);
const int TASK9_START_BIT = (1 << 5);
const int TASK10_START_BIT = (1 << 6);
const int TASK11_START_BIT = (1 << 7);
const int TASK12_START_BIT = (1 << 8);
const int TASK13_START_BIT = (1 << 9);

const int TASK4_END_BIT = (1 << 10);
const int TASK5_END_BIT = (1 << 11);
const int TASK6_END_BIT = (1 << 12);
const int TASK7_END_BIT = (1 << 13);
const int TASK8_END_BIT = (1 << 14);
const int TASK9_END_BIT = (1 << 15);
const int TASK10_END_BIT = (1 << 16);
const int TASK11_END_BIT = (1 << 17);
const int TASK12_END_BIT = (1 << 18);
const int TASK13_END_BIT = (1 << 19);


void setup() 
{
  Serial.begin(115200);

  // STEPPER Motor Initialization
  pinMode(pin_UpperDir, OUTPUT);
  pinMode(pin_UpperStep, OUTPUT);

  pinMode(pin_LowerDir, OUTPUT);
  pinMode(pin_LowerStep, OUTPUT);

  long stepsToMove = finalPosition - homePosition;

  if (stepsToMove != 0) {
    digitalWrite(pin_UpperDir, stepsToMove > 0 ? HIGH : LOW);
    stepsToMove = abs(stepsToMove);

    for (long i = 0; i < stepsToMove; i++) {
      digitalWrite(pin_UpperStep, HIGH);
      delayMicroseconds(StepRev);
      digitalWrite(pin_UpperStep, LOW);
      delayMicroseconds(StepRev);
    }

    homePosition = finalPosition;

    Serial.print("Stepper moved to position: ");
    Serial.println(homePosition);
  }

  // SPI Slave Initialization
  slave.setDataMode(SPI_MODE0);
  slave.begin();

  // Clear Buffers - SPI
  memset(spi_slave_tx_buf, 0, BUFFER_SIZE);
  memset(spi_slave_rx_buf, 0, BUFFER_SIZE);

  sema = xSemaphoreCreateMutex();

  // Create Queue for Data given from NVIDIA Jetsons
  queue = xQueueCreate(10, sizeof(MotorAngleReading));

  shoulderServo.attach(pin_Shoulder);
  shoulderrollServo.attach(pin_ShoulderRoll);
  elbowServo.attach(pin_Elbow);
  wristServo.attach(pin_Wrist);
  thumbServo.attach(pin_Thumb);
  indexServo.attach(pin_Index);
  middleServo.attach(pin_Middle);
  ringServo.attach(pin_Ring);

  // Left ARM : Straight Arm
  //shoulderServo.write(88);
  //shoulderrollServo.write(90);
  //elbowServo.write(90);
  //wristServo.write(90);

  //Left ARM : Bent Arm
  // shoulderServo.write(103);
  // shoulderrollServo.write(90);
  // elbowServo.write(50);
  // wristServo.write(90);

  //Raise its Arm in the air
  shoulderServo.write(88);
  shoulderrollServo.write(180);

  elbowServo.write(50);

   for (int i = 50; i >= 30; i--) {
    elbowServo.write(i);
    delay(30);
  }

  // Move from 30 → 50
  for (int i = 30; i <= 50; i++) {
    elbowServo.write(i);
    delay(30);
  }

  // Move from 50 → 30 again
  for (int i = 50; i >= 30; i--) {
    elbowServo.write(i);
    delay(30);
  }

  // Final move from 30 → 50
  for (int i = 30; i <= 50; i++) {
    elbowServo.write(i);
    delay(30);
  }

  // ============== DO NOT USE BELOW THIS =============================

  // int fromAngles[] = {50, 30, 50, 30};
  // int toAngles[]   = {30, 50, 30, 50};

  // // Move through each pair (from → to) smoothly
  // for (int i = 0; i < 4; i++) {
  //   int from = fromAngles[i];
  //   int to = toAngles[i];

  //   if (from < to) {
  //     for (int pos = from; pos <= to; pos++) {
  //       elbowServo.write(pos);
  //       delay(30);
  //     }
  //   } else {
  //     for (int pos = from; pos >= to; pos--) {
  //       elbowServo.write(pos);
  //       delay(30);
  //     }
  //   }
  // }


  // int angles[] = {50, 30, 50, 30, 50};

  // for (int i = 0; i < 5; i++) {
  //   elbowServo.write(angles[i]);
  //   delay(1000);
  // }




  wristServo.write(90);











  syncEventGroup = xEventGroupCreate();

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
                    Motor_Processing,   /* Task function. */
                    "TASK3",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK3,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    ShoulderMotor_Control,   /* Task function. */
                    "TASK4",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK4,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */

  xTaskCreatePinnedToCore(
                    ShoulderRollMotor_Control,   /* Task function. */
                    "TASK5",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK5,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    ElbowMotor_Control,   /* Task function. */
                    "TASK6",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK6,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    WristMotor_Control,   /* Task function. */
                    "TASK7",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK7,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    ThumbMotor_Control,   /* Task function. */
                    "TASK8",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK8,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */  

  xTaskCreatePinnedToCore(
                    IndexMotor_Control,   /* Task function. */
                    "TASK9",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK9,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    MiddleMotor_Control,   /* Task function. */
                    "TASK10",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK10,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    RingMotor_Control,   /* Task function. */
                    "TASK11",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK11,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */
              
  xTaskCreatePinnedToCore(
                    StepperUpperMotor_Control,   /* Task function. */
                    "TASK12",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK12,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
                    StepperLowerMotor_Control,   /* Task function. */
                    "TASK13",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &TASK13,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 1 */

  xEventGroupSetBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT);

}

void Communication(void * pvParameters)
{
  Serial.print("LEFT ARM Communication Running...Core: ");
  Serial.println(xPortGetCoreID());

  for (;;)
  {
    Serial.println();
    Serial.println("[Core 0]: Waiting for Handshake...");

    // Wait for handshake
    while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

    if (spi_slave_rx_buf[0] == 0xA2) //Left_ARM
    {
      Serial.println("Left ARM Controller Selected!");

      Serial.println("Receiving 8-byte command...");
      while (slave.transfer(spi_slave_tx_buf, spi_slave_rx_buf, BUFFER_SIZE) == 0);

      // Extract first 3 values
      controllerID = spi_slave_rx_buf[0];

      // Print raw received data
      Serial.print("Printing Received Data: ");
      for (int i = 0; i < 12; i++) 
      {
        Serial.print(spi_slave_rx_buf[i], HEX);
        Serial.print(" ");
      }
      Serial.println();

      // Print decoded info
      Serial.printf("Controller Address: 0x%02X\n", controllerID);

      // Notify task that data is ready
      xTaskNotifyGive(TASK2);
      vTaskDelay(pdMS_TO_TICKS(10));
    }
    else
    {
      Serial.println("[LEFT ARM]: No message for me!");
    }
  }
}

void Read_Write(void * pvParameters)  // TASK2
{
  Serial.print("ARM Read/Write Running...Core: ");
  Serial.println(xPortGetCoreID());

  //ServoCommand cmd;
  MotorAngleReading mar;

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xSemaphoreTake(sema, portMAX_DELAY))
    {
      //cmd.servo_id = spi_slave_rx_buf[1];
      //cmd.angle = spi_slave_rx_buf[2];

      mar.M1 = spi_slave_rx_buf[1];
      mar.M2 = spi_slave_rx_buf[2];
      mar.M3 = spi_slave_rx_buf[3];
      mar.M4 = spi_slave_rx_buf[4];
      mar.M5 = spi_slave_rx_buf[5];
      mar.M6 = spi_slave_rx_buf[6];
      mar.M7 = spi_slave_rx_buf[7];
      mar.M8 = spi_slave_rx_buf[8];
      mar.M9 = spi_slave_rx_buf[9];
      mar.M10 = spi_slave_rx_buf[10];

      xQueueSend(queue, &mar, portMAX_DELAY);
      xSemaphoreGive(sema);
    }

    xTaskNotifyGive(TASK3);

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void Motor_Processing(void * pvParameters)  // TASK3
{
  Serial.print("ARM Motor_Processing Running...Core: ");
  Serial.println(xPortGetCoreID());

  MotorAngleReading mar;

  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(xQueueReceive(queue, &mar, portMAX_DELAY))
    {
      // Store the angles in global arrays (or use the local motorAngle structure)
      ShoulderPA[0] = (int)((mar.M1 / 255.0f) * 180);
      ShoulderRollPA[0] = (int)((mar.M2 / 255.0f) * 180);
      ElbowPA[0] = (int)((mar.M3 / 255.0f) * 180);
      WristPA[0] = (int)((mar.M4 / 255.0f) * 180);
      ThumbPA[0] = (int)((mar.M5 / 255.0f) * 180);
      IndexPA[0] = (int)((mar.M6 / 255.0f) * 180);
      MiddlePA[0] = (int)((mar.M7 / 255.0f) * 180);
      RingPA[0] = (int)((mar.M8 / 255.0f) * 180);
      StepperMotorUpperPA[0] = (int)(mar.M9 * (500.0f / 255.0f) - 250);
      StepperMotorLowerPA[0] = (int)(mar.M10 * (370.0f / 255.0f) - 185);

      // Notify all motor tasks to update simultaneously
      xTaskNotifyGive(TASK4);
      xTaskNotifyGive(TASK5);
      xTaskNotifyGive(TASK6);
      xTaskNotifyGive(TASK7);
      xTaskNotifyGive(TASK8);
      xTaskNotifyGive(TASK9);
      xTaskNotifyGive(TASK10);
      xTaskNotifyGive(TASK11);
      xTaskNotifyGive(TASK12);
      xTaskNotifyGive(TASK13);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void ShoulderMotor_Control(void * pvParameters)  // TASK4
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(ShoulderPA[0] != ShoulderPA[1] && !ShoulderPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ShoulderPAWR = true;
      shoulderServo.write(ShoulderPA[0]);  // Write to Swing motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      ShoulderPA[2] = ShoulderPA[1];
      ShoulderPA[1] = ShoulderPA[0];  // Log the new position
      ShoulderPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ShoulderPAWR = true;
      shoulderServo.write(ShoulderPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      ShoulderPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK4_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

  }
}

void ShoulderRollMotor_Control(void * pvParameters)  // TASK5
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(ShoulderRollPA[0] != ShoulderRollPA[1] && !ShoulderRollPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ShoulderRollPAWR = true;
      shoulderrollServo.write(ShoulderRollPA[0]);  // Write to Raise motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      ShoulderRollPA[2] = ShoulderRollPA[1];
      ShoulderRollPA[1] = ShoulderRollPA[0];  // Log the new position
      ShoulderRollPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ShoulderRollPAWR = true;
      shoulderrollServo.write(ShoulderRollPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      ShoulderRollPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK5_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void ElbowMotor_Control(void * pvParameters)   // TASK6
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(ElbowPA[0] != ElbowPA[1] && !ElbowPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ElbowPAWR = true;
      elbowServo.write(ElbowPA[0]);  // Write to Knee motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      ElbowPA[2] = ElbowPA[1];
      ElbowPA[1] = ElbowPA[0];  // Log the new position
      ElbowPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ElbowPAWR = true;
      elbowServo.write(ElbowPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      ElbowPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK6_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void WristMotor_Control(void * pvParameters)   // TASK7
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(WristPA[0] != WristPA[1] && !WristPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      WristPAWR = true;
      wristServo.write(WristPA[0]);  // Write to Inner Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      WristPA[2] = WristPA[1];
      WristPA[1] = WristPA[0];  // Log the new position
      WristPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      WristPAWR = true;
      wristServo.write(WristPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      WristPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK7_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void ThumbMotor_Control(void * pvParameters)   // TASK8
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(ThumbPA[0] != ThumbPA[1] && !ThumbPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ThumbPAWR = true;
      thumbServo.write(ThumbPA[0]);  // Write to Outer Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      ThumbPA[2] = ThumbPA[1];
      ThumbPA[1] = ThumbPA[0];  // Log the new position
      ThumbPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      ThumbPAWR = true;
      thumbServo.write(ThumbPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      ThumbPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK8_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void IndexMotor_Control(void * pvParameters)   // TASK9
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(IndexPA[0] != IndexPA[1] && !IndexPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      IndexPAWR = true;
      indexServo.write(IndexPA[0]);  // Write to Outer Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      IndexPA[2] = IndexPA[1];
      IndexPA[1] = IndexPA[0];  // Log the new position
      IndexPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      IndexPAWR = true;
      indexServo.write(IndexPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      IndexPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK9_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void MiddleMotor_Control(void * pvParameters)   // TASK10
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(MiddlePA[0] != MiddlePA[1] && !MiddlePAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      MiddlePAWR = true;
      middleServo.write(MiddlePA[0]);  // Write to Outer Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      MiddlePA[2] = MiddlePA[1];
      MiddlePA[1] = MiddlePA[0];  // Log the new position
      MiddlePAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      MiddlePAWR = true;
      middleServo.write(MiddlePA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      MiddlePAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK10_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK11_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void RingMotor_Control(void * pvParameters)   // TASK11
{
  for(;;)
  {
    // Wait for notification from Task 2
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // Perform angle update
    if(RingPA[0] != RingPA[1] && !RingPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      RingPAWR = true;
      ringServo.write(RingPA[0]);  // Write to Outer Ankle motor
      vTaskDelay(pdMS_TO_TICKS(10)); // Minor delay to allow motor to move
      RingPA[2] = RingPA[1];
      RingPA[1] = RingPA[0];  // Log the new position
      RingPAWR = false;  // Reset flag
    }

    // Optional: If no movement needed, update motor to last position
    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      RingPAWR = true;
      ringServo.write(RingPA[1]);
      vTaskDelay(pdMS_TO_TICKS(10));  // Minor wait
      RingPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK11_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK12_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void StepperUpperMotor_Control(void * pvParameters)    // TASK12
{
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(StepperMotorUpperPA[0] != StepperMotorUpperPA[1] && !StepperMotorUpperPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      StepperMotorUpperPAWR = true;
      long stepsToMove = (StepperMotorUpperPA[0]) - currentPosition_Upper;

      digitalWrite(pin_UpperDir, stepsToMove > 0 ? HIGH : LOW);
      stepsToMove = abs(stepsToMove);

      for(long i = 0; i < stepsToMove; i++)
      {
        digitalWrite(pin_UpperStep, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(pin_UpperStep, LOW);
        delayMicroseconds(stepDelayMicros);
      } 

      currentPosition_Upper =  StepperMotorUpperPA[0];

      vTaskDelay(pdMS_TO_TICKS(10));
      StepperMotorUpperPA[2] = StepperMotorUpperPA[1];
      StepperMotorUpperPA[1] = StepperMotorUpperPA[0];
      StepperMotorUpperPAWR = false;
    }

    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      StepperMotorUpperPAWR = true;
      long stepsToMove = (StepperMotorUpperPA[1]) - currentPosition_Upper;

      digitalWrite(pin_UpperDir, stepsToMove > 0 ? HIGH : LOW);
      stepsToMove = abs(stepsToMove);

      for(long i = 0; i < stepsToMove; i++)
      {
        digitalWrite(pin_UpperStep, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(pin_UpperStep, LOW);
        delayMicroseconds(stepDelayMicros);
      } 

      currentPosition_Upper =  StepperMotorUpperPA[1];
      vTaskDelay(pdMS_TO_TICKS(10));
      StepperMotorUpperPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK12_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK13_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}

void StepperLowerMotor_Control(void * pvParameters)    // TASK13
{
  for(;;)
  {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    if(StepperMotorLowerPA[0] != StepperMotorLowerPA[1] && !StepperMotorLowerPAWR)
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      StepperMotorLowerPAWR = true;
      long stepsToMove = (StepperMotorLowerPA[0]) - currentPosition_Lower;

      digitalWrite(pin_LowerDir, stepsToMove > 0 ? HIGH : LOW);
      stepsToMove = abs(stepsToMove);

      for(long i = 0; i < stepsToMove; i++)
      {
        digitalWrite(pin_LowerStep, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(pin_LowerStep, LOW);
        delayMicroseconds(stepDelayMicros);
      } 

      currentPosition_Lower =  StepperMotorLowerPA[0];

      vTaskDelay(pdMS_TO_TICKS(10));
      StepperMotorLowerPA[2] = StepperMotorLowerPA[1];
      StepperMotorLowerPA[1] = StepperMotorLowerPA[0];
      StepperMotorLowerPAWR = false;
    }

    else
    {
      xEventGroupWaitBits(syncEventGroup, TASK4_START_BIT | TASK5_START_BIT | TASK6_START_BIT | TASK7_START_BIT | TASK8_START_BIT | TASK9_START_BIT | TASK10_START_BIT | TASK11_START_BIT | TASK12_START_BIT | TASK13_START_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
      StepperMotorLowerPAWR = true;
      long stepsToMove = (StepperMotorLowerPA[1]) - currentPosition_Lower;

      digitalWrite(pin_LowerDir, stepsToMove > 0 ? HIGH : LOW);
      stepsToMove = abs(stepsToMove);

      for(long i = 0; i < stepsToMove; i++)
      {
        digitalWrite(pin_LowerStep, HIGH);
        delayMicroseconds(stepDelayMicros);
        digitalWrite(pin_LowerStep, LOW);
        delayMicroseconds(stepDelayMicros);
      } 

      currentPosition_Lower =  StepperMotorLowerPA[1];
      vTaskDelay(pdMS_TO_TICKS(10));
      StepperMotorLowerPAWR = false;
    }

    xEventGroupSetBits(syncEventGroup, TASK13_END_BIT);
    xEventGroupWaitBits(syncEventGroup, TASK4_END_BIT | TASK5_END_BIT | TASK6_END_BIT | TASK7_END_BIT | TASK8_END_BIT | TASK9_END_BIT | TASK10_END_BIT | TASK11_END_BIT | TASK12_END_BIT, pdTRUE, pdFALSE, portMAX_DELAY);
  }
}


void loop() {
  // Empty, RTOS is handling it!
}