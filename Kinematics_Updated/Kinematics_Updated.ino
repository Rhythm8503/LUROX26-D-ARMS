#include <ESP32Servo.h>

Servo myServo;
Servo myServo2;
Servo myServo3;

float biseplength = 9.59;
float forarmlength = 9.59;
float xinput = 7.0;  // Max is 19.15 if y=0 and z=0
float yinput = 14.0; // Max is 19.15, min is 11 when x=0 and z=0
float zinput = 2.0;  // Max is 19.15 if y=0 and x=0

float x, x1, y, y11, z, z1;
float elboAngleRad, lowSholderAngleRad, xDirectionPlaceHolderRad, zDirectionPlaceHolderRad;
int xDirectionPlaceHolder, zDirectionPlaceHolder;
int lowSholderAngle, elboangle, lowSholderAngleTol, elboangleTol, upersholder;

ESP32PWM pwm;

void setup() {
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  myServo.attach(4);
  myServo2.attach(5);
  myServo3.attach(6);
  delay(1000);
  
  Serial.println("Enter x, y, z values in the format: x y z");
  Serial.println("Example: 7.0 14.0 2.0");
}

void loop() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove trailing newline characters

    // Split input into x, y, z values
    int firstSpace = input.indexOf(' ');
    int secondSpace = input.indexOf(' ', firstSpace + 1);

    if (firstSpace > 0 && secondSpace > firstSpace) 
    {
      xinput = input.substring(0, firstSpace).toFloat();
      yinput = input.substring(firstSpace + 1, secondSpace).toFloat();
      zinput = input.substring(secondSpace + 1).toFloat();

      if (validateInputs(xinput, yinput, zinput)) 
      {
        calculateServoAngles();

        Serial.println("Angles updated:");
        Serial.print("Elbow Angle: "); Serial.println(elboangle);
        Serial.print("Lower Shoulder Angle: "); Serial.println(lowSholderAngleTol + 90);
        Serial.print("Upper Shoulder Angle: "); Serial.println(90 - upersholder);
      }
    } 
    else 
    {
      Serial.println("Invalid input format. Please use: x y z");
    }
  }
}

bool validateInputs(float x, float y, float z) 
{
  bool isValid = true;

  if (y == 0 && z == 0 && x > 19.15) 
  {
    Serial.println("Error: x exceeds maximum value of 19.15 when y=0 and z=0.");
    isValid = false;
  }
  if (x == 0 && z == 0 && (y < 11 || y > 19.15)) 
  {
    Serial.println("Error: y must be between 11 and 19.15 when x=0 and z=0.");
    isValid = false;
  }
  if (y == 0 && x == 0 && z > 19.15) 
  {
    Serial.println("Error: z exceeds maximum value of 19.15 when y=0 and x=0.");
    isValid = false;
  }

  if (!isValid) 
  {
    Serial.println("Please re-enter valid values.");
  }

  return isValid;
}

void calculateServoAngles() 
{
  // Z arm movement
  zDirectionPlaceHolderRad = atan(zinput / yinput);
  zDirectionPlaceHolder = round(zDirectionPlaceHolderRad * (180 / PI));
  upersholder = zDirectionPlaceHolder;
  y = (yinput / cos(zDirectionPlaceHolderRad));

  // X arm movement
  xDirectionPlaceHolderRad = atan(xinput / y);
  xDirectionPlaceHolder = round(xDirectionPlaceHolderRad * (180 / PI));
  y11 = y / cos(xDirectionPlaceHolderRad);

  // Y arm movement
  elboAngleRad = acos((pow(biseplength, 2) + pow(forarmlength, 2) - pow(y11, 2)) / (2 * biseplength * forarmlength));
  elboangle = round(elboAngleRad * (180 / PI));
  lowSholderAngle = (180 - elboangle) / 2;

  // Add the shoulder angles together
  lowSholderAngleTol = lowSholderAngle - xDirectionPlaceHolder;

  // Update servos
  myServo.write(90 - upersholder);
  delay(100);
  myServo2.write(lowSholderAngleTol + 90);
  delay(100);

  if (elboangle >= 70) 
  {
    myServo3.write(elboangle);
  } 
  else 
  {
    myServo3.write(70);
  }
  
  delay(100);
}
