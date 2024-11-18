#include <ESP32Servo.h>

Servo myServo;
Servo myServo2;
Servo myServo3;

float biseplength= 9.59;
float forarmlength= 9.59;
float xinput= 7.0;  //   max is 19.15  if  y=0 and z=0 
float x;
float x1;
float yinput= 14.0; //   max is 19.15     min is 11 when x=0 and z=0
float y;
float y11;
float zinput= 2.0;  //   max is 19.15  if  y=0 and x=0
float z;
float z1;
float elboAngleRad;
float lowSholderAngleRad;
float xDirectionPlaceHolderRad;
float zDirectionPlaceHolderRad;
int xDirectionPlaceHolder;
int zDirectionPlaceHolder;
int lowSholderAngle;
int elboangle;
int lowSholderAngleTol;
int elboangleTol;
int upersholder;

ESP32PWM pwm;

void setup() 
{
  Serial.begin(115200);

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);

  myServo.attach(4);
  myServo2.attach(5);
  myServo3.attach(6);
  delay(1000);
  

  // z arm movment with z quardnet inpit 
  zDirectionPlaceHolderRad= atan(zinput/(yinput));
  zDirectionPlaceHolder= round(zDirectionPlaceHolderRad*(180/PI));
  upersholder= zDirectionPlaceHolder;
  y= (yinput/cos(zDirectionPlaceHolderRad));


  // x arm movment with x quardnet inpit 
  xDirectionPlaceHolderRad= atan(xinput/y);
  xDirectionPlaceHolder= round(xDirectionPlaceHolderRad*(180/PI));
  y11= y/cos(xDirectionPlaceHolderRad);


  // y arm movment with y quardnet inpit 
  elboAngleRad= acos((pow(biseplength,2) + pow(forarmlength,2) - pow(y11,2)) / (2*pow(biseplength,2))); 
  elboangle= round(elboAngleRad*(180/PI));
  lowSholderAngle= (180-elboangle)/2;


  //add the sholder angles todether  
  lowSholderAngleTol= lowSholderAngle - xDirectionPlaceHolder;


  Serial.println("elboangle");
  Serial.println(elboangle);
  Serial.println("low sholder");
  Serial.println(lowSholderAngleTol);
  Serial.println(lowSholderAngleTol+90);
  Serial.println("upersholder");
  Serial.println(upersholder);
  Serial.println(90-upersholder);


  myServo.write(90-upersholder);
  delay(100);
  myServo2.write(lowSholderAngleTol+90);
  delay(100);
  if (elboangle>=70){ 
   myServo3.write(elboangle);
    }
    else{ 
      myServo3.write(70);
      }
   
  delay(100);

}

void loop() 
{
 
}