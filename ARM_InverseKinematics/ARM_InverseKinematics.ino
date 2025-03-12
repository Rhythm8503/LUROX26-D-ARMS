#include <Arduino.h>
#include <math.h>
#include <stdio.h>

  const float L1 = 2.598;
  const float L2 = 2.44;
  const float L3 = 6.649;
  const float L4 = 7.071;
  const float L5 = 0.74;
  const float L6 = 1.00;   

  const float joint_limits[6][2] = 
  {
    {-5.0, 180.0},
    {-110.0, 110.0},
    {-180, 90},
    {-5, 120},
    {-90, 90},
    {-40, 100}
  };

  void crossProduct(float a[3], float b[3], float result[3])
  {
    result[0] = (a[1] * b[2]) - (a[2] * b[1]);
    result[1] = (a[2] * b[0]) - (a[0] * b[2]);
    result[2] = (a[0] * b[1]) - (a[1] * b[0]);
  }

  float dotProduct(float a[3], float b[3])
  {
    return (a[0] * b[0]) + (a[1] * b[1]) + (a[2] * b[2]);
  }

  float padenKahan1(float omega[3], float q[3], float target[3])
  {
    float v[3];
    float u[3] = {target[0] - q[0], target[1] - q[1], target[2] - q[2]};
    crossProduct(omega, u, v);
    return atan2(sqrt(dotProduct(v, v)), dotProduct(omega, u));
  }

  float clampAngle(float theta, int joint)
  {
    return max(joint_limits[joint][0], min(joint_limits[joint][1], theta));
  }

void setup() 
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Enter target x, y, z: ");
  Serial.print("Hello Brian!");
}

void loop() 
{
  if(Serial.available() > 0)
  {
    float x, y, z;
    x = Serial.parseFloat();
    y = Serial.parseFloat();
    z = Serial.parseFloat();

    float target_pos[3] = {x, y, z};

    float omega1[3] = {0, 1, 0};
    float omega4[3] = {0, 0, 1};
    float omega5[3] = {1, 0, 0};
    float omega6[3] = {0, 1, 0};

    float q1[3] = {0, 0, 0};
    float q4[3] = {L1 + L2 + L3, 0, 0};
    float q5[3] = {L1 + L2 + L3 + L4, 0, 0};
    float q6[3] = {L1 + L2 + L3 + L4 + L5, 0, 0};

    float theta1 = clampAngle(padenKahan1(omega1, q1, target_pos), 0);
    float theta4 = clampAngle(padenKahan1(omega4, q4, target_pos), 3);
    float theta5 = clampAngle(padenKahan1(omega5, q5, target_pos), 4);
    float theta6 = clampAngle(padenKahan1(omega6, q6, target_pos), 5);

    // Serial.printf("Computed Joint Angles:\n");
    // Serial.printf("Theta1: %.2f degrees\n", theta1 * 180/M_PI);
    // Serial.printf("Theta4: %.2f degrees\n", theta4 * 180/M_PI);
    // Serial.printf("Theta5: %.2f degrees\n", theta5 * 180/M_PI);
    // Serial.printf("Theta6: %.2f degrees\n", theta6 * 180/M_PI);
    // Serial.println("---------------------------------------");

    Serial.println("Computed Joint Angles:");
    Serial.print("Theta1: "); Serial.print(theta1 * 180 / M_PI, 2); Serial.println(" degrees");
    Serial.print("Theta4: "); Serial.print(theta4 * 180 / M_PI, 2); Serial.println(" degrees");
    Serial.print("Theta5: "); Serial.print(theta5 * 180 / M_PI, 2); Serial.println(" degrees");
    Serial.print("Theta6: "); Serial.print(theta6 * 180 / M_PI, 2); Serial.println(" degrees");
    Serial.println("---------------------------------------");


  }













}
