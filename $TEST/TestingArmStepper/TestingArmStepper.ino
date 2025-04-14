#include <AccelStepper.h>

// Pin definitions for DRV8825
#define dirPin 1
#define stepPin 2
#define motorInterfaceType 1  // 1: DRV8825 Driver

AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

long targetPosition = 0;  // Default position

void setup() {
  Serial.begin(115200);           // Start serial monitor
  stepper.setMaxSpeed(1200);     // steps per second
  stepper.setAcceleration(600);  // steps per second squared
  stepper.setCurrentPosition(0);  // Start at zero

  Serial.println("Enter a target position (in steps):");
}

void loop() {
  // Keep moving toward the target
  stepper.run();

  // Check for serial input
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read input until Enter is pressed
    input.trim();                                 // Remove whitespace
    if (input.length() > 0) {
      long newTarget = input.toInt();  // Convert input to integer
      targetPosition = newTarget;
      stepper.moveTo(targetPosition);
      Serial.print("New target set to: ");
      Serial.println(targetPosition);
    }
  }
}
