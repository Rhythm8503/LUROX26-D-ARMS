#include <AccelStepper.h>

// Pin definitions for DRV8825
#define dirPin 1
#define stepPin 2
#define motorInterfaceType 1  // 1: DRV8825 Driver

AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

long targetPosition = 0;  // Default position

void setup() {
  Serial.begin(115200);           // Start serial monitor
  stepper.setMaxSpeed(1000);     // steps per second
  stepper.setAcceleration(1000);  // steps per second squared
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

// #include <AccelStepper.h>

// // Pin definitions for DRV8825
// #define dirPin 1
// #define stepPin 2
// #define motorInterfaceType 1  // 1: DRV8825 Driver

// AccelStepper stepper(motorInterfaceType, stepPin, dirPin);

// long sequence[] = {-50, 50, 0};  // Positions to move to
// int currentStep = -1;            // Index of current target in sequence (-1 means idle)
// bool runningSequence = false;    // Flag to track if sequence is active

// void setup() {
//   Serial.begin(115200);
//   stepper.setMaxSpeed(200);
//   stepper.setAcceleration(100);
//   stepper.setCurrentPosition(0);

//   Serial.println("Press 'n' to run the swing sequence (-50 -> 50 -> 0) once.");
// }

// void loop() {
//   // Check for serial input
//   if (Serial.available()) {
//     char input = Serial.read();
//     if (input == 'n' && !runningSequence) {
//       runningSequence = true;
//       currentStep = 0;
//       stepper.moveTo(sequence[currentStep]);
//       Serial.println("Starting sequence...");
//     }
//   }

//   // Run stepper toward target
//   if (runningSequence) {
//     stepper.run();
//     if (stepper.distanceToGo() == 0) {
//       currentStep++;
//       if (currentStep < 3) {
//         stepper.moveTo(sequence[currentStep]);
//       } else {
//         runningSequence = false;  // Done with sequence
//         currentStep = -1;
//         Serial.println("Sequence complete.");
//       }
//     }
//   }
// }

