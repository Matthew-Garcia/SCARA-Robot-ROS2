/* 
   Title     : SCARA Robot (Arduino-based)
   Author    : Matthew Garcia
   Date      : November 26, 2025
   Objective : Implement a GUI-controlled SCARA robot controller that supports manual slider motion, 
               recording waypoints, and smooth looping playback using AccelStepper, with gripper control 
               and a safe STOP return to the last manual position.
*/

#include <AccelStepper.h>
#include <Servo.h>

// -------------------------------
//  STEP/DIR → CNC Shield Mapping
// -------------------------------
AccelStepper stepper1(1, 2, 5);   // J1
AccelStepper stepper2(1, 3, 6);   // J2
AccelStepper stepper3(1, 4, 7);   // J3
AccelStepper stepper4(1, 12, 13); // Z

Servo gripperServo;

// Angle → steps conversion
const float theta1AngleToSteps = 44.444444f;
const float theta2AngleToSteps = 35.555555f;
const float phiAngleToSteps    = 10.0f;
const float zDistanceToSteps   = 100.0f;

String content = "";
int data[10];

// Stored program arrays
int theta1Array[100];
int theta2Array[100];
int phiArray[100];
int zArray[100];
int gripperArray[100];
int positionsCounter = 0;

// Manual backup for STOP return
int manualJ1_backup = 0;
int manualJ2_backup = 0;
int manualJ3_backup = 0;
int manualZ_backup  = 0;

// Flags
bool ignoreManualUpdates = false;
bool saveManualPos = true;
bool firstSyncFromGUI = false;

// ---------------------------------
void homing() {
  stepper1.setCurrentPosition(0);
  stepper2.setCurrentPosition(0);
  stepper3.setCurrentPosition(0);
  stepper4.setCurrentPosition(0);
}
// ---------------------------------

void setup() {
  Serial.begin(115200);

  // Default motion limits
  stepper1.setMaxSpeed(4000);
  stepper1.setAcceleration(2000);

  stepper2.setMaxSpeed(4000);
  stepper2.setAcceleration(2000);

  stepper3.setMaxSpeed(4000);
  stepper3.setAcceleration(2000);

  stepper4.setMaxSpeed(4000);
  stepper4.setAcceleration(2000);

  // Fix Z direction
  stepper4.setPinsInverted(false, true);

  gripperServo.attach(A0, 600, 2500);
  gripperServo.write(180);
  delay(600);

  homing();
}

// ---------------------------------
void loop() {

  // ---------------------------------------
  // RECEIVE GUI PACKET
  // ---------------------------------------
  if (Serial.available()) {
    content = Serial.readString();

    for (int i = 0; i < 10; i++) {
      int index = content.indexOf(",");
      data[i] = atol(content.substring(0, index).c_str());
      content = content.substring(index + 1);
    }

    // Save a point
    if (data[0] == 1) {
      theta1Array[positionsCounter] = data[2] * theta1AngleToSteps;
      theta2Array[positionsCounter] = data[3] * theta2AngleToSteps;
      phiArray[positionsCounter]    = data[4] * phiAngleToSteps;
      zArray[positionsCounter]      = data[5] * zDistanceToSteps;
      gripperArray[positionsCounter] = data[6];
      positionsCounter++;
    }

    // Clear stored program
    if (data[0] == 2) {
      memset(theta1Array, 0, sizeof(theta1Array));
      memset(theta2Array, 0, sizeof(theta2Array));
      memset(phiArray, 0, sizeof(phiArray));
      memset(zArray, 0, sizeof(zArray));
      memset(gripperArray, 0, sizeof(gripperArray));
      positionsCounter = 0;
    }
  }

  // ---------------------------------------
  // RUN PROGRAM MODE
  // ---------------------------------------
  if (data[1] == 1) {

    ignoreManualUpdates = true;

    // Backup manual pose once
    if (saveManualPos) {
      manualJ1_backup = stepper1.currentPosition();
      manualJ2_backup = stepper2.currentPosition();
      manualJ3_backup = stepper3.currentPosition();
      manualZ_backup  = stepper4.currentPosition();
      saveManualPos = false;
    }

    // Apply speed/accel = maxSpeed
    stepper1.setMaxSpeed(data[7]);
    stepper2.setMaxSpeed(data[7]);
    stepper3.setMaxSpeed(data[7]);
    stepper4.setMaxSpeed(data[7]);

    stepper1.setAcceleration(data[8]);
    stepper2.setAcceleration(data[8]);
    stepper3.setAcceleration(data[8]);
    stepper4.setAcceleration(data[8]);

    // Loop through stored steps
    for (int i = 0; i < positionsCounter; i++) {

      // Break if STOP pressed
      if (data[1] == 0) break;

      stepper1.moveTo(theta1Array[i]);
      stepper2.moveTo(theta2Array[i]);
      stepper3.moveTo(phiArray[i]);
      stepper4.moveTo(zArray[i]);

      // Smooth motion to target
      while (stepper1.distanceToGo() != 0 ||
             stepper2.distanceToGo() != 0 ||
             stepper3.distanceToGo() != 0 ||
             stepper4.distanceToGo() != 0) {

        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();

        // LIVE STOP CHECK
        if (Serial.available()) {
          content = Serial.readString();
          int index = content.indexOf(",");
          data[1] = atol(content.substring(index + 1).c_str());
          if (data[1] == 0) break;
        }
      }

      if (data[1] == 0) break;

      // Gripper update
      if (i == 0 || gripperArray[i] != gripperArray[i - 1]) {
        gripperServo.write(gripperArray[i]);
        delay(300);
      }

      // Dwell time (smooth, robot-like)
      delay(300);   // <<< NEW
    }
  }

  // ---------------------------------------
  // STOP → return to manual position
  // ---------------------------------------
  if (data[1] == 0 && ignoreManualUpdates) {

    ignoreManualUpdates = false;

    stepper1.moveTo(manualJ1_backup);
    stepper2.moveTo(manualJ2_backup);
    stepper3.moveTo(manualJ3_backup);
    stepper4.moveTo(manualZ_backup);

    while (stepper1.distanceToGo() != 0 ||
           stepper2.distanceToGo() != 0 ||
           stepper3.distanceToGo() != 0 ||
           stepper4.distanceToGo() != 0) {

      stepper1.run();
      stepper2.run();
      stepper3.run();
      stepper4.run();
    }

    saveManualPos = true;
  }

  // ---------------------------------------
  // MANUAL SLIDER CONTROL
  // ---------------------------------------
  if (!ignoreManualUpdates) {

    int j1Steps = data[2] * theta1AngleToSteps;
    int j2Steps = data[3] * theta2AngleToSteps;
    int j3Steps = data[4] * phiAngleToSteps;
    int zSteps  = data[5] * zDistanceToSteps;

    if (!firstSyncFromGUI) {
      stepper1.setCurrentPosition(j1Steps);
      stepper2.setCurrentPosition(j2Steps);
      stepper3.setCurrentPosition(j3Steps);
      stepper4.setCurrentPosition(zSteps);
      firstSyncFromGUI = true;
    } else {
      stepper1.moveTo(j1Steps);
      stepper2.moveTo(j2Steps);
      stepper3.moveTo(j3Steps);
      stepper4.moveTo(zSteps);

      while (stepper1.distanceToGo() != 0 ||
             stepper2.distanceToGo() != 0 ||
             stepper3.distanceToGo() != 0 ||
             stepper4.distanceToGo() != 0) {

        stepper1.run();
        stepper2.run();
        stepper3.run();
        stepper4.run();
      }
    }

    gripperServo.write(data[6]);
    delay(200);
  }
}

