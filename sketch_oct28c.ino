#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(2);  // Motor on M3 (left motor)
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(3);  // Motor on M4 (right motor)

// Line sensor pins
int leftSensorPin = 5;   // Left line sensor connected to pin 5
int rightSensorPin = 6;  // Right line sensor connected to pin 6
int frontleftSensorPin = 2;  // Front left sensor for turning
int frontrightSensorPin = 3; // Front right sensor for turning

// Button pins
int startStopButtonPin = 7; // Button for start/stop
int forwardButtonPin = 8;   // Button for moving forward 5 seconds

// Variable to track system state (1 = running, 0 = stopped)
int systemRunning = 0;
bool isTurningRight = false;  // Flag for 90-degree right turn
bool isTurningLeft = false;   // Flag for 90-degree left turn
int tJunctionCount = 0;       // Counter to track how many T-junctions have been encountered

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - Dual Line Sensor Controlled Motors!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set up line sensor pins
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(frontleftSensorPin, INPUT);
  pinMode(frontrightSensorPin, INPUT);

  // Set up button pins
  pinMode(startStopButtonPin, INPUT);
  pinMode(forwardButtonPin, INPUT);

  // Set initial motor speeds
  myMotor1->setSpeed(150);  // Speed for Motor 1 (left)
  myMotor2->setSpeed(150);  // Speed for Motor 2 (right)
}

void loop() {
  // Check if the start/stop button has been pressed
  if (digitalRead(startStopButtonPin) == HIGH) {
    systemRunning = !systemRunning; // Toggle system state
    delay(300); // Debounce delay
    if (systemRunning) {
      Serial.println("System started");
    } else {
      Serial.println("System stopped");
      myMotor1->run(RELEASE);  // Stop both motors
      myMotor2->run(RELEASE);
      return; // Exit loop if system is stopped
    }
  }

  // If system is running, proceed with sensor detection and navigation logic
  if (systemRunning) {
    int leftVal = digitalRead(leftSensorPin);    // Read left line sensor
    int rightVal = digitalRead(rightSensorPin);  // Read right line sensor
    int frontleftVal = digitalRead(frontleftSensorPin);  // Read front left sensor
    int frontrightVal = digitalRead(frontrightSensorPin); // Read front right sensor

    // Print sensor values for debugging
    Serial.print(" | Left Sensor: ");
    Serial.print(leftVal);
    Serial.print(" | Right Sensor: ");
    Serial.print(rightVal);
    Serial.print(" | Front Left Sensor: ");
    Serial.print(frontleftVal);
    Serial.print(" | Front Right Sensor: ");
    Serial.println(frontrightVal);

    // Check for 90-degree right turn condition using front sensors
    if (frontleftVal == LOW && frontrightVal == HIGH) {
      isTurningRight = true;
      Serial.println("Executing 90-degree right turn");
      executeRightTurn();
      return; // Exit the loop during the turn
    }

    // Check for 90-degree left turn condition using front sensors
    if (frontleftVal == HIGH && frontrightVal == LOW) {
      isTurningLeft = true;
      Serial.println("Executing 90-degree left turn");
      executeLeftTurn();
      return; // Exit the loop during the turn
    }

    // Default condition: perform course correction based on left and right line sensors
    course_correction(leftVal, rightVal);
  }
}

// Function to execute a 90-degree right turn
void executeRightTurn() {
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150);
  myMotor1->run(BACKWARD);  
  myMotor2->run(FORWARD);   
  delay(1000);  // Fine-tune for a 90-degree turn
  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);  
  isTurningRight = false;
}

// Function to execute a 90-degree left turn
void executeLeftTurn() {
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150);
  myMotor1->run(FORWARD);   
  myMotor2->run(BACKWARD);  
  delay(1000);  // Fine-tune for a 90-degree turn
  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);  
  isTurningLeft = false;
}

// Function to perform course correction
void course_correction(int leftVal, int rightVal) {
  myMotor1->setSpeed(150);
  myMotor2->setSpeed(150);

  if (leftVal == HIGH && rightVal == HIGH) {
    // Both sensors detect the line, move straight forward
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    Serial.println("Driving forward on the line");
  } else if (leftVal == LOW && rightVal == HIGH) {
    // Left sensor is off the line, adjust slightly to the left
    myMotor2->setSpeed(130);  // Slow down left motor to adjust
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    Serial.println("Adjusting left");
  } else if (leftVal == HIGH && rightVal == LOW) {
    // Right sensor is off the line, adjust slightly to the right
    myMotor1->setSpeed(130);  // Slow down right motor to adjust
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    Serial.println("Adjusting right");
  }
  delay(5); // Small delay for adjustment
}

