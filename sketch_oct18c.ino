#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(3);  // Motor on M3 (left motor)
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);  // Motor on M4 (right motor)

// Line sensor pins
int leftSensorPin = 3;   // Left line sensor connected to pin 2
int rightSensorPin = 2;  // Right line sensor connected to pin 3
int frontSensorPin = 6;
int backSensorPin = 5;

// Button pins
int startStopButtonPin = 7; // First button for start/stop
int forwardButtonPin = 8;   // Second button for moving forward 5 seconds

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
  pinMode(frontSensorPin, INPUT);
  pinMode(backSensorPin, INPUT);

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

  // If system is running and not turning, check the second button for 5-second forward movement
  if (systemRunning && !isTurningRight && !isTurningLeft) {
    if (digitalRead(forwardButtonPin) == HIGH) {
      Serial.println("Moving forward for 5 seconds");
      myMotor1->run(FORWARD);  // Move left motor forward
      myMotor2->run(FORWARD);  // Move right motor forward
      delay(5000);  // Move forward for 5 seconds
      myMotor1->run(RELEASE);  // Stop both motors after 5 seconds
      myMotor2->run(RELEASE);
      delay(300);  // Debounce delay
    }

    // If the forward button is not pressed, execute the normal line-following logic
    int leftVal = digitalRead(leftSensorPin);    // Read left line sensor
    int rightVal = digitalRead(rightSensorPin);  // Read right line sensor
    int frontVal = digitalRead(frontSensorPin);
    int backVal = digitalRead(backSensorPin);

    // Print sensor values for debugging
    Serial.print(" | Left Sensor: ");
    Serial.print(leftVal);
    Serial.print(" | Right Sensor: ");
    Serial.print(rightVal);
    Serial.print(" | Front Sensor: ");
    Serial.print(frontVal);
    Serial.print(" | Back Sensor: ");
    Serial.println(backVal);

    // Check for cross-junction condition (all sensors are HIGH)
    if (leftVal == HIGH && rightVal == HIGH && frontVal == HIGH) {
      // Cross-junction detected: Move forward
      Serial.println("Cross-junction detected: Moving forward");
      myMotor1->run(FORWARD);  // Continue moving forward
      myMotor2->run(FORWARD);
      delay(1000);  // Move forward for a bit before checking again
      return;  // Continue moving forward, do not turn
    }

    // Check for 90-degree right turn condition
    if (leftVal == LOW && frontVal == LOW && rightVal == HIGH) {
      // Begin 90-degree right turn
      isTurningRight = true;
      Serial.println("Executing 90-degree right turn");
      executeRightTurn();
      return; // Exit the loop during the turn
    }

    // Check for 90-degree left turn condition
    if (leftVal == HIGH && frontVal == LOW && rightVal == LOW) {
      // Begin 90-degree left turn
      isTurningLeft = true;
      Serial.println("Executing 90-degree left turn");
      executeLeftTurn();
      return; // Exit the loop during the turn
    }

    // Check for T-junction condition (both left and right sensors are HIGH, front is LOW)
    if (leftVal == HIGH && rightVal == HIGH && frontVal == LOW) {
      tJunctionCount++;  // Increment T-junction count
      if (tJunctionCount == 1) {
        // First T-junction: Turn left
        isTurningLeft = true;
        Serial.println("T-junction 1: Turning left");
        executeLeftTurn();
      } else if (tJunctionCount == 2) {
        // Second T-junction: Turn right
        isTurningRight = true;
        Serial.println("T-junction 2: Turning right");
        executeRightTurn();
      }
      return; // Exit the loop during the turn
    }

    // Control logic based on sensor readings
    if (leftVal == LOW && rightVal == LOW && frontVal == HIGH) {
      myMotor1->run(FORWARD);  // Move left motor 
      myMotor2->run(FORWARD);  // Move right motor forward
      Serial.println("Driving forward");
    } else if (leftVal == LOW && rightVal == HIGH && frontVal == HIGH) {
      myMotor2->run(BACKWARD);  // Move right motor 
      myMotor1->run(FORWARD);  // Move left motor forward
      Serial.println("Adjusting left");
    } else if (leftVal == HIGH && rightVal == LOW && frontVal == HIGH) {
      myMotor2->run(FORWARD);  // Move right motor forward
      myMotor1->run(BACKWARD);  // Stop left motor
      Serial.println("Adjusting right");
    } else {
      myMotor2->run(RELEASE);
      myMotor1->run(RELEASE);
      Serial.println("Stopping, no line detected");
    }

    // Small delay to avoid rapid looping
    delay(50);
  }
}

// Function to execute a 90-degree right turn
void executeRightTurn() {
  // Set both motors to turn the robot right
  myMotor1->setSpeed(150);  // Set left motor speed
  myMotor2->setSpeed(150);  // Set right motor speed

  // Execute a right turn by running the left motor backward and right motor forward
  myMotor1->run(BACKWARD);  // Left motor moves backward
  myMotor2->run(FORWARD);   // Right motor moves forward

  // Adjust the delay to achieve a 90-degree turn (may need tuning based on motor power and turn duration)
  delay(1000);  // This delay should be fine-tuned to complete the 90-degree turn

  // Stop the motors after completing the turn
  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);  

  // Turn completed, reset the flag
  isTurningRight = false;
}

// Function to execute a 90-degree left turn
void executeLeftTurn() {
  // Set both motors to turn the robot left
  myMotor1->setSpeed(150);  // Set left motor speed
  myMotor2->setSpeed(150);  // Set right motor speed

  // Execute a left turn by running the right motor backward and left motor forward
  myMotor1->run(FORWARD);   // Left motor moves forward
  myMotor2->run(BACKWARD);  // Right motor moves backward

  // Adjust the delay to achieve a 90-degree turn (may need tuning based on motor power and turn duration)
  delay(1000);  // This delay should be fine-tuned to complete the 90-degree turn

  // Stop the motors after completing the turn
  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);  

  // Turn completed, reset the flag
  isTurningLeft = false;
}
