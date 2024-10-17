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

  // If system is running, check the second button for 5-second forward movement
  if (systemRunning) {
    if (digitalRead(forwardButtonPin) == HIGH) {
      Serial.println("Moving forward for 5 seconds");
      myMotor1->run(BACKWARD);  // Move left motor forward
      myMotor2->run(BACKWARD);  // Move right motor forward
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
    Serial.print("Left Sensor: ");
    Serial.print(leftVal);
    Serial.print(" | Right Sensor: ");
    Serial.print(rightVal);
    Serial.print(" | Front Sensor: ");
    Serial.print(frontVal);
    Serial.print(" | Back Sensor: ");
    Serial.println(backVal);

    // Control logic based on sensor readings
    if (leftVal == LOW && rightVal == LOW && frontVal == HIGH) {
      myMotor1->run(BACKWARD);  // Move left motor 
      myMotor2->run(BACKWARD);  // Move right motor forward
      Serial.println("Driving forward");
    } else if (leftVal == LOW && rightVal == HIGH && frontVal == HIGH) {
      myMotor2->run(FORWARD);  // Move right motor 
      myMotor1->run(BACKWARD);  // Move left motor forward
      Serial.println("Adjusting left");
    } else if (leftVal == HIGH && rightVal == LOW && frontVal == HIGH) {
      myMotor2->run(BACKWARD);  // Move right motor forward
      myMotor1->run(RELEASE);  // Stop left motor
      Serial.println("Adjusting right");
    } else if (leftVal == LOW && rightVal == HIGH && frontVal == LOW) {
      myMotor2->run(RELEASE);  // Stop left motor
      myMotor1->run(BACKWARD);  // Right motor moves
      Serial.println("Turning right");
    } else if (leftVal == HIGH && rightVal == LOW && frontVal == LOW) {
      myMotor2->run(BACKWARD);  // Left motor moves
      myMotor1->run(RELEASE);  // Stop right motor
      Serial.println("Turning left");
    } else if (leftVal == HIGH && rightVal == HIGH && frontVal == LOW) {
      myMotor2->run(BACKWARD);  // Move left motor forward
      myMotor1->run(RELEASE);  // Stop right motor
      Serial.println("Turning right at T-junction");
    } else if (leftVal == HIGH && rightVal == HIGH && frontVal == HIGH) {
      myMotor2->run(BACKWARD);  // Move left motor forward
      myMotor1->run(BACKWARD);  // Move right motor forward
      Serial.println("Keep Moving");
    } else {
      myMotor2->run(RELEASE);
      myMotor1->run(RELEASE);
      Serial.println("Stopping, no line detected");
    }

    // Small delay to avoid rapid looping
    delay(50);
  }
}

