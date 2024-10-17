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
  // Set initial motor speeds
  myMotor1->setSpeed(150);  // Max speed for Motor 1
  myMotor2->setSpeed(150);  // Max speed for Motor 2
}

void loop() {
  // Read the values from the line sensors
  int leftVal = digitalRead(leftSensorPin);    // Read left line sensor
  int rightVal = digitalRead(rightSensorPin);  // Read right line sensor
  int frontVal = digitalRead(frontSensorPin);
  int backVal = digitalRead(backSensorPin);

  // Print sensor values for debugging
  Serial.print("Left Sensor: ");
  Serial.print(leftVal);
  Serial.print("Right Sensor: ");
  Serial.println(rightVal);
  Serial.print("front Sensor: ");
  Serial.print(frontVal);
  Serial.print("back Sensor: ");
  Serial.print(backVal);
  // Control logic based on sensor readings
  if (leftVal == LOW && rightVal == LOW && frontVal == HIGH) {
    // Both sensors see black (move forward)
    myMotor1->run(BACKWARD);  // Move left motor 
    myMotor2->run(BACKWARD);  // Move right motor forward
    Serial.println("Driving forward");
    
  } else if (leftVal == LOW && rightVal == HIGH && frontVal == HIGH) {
    // Left sensor sees black, right sensor sees white (turn left)
    myMotor2->run(FORWARD);  // run motor 
    myMotor1->run(BACKWARD);  // Move right motor forward
    Serial.println("adjusting left");
    
  } else if (leftVal == HIGH && rightVal == LOW && frontVal == HIGH) {
    // Right sensor sees black, left sensor sees white (turn right)
    myMotor2->run(BACKWARD);  // Move left motor forward
    myMotor1->run(RELEASE);  // Stop right motor
    Serial.println("adjusting right");
    
  } else if (leftVal == LOW && rightVal == HIGH && frontVal == LOW) {
    // Corner(Turning Right)
    myMotor2->run(RELEASE);  // Move left motor forward
    myMotor1->run(BACKWARD);  // Stop right motor
    Serial.println("Turning right");
  } else if (leftVal == HIGH && rightVal == LOW && frontVal == LOW) {
    // Corner(turning left)
    myMotor2->run(BACKWARD);  // Stop left motor
    myMotor1->run(RELEASE);  // Move right motor forward
    Serial.println("turning left");
  } else if (leftVal == HIGH && rightVal == HIGH && frontVal == LOW){
    //T-junction
    myMotor2->run(BACKWARD);  // Move left motor forward
    myMotor1->run(RELEASE);  // Stop right motor
    Serial.println("Turning right");
  } else if (leftVal == HIGH && rightVal == HIGH && frontVal == HIGH){
    myMotor2->run(BACKWARD);  // Move left motor forward
    myMotor1->run(BACKWARD);  // Move right motor forward
    Serial.println("Keep Moving");
  }  else {
    myMotor2->run(FORWARD);
    myMotor1->run(FORWARD);
    Serial.println("Stopping, no line detected");
  }
  delay(100);  // Small delay to avoid rapid looping
}