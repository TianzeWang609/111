#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
Adafruit_DCMotor *left_motor = AFMS.getMotor(4);  // Motor on port 4 (left motor)

// Line sensor pins
int leftSensorPin = 3;   // initialising variables
int rightSensorPin = 2;  
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
  right_motor ->setSpeed(158);  // Max speed for right motor
  left_motor ->setSpeed(150);  // Max speed for left motor
}


void loop() {
  // Read the values from the line sensors
  //right_motor -> run(FORWARD) ;
  //left_motor -> run(FORWARD) ;
  course_correction() ;
}

void course_correction() {
  // Read the values from the line sensors
  int leftVal = digitalRead(leftSensorPin);    // Read the line sensors
  int rightVal = digitalRead(rightSensorPin);
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
  right_motor ->setSpeed(158);  // Max speed for right motor
  left_motor ->setSpeed(150);  // Max speed for left motor
  if (leftVal == LOW && rightVal == LOW && frontVal == HIGH) {
    // Both sensors see black (move forward)
    right_motor->run(FORWARD);  // Move right motor forward
    left_motor->run(FORWARD);  // Move left motor forward
    Serial.println("Driving forward");
    
  } else if (leftVal == LOW && rightVal == HIGH && frontVal == HIGH) {
    // Left sensor sees black, right sensor sees white (turn left)
    right_motor ->setSpeed(140);
    right_motor ->run(FORWARD);  // Move right motor  backwards
    left_motor ->run(FORWARD);  // Move left motor forward
    Serial.println("adjusting left");

  }else if (leftVal == LOW && rightVal == HIGH && frontVal == LOW) {
    // Left sensor sees black, right sensor sees white (turn left extreme)
    right_motor -> setSpeed(120);
    right_motor ->run(FORWARD);  // Move right motor  backwards
    left_motor ->run(FORWARD);  // Move left motor forward
    Serial.println("adjusting left");
    
  } else if (leftVal == HIGH && rightVal == LOW && frontVal == HIGH) {
    // Right sensor sees black, left sensor sees white (turn right)
    left_motor -> setSpeed(140);
    right_motor->run(FORWARD);  // Move the right motor forward
    left_motor->run(FORWARD);  // Move the left motor backwards
    Serial.println("adjusting right");
  } else if (leftVal == HIGH && rightVal == LOW && frontVal == LOW) {
    // Right sensor sees black, left sensor sees white (turn right extreme)
    left_motor -> setSpeed(120);
    right_motor->run(FORWARD);  // Move the right motor forward
    left_motor->run(FORWARD);  // Move the left motor backwards
    Serial.println("adjusting right");
  }  else {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    Serial.println("Stopping, no line detected");
  }
  delay(5); 
}







