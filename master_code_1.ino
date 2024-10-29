#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
Adafruit_DCMotor *left_motor = AFMS.getMotor(4);  // Motor on port 4 (left motor)

// Line sensor pins
int leftSensorPin = 3;   // initialising variables
int rightSensorPin = 2;  
int frontrightSensorPin = 6;
int frontleftSensorPin = 5;
int leftVal = 0;    // creating global variable for values of pin
int rightVal = 0;
int frontrightVal = 0;
int frontleftVal = 0 ;
int tJunctionCount = 0 ;
int right_turn_counter = 0 ;
int left_turn_counter = 0 ;

void course_correction() {
  right_motor ->setSpeed(218);  // Max speed for right motor
  left_motor ->setSpeed(210);  // Max speed for left motor
  if (leftVal == HIGH && rightVal == HIGH ) {
    // all sensors see white (move forward)
    right_motor->run(BACKWARD);  
    left_motor->run(BACKWARD);  
    Serial.println("Driving forward");

  }else if (leftVal == LOW && rightVal == HIGH) {
    // Left sensor sees black, right sensor sees white (veer right)
    right_motor -> setSpeed(158);
    
    right_motor ->run(BACKWARD);  
    left_motor ->run(BACKWARD);  
    Serial.println("adjusting left");
    
  } else if (leftVal == HIGH && rightVal == LOW) {
    // Right sensor sees black, left sensor sees white (veer left)
    left_motor -> setSpeed(150);
    right_motor->run(BACKWARD);  
    left_motor->run(BACKWARD);  
    Serial.println("adjusting right");
  } else {
    left_motor->run(RELEASE);  
    right_motor->run(RELEASE);  
    Serial.println("Finished");
  }
  delay(1000); 
}


void right_turn() {
  // programme to let the robot turn to the right
  left_motor -> setSpeed(150);
  right_motor -> setSpeed(158);
  right_motor ->run(BACKWARD);  
  left_motor ->run(FORWARD);  
  Serial.println(" Turning right ");
  delay(1000) ;
}
   
void left_turn() {
  // programme to let the robot turn to the left
  left_motor -> setSpeed(150);
  right_motor -> setSpeed(158);
  right_motor ->run(FORWARD);  
  left_motor ->run(BACKWARD);  
  Serial.println(" Turning left ");
  delay(1000) ;
}
void junction_checker() {
  // delay to ensure that reading is accurate in case robot is at strange angle
  if (frontrightVal == HIGH && frontleftVal == LOW) {
    delay(100) ;
  } else if (frontrightVal == LOW && frontleftVal == HIGH) { 
    delay(100) ;
  }

  // re-read line data after a delay
  leftVal = digitalRead(leftSensorPin);    // Read the line sensors
  rightVal = digitalRead(rightSensorPin);
  frontrightVal = digitalRead(frontrightSensorPin);
  frontleftVal = digitalRead(frontleftSensorPin);

 // add appropriate counts to turns detected 

  if ( frontrightVal == HIGH && frontleftVal == HIGH){
  // both left and right see white t junction
    tJunctionCount = tJunctionCount + 1 ;
    Serial.println(" T Junction detected ");
  }
  if ( frontrightVal == HIGH && frontleftVal == LOW){
  // right sees white , left sees black
    right_turn_counter = right_turn_counter + 1 ;
    Serial.println(" right turn rejected ");
  }
  if ( frontrightVal == LOW && frontleftVal == HIGH) {
  // left sees white , left sees black
    left_turn_counter = left_turn_counter + 1 ;
    Serial.println(" left turn detected ");
  }
}

void navigation () {
  if (tJunctionCount == 2) {
    left_turn() ;
  }

}
// Button pins
//int startStopButtonPin = 7; // First button for start/stop
//int forwardButtonPin = 8;   // Second button for moving forward 5 seconds

// Variable to track system state (1 = running, 0 = stopped)
//int systemRunning = 0;

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
  pinMode(frontrightSensorPin, INPUT);
  pinMode(frontleftSensorPin, INPUT);
  // Set initial motor speeds
  right_motor ->setSpeed(158);  // Max speed for right motor
  left_motor ->setSpeed(150);  // Max speed for left motor
}

void loop() {
  // Read the values from the line sensors
  int leftVal = digitalRead(leftSensorPin);    // Read the line sensors
  int rightVal = digitalRead(rightSensorPin);
  int frontrightVal = digitalRead(frontrightSensorPin);
  int frontleftVal = digitalRead(frontleftSensorPin);

  // Print sensor values for debugging
  Serial.print("Left Sensor: ");
  Serial.print(leftVal);
  Serial.print("Right Sensor: ");
  Serial.println(rightVal);
  Serial.print("front right Sensor: ");
  Serial.print(frontrightVal);
  Serial.print("front left Sensor: ");
  Serial.print(frontleftVal);
  // start subroutines for line following and routing 
  junction_checker() ;
  //navigation() ;
  course_correction() ;

}

