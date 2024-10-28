#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
Adafruit_DCMotor *left_motor = AFMS.getMotor(2);  // Motor on port 4 (left motor)

// Line sensor pins
int leftSensorPin = 3;   // initialising variables
int rightSensorPin = 2;  
int frontrightSensorPin = 6;
int frontleftSensorPin = 5;
int leftVal = 0;    // creating global variable for values of pin
int rightVal = 0;
int frontrightVal = 0;
int frontleftVal = 0 ;

// Button pins
int startStopButtonPin = 7; // First button for start/stop
int forwardButtonPin = 8;   // Second button for moving forward 5 seconds

// Variable to track system state (1 = running, 0 = stopped)
int systemRunning = 0;
int tJunctionCount = 0;       // Counter to track how many T-junctions have been encountered
int left_turn_counter = 0 ;
int right_turn_counter = 0;
bool junctionDetected = false;
bool pathComplete = false;

void setup() {
  Serial.begin(9600);         
  Serial.println("Adafruit Motorshield v2 - Dual Line Sensor Controlled Motors!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
  // Check if the start/stop button has been pressed
  if (digitalRead(startStopButtonPin) == HIGH) {
    systemRunning = !systemRunning; // Toggle system state
    delay(300); // Debounce delay
    if (systemRunning) {
      Serial.println("System started");
    } else {
      Serial.println("System stopped");
      right_motor->run(RELEASE);  // Stop both motors
      left_motor->run(RELEASE);
      return; // Exit loop if system is stopped
    }
  }
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
  //course_correction() ;
  goFrom1To2();
}

void course_correction() {
  right_motor ->setSpeed(150);  // Max speed for right motor
  left_motor ->setSpeed(150);  // Max speed for left motor
  if (leftVal == HIGH && rightVal == HIGH ) {
    // BACK 2 sensors see white (move forward)
    right_motor->run(FORWARD);  
    left_motor->run(FORWARD);  
    Serial.println("Driving forward");

  }else if (leftVal == LOW && rightVal == HIGH) {
    // Left sensor sees black, right sensor sees white (turn left extreme)
    left_motor -> setSpeed(130);
    right_motor ->run(FORWARD);  
    left_motor ->run(FORWARD);  
    Serial.println("adjusting left");
    
  } else if (leftVal == HIGH && rightVal == LOW) {
    // Right sensor sees black, left sensor sees white (turn right) , 2 front see black
    right_motor -> setSpeed(130);
    right_motor->run(FORWARD);  
    left_motor->run(FORWARD);  
    Serial.println("adjusting right");
  }
  delay(5); 
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
    junctionDetected == true;
    Serial.println(" T Junction detected ");
  }
  if ( frontrightVal == HIGH && frontleftVal == LOW){
  // right sees white , left sees black
    right_turn_counter = right_turn_counter + 1 ;
    junctionDetected == true;
    Serial.println(" right turn detected ");
  }
  if ( frontrightVal == LOW && frontleftVal == HIGH) {
  // left sees white , left sees black
    left_turn_counter = left_turn_counter + 1 ;
    junctionDetected == true;
    Serial.println(" left turn detected ");
  }
}


void goFrom1To2() {
  Serial.println("Starting path 1 to 2");
  while (pathComplete == false) {
    if (junctionDetected == true) {
      junction_checker();


      if (tJunctionCount == 3) {
        left_turn() ;
      }

      else if (tJunctionCount == 5) {
      right_turn() ;
      }

      else if (right_turn_counter == 2 || right_turn_counter == 3 || right_turn_counter == 4) {
        right_turn() ;
      }

      else if (right_turn_counter == 4 && tJunctionCount == 5) {
        pathComplete = true ;
      }
      junctionDetected = false;
    }  
    else {
      course_correction();
    }
  }
  right_turn_counter = 0 ;
  tJunctionCount = 0 ;
  

}

void goFrom2To4(){
  Serial.println("Starting path 2 to 4");
  while (pathComplete == false) {
    if (junctionDetected == true) {
      junction_checker();
      if (tJunctionCount == 3) {
        pathComplete = true ;
      }

      else if (tJunctionCount == 1 || tJunctionCount == 2) {
        right_turn() ;
      }

      junctionDetected = false;
    }  
    else {
      course_correction();
    }
  }
  tJunctionCount = 0;
}

/*void goFrom2To5(){

}

void goFrom2To8(){

}

void goFrom2To7(){

}

void goFrom3To4(){

}

void goFrom3To5(){

}

void goFrom3To8(){

}

void goFrom3To7(){

}

void goFrom2To3(){

}

void goFrom7To1(){

}*/