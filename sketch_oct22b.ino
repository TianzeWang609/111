#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(3);  // Motor on M3 (left motor)
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);  // Motor on M4 (right motor)

// Line sensor pins
int leftOuterSensorPin = 3;   // Left outer line sensor connected to pin 2
int rightOuterSensorPin = 2;  // Right outer line sensor connected to pin 3
int leftInnerSensorPin = 6;   // Left inner line sensor connected to pin 6
int rightInnerSensorPin = 5;  // Right inner line sensor connected to pin 5

// Button pins
int startStopButtonPin = 7;   // First button for start/stop
int forwardButtonPin = 8;     // Second button for moving forward 5 seconds

// System states
int systemRunning = 0;
bool isTurningRight = false;  
bool isTurningLeft = false;   
int tJunctionCount = 0;       
bool crossChecker = false;    

void setup() {
  Serial.begin(9600);         
  Serial.println("Adafruit Motorshield v2 - Dual Line Sensor Controlled Motors!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set up line sensor pins
  pinMode(leftOuterSensorPin, INPUT);
  pinMode(rightOuterSensorPin, INPUT);
  pinMode(leftInnerSensorPin, INPUT);
  pinMode(rightInnerSensorPin, INPUT);

  // Set up button pins
  pinMode(startStopButtonPin, INPUT);
  pinMode(forwardButtonPin, INPUT);
  
  // Set initial motor speeds
  myMotor1->setSpeed(150);  
  myMotor2->setSpeed(150);  
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
  if (systemRunning) {
    goFrom1To2();  // Example: From point 1 to point 2
  }
}

// Function to go from point 1 to point 2
void goFrom1To2() {
  Serial.println("Starting path 1 to 2");
  moveForward();
  ignoreCrossJunction();
  detectAndTurnAtTJunction('L');
  moveForward();
  ignoreCrossJunction();
  detectAndTurnAtTJunction('R');
  ignoreCrossJunction();
  moveForward();
  ignoreNextJunction();
  moveForward();
  detectAndTurnAtCorner('R');
  moveForward();
  detectAndTurnAtCorner('R');
  moveForward();
  
  Serial.println("Reached point 2");
}
// Function to go from point 2 to point 3
void goFrom2To3() {
  Serial.println("Starting path 2 to 3");
  movebackward();
  detectAndTurnAtTJunction('R');
  moveForward();
  detectAndTurnAtTJunction('R')
  moveForward();
  detectAndTurnAtTJunction('R')
  moveForward();
  Serial.println("Reached point 3");
}

// Function to go from point 2 to point 4
void goFrom2To4() {
  Serial.println("Starting path 2 to 4");
  movebackward();
  detectAndTurnAtTJunction('L');
  moveForward();
  detectAndTurnAtTJunction('R');
  moveForward();
  moveUntilCrossJunction();
  Serial.println("Reached point 4");
}

// Function to go from point 3 to point 4
void goFrom3To4() {
  Serial.println("Starting path 3 to 4");
  movebackward();
  detectAndTurnAtTJunction('R');
  moveForward();
  detectAndTurnAtTJunction('L');
  moveForward();
  ignoreNextJunction();
  detectAndTurnAtTJunction('L');
  moveForward();
  moveUntilCrossJunction()
  Serial.println("Reached point 4");
}

// Function to go from point 2 to point 5
void goFrom2To5() {
  Serial.println("Starting path 2 to 5");
  movebackward();
  detectAndTurnAtTJunction('L');
  moveForward();
  detectAndTurnAtTJunction('L');
  moveForward();
  moveUntilCrossJunction()
  Serial.println("Reached point 5");
}

// Function to go from point 3 to point 5
void goFrom3To5() {
  Serial.println("Starting path 3 to 5");

  // Move forward and ignore the first cross-junction
  moveForward();
  ignoreCrossJunction();
  
  // Continue forward until reaching point 5
  moveForward();
  Serial.println("Reached point 5");
}

// Function to go from point 6 to point 2
void goFrom6To2() {
  Serial.println("Starting path 6 to 2");

  // Move forward and ignore the first cross-junction
  moveForward();
  ignoreCrossJunction();

  // Move forward, reach the T-junction and turn right
  moveForward();
  detectAndTurnAtTJunction('R');

  // Continue forward until reaching point 2
  moveForward();
  Serial.println("Reached point 2");
}

// Function to go from point 7 to point 2
void goFrom7To2() {
  Serial.println("Starting path 7 to 2");

  // Move forward, ignore the first cross-junction
  moveForward();
  ignoreCrossJunction();

  // Move forward, reach the T-junction and turn right
  moveForward();
  detectAndTurnAtTJunction('R');

  // Continue forward until reaching point 2
  moveForward();
  Serial.println("Reached point 2");
}

// Function to go from point 8 to point 2
void goFrom8To2() {
  Serial.println("Starting path 8 to 2");

  // Move forward and ignore the first cross-junction
  moveForward();
  ignoreCrossJunction();
  
  // Move forward until reaching point 2
  moveForward();
  Serial.println("Reached point 2");
}

// Function to detect and turn at a T-junction (Left or Right)
void Movebackward() {
  myMotor1->run(BACKWARD);
  myMotor2->run(BACKWARD);
  delay(500);  // Adjust delay for the distance you want to move
  Serial.println("Moving BACKWARD...");
}
void detectAndTurnAtTJunction(char direction) {
  while (true) {
    int leftOuterVal = digitalRead(leftOuterSensorPin);
    int rightOuterVal = digitalRead(rightOuterSensorPin);
    int leftInnerVal = digitalRead(leftInnerSensorPin);
    int rightInnerVal = digitalRead(rightInnerSensorPin);
    
    if (leftOuterVal == HIGH && rightOuterVal == HIGH && leftInnerVal == LOW && rightInnerVal == LOW) {
      if (direction == 'L') {
        executeLeftTurn();
      } else if (direction == 'R') {
        executeRightTurn();
      }
      break;
    }
  }
}

// Function to ignore cross junctions
void ignoreCrossJunction() {
  while (true) {
    int leftOuterVal = digitalRead(leftOuterSensorPin);
    int rightOuterVal = digitalRead(rightOuterSensorPin);
    int leftInnerVal = digitalRead(leftInnerSensorPin);
    int rightInnerVal = digitalRead(rightInnerSensorPin);
    
    if (leftOuterVal == HIGH && rightOuterVal == HIGH) {
      // Cross-junction detected, move straight through
      moveForward();
      break;
    }
  }
}

// Function to detect and turn at a corner (Left or Right)
void detectAndTurnAtCorner(char direction) {
  while (true) {
    int leftOuterVal = digitalRead(leftOuterSensorPin);
    int rightOuterVal = digitalRead(rightOuterSensorPin);
    int leftInnerVal = digitalRead(leftInnerSensorPin);
    int rightInnerVal = digitalRead(rightInnerSensorPin);
    
    if (leftOuterVal == HIGH || rightOuterVal == HIGH) {
      if (direction == 'R') {
        executeRightTurn();
      } else if (direction == 'L') {
        executeLeftTurn();
      }
      break;
    }
  }
}

// Function to move forward
void moveForward() {
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  delay(500);  // Adjust delay for the distance you want to move
  Serial.println("Moving forward...");
}

// Function to execute a 90-degree right turn
void executeRightTurn() {
  myMotor1->setSpeed(150);  
  myMotor2->setSpeed(150);  
  myMotor1->run(BACKWARD);  
  myMotor2->run(FORWARD);   
  delay(1000);  
  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);  
  Serial.println("Executed right turn");
}

// Function to execute a 90-degree left turn
void executeLeftTurn() {
  myMotor1->setSpeed(150);  
  myMotor2->setSpeed(150);  
  myMotor1->run(FORWARD);   
  myMotor2->run(BACKWARD);  
  delay(1000);  
  myMotor1->run(RELEASE);  
  myMotor2->run(RELEASE);  
  Serial.println("Executed left turn");
}
// Function to move forward until cross-junction is detected, then stop
void moveUntilCrossJunction() {
  Serial.println("Moving forward until cross-junction...");
  
  while (true) {
    int leftOuterVal = digitalRead(leftOuterSensorPin);
    int rightOuterVal = digitalRead(rightOuterSensorPin);
    int leftInnerVal = digitalRead(leftInnerSensorPin);
    int rightInnerVal = digitalRead(rightInnerSensorPin);
    
    // Check if all sensors detect a cross-junction (all HIGH)
    if (leftOuterVal == HIGH && rightOuterVal == HIGH) {
      Serial.println("Cross-junction detected: Stopping.");
      myMotor1->run(RELEASE);  // Stop both motors
      myMotor2->run(RELEASE);
      break;  // Exit the loop and stop
    }
    
    // Continue moving forward if not at a cross-junction
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    delay(50);  // Small delay to avoid rapid looping
  }
}

// Function to ignore the next T-junction and continue forward
void ignoreNextJunction() {
  Serial.println("Ignoring the next T-junction...");

  while (true) {
    int leftOuterVal = digitalRead(leftOuterSensorPin);
    int rightOuterVal = digitalRead(rightOuterSensorPin);
    int leftInnerVal = digitalRead(leftInnerSensorPin);
    int rightInnerVal = digitalRead(rightInnerSensorPin);

    // Check if we are at a T-junction (outer sensors HIGH, inner sensors LOW)
    if (leftOuterVal == HIGH || rightOuterVal == HIGH) {
      Serial.println("T-junction detected: Ignoring and continuing forward.");
      // Continue moving forward, don't turn
      myMotor1->run(FORWARD);
      myMotor2->run(FORWARD);
      delay(1000);  // Move forward for a bit to "ignore" the junction
      break;  // Exit the loop after ignoring the T-junction
    }

    // Continue moving forward
    myMotor1->run(FORWARD);
    myMotor2->run(FORWARD);
    delay(50);  // Small delay to avoid rapid looping
  }
}
