#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *myMotor1 = AFMS.getMotor(3);  // Motor on M3 (left motor)
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(4);  // Motor on M4 (right motor)

// Line sensor pins
int leftSensorPin = 3;
int rightSensorPin = 2;
int frontSensorPin = 6;
int backSensorPin = 5;

// Button pins
int startStopButtonPin = 7;
int forwardButtonPin = 8;

// Variable to track system state (1 = running, 0 = stopped)
int systemRunning = 0;
bool isTurningRight = false;
bool isTurningLeft = false;
int tJunctionCount = 0;  // Counter for T-junctions

// Position tracking
int posX = 0;    // X-coordinate
int posY = 0;    // Y-coordinate
int direction = 0; // Initial direction (0 = North, 90 = East, 180 = South, 270 = West)

// Mapping system
struct Node {
  int x;
  int y;
  String junctionType;  // "T-junction", "Cross-junction", "Left turn", "Right turn"
};

Node path[100];  // Array to store the path (maximum 100 nodes)
int pathIndex = 0;  // Index to track the current path

void setup() {
  Serial.begin(9600);
  
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

  // If system is running and not turning
  if (systemRunning && !isTurningRight && !isTurningLeft) {
    if (digitalRead(forwardButtonPin) == HIGH) {
      moveForward();  // Move forward for 5 seconds
    }

    int leftVal = digitalRead(leftSensorPin);
    int rightVal = digitalRead(rightSensorPin);
    int frontVal = digitalRead(frontSensorPin);
    
    // Detect cross-junction
    if (leftVal == HIGH && rightVal == HIGH && frontVal == HIGH) {
      recordNode("Cross-junction");
      moveForward();
    }
    // Detect T-junction
    else if (leftVal == HIGH && rightVal == HIGH && frontVal == LOW) {
      tJunctionCount++;
      if (tJunctionCount == 1) {
        recordNode("T-junction - Left Turn");
        executeLeftTurn();
      } else if (tJunctionCount == 2) {
        recordNode("T-junction - Right Turn");
        executeRightTurn();
      }
    }
    // Detect right turn
    else if (leftVal == LOW && frontVal == LOW && rightVal == HIGH) {
      recordNode("Right Turn");
      executeRightTurn();
    }
    // Detect left turn
    else if (leftVal == HIGH && frontVal == LOW && rightVal == LOW) {
      recordNode("Left Turn");
      executeLeftTurn();
    }

    delay(50);  // Avoid rapid looping
  }
}

// Function to record a node in the path
void recordNode(String junctionType) {
  path[pathIndex].x = posX;
  path[pathIndex].y = posY;
  path[pathIndex].junctionType = junctionType;
  pathIndex++;

  Serial.print("Recorded Node: ");
  Serial.print(junctionType);
  Serial.print(" at (");
  Serial.print(posX);
  Serial.print(", ");
  Serial.print(posY);
  Serial.println(")");
}

// Function to update position when moving forward
void updatePosition() {
  if (direction == 0) {
    posY++;  // Moving North
  } else if (direction == 90) {
    posX++;  // Moving East
  } else if (direction == 180) {
    posY--;  // Moving South
  } else if (direction == 270) {
    posX--;  // Moving West
  }
}

// Function to execute a 90-degree right turn
void executeRightTurn() {
  direction = (direction + 90) % 360;  // Update direction
  updatePosition();  // Update position after the turn
  Serial.println("Executing right turn...");
  // Motor control logic for turning right
  myMotor1->run(BACKWARD);
  myMotor2->run(FORWARD);
  delay(1000);  // Adjust this delay for your specific turning needs
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  isTurningRight = false;
}

// Function to execute a 90-degree left turn
void executeLeftTurn() {
  direction = (direction + 270) % 360;  // Update direction (turning left is subtracting 90)
  updatePosition();
  Serial.println("Executing left turn...");
  // Motor control logic for turning left
  myMotor1->run(FORWARD);
  myMotor2->run(BACKWARD);
  delay(1000);  // Adjust this delay for your specific turning needs
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
  isTurningLeft = false;
}

// Function to move forward
void moveForward() {
  updatePosition(); 
  Serial.println("Moving forward...");
  myMotor1->run(FORWARD);
  myMotor2->run(FORWARD);
  delay(1000);
  myMotor1->run(RELEASE);
  myMotor2->run(RELEASE);
}
