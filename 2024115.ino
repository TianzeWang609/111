#include <Adafruit_MotorShield.h>
#include <Servo.h>  // Include Servo library
#include <Wire.h>   // Include Wire library for I2C
#include <DFRobot_VL53L0X.h>  // Include ToF sensor library

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
Adafruit_DCMotor *left_motor = AFMS.getMotor(4);   // Motor on port 4 (left motor)
Servo myservo;  // Create servo object
DFRobot_VL53L0X sensor;  // Create ToF sensor object

// Line sensor pins
int leftSensorPin = 11;
int rightSensorPin = 10;
int frontrightSensorPin = 9;
int frontleftSensorPin = 12;

// Button, LED, Magnetic Sensor, Servo, and ToF Sensor pins
int startStopButtonPin = 5; // Start/stop button
int ledPin = 2;             // LED for flashing indicator
int magneticSensorPin = 8;  // Magnetic sensor pin
int servoPin = 6;           // Servo control pin
int tofSensorPin = 13;      // Time of Flight sensor (I2C)

// System state variables
bool systemRunning = false;     // Tracks if the system is running
int lastButtonState = HIGH;     // Stores the previous button state for edge detection
bool from1to2Complete = false;  // Indicates completion of goFrom1to2
bool boxCollected = false;      // Tracks if the box has been collected
bool reachedDestination = false; // Tracks if destination (4 or 8) is reached

// LED flashing timing variables
unsigned long previousMillis = 0;
const long ledInterval = 200;  // LED blink interval in milliseconds

// Sensor variables
int leftVal = 0;
int rightVal = 0;
int frontrightVal = 0;
int frontleftVal = 0;
int tJunctionCount = 0;
int right_turn_counter = 0;
int left_turn_counter = 0;
bool tJunctionDetected = false;

// Function prototypes
void course_correction();
void right_turn();
void left_turn();
void junction_checker();
void goFrom1to2();
void goFrom2to4();
void goFrom4to2();
void goFrom8to2();
void goFrom2to8();
void resetState(); // Reset all state variables
void checkToFSensorAndCollectBox(); // Check ToF sensor and collect box if within range

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - Dual Line Sensor Controlled Motors!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Initialize pins
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(frontrightSensorPin, INPUT);
  pinMode(frontleftSensorPin, INPUT);
  pinMode(startStopButtonPin, INPUT_PULLUP); // Use internal pull-up for button
  pinMode(ledPin, OUTPUT);
  pinMode(magneticSensorPin, INPUT);         // Magnetic sensor input

  // Initialize servo and set it to vertical position
  myservo.attach(servoPin);
  myservo.write(0);  // Start with servo at 0 degrees (vertical position)

  // Initialize Time of Flight sensor
  Wire.begin();
  sensor.begin(0x50); // Initialize the sensor at address 0x50
  sensor.setMode(sensor.eContinuous, sensor.eHigh); // Continuous and high-precision mode
  sensor.start(); // Start the sensor

  // Set initial motor speeds
  right_motor->setSpeed(158);  // Max speed for right motor
  left_motor->setSpeed(150);   // Max speed for left motor
}

void loop() {
  // Read the button state
  int buttonState = digitalRead(startStopButtonPin);

  // Check if the button state changed from HIGH to LOW
  if (buttonState == LOW && lastButtonState == HIGH) {
    systemRunning = !systemRunning;  // Toggle system running state
    Serial.println(systemRunning ? "System Started" : "System Stopped");

    if (systemRunning) {
      resetState();  // Reset all counters and state variables on start
      Serial.println("All states reset");
    }
  }

  // Update last button state
  lastButtonState = buttonState;

  // LED flashing indicator when system is running, using millis() instead of delay()
  if (systemRunning) {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= ledInterval) {
      previousMillis = currentMillis;
      digitalWrite(ledPin, !digitalRead(ledPin)); // Toggle LED state
    }

    // Start sequence: goFrom1to2, then check magnetic sensor
    if (!from1to2Complete) {
      goFrom1to2();
    } else if (from1to2Complete && !boxCollected) {
      checkToFSensorAndCollectBox();  // Check ToF sensor and collect box if within range
    } else if (boxCollected && !reachedDestination) {
      // Decide destination based on magnetic sensor status
      int magneticSensorValue = digitalRead(magneticSensorPin);
      if (magneticSensorValue == HIGH) {
        goFrom2to8();
      } else {
        goFrom2to4();
      }
    }
  } else {
    // If system is stopped, turn off motors, LED, and reset servo
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
    digitalWrite(ledPin, LOW); // Ensure LED is off when stopped
  }

  // After reaching destination, rotate servo back to release the box
  if (reachedDestination) {
    myservo.write(0);  // Rotate servo back to 0 degrees to release the box
    delay(1000);       // Wait for servo to reach position
    Serial.println("Box released by rotating servo back to 0 degrees");
    reachedDestination = false; // Reset after releasing the box
  }
}

void checkToFSensorAndCollectBox() {
  // Rotate servo to parallel position with ground
  myservo.write(90);  // Rotate servo to 90 degrees
  delay(500);         // Allow time for servo to reach position

  // Check distance with Time of Flight sensor
  int distance = sensor.getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < 100) {  // If an object is detected within 100mm
    Serial.println("Object detected within 100mm, collecting box...");
    myservo.write(180);  // Rotate servo to 180 degrees to collect the box
    delay(1000);         // Wait for servo to reach position
    boxCollected = true;
  }
}

void goFrom1to2() {
  static bool initialMoveDone = false;
  static bool leftTurnDone = false;
  static bool rightTurnDone = false;

  if (!initialMoveDone) {
    right_motor->setSpeed(200);
    left_motor->setSpeed(200);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    delay(2000);
    initialMoveDone = true;
    Serial.println("Initial forward move complete");
  }
  
  junction_checker();
  course_correction();

  if (tJunctionCount == 2 && !leftTurnDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    left_turn();
    leftTurnDone = true;
    tJunctionDetected = false;
    Serial.println("Left turn complete");
  }

  if (leftTurnDone) {
    course_correction();
  }

  if (tJunctionCount == 3 && !rightTurnDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone = true;
    tJunctionDetected = false;
    Serial.println("Right turn complete at second junction");
    tJunctionCount = 0; // Reset junction count
    from1to2Complete = true; // Mark goFrom1to2 as complete

    // Rotate the servo to parallel to ground and start ToF detection
    myservo.write(90);  // Parallel to ground
    delay(500);
    checkToFSensorAndCollectBox();  // Start ToF sensor check after final turn
  }
}

void goFrom2to4() {
  static bool reversedToFirstJunction = false;
  static bool firstLeftTurnDone = false; 
  static bool firstRightTurnDone = false; 
  static bool finalStopDone = false;  

  // Step 1: Reverse until reaching the first T-junction
  if (!reversedToFirstJunction && !finalStopDone) {
    right_motor->setSpeed(158);
    left_motor->setSpeed(145);
    right_motor->run(FORWARD); 
    left_motor->run(FORWARD);

    junction_checker();

    if (tJunctionCount == 1) { 
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      reversedToFirstJunction = true;
      tJunctionDetected = false;  
      Serial.println("Reversed to first T-junction");
    }
      }

  // Step 2: Custom left turn at first T-junction
  if (reversedToFirstJunction && !firstLeftTurnDone && !finalStopDone) {
    left_motor->setSpeed(255); // Increase speed for sharper turn
    right_motor->setSpeed(60);  
    right_motor->run(FORWARD);
    left_motor->run(BACKWARD);
    delay(2700);  // Increased delay for full turn, adjust as needed
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    
    firstLeftTurnDone = true;
    tJunctionDetected = false; 
    Serial.println("Left turn at first T-junction complete");
  }

  // Step 3: Move forward to next T-junction and custom right turn
  if (firstLeftTurnDone && !firstRightTurnDone && !finalStopDone) {
    course_correction(); 
    junction_checker();

    if (tJunctionCount == 2) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      left_motor->setSpeed(70);  
      right_motor->setSpeed(255); 
      right_motor->run(BACKWARD);
      left_motor->run(FORWARD);
      delay(2700);  // Increased delay for full turn, adjust as needed
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      firstRightTurnDone = true;
      tJunctionDetected = false; 
      Serial.println("Right turn at second T-junction complete");
    }
  }

  // Step 4: Move forward and stop at third T-junction
  if (firstRightTurnDone && !finalStopDone) {
    course_correction();
    junction_checker();

    if (tJunctionCount == 3) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      finalStopDone = true;
      Serial.println("Stopped at final T-junction");
    }
  reachedDestination = true; // Mark as destination reached
}


void resetState() {
  // Reset all state variables to their initial values
  tJunctionCount = 0;
  right_turn_counter = 0;
  left_turn_counter = 0;
  from1to2Complete = false;
  boxCollected = false;
  reachedDestination = false;
  myservo.write(0);  // Reset servo to 0 degrees
}

void course_correction() {
  int baseSpeed = 180;
  int adjustment = 30;

  if (leftVal == HIGH && rightVal == HIGH) {
    right_motor->setSpeed(baseSpeed);
    left_motor->setSpeed(baseSpeed);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    Serial.println("Driving forward");
  } 
  else if (leftVal == LOW && rightVal == HIGH) {
    right_motor->setSpeed(baseSpeed + adjustment);
    left_motor->setSpeed(baseSpeed - adjustment);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    Serial.println("Adjusting right");
  } 
  else if (leftVal == HIGH && rightVal == LOW) {
    right_motor->setSpeed(baseSpeed - adjustment);
    left_motor->setSpeed(baseSpeed + adjustment);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    Serial.println("Adjusting left");
  } 
  else {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    Serial.println("Stopping or off track");
  }
}

void right_turn() {
  left_motor->setSpeed(50);
  right_motor->setSpeed(235);
  right_motor->run(BACKWARD);
  left_motor->run(FORWARD);
  Serial.println("Turning right");
  delay(2800);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void left_turn() {
  left_motor->setSpeed(235);
  right_motor->setSpeed(50);
  right_motor->run(FORWARD);
  left_motor->run(BACKWARD);
  Serial.println("Turning left");
  delay(2800);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void junction_checker() {
  leftVal = digitalRead(leftSensorPin);
  rightVal = digitalRead(rightSensorPin);
  frontrightVal = digitalRead(frontrightSensorPin);
  frontleftVal = digitalRead(frontleftSensorPin);

  if (frontrightVal == HIGH && frontleftVal == HIGH && !tJunctionDetected) {
    tJunctionCount++;
    tJunctionDetected = true;
    Serial.println("T Junction detected");
  } 
  else if (frontrightVal == LOW || frontleftVal == LOW) {
    tJunctionDetected = false;
  }
}

void goFrom4to2() {
  static bool reversedToFirstRightTurn = false;  // Tracks if reversing to the first right-turn corner is complete
  static bool firstRightTurnDone = false;        // Tracks if the first right turn is complete
  static bool firstLeftTurnDone = false;         // Tracks if the first left turn is complete

  // Step 1: Reverse until reaching the first right-turn corner
  if (!reversedToFirstRightTurn) {
    right_motor->setSpeed(158);  // Set speed for reverse, same as in goFrom2to4
    left_motor->setSpeed(145);
    right_motor->run(FORWARD);   // Reverse by running motors forward
    left_motor->run(FORWARD);

    // Check for right-turn corner
    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      reversedToFirstRightTurn = true;
      delay(500);  // Small delay to stabilize before turning
      Serial.println("Reversed to first right-turn corner");
    }
  }

  // Step 2: Perform a right turn at the first right-turn corner
  if (reversedToFirstRightTurn && !firstRightTurnDone) {
    // Custom right turn with specific speed and delay
    left_motor->setSpeed(255);  // Higher speed for sharper turn
    right_motor->setSpeed(60);
    right_motor->run(BACKWARD);
    left_motor->run(FORWARD);
    delay(2700);  // Adjusted delay for full turn
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);

    firstRightTurnDone = true;
    Serial.println("Right turn at first right-turn corner complete");
  }

  // Step 3: Move forward to the first left-turn corner and turn left
  if (firstRightTurnDone && !firstLeftTurnDone) {
    course_correction(); // Move forward using line-following correction

    // Check for left-turn corner
    if (detectLeftTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      delay(500);  // Small delay to stabilize before turning

      // Perform a left turn
      left_motor->setSpeed(255);  // Higher speed for sharper turn
      right_motor->setSpeed(60);
      right_motor->run(FORWARD);
      left_motor->run(BACKWARD);
      delay(2700);  // Adjusted delay for full turn
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      firstLeftTurnDone = true;
      Serial.println("Left turn at first left-turn corner complete, stopping now");
    }
  }

  // Step 4: Stop after completing the left turn
  if (firstLeftTurnDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    Serial.println("Stopped after completing route to position 2");
  }
}

void goFrom2to8() {
  static bool reversedToFirstJunction = false;
  static bool firstLeftTurnDone = false;
  static bool secondLeftTurnDone = false;
  static int ignoredTJunctions = 0;
  static bool thirdRightTurnDone = false;
  static bool rightTurnCornerIgnored = false;
  static int rightTurnCornerCount = 0;
  static bool finalStopDone = false;

  // Step 1: Reverse until reaching the first T-junction
  if (!reversedToFirstJunction && !finalStopDone) {
    right_motor->setSpeed(158);
    left_motor->setSpeed(145);
    right_motor->run(FORWARD);
    left_motor->run(FORWARD);

    junction_checker();

    if (tJunctionCount == 1) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      reversedToFirstJunction = true;
      Serial.println("Reversed to first T-junction");
    }
  }

  // Step 2: Perform a right turn at the first right-turn corner
  if (reversedToFirstJunction && !firstLeftTurnDone) {
    left_motor->setSpeed(255);
    right_motor->setSpeed(60);
    right_motor->run(BACKWARD);
    left_motor->run(FORWARD);
    delay(2700);
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    firstLeftTurnDone = true;
    Serial.println("Right turn at first corner complete");
  }

  // Step 3: Ignore the next four T-junctions
  if (firstLeftTurnDone && ignoredTJunctions < 4) {
    course_correction();
    junction_checker();
    if (tJunctionDetected) {
      ignoredTJunctions++;
      Serial.println("Ignoring T-junction");
      tJunctionDetected = false;
    }
  }

  // Step 4: Right turn at the fifth T-junction
  if (ignoredTJunctions == 4 && !thirdRightTurnDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    thirdRightTurnDone = true;
    Serial.println("Right turn at fifth T-junction complete");
  }

  // Step 5: Ignore the first right-turn corner after the fifth T-junction
  if (thirdRightTurnDone && !rightTurnCornerIgnored) {
    course_correction();
    if (detectRightTurnCorner()) {
      Serial.println("Ignoring first right-turn corner after fifth T-junction");
      rightTurnCornerIgnored = true;
    }
  }

  // Step 6: Right turns at subsequent right-turn corners
  if (rightTurnCornerIgnored && rightTurnCornerCount < 4) {
    course_correction();
    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      right_turn();
      rightTurnCornerCount++;
      Serial.print("Right turn at right-turn corner ");
      Serial.println(rightTurnCornerCount);
    }
  }

  // Step 7: Stop after the final right turn
  if (rightTurnCornerCount == 4) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    finalStopDone = true;
    Serial.println("Stopped after final right turn");
  }
}
void goFrom8to2() {
  static bool reversedToFirstRightTurn = false; // Tracks if reversing to the first right-turn corner is complete
  static bool firstRightTurnDone = false;       // Tracks if the first right turn is complete
  static int ignoredTJunctions = 0;             // Tracks the number of T-junctions ignored
  static bool secondRightTurnDone = false;      // Tracks if the right turn at the fifth T-junction is complete
  static int ignoredRightTurnCorners = 0;       // Tracks ignored right-turn corners after fifth T-junction
  static int rightTurnCornerCount = 0;          // Tracks the number of right turns completed after the fifth T-junction
  static bool finalStopDone = false;            // Tracks if the final stop has been reached

  // Step 1: Reverse until reaching the first right-turn corner
  if (!reversedToFirstRightTurn && !finalStopDone) {
    right_motor->setSpeed(158);  // Set speed for reverse, same as in goFrom2to4
    left_motor->setSpeed(145);
    right_motor->run(FORWARD);   // Reverse by running motors forward
    left_motor->run(FORWARD);

    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      reversedToFirstRightTurn = true;
      delay(500);  // Small delay to stabilize before turning
      Serial.println("Reversed to first right-turn corner");
    }
  }

  // Step 2: Perform a right turn at the first right-turn corner
  if (reversedToFirstRightTurn && !firstRightTurnDone && !finalStopDone) {
    // Custom right turn with specific speed and delay
    left_motor->setSpeed(255);  // Higher speed for sharper turn
    right_motor->setSpeed(60);
    right_motor->run(BACKWARD);
    left_motor->run(FORWARD);
    delay(2700);  // Adjusted delay for full turn
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);

    firstRightTurnDone = true;
    Serial.println("Right turn at first right-turn corner complete");
  }

  // Step 3: Move forward, ignoring the next four T-junctions and all corners
  if (firstRightTurnDone && ignoredTJunctions < 4 && !finalStopDone) {
    course_correction(); // Move forward using line-following correction
    junction_checker();

    // Ignore T-junctions
    if (tJunctionDetected && tJunctionCount <= 4) {
      ignoredTJunctions++;
      tJunctionDetected = false; // Reset T-junction detection for each ignored junction
      Serial.println("Ignoring T-junction");
    }
  }

  // Step 4: Right turn at the fifth T-junction
  if (ignoredTJunctions == 4 && !secondRightTurnDone && !finalStopDone) {
    course_correction();
    junction_checker();

    if (tJunctionDetected && tJunctionCount == 5) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      // Custom right turn
      left_motor->setSpeed(255);  // Higher speed for sharper turn
      right_motor->setSpeed(60);
      right_motor->run(BACKWARD);
      left_motor->run(FORWARD);
      delay(2700);  // Adjusted delay for full turn
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      secondRightTurnDone = true;
      ignoredTJunctions = 0;  // Reset ignored T-junction count
      tJunctionDetected = false;
      Serial.println("Right turn at fifth T-junction complete");
    }
  }

  // Step 5: Ignore the first right-turn corner after the fifth T-junction
  if (secondRightTurnDone && ignoredRightTurnCorners < 1 && !finalStopDone) {
    course_correction();

    if (detectRightTurnCorner()) {
      ignoredRightTurnCorners++;
      Serial.println("Ignoring first right-turn corner after fifth T-junction");
    }
  }

  // Step 6: Right turns at the second, third, fourth, and fifth right-turn corners
  if (ignoredRightTurnCorners == 1 && rightTurnCornerCount < 4 && !finalStopDone) {
    course_correction();

    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      // Perform a right turn
      left_motor->setSpeed(255);  // Higher speed for sharper turn
      right_motor->setSpeed(60);
      right_motor->run(BACKWARD);
      left_motor->run(FORWARD);
      delay(2700);  // Adjusted delay for full turn
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      rightTurnCornerCount++;
      Serial.print("Right turn at right-turn corner ");
      Serial.println(rightTurnCornerCount + 1); // +1 because corner count starts from 0
      delay(500);  // Small delay to stabilize after turning
    }
  }

  // Step 7: Stop after completing the fifth right turn
  if (rightTurnCornerCount == 4 && !finalStopDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    finalStopDone = true;
    Serial.println("Stopped after final right turn");
  }
}

// Helper function to detect right-turn corners
bool detectRightTurnCorner() {
  return (frontrightVal == HIGH && frontleftVal == LOW);
}
