#include <Adafruit_MotorShield.h>
#include <Servo.h>  // Include Servo library
#include <Wire.h>   // Include Wire library for I2C
#include "DFRobot_VL53L0X.h"
#include <Servo.h>

// create TOF sensor object
DFRobot_VL53L0X TOF;

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
Adafruit_DCMotor *left_motor = AFMS.getMotor(4);   // Motor on port 4 (left motor)

// Servo objects
Servo gate_servo;  // Create gate servo
Servo ramp_servo;  // create ramp servo

// Line sensor pins
int leftSensorPin = 11;
int rightSensorPin = 10;
int frontrightSensorPin = 9;
int frontleftSensorPin = 12;
int magnetsensor = 8;
// Button and LED pins
int startStopButtonPin = 5; // Start/stop button
int ledPin = 2;             // LED for flashing indicator
int magnetLED = 4;
// System state variables
bool systemRunning = false;  // Tracks if the system is running
int lastButtonState = HIGH;  // Stores the previous button state for edge detection
bool from1to2Complete = false; // Indicates completion of goFrom1to2

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

int onetotwocomplete = 0;
bool initialMoveDone = false;   // Tracks if initial move is complete
bool leftTurnDone = false;
bool rightTurnDone = false; 
bool rightTurnDone2 = false;
bool rightTurnDone3 = false;
bool rightTurnDone4 = false;    // Tracks if initial left turn is complete
bool tJunctionDetected = false; // Ensures each T-junction is only counted once

// Function prototypes
void course_correction();
void right_turn();
void left_turn();
void junction_checker();
void goFrom1to2();
void goFrom2to4();
void setGateServoUp();
void setGateServoHorizontal();
void setGateServoDown();
void search_and_find() ;

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - Dual Line Sensor Controlled Motors!");

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set up pins
  pinMode(leftSensorPin, INPUT);
  pinMode(rightSensorPin, INPUT);
  pinMode(frontrightSensorPin, INPUT);
  pinMode(frontleftSensorPin, INPUT);
  pinMode(startStopButtonPin, INPUT_PULLUP); // Use internal pull-up for button
  pinMode(ledPin, OUTPUT);
  pinMode(magnetLED,OUTPUT);
  pinMode(magnetsensor,INPUT);
  // Attach and initialize gate servo
  gate_servo.attach(6);
  gate_servo.write(0) ;

  ramp_servo.attach(7) ;
  ramp_servo.write(0) ;


  // Set initial motor speeds
  right_motor->setSpeed(158);  // Max speed for right motor
  left_motor->setSpeed(150);   // Max speed for left motor


    //join i2c bus (address optional for master)
  Wire.begin();
  //Set I2C sub-device address
  TOF.begin(0x50);
  //Set to Back-to-back mode and high precision mode
  TOF.setMode(TOF.eContinuous,TOF.eHigh);
  //Laser rangefinder begins to work
  TOF.start();
}

void loop() {
  // Read the button state
  int buttonState = digitalRead(startStopButtonPin);

  // Check if the button state changed from HIGH to LOW
  if (buttonState == LOW && lastButtonState == HIGH) {
    systemRunning = !systemRunning;  // Toggle system running state
    Serial.println(systemRunning ? "System Started" : "System Stopped");

    // Reset counters and state variables
    if (systemRunning) {
      tJunctionCount = 0;
      right_turn_counter = 0;
      left_turn_counter = 0;
      onetotwocomplete = 0;
      initialMoveDone = false;
      leftTurnDone = false;
      tJunctionDetected = false;
      from1to2Complete = false; // Reset completion flag for goFrom1to2
      Serial.println("Counters and state reset");
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
    //Perform the sequence of operations
    if (!from1to2Complete) {
    // Keep gate servo up before starting goFrom1to2
    setGateServoUp();
    goFrom1to2();
    from1to2Complete = true;  // Mark goFrom1to2 as complete
} else {
    // Set gate servo down after goFrom1to2 completes
    setGateServoDown();
    delay(1000);  // Wait for gate to fully move down

    // Check magnetic sensor to decide which function to execute
    int sensorValue = digitalRead(magnetsensor);  // Read the magnetic sensor

    if (sensorValue == HIGH) {
        // Magnet detected, execute goFrom2to8
        goFrom2to8();
        setGateServoUp();
        delay(1000);  // Wait for gate to fully move up
        goFrom8to2();
        Serial.println("Magnet detected, executing goFrom2to8");
    } else {
        // No magnet detected, execute goFrom2to4
        goFrom2to4();
        Serial.println("No magnet detected, executing goFrom2to4");
        setGateServoUp();
        delay(1000);  // Wait for gate to fully move up
        goFrom4to2();
        }
    // Stop the system after completing the sequence
    systemRunning = false;
  }
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
  right_motor->setSpeed(250);
  right_motor->run(BACKWARD);
  left_motor->run(FORWARD);
  Serial.println("Turning right");
  delay(2700);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}

void left_turn() {
  left_motor->setSpeed(235);
  right_motor->setSpeed(50);
  right_motor->run(FORWARD);
  left_motor->run(BACKWARD);
  Serial.println("Turning left");
  delay(2900);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}


void junction_checker() {
  if (frontrightVal == HIGH && frontleftVal == LOW) {
    delay(100);
  } else if (frontrightVal == LOW && frontleftVal == HIGH) {
    delay(100);
  } else {
    delay(100) ;
  }

  leftVal = digitalRead(leftSensorPin);
  rightVal = digitalRead(rightSensorPin);
  frontrightVal = digitalRead(frontrightSensorPin);
  frontleftVal = digitalRead(frontleftSensorPin);

  // 检查T形路口，仅当tJunctionDetected为false时进行计数
  if (frontrightVal == HIGH && frontleftVal == HIGH && !tJunctionDetected) {
    tJunctionCount++;
    tJunctionDetected = true;  // 避免重复检测
    Serial.println("T Junction detected");
  } 
  else if (frontrightVal == LOW || frontleftVal == LOW) {
    tJunctionDetected = false;  // 重置T形路口检测状态
  }
  if (frontrightVal == HIGH && frontleftVal == LOW) {
    right_turn_counter++ ;
    Serial.println("Right Junction detected");
  }
  if (frontrightVal == LOW && frontleftVal == HIGH ) {
    left_turn_counter++ ;
    Serial.println("left Junction detected");
  }
}

// Modified goFrom2to4 with specific left and right turn parameters
void goFrom2to4() {
  static bool reversedToFirstJunction = false;
  static bool firstLeftTurnDone = false; 
  static bool firstRightTurnDone = false; 
  static bool finalStopDone = false;  

  // Step 1: Reverse until reaching the first T-junction
  setGateServoDown();  // Ensure the servo is down before starting
  delay(1500);  // Delay to allow servo movement

  if (!reversedToFirstJunction && !finalStopDone) {
    right_motor->setSpeed(158);
    left_motor->setSpeed(145);
    right_motor->run(FORWARD); 
    left_motor->run(FORWARD);

    junction_checker();  // Check for T-junction

    if (tJunctionCount == 1) { 
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      reversedToFirstJunction = true;
      tJunctionDetected = false;  // Reset detection for the next junctions
      Serial.println("Reversed to first T-junction");
    }
  }

  // Step 2: Custom left turn at first T-junction
  if (reversedToFirstJunction && !firstLeftTurnDone && !finalStopDone) {
    left_motor->setSpeed(255);   // Increase speed for sharper turn
    right_motor->setSpeed(70);   // Slightly slower speed for controlled turn
    right_motor->run(FORWARD);
    left_motor->run(BACKWARD);
    delay(2700);  // Adjust delay for sharper turn
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    
    firstLeftTurnDone = true;
    tJunctionDetected = false;   // Reset detection for the next junctions
    Serial.println("Left turn at first T-junction complete");
  }

  // Step 3: Move forward to the next T-junction and custom right turn
  if (firstLeftTurnDone && !firstRightTurnDone && !finalStopDone) {
    course_correction(); 
    junction_checker();

    if (tJunctionCount == 2) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      // Perform a custom right turn
      left_motor->setSpeed(70);  
      right_motor->setSpeed(255); 
      right_motor->run(BACKWARD);
      left_motor->run(FORWARD);
      delay(2700);  // Adjust delay for sharper turn
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
      setGateServoUp();  // Move the servo up at the end
    }
  }
}



void goFrom2to8() {
  static bool reversedToFirstJunction = false;  // Tracks if reversing to the first T-junction is complete
  static bool firstLeftTurnDone = false;        // Tracks if the first left turn is complete
  static bool secondLeftTurnDone = false;       // Tracks if the second left turn is complete
  static bool thirdRightTurnDone = false;       // Tracks if the third right turn is complete
  static bool rightTurnCornerIgnored = false;   // Tracks if the right-turn corner is ignored
  static bool reachedFinalCorner = false;       // Tracks if the final corner has been reached for stopping
  unsigned long cornerStartTime = 0;            // Tracks the start time after detecting the final corner

  // Step 1: Reverse until reaching the first T-junction, then perform a custom left turn
  if (!reversedToFirstJunction) {
    right_motor->setSpeed(158);  // Set speed similar to goFrom2to4
    left_motor->setSpeed(145);
    right_motor->run(FORWARD);   // Reverse by running motors forward
    left_motor->run(FORWARD);
    tJunctionCount == 0;
    junction_checker();  // Check for T-junctions

    if (tJunctionCount == 1) { // Stop when the first T-junction is detected
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      
      // Custom left turn parameters as in goFrom2to4
      left_motor->setSpeed(255);  // Higher speed for sharper left turn
      right_motor->setSpeed(60);
      right_motor->run(FORWARD);
      left_motor->run(BACKWARD);
      delay(2700);                // Adjusted delay for full turn
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      reversedToFirstJunction = true;
      firstLeftTurnDone = true;
      tJunctionDetected = false;  // Reset T-junction detection
      Serial.println("Reversed to first T-junction and left turn complete");
    }
  }

  // Step 2: Go straight to the next T-junction and perform a left turn
  if (firstLeftTurnDone && !secondLeftTurnDone) {
    course_correction();
    junction_checker();

    if (tJunctionCount == 2) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);

      // Perform a standard left turn for this step
      left_turn();
      secondLeftTurnDone = true;
      tJunctionDetected = false;  // Reset T-junction detection
      Serial.println("Second left turn at T-junction complete");
    }
  }

  // Step 3: Ignore the next two T-junctions
  if (secondLeftTurnDone) {
    course_correction();
    junction_checker();

    if (tJunctionCount == 3 || tJunctionCount == 4) {
      course_correction();
      Serial.println("Ignoring T-junction");
      tJunctionDetected = false;  // Reset T-junction detection for ignored junctions
    }
  }

  // Step 4: Perform a right turn at the third T-junction after ignoring two
  if (!thirdRightTurnDone) {
    junction_checker();

    if (tJunctionCount == 7) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      right_turn();
      thirdRightTurnDone = true;
      tJunctionDetected = false;  // Reset T-junction detection after turn
      Serial.println("Right turn at third T-junction complete");
    }
  }

  // Step 5: Ignore the next right-turn corner
  if (thirdRightTurnDone) {
    course_correction();
    }
}

// Helper function to detect right-turn corners
bool detectRightTurnCorner() {
  return (frontrightVal == HIGH && frontleftVal == LOW);
}

// Helper function to detect regular corners
bool detectRegularCorner() {
  return (frontrightVal == LOW && frontleftVal == LOW);  // Modify based on your corner detection criteria
}

void goFrom4to2() {
  static bool initialStraightDone = false; 
  static bool firstRightTurnDone = false; 
  static int rightTurnCornerCount = 0; 
  static bool reachedEndOfCourse = false; 
  // Step 1: Initial straight movement until the second T-junction, then perform a right turn
  if (!initialStraightDone) {
    course_correction(); 
    junction_checker(); 

    if (tJunctionCount == 2) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      right_turn();
      firstRightTurnDone = true;
      initialStraightDone = true;
      tJunctionDetected = false; 
      Serial.println("Right turn at second T-junction complete");
    }
  }

  // Step 2: Move forward until reaching the first right-turn corner, then turn right
  if (firstRightTurnDone && rightTurnCornerCount == 0) {
    course_correction();
    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      right_turn();
      rightTurnCornerCount++;
      Serial.println("Right turn at first right-turn corner complete");
    }
  }

  // Step 3: Move forward to the second right-turn corner, then turn right
  if (rightTurnCornerCount == 1) {
    course_correction();
    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      right_turn();
      rightTurnCornerCount++;
      Serial.println("Right turn at second right-turn corner complete");
    }
  }

  // Step 4: Move forward to the third right-turn corner, then turn right
  if (rightTurnCornerCount == 2) {
    course_correction();
    if (detectRightTurnCorner()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      right_turn();
      rightTurnCornerCount++;
      Serial.println("Right turn at third right-turn corner complete");
    }
  }

  // Step 5: Continue forward until `course_correction` loses the white line, then stop
  if (rightTurnCornerCount == 3 && !reachedEndOfCourse) {
    course_correction();

    // Check if course_correction detects no white line
    if (!isOnTrack()) {
      right_motor->run(RELEASE);
      left_motor->run(RELEASE);
      reachedEndOfCourse = true;
      Serial.println("Stopped as course correction detects no white line");
    }
  }
}



// Helper function to check if on track (white line detected)
bool isOnTrack() {
  leftVal = digitalRead(leftSensorPin);
  rightVal = digitalRead(rightSensorPin);
  return (leftVal == HIGH || rightVal == HIGH); 
}
void goFrom1to2() {

  // 第一步：前进1秒
  if (!initialMoveDone) {
    right_motor->setSpeed(200);
    left_motor->setSpeed(200);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    delay(2000);
    initialMoveDone = true;
    Serial.println("Initial forward move complete");
  }
  
  // 检查 T形路口
  junction_checker();
  course_correction() ;

  // 第二步：到达第一个 T形路口后左转
  if (tJunctionCount == 2 && !leftTurnDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    left_turn();
    leftTurnDone = true;
    tJunctionDetected = false; // 重置T形路口检测状态
    Serial.println("Left turn complete");

  }

  // 第三步：执行左转后的 course_correction 直到第二个 T形路口，然后右转
//Processing Adafruit SSD130
  if (leftTurnDone == true) {
    course_correction(); // 在左转后继续前进
  }
  if (tJunctionCount == 3 && !rightTurnDone) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone = true;
    tJunctionDetected = false; // 重置T形路口检测状态
    //tJunctionAfterLeftTurn = tJunction; // 重置左转后计数，准备继续检测T形路口
    Serial.println("right turn complete second junction");
    tJunctionCount = 0 ; 
  }
 
  
  if ((right_turn_counter == 5) && !rightTurnDone2) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone2 = true;
    Serial.println("right turn complete forth junction");
  }
  if (rightTurnDone2 == true) {
    course_correction(); // 在左转后继续前进
  }

  if ((right_turn_counter == 6) && !rightTurnDone3) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone3 = true;
    Serial.println("right turn complete forth junction");
  }
  if (rightTurnDone3 == true) {
    course_correction(); // 在左转后继续前进
  }
  if ((right_turn_counter == 7) && !rightTurnDone4) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    left_motor->setSpeed(70);  
    right_motor->setSpeed(255); 
    right_motor->run(BACKWARD);
    left_motor->run(FORWARD);
    delay(2600);  // Increased delay for full turn, adjust as needed
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    rightTurnDone4 = true;
    Serial.println("right turn complete forth junction");
  }
  if (rightTurnDone4 == true) {
    course_correction(); 
    search_and_find() ;


  }
}

void search_and_find() {
  // Set the gate servo to the horizontal position
  setGateServoHorizontal();
  delay(1000);  // Wait for the gate to move to the horizontal position

  // Check the distance using the ToF sensor
  if (TOF.getDistance() <= 100) {
    // Stop the motors
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);

    // Lower the ramp
    ramp_servo.write(30);
    delay(1000);  // Wait for the ramp to lower

    // Set the gate servo to the down position
    setGateServoDown();
    delay(1000);  // Wait for the gate to move down

    // Reset the ramp position
    ramp_servo.write(0);

    // Check the magnetic sensor
    int sensorValue = digitalRead(magnetsensor);  // Read the magnetic sensor

    if (sensorValue == HIGH) {
      // Magnet detected, turn on the LED
      digitalWrite(magnetLED, HIGH);
      Serial.println("Magnet detected, LED turned ON");
    } else {
      // No magnet detected, turn off the LED
      digitalWrite(magnetLED, LOW);
      Serial.println("No magnet detected, LED turned OFF");
    }
  }
}


void setGateServoUp() {
  gate_servo.write(0);  // Set servo to 0 degrees
  Serial.println("Gate servo set to vertical up");
}

// Function to set gate_servo to horizontal position
void setGateServoHorizontal() {
  gate_servo.write(60);  // Set servo to 90 degrees
  Serial.println("Gate servo set to horizontal");
}

// Function to set gate_servo to vertical down position
void setGateServoDown() {
  gate_servo.write(160);  // Set servo to 180 degrees
  Serial.println("Gate servo set to vertical down");
}
void oneighty_turn() {
  left_motor->setSpeed(235);
  right_motor->setSpeed(235);
  right_motor->run(FORWARD);
  left_motor->run(BACKWARD);
  Serial.println("Turning left");
  delay(2900);
  right_motor->run(RELEASE);
  left_motor->run(RELEASE);
}
void goFrom8to2() {
  static bool turnAroundDone = false;
  static int leftTurnCounter = 0;
  static int tJunctionCounter = 0;
  static bool firstLeftTurnDone = false;
  static bool secondLeftTurnDone = false;
  static bool rightTurnDone = false;
  static bool finalCorrectionDone = false;

  // Step 1: Perform a 180-degree turn (U-turn)
  if (!turnAroundDone) {
    left_motor->setSpeed(255);    // Max speed for sharp turn
    right_motor->setSpeed(255);
    left_motor->run(BACKWARD);
    right_motor->run(FORWARD);
    delay(3000);                  // Adjust delay to complete 180-degree turn
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);

    turnAroundDone = true;
    Serial.println("180-degree turn completed");
  }

  // Step 2: Ignore the first two left-turn junctions
  if (turnAroundDone && !firstLeftTurnDone) {
    course_correction();   // Keep the robot on track
    junction_checker();    // Check for junctions

    if (leftTurnDetected()) {  // Detect left turn junction
      leftTurnCounter++;
      Serial.print("Left turn junction detected: ");
      Serial.println(leftTurnCounter);

      // Skip the first two left-turn junctions
      if (leftTurnCounter > 2) {
        // Stop at the third T-junction for the left turn
        left_motor->run(RELEASE);
        right_motor->run(RELEASE);
        
        left_motor->setSpeed(255); // Speed settings for left turn
        right_motor->setSpeed(70); 
        right_motor->run(FORWARD);
        left_motor->run(BACKWARD);
        delay(2500);  // Adjust delay as needed for full turn

        left_motor->run(RELEASE);
        right_motor->run(RELEASE);
        
        firstLeftTurnDone = true;
        Serial.println("Left turn at third T-junction completed");
      }
    }
  }

  // Step 3: Perform a left turn at the second left-turn junction after the first left turn
  if (firstLeftTurnDone && !secondLeftTurnDone) {
    course_correction();
    junction_checker();

    if (leftTurnDetected()) {  // Detect left turn junction
      left_motor->run(RELEASE);
      right_motor->run(RELEASE);

      left_motor->setSpeed(255);
      right_motor->setSpeed(70);
      right_motor->run(FORWARD);
      left_motor->run(BACKWARD);
      delay(2500);  // Adjust delay as needed for full turn

      left_motor->run(RELEASE);
      right_motor->run(RELEASE);

      secondLeftTurnDone = true;
      Serial.println("Second left turn after the first left turn completed");
    }
  }

  // Step 4: Perform a right turn at the next right-turn corner
  if (secondLeftTurnDone && !rightTurnDone) {
    course_correction();
    junction_checker();

    if (rightTurnDetected()) {  // Detect right turn corner
      left_motor->run(RELEASE);
      right_motor->run(RELEASE);

      left_motor->setSpeed(70);
      right_motor->setSpeed(255);
      right_motor->run(BACKWARD);
      left_motor->run(FORWARD);
      delay(2500);  // Adjust delay as needed for full turn

      left_motor->run(RELEASE);
      right_motor->run(RELEASE);

      rightTurnDone = true;
      Serial.println("Right turn at right-turn corner completed");
    }
  }

  // Step 5: Finish with course correction
  if (rightTurnDone && !finalCorrectionDone) {
    course_correction();
    finalCorrectionDone = true;
    Serial.println("Final course correction completed");
  }
}
