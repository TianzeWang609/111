#include <Adafruit_MotorShield.h>

// Create the motor shield object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
Adafruit_DCMotor *left_motor = AFMS.getMotor(4);   // Motor on port 4 (left motor)

// Line sensor pins
int leftSensorPin = 3;
int rightSensorPin = 2;
int frontrightSensorPin = 6;
int frontleftSensorPin = 5;

// Button and LED pins
int startStopButtonPin = 7; // Start/stop button
int ledPin = 13;            // LED for flashing indicator

// System state variables
bool systemRunning = false;  // Tracks if the system is running
int lastButtonState = HIGH;  // Stores the previous button state for edge detection

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
bool initialMoveDone = false;   // 用于记录是否已完成初始前进
bool leftTurnDone = false;
bool rightTurnDone = false; 
bool rightTurnDone2 = false;
bool rightTurnDone3 = false;
bool rightTurnDone4 = false;  // 用于记录是否已完成第一次左转
bool tJunctionDetected = false; // 用于确保每个T形路口只计数一次

// Function prototypes
void course_correction();
void right_turn();
void left_turn();
void junction_checker();
void goFrom1to2();

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

    // 重置计数器和状态变量
    if (systemRunning) {
      tJunctionCount = 0;
      right_turn_counter = 0;
      left_turn_counter = 0;
      onetotwocomplete = 0;  // 重置任务完成标记
      initialMoveDone = false;   // 重置初始前进标记
      leftTurnDone = false;      // 重置左转完成标记
      tJunctionDetected = false; // 重置T形路口检测状态
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

    // Start subroutines for line following and routing only when running
    goFrom1to2();
  } else {
    // If system is stopped, turn off motors and LED
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
    digitalWrite(ledPin, LOW); // Ensure LED is off when stopped
  }
}

void course_correction() {
  int baseSpeed = 180;   // 基础速度
  int adjustment = 30;   // 调整幅度

  // 根据传感器状态调整速度
  if (leftVal == HIGH && rightVal == HIGH) {
    // 两个传感器都检测到白色，保持直线行驶
    right_motor->setSpeed(baseSpeed);
    left_motor->setSpeed(baseSpeed);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    Serial.println("Driving forward");
  } 
  else if (leftVal == LOW && rightVal == HIGH) {
    // 左传感器检测到黑线，右传感器检测到白色，向右轻微调整
    right_motor->setSpeed(baseSpeed - adjustment); // 减速右轮
    left_motor->setSpeed(baseSpeed + adjustment);  // 加速左轮
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    Serial.println("Adjusting right");
  } 
  else if (leftVal == HIGH && rightVal == LOW) {
    // 右传感器检测到黑线，左传感器检测到白色，向左轻微调整
    right_motor->setSpeed(baseSpeed + adjustment); // 加速右轮
    left_motor->setSpeed(baseSpeed - adjustment);  // 减速左轮
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    Serial.println("Adjusting left");
  } 
  else {
    // 停止，或适当的默认行为
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    Serial.println("Stopping or off track");
  }
}

void right_turn() {
  left_motor->setSpeed(50);
  right_motor->setSpeed(195);
  right_motor->run(BACKWARD);
  left_motor->run(FORWARD);
  Serial.println("Turning right");
  delay(2000);
}

void left_turn() {
  left_motor->setSpeed(230);
  right_motor->setSpeed(50);
  right_motor->run(FORWARD);
  left_motor->run(BACKWARD);
  Serial.println("Turning left");
  delay(2000);
}

void goFrom1to2() {
  static int tJunctionAfterLeftTurn = 0; // 用于记录左转后的T形路口数

  // 第一步：前进1秒
  if (!initialMoveDone) {
    right_motor->setSpeed(120);
    left_motor->setSpeed(120);
    right_motor->run(BACKWARD);
    left_motor->run(BACKWARD);
    delay(1000);
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
    //tJunctionAfterLeftTurn = tJunction; // 重置左转后计数，准备继续检测T形路口
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
 
  
  if ((right_turn_counter == 4) && !rightTurnDone2) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone2 = true;
    Serial.println("right turn complete forth junction");
  }
  if (rightTurnDone2 == true) {
    course_correction(); // 在左转后继续前进
  }

  if ((right_turn_counter == 5) && !rightTurnDone3) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone3 = true;
    Serial.println("right turn complete forth junction");
  }
  if (rightTurnDone3 == true) {
    course_correction(); // 在左转后继续前进
  }
  if ((right_turn_counter == 6) && !rightTurnDone4) {
    right_motor->run(RELEASE);
    left_motor->run(RELEASE);
    right_turn();
    rightTurnDone4 = true;
    Serial.println("right turn complete forth junction");
  }
  if (rightTurnDone4 == true) {
    course_correction(); // 在左转后继续前进
  }
  


  


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
