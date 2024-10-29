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

// Function prototypes
void course_correction();
void right_turn();
void left_turn();
void junction_checker();
void navigation();

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
    delay(50);  // Simple debounce
    systemRunning = !systemRunning;  // Toggle system running state
    Serial.println(systemRunning ? "System Started" : "System Stopped");
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

    // Read the values from the line sensors
    leftVal = digitalRead(leftSensorPin);
    rightVal = digitalRead(rightSensorPin);
    frontrightVal = digitalRead(frontrightSensorPin);
    frontleftVal = digitalRead(frontleftSensorPin);

    // Print sensor values for debugging
    Serial.print("Left Sensor: ");
    Serial.print(leftVal);
    Serial.print(" Right Sensor: ");
    Serial.println(rightVal);
    Serial.print(" Front Right Sensor: ");
    Serial.print(frontrightVal);
    Serial.print(" Front Left Sensor: ");
    Serial.println(frontleftVal);

    // Start subroutines for line following and routing
    junction_checker();
    course_correction();
  } else {
    // If system is stopped, turn off motors and LED
    left_motor->run(RELEASE);
    right_motor->run(RELEASE);
    digitalWrite(ledPin, LOW); // Ensure LED is off when stopped
  }
}

void course_correction() {
  int baseSpeed = 200;   // 基础速度
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

  // 去除不必要的延时，减少调整时的延迟
}


void right_turn() {
  left_motor->setSpeed(150);
  right_motor->setSpeed(158);
  right_motor->run(BACKWARD);
  left_motor->run(FORWARD);
  Serial.println("Turning right");
  delay(1000);
}

void left_turn() {
  left_motor->setSpeed(150);
  right_motor->setSpeed(158);
  right_motor->run(FORWARD);
  left_motor->run(BACKWARD);
  Serial.println("Turning left");
  delay(1000);
}

void junction_checker() {
  if (frontrightVal == HIGH && frontleftVal == LOW) {
    delay(100);
  } else if (frontrightVal == LOW && frontleftVal == HIGH) {
    delay(100);
  }

  leftVal = digitalRead(leftSensorPin);
  rightVal = digitalRead(rightSensorPin);
  frontrightVal = digitalRead(frontrightSensorPin);
  frontleftVal = digitalRead(frontleftSensorPin);

  if (frontrightVal == HIGH && frontleftVal == HIGH) {
    tJunctionCount++;
    Serial.println("T Junction detected");
  }
  if (frontrightVal == HIGH && frontleftVal == LOW) {
    right_turn_counter++;
    Serial.println("Right turn rejected");
  }
  if (frontrightVal == LOW && frontleftVal == HIGH) {
    left_turn_counter++;
    Serial.println("Left turn detected");
  }
}
