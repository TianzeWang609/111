
#include <Adafruit_MotorShield.h>
#include <Servo.h>  // Include Servo library
#include <Wire.h>   // Include Wire library for I2C
#include <DFRobot_VL53L0X.h>  // Include ToF sensor library


Servo gate_servo;  // Create gate servo
Servo ramp_servo ; // create ramp servo
DFRobot_VL53L0X sensor;  // Create ToF sensor object

void setup() {
  Serial.begin(9600);
  Serial.println("Adafruit Motorshield v2 - Dual Line Sensor Controlled Motors!");

  // Create the motor shield object
  Adafruit_MotorShield AFMS = Adafruit_MotorShield();
  Adafruit_DCMotor *right_motor = AFMS.getMotor(3);  // Motor on port 3 (right motor)
  Adafruit_DCMotor *left_motor = AFMS.getMotor(4);   // Motor on port 4 (left motor)

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.") ;
  pinMode(8, INPUT);         // Magnetic sensor input

  // Initialize servo and set it to vertical position

  ramp_servo.attach(7) ;
  ramp_servo.write(0) ;

  gate_servo.attach(6);
  gate_servo.write(0);  // Start with servo at 0 degrees to start of with

  // Initialize Time of Flight sensor
  Wire.begin();
  sensor.begin(0x50); // Initialize the sensor at address 0x50
  sensor.setMode(sensor.eContinuous, sensor.eHigh); // Continuous and high-precision mode
  sensor.start(); // Start the sensor

  // Set initial motor speeds
  right_motor->setSpeed(158);  // Max speed for right motor
  left_motor->setSpeed(150);   // Max speed for left motor
  //right_motor-> run(BACKWARD) ;
  //left_motor ->run(BACKWARD) ;
}


void loop() {
  ramp_servo.write(20);
  //delay(1000) ;
  //ramp_servo.write(0) ;
  delay(1000) ;
  gate_servo.write(35) ;
  delay(1000) ;
  gate_servo.write(130) ;
  delay(1000) ;
  ramp_servo.write(0);
  delay(1000) ;
  //ramp_servo.write(0) ;

  


}
