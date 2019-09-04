/**
 * This file provides a quick sanity check that
 * the robot is working.
 */
 
// Library for controlling the hobby servo
#include <Servo.h>

// Motor driver pin connections
#define ENB 5  // left motor enable
#define IN1 7  // left motor control
#define IN2 8  // left motor control
#define ENA 6  // right motor enable
#define IN3 9  // right motor control
#define IN4 11 // right motor control

// Ultrasonic rangefinder pin connections
#define Echo A4
#define Trig A5

// Servo pins
#define ServoPin 3

// global variables
Servo head;  // create servo object to control the looking direction

/** 
 *  Ultrasonic distance measurement.  Returns distance in cm.
 *  Max distance is ~1.5m, based on the timeout of pulseIn().
 *  This is an example of a lowest-level function, since it
 *  interacts with, and depends on, the lowest level of hardware
 *  of the processor and sensor.
 */
int getDistance() {
  digitalWrite(Trig, LOW);  // ensure ping is off
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH); // start ping
  delayMicroseconds(10); 
  digitalWrite(Trig, LOW);  // end ping
  return (int)pulseIn(Echo, HIGH, 10000) / 58; // measure time of flight and convert to cm
}

void setup(){
  // Start serial comm in case you want to debug with it
  Serial.begin(9600);

  // Configure the pins that are outputs
  pinMode(Trig, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(ServoPin, OUTPUT);

  // Attach the servo controller to the servo pin
  head.attach(ServoPin);  
  
  // Set the head looking to the side.
  head.write(0);
  delay(2000);
  // Set the head looking to the other side.
  head.write(180);
  delay(2000);
  // and to the middle
  head.write(90);
  delay(1000);

  // test the left motors
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 128);
  delay(2000);
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(ENB, 0);
  delay(1000);
  
  // test the right motors
  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENA, 128);
  delay(2000);
  digitalWrite(IN4, LOW);
  digitalWrite(IN3, LOW);
  analogWrite(ENA, 0);
  delay(1000);
}

void loop() {
  
  // Example of reading the ultrasonic rangefinder and printing to
  // the serial port.
  Serial.println(getDistance());

}
