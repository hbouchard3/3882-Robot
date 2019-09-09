/**
 * This file provides some starting structure for controlling
 * the Elegoo robot car and interfacing with its sensors.
 */
 
// Library for controlling the hobby servo
#include <Servo.h>

// Configuration
// Period of the loop, in ms
#define LoopTime 100

// Motor driver pin connections
#define L_EN 5  // left motor R_ENble
#define L_1 7  // left motor control
#define L_2 8  // left motor control
#define R_EN 6  // right motor R_ENble
#define R_1 9  // right motor control
#define R_2 11 // right motor control

// state machine states
#define MOVE 12
#define OBJECT_IN_PATH 13
#define AT_STATION 14

// Ultrasonic rangefinder pin connections
#define Echo A4
#define Trig A5

// Servo pins
#define ServoPin 3

// Macros to read the three line tracking sensors.
// These will return '0' with a light surface and '1'
// with a dark surface.
#define LT_R !digitalRead(10)
#define LT_M !digitalRead(4)
#define LT_L !digitalRead(2)

int readDistance();
void stopRobot();


// global variables
Servo head;  // create servo object to control the looking direction
long prevMillis; // used to time loop()
short state;

/**
 *  Create reusable functions here or in additional files that
 *  you #include.  For example, functions could
 *  set the speed and direction of the left side and right side,
 *  or make the robot as a whole turn left or right or go
 *  straight (or you could do the former and then use that to
 *  create the latter), and functions could interpret sensor
 *  data, change states, enact behaviors, etc.
 */


/*
 * Function for state "AT_STATION"
 * 
 * Waits for one second, then checks whether there is an object to the right of the robot within 12 inches.
 */
int atStation(){
  delay(1000); // wait one second
  head.write(180); // look to the right.
  while((readDistance() != 0) || (readDistance() < 31))
  {
    // do nothing.
  }

  head.write(90); // look forward.
  return MOVE; // move on to MOVE state.
}

/*
 * Function for state "MOVE"
 */
int move(){

  // Example of how the sensor macros can be used.  Whether or not this
  // type of sensor interaction belongs in loop() is up to your
  // code structure.

  /*
  if (LT_M) 
  {
    analogWrite(R_EN, 128);
    analogWrite(L_EN, 128);
    digitalWrite(L_1, HIGH);
    digitalWrite(L_2, LOW);
    digitalWrite(R_1, HIGH);
    digitalWrite(R_2, HIGH);
    Serial.println ("LTM");
  }
  */
  
  if(!LT_R)     // Turn right
  {
    analogWrite(R_EN, 128);
    analogWrite(L_EN, 128);
    digitalWrite(R_1, HIGH);
    digitalWrite(R_2, HIGH);
    digitalWrite(L_1, LOW);
    digitalWrite(L_2, LOW);
    Serial.println("LTR");
   
  }
  
  if(!LT_L)    // turn left
  { 
    analogWrite(6, 128);
    analogWrite(5, 128);
    digitalWrite(9, LOW);
    digitalWrite(11, LOW);
    digitalWrite(7, HIGH);
    digitalWrite(8, LOW);
    Serial.println("LTL");
    
  }

/*
  if((LT_L) && (LT_R) && (LT_M))     // stop for station
  { 
    stopRobot();
    head.write(0);// manufacturing stop at 12 inches as mentioned in the document
    Serial.println("Station");
    return AT_STATION;
    
  }
  
  if(readDistance()<=30)  // stop for obstruction
  {
    stopRobot();
    Serial.println("Object");
    return OBJECT_IN_PATH;
  
  }
*/
  return MOVE;
  
}

/*
 * Function for state "OBJECT_IN_PATH"
 * 
 * Waits until the object is no longer in the path.
 */
int objectInPath(){
  while((readDistance() != 0) || (readDistance() < 10))
  {
    // do nothing.
  }
  return MOVE;
}

/**
 * You may or may not want functions like stopRobot().  With your chosen
 * breakdown of functions, is it useful and sensible to have a
 * function that stops the robot?
 */
void stopRobot(){
  // Disable the motors?  Set a speed variable to 0?
  // Depends on your hierarchy and where this function
  // fits into it.
} 

/**
 *  Motor controller control information:
 *  Setting IN1 high and IN2 low sets the left motor forwards.
 *  IN3 low and IN4 high sets the right motor forwards.
 *  Swap those polarities to make the motors turn backwards.
 *  Setting both control signals low applies a brake.
 *  When the IN_ signals are set for forwards or backwards,
 *  the ENA and ENB signals can be modulated with PWM
 *  to control the speed of the motors.  See the analogWrite()
 *  Arduino function for an easy way to create PWM.
 */

/** 
 *  Ultrasonic distance measurement.  Returns distance in cm.
 *  Max distance is ~1.5m, based on the timeout of pulseIn().
 *  This is an example of a lowest-level function, since it
 *  interacts with, and depends on, the lowest level of hardware
 *  of the processor and sensor.
 */
int readDistance() {
  digitalWrite(Trig, LOW);  // ensure ping is off
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH); // start ping
  delayMicroseconds(10); 
  digitalWrite(Trig, LOW);  // end ping
  return (int)pulseIn(Echo, HIGH, 10000) / 58; // measure time of flight and convert to cm
}

/**
 * waitForTick() blocks until a periodic time based on the global millis() time,
 * which can be used to force loop() to run at a predictable rate (as long as
 * the code in loop() is faster than the tick time).
 * This may or may not be desirable for your code, but it usually is.
 */
void waitForTick(){
  // block until the specified time
  while((millis() - prevMillis) <= LoopTime);
  prevMillis = millis();
  return;
}


void setup(){
  // Start serial comm in case you want to debug with it
  Serial.begin(9600);

  


  state = MOVE;

  // Configure the pins that are outputs
  pinMode(Trig, OUTPUT);
  pinMode(L_1, OUTPUT);
  pinMode(L_2, OUTPUT);
  pinMode(R_1, OUTPUT);
  pinMode(R_2, OUTPUT);
  pinMode(R_EN, OUTPUT);
  pinMode(L_EN, OUTPUT);
  pinMode(ServoPin, OUTPUT);


  // Attach the servo controller to the servo pin
  head.attach(ServoPin);
  // Set the head looking forward.
  head.write(90);

  // record the current time for the timing function
  prevMillis = millis();
}

void loop() {
  // calling waitForTick() at the beginning of loop will keep it periodic
  waitForTick(); 

  // State Machine Manager
  switch(state)
  {
    case MOVE:
      Serial.println("hello");
      state = move();
      Serial.println("end");
    break;
    case OBJECT_IN_PATH:
      state = objectInPath();
    break;
    case AT_STATION:
      state = atStation();
    break;
  }
  







  
  
  // Example of reading the ultrasonic rangefinder and printing to
  // the serial port.
  // Note that readDistance() is blocking, meaning that it will prevent
  // any other code from executing until it returns.  This will
  // take a variable amount of time, up to ~10 ms.
  Serial.println(readDistance());
  
  // Example of how the sensor macros can be used.  Whether or not this
  // type of sensor interaction belongs in loop() is up to your
  // code structure.
  if(LT_M){
    // Do something based on the middle sensor detecting a dark surface
  }

}
