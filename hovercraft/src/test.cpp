#include <Arduino.h>

#include <Servo.h>
#include <util.hpp>


#include <PID_v1.h>

#define DEBUG


#define OUTPUT_THRUST_FAN 6


#define BATTERY_STATUS A6


#define SERVO_PIN 9

#define LED_PIN 5

#define US_SENSOR 3


Servo servo;



/**
 * @brief PID controller set for adjusting US Sensors input values
 * 
 */

unsigned long us_sensor_buffer;
double us_cm_input, us_cm_output, setPoint;
PID pid_controller(&us_cm_input, &us_cm_output, &setPoint, 2.0,  5.0, 1.0, P_ON_M, DIRECT);
unsigned long windowStartTime;
int windowSize = 1000;


void setup() {

  windowStartTime = millis();


  Serial.begin(9600);


  servo.attach(SERVO_PIN);

  while(!Serial);

  pinMode(OUTPUT_THRUST_FAN, OUTPUT);
  pinMode(10, OUTPUT);






}




void loop() {

    analogWrite(OUTPUT_THRUST_FAN, 255);

    analogWrite(10, 255);

    servo.writeMicroseconds(MIN_PULSE_WIDTH);
}