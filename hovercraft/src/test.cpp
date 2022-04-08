// #include <Arduino.h>

// #include <Servo.h>
// #include <util.hpp>


// #include <PID_v1.h>

// #define DEBUG


// #define OUTPUT_THRUST_FAN 6


// #define BATTERY_STATUS A6


// #define SERVO_PIN 9

// #define LED_PIN 5

// #define US_SENSOR 3


// Servo servo;



// /**
//  * @brief PID controller set for adjusting US Sensors input values
//  * 
//  */

// unsigned long us_sensor_buffer;
// double us_cm_input, us_cm_output, setPoint;
// PID pid_controller(&us_cm_input, &us_cm_output, &setPoint, 2.0,  5.0, 1.0, P_ON_M, DIRECT);
// unsigned long windowStartTime;
// int windowSize = 1000;


// void setup() {

//   windowStartTime = millis();


//   Serial.begin(9600);


//   servo.attach(SERVO_PIN);

//   while(!Serial);

//   pinMode(OUTPUT_THRUST_FAN, OUTPUT);

//   pinMode(INPUT, BATTERY_STATUS);
  
//   pinMode(INPUT, US_SENSOR);

//   pinMode(OUTPUT, SERVO_PIN);

//   pinMode(OUTPUT, LED_PIN);

//   digitalWrite(LED_PIN, HIGH);




//   setPoint = 20.0;

//   // pid_controller.SetOutputLimits(0, windowSize);
//   pid_controller.SetMode(AUTOMATIC);




// }

// int i = 0;



// void loop() {
  


//   analogWrite(OUTPUT_THRUST_FAN, 255);

  
//   // i = (i + 1) % 255;


//   float battery_voltage = hc::Util::map(
//     analogRead(BATTERY_STATUS), 0, 1023, 0, 7
//   );
//   // Serial.println(battery_voltage);



//   int servo_angle = hc::Util::map(i, 0, 255, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);

//   servo.writeMicroseconds(servo_angle);



//   us_sensor_buffer = pulseIn(US_SENSOR, HIGH);

//   if((us_sensor_buffer > 0) && (us_sensor_buffer <= 37500)){
//     us_cm_input = (double) us_sensor_buffer / 147 * 2.54;
//     pid_controller.Compute();
//   }

  
  
//   // if(millis() - windowStartTime > windowSize){
//   //   windowStartTime += windowSize;
//   // }

//   // if(us_cm_output < millis() - windowStartTime){

//     // analogWrite(LED_PIN, 255);
//   // }
//   // else{
    

    
//     // analogWrite(LED_PIN, LOW);
//   // }



//   // analogWrite(LED_PIN, us_cm_output);


//   Serial.print(us_cm_output);
//   // Serial.print(" | ");

  
//   // Serial.print(us_cm_input);
//   // Serial.print(" cm");
//   Serial.print("\t\t\t\t\r");



//   delay(50);
// }