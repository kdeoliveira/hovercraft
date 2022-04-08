#include <Arduino.h>

#include <common.hpp>
#include <actuator.hpp>
#include <Servo.h>
#include <util.hpp>

#include <AutoPID.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (* /*func*/ )()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() { }

void setupUSB() __attribute__((weak));
void setupUSB() { }






const int min_us_sensor = 20;


#define US_SENSOR_LEFT US_SENSOR_1
#define US_SENSOR_RIGHT US_SENSOR_2
#define US_SENSOR_FRONT US_SENSOR_3


int main(void)
{
	init();

	initVariant();

#if defined(USBCON)
	USBDevice.attach();
#endif
	
    /**
     * @brief Begin of setup()
     * 
     */

    Serial.begin(9600);

    hc::Actuator us_sensor_controller{5,2,1};

    double distance_left, distance_front, distance_right, distance_output_left;
    double set_point;


    AutoPID pid_controller(&distance_left, &set_point, &distance_output_left, 0, 255, 0.12, 0.0003, 0);

    pid_controller.setBangBang(4.5);

    pid_controller.setTimeStep(1000);

    set_point = 20.0;

    Servo servo_motor;

    servo_motor.attach(SERVO);


    // hc::STATE current_state = hc::STATE::LEFT;
    // 0 : left
    // 1 : front
    // 2 : right
    int current_state = 0;



    

    pinMode(LIFT_FAN, OUTPUT);
    pinMode(THRUST_FAN, OUTPUT);

    pinMode(US_SENSOR_LEFT, INPUT);
    pinMode(US_SENSOR_RIGHT, INPUT);
    pinMode(US_SENSOR_FRONT, INPUT);


    pinMode(LED_GREEN, OUTPUT);
    pinMode(SERVO, OUTPUT);
    pinMode(LED_WARNING, OUTPUT);

    unsigned long us_sensor_buffer[3];

    int i = 0;

    
	while(true){
        // servo_motor.writeMicroseconds(MIN_PULSE_WIDTH);

        analogWrite(LIFT_FAN, 255);
        analogWrite(THRUST_FAN, 0);
		
        us_sensor_buffer[0] = pulseIn(US_SENSOR_LEFT, HIGH);
        us_sensor_buffer[1] = pulseIn(US_SENSOR_FRONT, HIGH);
        us_sensor_buffer[2] = pulseIn(US_SENSOR_RIGHT, HIGH);

        if( (us_sensor_buffer[0] > 0) && (us_sensor_buffer[0] <= 37500) ){
            distance_left = us_sensor_buffer[0] / 147 * 2.54;
        }

        if( (us_sensor_buffer[1] > 0) && (us_sensor_buffer[1] <= 37500) ){
            distance_front = us_sensor_buffer[1] / 147 * 2.54;
        }

        if( (us_sensor_buffer[2] > 0) && (us_sensor_buffer[2] <= 37500) ){
            distance_right = us_sensor_buffer[2] / 147 * 2.54;
        }

        // Serial.print("LEFT: ");
        // Serial.print(distance_left);
        // Serial.print("\tFRONT: ");
        // Serial.print(distance_front);
        // Serial.print("\tRIGHT: ");
        // Serial.println(distance_right);

        
        
        
        if(
            (distance_left - distance_front) > 0 && (distance_left - distance_right) > 0
        ){

            servo_motor.writeMicroseconds(
                MIN_PULSE_WIDTH
            );
            
            Serial.println("LEFT");
            analogWrite(THRUST_FAN, 255);

        }
        else if(
            (distance_front - distance_left) > 0 && (distance_front - distance_right) > 0
        ){
            servo_motor.writeMicroseconds(
                (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) /2
            );

            Serial.println("FRONT");
            analogWrite(THRUST_FAN, 255);
        }
        else if(
            (distance_right - distance_front) > 0 && (distance_right - distance_left) > 0
        ){
            servo_motor.writeMicroseconds(
                MAX_PULSE_WIDTH
            );
            Serial.println("RIGHT");
            analogWrite(THRUST_FAN, 255);
        }



        



        // if( (us_sensor_buffer[0] > 0) && (us_sensor_buffer[0] <= 37500) ){
        //     distance_left = us_sensor_buffer[0] / 147 * 2.54;
        //     Serial.print(distance_left);

        //     pid_controller.run();
            
        //     Serial.print("\t|\t");
        //     if(pid_controller.atSetPoint(1)) continue;

        //     if(distance_output_left > 0 && distance_left - 20.0 < 0){
        //         int servo_angle = hc::Util::map(abs(distance_left - 20.0), 0, 5, MIN_PULSE_WIDTH, (MIN_PULSE_WIDTH+MAX_PULSE_WIDTH)/3);
        //         Serial.print(servo_angle);
        //         servo_motor.writeMicroseconds(servo_angle);
        //     }

            //  analogWrite(LED_WARNING, pid_controller.atSetPoint(1));
            //  Serial.print("\t|\t");

            //  Serial.println(distance_output_left);

            
            // us_sensor_controller.start(distance);
            
            
        // }



        
        // int servo_angle = hc::Util::map(i, 0, 255, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
        // servo_motor.writeMicroseconds(MIN_PULSE_WIDTH);
        
        

        // i  = (i +  1) % 255;

        // if(us_sensor_controller.output_signal(millis())){
        //     analogWrite(LED_GREEN, 255);
        // }else{
        //     analogWrite(LED_GREEN, 0);
        // }
        
        // Serial.println((us_sensor_controller.get_output()));






		if (serialEventRun) serialEventRun();

        delay(10);
	}
        
	return 0;
}
