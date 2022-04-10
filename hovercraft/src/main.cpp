#include <Arduino.h>

#include <common.hpp>
#include <Servo.h>
#include <util.hpp>

#include <PID_v1.h>

// Declared weak in Arduino.h to allow user redefinitions.
int atexit(void (*/*func*/)()) { return 0; }

// Weak empty variant initialization function.
// May be redefined by variant files.
void initVariant() __attribute__((weak));
void initVariant() {}

void setupUSB() __attribute__((weak));
void setupUSB() {}



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


    double set_point, input, output;
    PID pid_controller(&input, &output, &set_point, 0,10,0, P_ON_M, DIRECT);
    set_point = 22.0;
    pid_controller.SetOutputLimits(0, 255);
    pid_controller.SetMode(AUTOMATIC);

    unsigned long us_sensor_buffer[3];

    double distance_left, distance_front, distance_right;

    Servo servo_motor;

    servo_motor.attach(SERVO);

    hc::StateMachine state_machine(20.0);

    pinMode(LIFT_FAN, OUTPUT);
    pinMode(THRUST_FAN, OUTPUT);

    pinMode(US_SENSOR_LEFT, INPUT);
    pinMode(US_SENSOR_RIGHT, INPUT);
    pinMode(US_SENSOR_FRONT, INPUT);

    pinMode(LED_GREEN, OUTPUT);
    pinMode(SERVO, OUTPUT);
    pinMode(LED_WARNING, OUTPUT);



    while (true)
    {
        // servo_motor.writeMicroseconds(MIN_PULSE_WIDTH);

        analogWrite(LIFT_FAN, 255);
        analogWrite(THRUST_FAN, 0);

        us_sensor_buffer[0] = pulseIn(US_SENSOR_LEFT, HIGH);
        us_sensor_buffer[1] = pulseIn(US_SENSOR_FRONT, HIGH);
        us_sensor_buffer[2] = pulseIn(US_SENSOR_RIGHT, HIGH);

        if ((us_sensor_buffer[0] > 0) && (us_sensor_buffer[0] <= 37500))
        {
            distance_left = us_sensor_buffer[0] / 147 * 2.54;
        }

        if ((us_sensor_buffer[1] > 0) && (us_sensor_buffer[1] <= 37500))
        {
            distance_front = us_sensor_buffer[1] / 147 * 2.54;
        }

        if ((us_sensor_buffer[2] > 0) && (us_sensor_buffer[2] <= 37500))
        {
            distance_right = us_sensor_buffer[2] / 147 * 2.54;
        }

        // Serial.print("LEFT: ");
        // Serial.print(distance_left);
        // Serial.print("\tFRONT: ");
        // Serial.print(distance_front);
        // Serial.print("\tRIGHT: ");
        // Serial.println(distance_right);

        hc::STATE current_state = state_machine.run(distance_left, distance_front, distance_right);

        //PID controller currently set only for one state 
        if (current_state == hc::STATE::LEFT)
        {
            input = distance_front;
            // input =  hc::Util::map(distance_front, 13.0, 40.0, 0, 255);
            pid_controller.Compute();

            //PID should output to correct value
            servo_motor.writeMicroseconds(
                hc::Util::map(output, 0, 255, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH)
            );

            Serial.println("LEFT");
            analogWrite(THRUST_FAN, 255);
        }
        else if (current_state == hc::STATE::FRONT)
        {
            servo_motor.writeMicroseconds(
                (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2);

            Serial.println("FRONT");
            analogWrite(THRUST_FAN, 255);
        }

        else if (current_state == hc::STATE::RIGHT)
        {
            servo_motor.writeMicroseconds(MAX_PULSE_WIDTH);
            Serial.println("RIGHT");
            analogWrite(THRUST_FAN, 255);
        }

        if (serialEventRun)
            serialEventRun();

        delay(10);
    }

    return 0;
}
