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
     * @brief Begin of setup() and initialization of variables and objects
     *
     */

    Serial.begin(9600);

    /**
     * @brief PID controller used for front sensor
     * 
     */
    double set_point, input, output;
    PID pid_controller(&input, &output, &set_point, 0, 10, 0, P_ON_M, DIRECT);
    set_point = 20.0;
    pid_controller.SetOutputLimits(0, 255);
    pid_controller.SetMode(AUTOMATIC);


    /**
     * @brief Input variable for US sensors
     * 
     */
    unsigned long us_sensor_buffer[3];

    double distance_left, distance_front, distance_right;

    /**
     * @brief Servo motor object
     * 
     */
    Servo servo_motor;

    servo_motor.attach(SERVO);

    hc::StateMachine state_machine(20.0);

    // pinMode(LIFT_FAN, OUTPUT);
    pinMode(THRUST_FAN, OUTPUT);

    pinMode(US_SENSOR_LEFT, INPUT);
    pinMode(US_SENSOR_RIGHT, INPUT);
    pinMode(US_SENSOR_FRONT, INPUT);

    pinMode(LED_GREEN, OUTPUT);
    pinMode(SERVO, OUTPUT);
    pinMode(LED_WARNING, OUTPUT);

    /**
     * @brief Super-loop
     * 
     */
    while (true)
    {
        
        
        analogWrite(LIFT_FAN, 255);
        analogWrite(THRUST_FAN, 255);

        /**
         * @brief Measures the time of a pulse length when in HIGH state
         * Value returns the speed of sound which can then be converted to cm
         * 
         */

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

        /**
         * @brief runs the state machine logic for next state value
         * 
         */

        hc::STATE current_state = state_machine.run(distance_left, distance_front, distance_right);

        
        /**
         * @brief Init of output logic based on next state and input value
         * This is a representation of a miley machine which outputs the required angle based on the currents state of the hovercraft and the input provided.
         * Servo is defined as an actuator which can perform rotation from -90 to +90 
         * if on state LEFT: servo is at -90 degrees (MIN_PULSE_WIDTH)
         * if on state FRONT: servo is at 0 degree (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2
         * if on state RIGHT: servo is at +90 degree (MAX_PULSE_WIDTH)
         * 
         * Note that an additional check is made in case the state has changed. It ensures that the servo performs a smooth rotation preventing the hovercraft to enter in an undesired state.
         * 
         * Note that thrust fan speed is decreased for smooth rotation
         */
        if (current_state == hc::STATE::LEFT)
        {

            if (state_machine.get_changed_state())
            {
                
                analogWrite(THRUST_FAN, 75);
                int temp_init = MIN_PULSE_WIDTH;
                int temp_limit = servo_motor.readMicroseconds();

                for (int i = temp_limit; i >= temp_init; i--)
                {
                    servo_motor.writeMicroseconds(i);
                    delay(1);
                }
            }

            servo_motor.writeMicroseconds(MIN_PULSE_WIDTH + 25);

            Serial.print("[LEFT] front_sensor: ");
            Serial.println(distance_front);

            analogWrite(THRUST_FAN, 215);
        }
        else if (current_state == hc::STATE::FRONT)
        {
            if (state_machine.get_changed_state())
            {

                analogWrite(THRUST_FAN, 75);
                int temp_init = servo_motor.readMicroseconds();
                int temp_limit = (MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2;

                if (temp_init < temp_limit)
                    for (int i = temp_init; i <= temp_limit; i++)
                    {
                        servo_motor.writeMicroseconds(i);
                        delay(1);
                    }

                else
                {
                    Serial.println("CHANGED TO FRONT");

                    for (int i = temp_init; i >= temp_limit; i--)
                    {
                        servo_motor.writeMicroseconds(i);
                        delay(1);
                    }
                }
            }

            /**
             * @brief PID CONTROLLER for front sensor during turning. Since this is a critical state for the hovercraft, we ensure that the servo smoothly adjusts its angle based on the left sensor value.
             * 
             */

            input = distance_left;
            // input =  hc::Util::map(distance_left, 13.0, 40.0, 0, 255);
            pid_controller.Compute();

            // PID should output to correct value
             servo_motor.writeMicroseconds(
                 hc::Util::map(output, 0, 255, 1525 + 50, 1525 - 50 )
             );

            // servo_motor.writeMicroseconds((MIN_PULSE_WIDTH + MAX_PULSE_WIDTH) / 2);

            Serial.println("FRONT");
            analogWrite(THRUST_FAN, 215);
        }

        else if (current_state == hc::STATE::RIGHT)
        {
            if (state_machine.get_changed_state())
            {
                analogWrite(THRUST_FAN, 75);
                int temp_init = min(servo_motor.readMicroseconds(), MAX_PULSE_WIDTH);
                int temp_limit = max(servo_motor.readMicroseconds(), MAX_PULSE_WIDTH);

                for (int i = temp_init; i <= temp_limit; i++)
                {
                    servo_motor.writeMicroseconds(i);
                    delay(1);
                }
            }

            servo_motor.writeMicroseconds(MAX_PULSE_WIDTH);
            Serial.println("RIGHT");
            analogWrite(THRUST_FAN, 215);
        }

        if (serialEventRun)
            serialEventRun();

        /**
         * @brief Overall delay
         * 
         */
        delay(10);
    }

    return 0;
}
