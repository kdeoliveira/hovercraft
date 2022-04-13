#pragma once

/**
 * @brief PINOUT definition for the microcontroller
 * 
 */
#define LIFT_FAN 6
#define THRUST_FAN 10
#define BATTERY_STATUS A6
#define SERVO 9
#define LED_GREEN 5
#define LED_WARNING 13
// PD3 - 1x3
#define US_SENSOR_1 3
// PD4 - 1x2
#define US_SENSOR_2 4
// PD2 - 2x3
#define US_SENSOR_3 2

namespace hc
{
    /**
     * @brief Enumaration of state transition for the state machine
     * 
     */
    enum class STATE : int
    {
        START,
        LEFT,
        FRONT,
        RIGHT,
        END
    };

    /**
     * @brief Struct defining integer object performs cyclic iterations.
     * 
     */
    struct int_cycle
    {
        int cycles = 0;

        int_cycle &operator++()
        {
            (*this).cycles = ((*this).cycles + 1) % 5;
            return *this;
        }

        int_cycle operator++(int)
        {
            (*this).cycles = ((*this).cycles + 1) % 5;
            return *this;
        }

        int_cycle &operator--()
        {
            if ((*this).cycles == 0 ) return *this;
            (*this).cycles = ((*this).cycles - 1) % 5;
            return *this;
        }

        int_cycle operator--(int)
        {
            if ((*this).cycles == 0 ) return *this;
            (*this).cycles = ((*this).cycles - 1) % 5;
            return *this;
        }
    };

    /**
     * @brief Definition of state machine object that it's used to model the behavior of our hovercraft based on the input sensors
     * Output logic is performed outside this object
     * 
     */
    class StateMachine
    {

    private:
        STATE m_state;
        int_cycle m_type;
        double m_transition_value;
        bool changed_state;

    public:
    /**
     * @brief Construct a new State Machine object
     * 
     * @param transition Enabled transition variable
     */
        StateMachine(double transition) : m_transition_value{transition}, changed_state{false}
        {
            m_state = STATE(m_type.cycles);
        }

        /**
         * @brief Runs the state logic of this state machine and verifies next state based on input sensors.
         * 
         * @param sensor_left 
         * @param sensor_front 
         * @param sensor_right 
         * @return STATE 
         */
        STATE run(double sensor_left, double sensor_front, double sensor_right)
        {
            this->changed_state = false;

            if (m_type.cycles == 0)
            {
                this->changed_state = true;
                ++m_type;
                return this->m_state = STATE(m_type.cycles);
            }

            else if (m_type.cycles == 1)
            {
                if (sensor_left < m_transition_value)
                {
                    this->changed_state = true;
                    ++m_type;
                    return m_state = STATE(m_type.cycles);
                }
            }
            else if (m_type.cycles == 2)    //LEFT
            {
                if (sensor_front < m_transition_value && sensor_left > m_transition_value)
                {
                    this->changed_state = true;
                    --m_type;
                    return m_state = STATE(m_type.cycles);
                }
                else if (sensor_front < m_transition_value && sensor_right > m_transition_value)
                {
                    this->changed_state = true;
                    ++m_type;
                    return m_state = STATE(m_type.cycles);
                }
            }
            else if (m_type.cycles == 3)    //RIGHT
            {
                if (sensor_right < m_transition_value)
                {   
                    this->changed_state = true;
                    --m_type;
                    return m_state = STATE(m_type.cycles);
                }
                // else if (sensor_right < m_transition_value)
                // {
                //     ++m_type;
                //     return this->m_state = STATE(m_type.cycles);
                // }
            }
            return this->m_state;
        }
        /**
         * @brief Flag that returns if the current state has changed
         * 
         * @return bool 
         */
        bool get_changed_state(){
        return this->changed_state;
    }
    };

    

}