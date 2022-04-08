#pragma once

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

    enum class STATE : int
    {
        START,
        LEFT,
        FRONT,
        RIGHT,
        END
    };

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

    class StateMachine
    {

    private:
        STATE m_state;
        int_cycle m_type;
        double m_transition_value;

    public:
        StateMachine(double transition) : m_transition_value{transition}
        {
            m_state = STATE(m_type.cycles);
        }

        STATE run(double sensor_left, double sensor_front, double sensor_right)
        {
            if (m_type.cycles == 0)
            {
                ++m_type;
                return this->m_state = STATE(m_type.cycles);
            }

            else if (m_type.cycles == 1)
            {
                if (sensor_left < m_transition_value)
                {
                    ++m_type;
                    return m_state = STATE(m_type.cycles);
                }
            }
            else if (m_type.cycles == 2)
            {
                if (sensor_front < m_transition_value && sensor_left > m_transition_value)
                {
                    --m_type;
                    return m_state = STATE(m_type.cycles);
                }
                else if (sensor_front < m_transition_value && sensor_right > m_transition_value)
                {
                    ++m_type;
                    return m_state = STATE(m_type.cycles);
                }
            }
            else if (m_type.cycles == 3)
            {
                if (sensor_right > m_transition_value)
                {
                    --m_type;
                    return m_state = STATE(m_type.cycles);
                }
                else if (sensor_right < m_transition_value)
                {
                    ++m_type;
                    return this->m_state = STATE(m_type.cycles);
                }
            }
            return this->m_state;
        }
    };

}