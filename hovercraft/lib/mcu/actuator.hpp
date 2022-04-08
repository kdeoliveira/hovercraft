#pragma once
#include <PID_v1.h>


namespace hc{
    class Actuator{
        private:
            PID pid_controller;
            double m_input;
            double m_output;
            double * m_set_point;
            const int m_window_size = 1000;
            unsigned long m_window_start_time;
        public:
            Actuator(int kp,int ki,int kd) : m_set_point{new double(20)}, pid_controller{&m_input, &m_output, m_set_point, kp, ki, kd, P_ON_M, DIRECT}{
                // this->pid_controller.SetOutputLimits(0, this->m_window_size);
                this->pid_controller.SetMode(AUTOMATIC);
                m_window_start_time = 0;
            }

            Actuator(const Actuator&) = delete;

            ~Actuator(){

                delete m_set_point;
            }

            void set_window_start_time(unsigned long x){
                this->m_window_start_time = x;
            }


            void set_point(double x){
                *m_set_point = x;
            }

            bool start(double in){
                this->m_input = in;

                return this->pid_controller.Compute();
            }

            double get_output(){
                return this->m_output;
            }
            

            bool output_signal(unsigned long current_time){
                if(current_time - this->m_window_start_time > m_window_size){
                    this->m_window_start_time += this->m_window_size;
                }

                return (this->m_output < current_time - this->m_window_start_time);
            }
        
    };
}