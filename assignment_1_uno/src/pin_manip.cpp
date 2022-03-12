#include <Arduino.h>
// 0/1

// DDRx: Setting INPUT/OUTPUT into pins

// DDRD = pin 0 to 7
// DDRB = pin 8 to 13
// DDRC = pin 0 to 7

// PORTx: Writing LOW/HIGH into pins


// Timer:

// outputDutyCycle = 60;
// pwmFrequency = (F_CPU / (timer1Prescaler)) - 1;
// dutyCycleDivisor = 100 / outputDutyCycle;
// pwmValueWithDutyCycle = pwmFrequency  / dutyCycleDivisor;

// ICR1 = (uint16_t)pwmFrequency;
// OCR1A = (uint16_t)pwmValueWithDutyCycle

// int main(){
//     init();

//     DDRD |= (1 << PD5);
//     DDRD |= (1 << PD4);

//     TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM20);
//     TCCR2B = _BV(WGM22) | _BV(CS22);
//     OCR0B = 180;
    


//     while(true){
//         PORTD |= (1 << PD5);
//         PORTD |= (1 << PD4);
//     }


//     return 0;
// }



