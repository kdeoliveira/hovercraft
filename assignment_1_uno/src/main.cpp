#include <Arduino.h>
#include "common.h"

byte led_value = 0;
unsigned long recv_buf = 0;
unsigned long length_mm = 0;

#define TRIG_PIN 3
#define ECHO_PIN 7
#define LED_PIN 5
#define LED_WARNING 4
#define BTN_PIN 2



#ifdef __AVR_ATmega328P__
    #define IR_PIN A3
#else
    #define IR_PIN A0
#endif

volatile bool btn_state = false;


void interrupt_change_state(void){
  btn_state = !btn_state;
}


int ir_data;

void setup(void)
{
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);
  pinMode(BTN_PIN, INPUT_PULLUP);
  pinMode(IR_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BTN_PIN), interrupt_change_state, FALLING);

}

void loop()
{
  if(btn_state){
    ir_data = analogRead(IR_PIN);
    Serial.println(106500.8 * pow(ir_data,-0.935) - 10);

    delay(10);

  }else{
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(50);
    digitalWrite(TRIG_PIN, LOW);

    recv_buf = pulseIn(ECHO_PIN, HIGH);
    if(recv_buf > 1 && recv_buf < 60000){
      length_mm = recv_buf*0.34/2;
      // Serial.print("length (mm): ");
      Serial.println(length_mm, DEC);
      // Serial.print(hovercraft::whitespace);
      // Serial.write(CARRIAGE_RETURN);
      

      led_value = hovercraft::map(length_mm, 150, 600,0, 255);
      
      analogWrite(LED_PIN, led_value);

      digitalWrite(LED_WARNING, led_value == 0);
      delay(10);

    }
  }
}

