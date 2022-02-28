#include <Arduino.h>


byte led_value = 0;
unsigned long recv_buf = 0;
unsigned long length_mm = 0;

#define TRIG_PIN 3
#define ECHO_PIN 2
#define LED_PIN 5
#define LED_WARNING 4



uint16_t high_pulse, low_pulse;


long map(long x, long min, long max, long out_min, long out_max){
  if(x <= min){
    return out_min;
  }
  else if(x >= max){
    return out_min;
  }

  return (x - min)*(out_max -out_min) / (max - min) + out_min;
}

const char whitespace[5] = {0x20,0x20,0x20, 0x20,0x20};


void setup(void)
{
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);

}

void loop()
{
  Serial.flush();

  

  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(50);
  digitalWrite(TRIG_PIN, LOW);

  recv_buf = pulseIn (ECHO_PIN, HIGH);
  if(recv_buf > 1 && recv_buf < 60000){
    length_mm = recv_buf*0.34/2;
    Serial.print("length (mm): ");
    Serial.print(length_mm, DEC);
    Serial.print(whitespace);
    Serial.write(0x0D);

    led_value = map(length_mm, 150, 600,0, 255);
    
    analogWrite(LED_PIN, led_value);

    digitalWrite(LED_WARNING, led_value == 0);
  }
  delay(10);
}