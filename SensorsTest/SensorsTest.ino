//#define SENSOR_PC3_PIN 17
#define LED_PIN 5
#define LED_WARNING 12

// Please comment/uncomment ONLY of the next 2 lines to select IR or US sensor
//#define IR_SENSOR
#define US_SENSOR

#if defined(IR_SENSOR) && defined(US_SENSOR) || !defined(IR_SENSOR) && !defined(US_SENSOR)
#error Both sensors defined, please only use one. (comment out the other)
#endif

#if defined(IR_SENSOR)			// led BRIGHTNESS
#define ADC_VALUE_AT_15CM 579 	// 255
#define ADC_VALUE_AT_40CM 306	// 0
#endif

#if defined(US_SENSOR)
#define ADC_VALUE_AT_15CM 11	//	255
#define ADC_VALUE_AT_40CM 29	//	0
#endif

void setup()
{
  Serial.begin(9600);

  Serial.println("Initializing ADC...");

  // PC3/A3 direction input
  DDRC |= (0 << PC3);

  // Info:  AVcc = 5V => REFS[1:0] = 0b01 => GOOD CHOICE FOR IR/US sensors
  //        AREF = UNCONNECTED  X
  //        1.1 Voltage reference => REFS[1:0] = 0b11 (NOT GOOD ENOUGH)
  //        REFS[1:0] = 0b00 -> AREF as reference... but unconnected so no
  ADMUX |= (1 << REFS0);

  // ADLAR: left adjust ADC result (8-bit result in ADCH reg) 
  ADMUX |= (1 << ADLAR);

  // 0x03: ADC analog pin input (ADC3 / PC3)
  ADMUX |= (0x03);

  // divide by 2 prescaler to ADC clock (minimum)
  ADCSRA |= (1 << ADPS0);

  // Enable ADC
  ADCSRA |= (1 << ADEN);

  Serial.print("Starting first conversion and ADC init... ");
  
  // start first conversion and ADC init (25 clks first time)
  ADCSRA |= (1 << ADSC);

  // wait for first conversion to finish
  while (ADCSRA & (1 << ADSC));
  
  Serial.println("Done!");
  Serial.print("Initializing LEDs... ");

  pinMode(LED_PIN, OUTPUT);
  pinMode(LED_WARNING, OUTPUT);

  digitalWrite(LED_PIN, HIGH);
  digitalWrite(LED_WARNING, HIGH);
  
  Serial.println("Done!");
  Serial.println("ADC measurements:");
}

void loop()
{
  delay(10);
  
  ADCSRA |= (1 << ADSC);        // Start ADC conversion
  while (ADCSRA & (1 << ADSC)); // wait for conversion to finish

  unsigned long adcL = ADCL;  // ls bits in ADCL bit 7 and 6 (ADC_RESULT[1:0])
  unsigned long adcH = ADCH;  // msbyte in ADCH (ADC_RESULT[9:2]
  unsigned long adc_val = (adcH << 2) | (adcL >> 6);  // get result from ADCH and ADCL registers

  double a = (40.0 - 15.0) / (ADC_VALUE_AT_40CM - ADC_VALUE_AT_15CM);
  double b = 40.0 - a * ADC_VALUE_AT_40CM;
  double dist = a * ((double) adc_val) + b;

  Serial.print("adc reading: ");
  Serial.print(adc_val);
  Serial.print(" - dist (cm): ");
  Serial.println(dist);

  if (dist <= 15)
  {
    digitalWrite(LED_WARNING, LOW);
    digitalWrite(LED_PIN, LOW);
  }
  else if (dist > 40.0)
  {
    digitalWrite(LED_WARNING, LOW);
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_WARNING, HIGH);
    byte led_bright = (255 - 0) / (40.0 - 15.0) * dist - 153;
    analogWrite(LED_PIN, led_bright);
  }  
}
