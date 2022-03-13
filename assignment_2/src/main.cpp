#include "quaternionFilters.h"
#include "MPU9250.h"
#include <Servo.h>

#define SerialDebug true

#define LED_PIN 5
#define LED_WARNING 12

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 myIMU(MPU9250_ADDRESS, I2Cport, I2Cclock);



Servo myservo; // create servo object to control a servo


long map(long x, long min, long max, long out_min, long out_max)
{
  if (x <= min)
  {
    return 0;
  }
  else if (x >= max)
  {
    return 0;
  }

  return (x - min) * (out_max - out_min) / (max - min) + out_min;
}

void setup()
{



  Serial.begin(38400);
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
  while (!Serial);

  Wire.begin();

  pinMode(LED_WARNING, OUTPUT);

  pinMode(LED_PIN, OUTPUT);


  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5], 1);
    Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);


    // =====================
    // Setting and initializing IMU
    // =====================

    //Setting the gyro full scale range
    myIMU.Gscale = MPU9250::GFS_2000DPS; 
    //Setting the accelerator full scale range
    myIMU.Ascale = MPU9250::AFS_16G;

    myIMU.initMPU9250();




    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();

    // =====================
    // Setting and initializing IMU
    // =====================

    //Setting sample rate
    //sample rates possible are 32 kHz (GYRO_CONFIG[1:0]=0x10), 8 kHz (CONFIG[2:0]=0x00), or 1 kHz (CONFIG[2:0]=0x03)
    myIMU.writeByte(myIMU._I2Caddr, CONFIG, 0x03);
    uint8_t gyro_config = myIMU.readByte(myIMU._I2Caddr,GYRO_CONFIG);
    // F_CHOICE[1:0]
    // 00 => Allow CONFIG TO BE READ
    // 1x => Uses the GYRO_CONFIG [1:0] bits
    gyro_config &= ~0x02;
    myIMU.writeByte(myIMU._I2Caddr, GYRO_CONFIG, gyro_config);



    //    delay(2000); // Add delay to see results before serial spew of data

    if (SerialDebug)
    {
          Serial.println("AK8963 mag biases (mG)");
    Serial.println(myIMU.magBias[0]);
    Serial.println(myIMU.magBias[1]);
    Serial.println(myIMU.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(myIMU.magScale[0]);
    Serial.println(myIMU.magScale[1]);
    Serial.println(myIMU.magScale[2]);

      Serial.println("Magnetometer:");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(myIMU.factoryMagCalibration[2], 2);
    }

  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }
}


void loop()
{

  


  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount); // Read the x/y/z adc values

    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount); // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount); // Read the x/y/z adc values

    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  }
  myIMU.updateTime();
  //https://forum.arduino.cc/t/fatal-bug-in-sparkfun-and-kris-winers-mpu9250-code/619810
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, -myIMU.mz, myIMU.deltat);
  
  // CALCULATE
  myIMU.delt_t = millis() - myIMU.count;



  myIMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));

  myIMU.yaw *= RAD_TO_DEG;
  myIMU.yaw -= 8.5;

  myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                * *(getQ()+2)));
  myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                * *(getQ()+3));
  myIMU.pitch *= RAD_TO_DEG;
  myIMU.roll *= RAD_TO_DEG;

    analogWrite(LED_PIN, 255 - map(1000*abs(myIMU.ax), 1000*0.01, 1000, 0, 255));

  if(myIMU.yaw + 180 < 180 || myIMU.yaw + 180 > 0){
    int result_servo = (int)map( floor( myIMU.yaw) + 180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    if(result_servo == 0){
        digitalWrite(LED_WARNING, LOW);
        Serial.print("Out of bound\r");
    }else{
      digitalWrite(LED_WARNING, HIGH);
      myservo.writeMicroseconds( result_servo);
    }
  }

  if (myIMU.delt_t > 1000) //1s for serial print
  {
    if (SerialDebug)
    {
      Serial.print("ax = ");  
      Serial.print((int)1000* myIMU.ax);
      Serial.print(" mg  [");

      Serial.print(myIMU.ax, 4);
      Serial.print("g]");

      Serial.print("\tYaw: ");
      Serial.print(myIMU.yaw + 180, 2);

      Serial.print("\tPitch: ");
      Serial.print(myIMU.pitch, 2);

      Serial.print("\tRoll: ");
      Serial.print(myIMU.roll, 2);

      Serial.print("\trate = ");
      Serial.print((float)myIMU.sumCount / myIMU.sum, 2);
      Serial.println(" Hz");

      

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    }
  }
}