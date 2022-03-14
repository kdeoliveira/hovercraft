#include "quaternionFilters.h"
#include <Servo.h>
#include <util.hpp>

#define SerialDebug true

#define LED_PIN 5
#define LED_WARNING 12

#define I2Cclock 400000
#define I2Cport Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0

MPU9250 mpu_imu(MPU9250_ADDRESS, I2Cport, I2Cclock);

Servo myservo; // create servo object to control a servo

void setup()
{

  Serial.begin(38400);
  myservo.attach(9); // attaches the servo on pin 9 to the servo object
  while (!Serial);

  Wire.begin();

  pinMode(LED_WARNING, OUTPUT);

  pinMode(LED_PIN, OUTPUT);

  byte c = mpu_imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) // WHO_AM_I should always be 0x71
  {
    Serial.println(F("MPU9250 is online..."));

    print_self_test_imu(mpu_imu);

    // Calibrate gyro and accelerometers, load biases in bias registers
    mpu_imu.calibrateMPU9250(mpu_imu.gyroBias, mpu_imu.accelBias);

    // =====================
    // Setting and initializing IMU
    // =====================

    // Setting the gyro full scale range
    mpu_imu.Gscale = MPU9250::GFS_2000DPS;
    // Setting the accelerator full scale range
    mpu_imu.Ascale = MPU9250::AFS_16G;

    mpu_imu.initMPU9250();

    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = mpu_imu.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
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
    mpu_imu.initAK8963(mpu_imu.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (SerialDebug)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(mpu_imu.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(mpu_imu.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(mpu_imu.factoryMagCalibration[2], 2);
    }

    // Get sensor resolutions, only need to do this once
    mpu_imu.getAres();
    mpu_imu.getGres();
    mpu_imu.getMres();

    // =====================
    // Setting and initializing IMU
    // =====================

    // Setting sample rate
    // sample rates possible are 32 kHz (GYRO_CONFIG[1:0]=0x10), 8 kHz (CONFIG[2:0]=0x00), or 1 kHz (CONFIG[2:0]=0x03)
    mpu_imu.writeByte(mpu_imu._I2Caddr, CONFIG, 0x06);
    uint8_t gyro_config = mpu_imu.readByte(mpu_imu._I2Caddr, GYRO_CONFIG);
    // F_CHOICE[1:0]
    // 00 => Allow CONFIG TO BE READ
    // 1x => Uses the GYRO_CONFIG [1:0] bits
    gyro_config &= ~0x02;
    mpu_imu.writeByte(mpu_imu._I2Caddr, GYRO_CONFIG, gyro_config);

    if (SerialDebug)
    {
      print_info_imu(mpu_imu);
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

  if (mpu_imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    mpu_imu.readAccelData(mpu_imu.accelCount); // Read the x/y/z adc values

    mpu_imu.ax = (float)mpu_imu.accelCount[0] * mpu_imu.aRes; // - mpu_imu.accelBias[0];
    mpu_imu.ay = (float)mpu_imu.accelCount[1] * mpu_imu.aRes; // - mpu_imu.accelBias[1];
    mpu_imu.az = (float)mpu_imu.accelCount[2] * mpu_imu.aRes; // - mpu_imu.accelBias[2];

    mpu_imu.readGyroData(mpu_imu.gyroCount); // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    mpu_imu.gx = (float)mpu_imu.gyroCount[0] * mpu_imu.gRes;
    mpu_imu.gy = (float)mpu_imu.gyroCount[1] * mpu_imu.gRes;
    mpu_imu.gz = (float)mpu_imu.gyroCount[2] * mpu_imu.gRes;

    mpu_imu.readMagData(mpu_imu.magCount); // Read the x/y/z adc values

    mpu_imu.mx = (float)mpu_imu.magCount[0] * mpu_imu.mRes * mpu_imu.factoryMagCalibration[0] - mpu_imu.magBias[0];
    mpu_imu.my = (float)mpu_imu.magCount[1] * mpu_imu.mRes * mpu_imu.factoryMagCalibration[1] - mpu_imu.magBias[1];
    mpu_imu.mz = (float)mpu_imu.magCount[2] * mpu_imu.mRes * mpu_imu.factoryMagCalibration[2] - mpu_imu.magBias[2];
  }
  mpu_imu.updateTime();
  // https://forum.arduino.cc/t/fatal-bug-in-sparkfun-and-kris-winers-mpu9250-code/619810
  MahonyQuaternionUpdate(mpu_imu.ax, mpu_imu.ay, mpu_imu.az, mpu_imu.gx * DEG_TO_RAD,
                         mpu_imu.gy * DEG_TO_RAD, mpu_imu.gz * DEG_TO_RAD, mpu_imu.my,
                         mpu_imu.mx, -mpu_imu.mz, mpu_imu.deltat);

  // CALCULATE
  mpu_imu.delt_t = millis() - mpu_imu.count;

  mpu_imu.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() * *(getQ() + 3)), *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));

  mpu_imu.yaw *= RAD_TO_DEG;
  mpu_imu.yaw -= 8.5;

  mpu_imu.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() * *(getQ() + 2)));
  mpu_imu.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) * *(getQ() + 3)), *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
  mpu_imu.pitch *= RAD_TO_DEG;
  mpu_imu.roll *= RAD_TO_DEG;

  analogWrite(LED_PIN, 255 - map(1000 * abs(mpu_imu.ax), 1000 * 0.01, 1000, 0, 255));

  if (mpu_imu.yaw + 180 < 180 || mpu_imu.yaw + 180 > 0)
  {
    int result_servo = (int)map(floor(mpu_imu.yaw) + 180, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
    if (result_servo == 0)
    {
      digitalWrite(LED_WARNING, LOW);
      Serial.print("Out of bound\r");
    }
    else
    {
      digitalWrite(LED_WARNING, HIGH);
      myservo.writeMicroseconds(result_servo);
    }
  }

  if (mpu_imu.delt_t > 1000) // 1s for serial print
  {
    if (SerialDebug)
    {
      Serial.print("CONFIG register: ");
      Serial.print(
          mpu_imu.readByte(mpu_imu._I2Caddr, CONFIG), BIN);

      Serial.print(" - ");
      Serial.print("GYRO_CONFIG register: ");
      Serial.print(
          mpu_imu.readByte(mpu_imu._I2Caddr, GYRO_CONFIG), BIN);
      Serial.print(
          " : ");
      Serial.print("ax = ");
      Serial.print((int)1000 * mpu_imu.ax);
      Serial.print(" mg  [");

      Serial.print(mpu_imu.ax, 4);
      Serial.print("g]");

      Serial.print("\tYaw: ");
      Serial.print(mpu_imu.yaw + 180, 2);

      Serial.print("\tPitch: ");
      Serial.print(mpu_imu.pitch, 2);

      Serial.print("\tRoll: ");
      Serial.print(mpu_imu.roll, 2);

      Serial.print("\trate = ");
      Serial.print((float)mpu_imu.sumCount / mpu_imu.sum, 2);
      Serial.println(" Hz");

      mpu_imu.count = millis();
      mpu_imu.sumCount = 0;
      mpu_imu.sum = 0;
    }
  }
}