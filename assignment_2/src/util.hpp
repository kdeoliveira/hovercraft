#pragma once

#include "MPU9250.h"

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




void print_info_imu(MPU9250 imu){
    Serial.println("AK8963 mag biases (mG)");
    Serial.println(imu.magBias[0]);
    Serial.println(imu.magBias[1]);
    Serial.println(imu.magBias[2]);

    Serial.println("AK8963 mag scale (mG)");
    Serial.println(imu.magScale[0]);
    Serial.println(imu.magScale[1]);
    Serial.println(imu.magScale[2]);

    Serial.println("Magnetometer:");
    Serial.print("X-Axis sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[0], 2);
    Serial.print("Y-Axis sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[1], 2);
    Serial.print("Z-Axis sensitivity adjustment value ");
    Serial.println(imu.factoryMagCalibration[2], 2);
}



void print_self_test_imu(MPU9250 imu){
        // Start by performing self test and reporting values
    imu.MPU9250SelfTest(imu.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(imu.selfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(imu.selfTest[5], 1);
    Serial.println("% of factory value");
}


void print_calibration_imu(MPU9250 imu){
  Serial.print("X-Axis factory sensitivity adjustment value ");
      Serial.println(imu.factoryMagCalibration[0], 2);
      Serial.print("Y-Axis factory sensitivity adjustment value ");
      Serial.println(imu.factoryMagCalibration[1], 2);
      Serial.print("Z-Axis factory sensitivity adjustment value ");
      Serial.println(imu.factoryMagCalibration[2], 2);
}