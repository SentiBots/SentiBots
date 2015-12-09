#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU9250.h>
#include <Wire.h>
#include <Servo.h>
#include <L3G.h>
#include <LSM303.h>

int16_t xAcc, yAcc, zAcc, xGyr, yGyr, zGyr, mx, my, mz;
MPU9250 accelgyro;

char report[80];

L3G gyro;
LSM303 compass;

void setup() {
  // put your setup code here, to run once:

 Wire.begin();
  
  Serial.begin(9600);

  gyro.init();
  gyro.enableDefault();

  compass.init();
  compass.enableDefault();
}

void loop() {
  // put your main code here, to run repeatedly:

  gyro.read();

  compass.read();

  snprintf(report, sizeof(report), "A: %6d",
    compass.a.x + compass.a.y + compass.a.z);
  Serial.println(report);

  delay(100);

}
