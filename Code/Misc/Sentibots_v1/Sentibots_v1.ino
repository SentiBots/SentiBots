#include <PID_v1.h>
#include <I2Cdev.h>
#include <MPU9250.h>
#include <Wire.h>
#include <Servo.h>
#include <L3G.h>
#include <LSM303.h>

//Library
L3G gyro;
LSM303 compass;

char report[100];

//Filter Constants
#define ACCELEROMETER_SENSITIVITY 8192.0
#define GYROSCOPE_SENSITIVITY 133.746349//65.536
#define M_PI 3.14159265359

//State Constants
#define OFF 0
#define IDLE 1
#define TAKE_OFF 2
#define HOVER 3
#define LANDING 4

//Filter Variables
double prevTime;

//PID Variables
double xSetpoint, xInput, xOutput;
double ySetpoint, yInput, yOutput;
double zSetpoint, zInput, zOutput;

double Kp=2, Ki=5, Kd=1;
PID xPID(&xInput, &xOutput, &xSetpoint, Kp, Ki, Kd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, Kp, Ki, Kd, DIRECT);
PID zPID(&zInput, &zOutput, &zSetpoint, Kp, Ki, Kd, DIRECT);

//Pin Variables
int16_t xAcc, yAcc, zAcc, xGyr, yGyr, zGyr, mx, my, mz;
int16_t acc[3], gyr[3];
int16_t prevAcc[3], prevGyr[3];
int16_t prevXAcc, prevYAcc, prevZAcc, prevXGyr, prevYGyr, prevZGyr;

//IMU declaration
MPU9250 accelgyro;

//Servo declaration
Servo servo1;
Servo servo2;
//Servo servo3;
Servo servo4;

//State Variables
int state = OFF;

void setup() {
  //Setting up I2C comms
    Wire.begin();

  Serial.begin(9600);

  if (!gyro.init()) {
    Serial.println("Failed to autodetect gyro type!");
    while (1);
  }

  gyro.enableDefault();

  compass.init();
  compass.enableDefault();
  
  servo1.attach(10);
  servo2.attach(11);
  //servo3.attach(5);
  servo4.attach(6);

  //Set Default
  servo1.write(80);
  servo2.write(80);
  //servo3.write(80);
  servo4.write(80);

  //Other setups
  state = HOVER;
  getInput();
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(40, 120);
  yPID.SetOutputLimits(40, 120);
  zPID.SetOutputLimits(40, 120);
  xSetpoint = 0;
  ySetpoint = 0;
  zSetpoint = 0;
  
  prevTime = millis();
}

void loop() {
  if (state == OFF) {
    
  }
  else if (state == IDLE) {
    
  }
  else {
    //Tilt while lift off/hover/landing
    getInput();
    getTilt();
    xPID.Compute();
    yPID.Compute();
    zPID.Compute();
    //Stabilisation Output
    //Read the docs of Servo.h
    compensate();
  }
}

/*
void getTilt() {
  float pitchAcc, rollAcc;
  double now = millis();  
  double dt = now - prevTime;
  xInput += ((float)xGyr / GYROSCOPE_SENSITIVITY) * dt;
  zInput -= ((float)zGyr / GYROSCOPE_SENSITIVITY) * dt;
 
  int forceMagnitudeApprox = abs(xAcc) + abs(yAcc) + abs(zAcc);
  if (forceMagnitudeApprox > 8192 && forceMagnitudeApprox < 32768)
  {
    pitchAcc = atan2f((float)zAcc, (float)yAcc) * 180 / M_PI;
    xInput = xInput * 0.98 + pitchAcc * 0.02;
    rollAcc = atan2f((float)xAcc, (float)yAcc) * 180 / M_PI;
    zInput = zInput * 0.98 + rollAcc * 0.02;
  }
  prevTime = now;
}
*/

void getTilt() {
  xInput += gyr[0];
  yInput += gyr[1];
}

void getInput() {
  gyro.read();

  compass.read();

  for (int i = 0; i < acc.length; i++) {
    prevAcc[i] = acc[i];
    prevGyr[i] = gyr[i];
  }
  
  acc[0] = compass.a.y;
  acc[1] = -compass.a.x;
  acc[2] = compass.a.z;

  gyr[0] = gyro.g.y;
  gyr[1] = -gyro.g.x;
  gyr[2] = gyro.g.z;

  //Filter
  for (int i = 0; i < acc.length; i++) {
    double rChange = (acc[i] - prevAcc[i]) / prevAcc[i];
    if (rChange < 1.5 && rChange > 0.001) {
      acc[i] = prevAcc[i];
    }
  }
  for (int i = 0; i < gyr.length; i++ {
    double rChange = (gyr[i] - prevGyr[i]) / prevGyr[i];
    if (rChange < 1.5 && rChange > 0.001) {
      gyr[i] = prevGyr[i];
    }
  }

  snprintf(report, sizeof(report), "G: %8d %8d %8d    A: %8d %8d %8d",
    gyr[0], gyr[1], gyr[2],
    acc[0], acc[1], acc[2]);
  Serial.println(report);

  delay(100);
}

void checkCommands() {
  if (Serial.available() > 1) {
    int command = Serial.read();
    if (command == 0) {
      state = Serial.read();
    }
    else if (command < 20) {
      
    }
  }
}

void compensate() {

servo2.write(xOutput);
servo4.write(160 - xOutput);
  
}

