#include <PID_v1.h>
#include <I2Cdev.h>
#include <Wire.h>
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

double prevY;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


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
double xSetpoint, xInput, xOutput, xRealOutput;
double ySetpoint, yInput, yOutput, yRealOutput;
double zSetpoint, zInput, zOutput, zRealOutput;

double Kp=2, Ki=0, Kd=0;
PID xPID(&xInput, &xOutput, &xSetpoint, Kp, Ki, Kd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, Kp, Ki, Kd, DIRECT);
PID zPID(&zInput, &zOutput, &zSetpoint, Kp, Ki, Kd, DIRECT);

//Servo declaration
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

//State Variables
int state = OFF;

bool negative = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
   Serial.begin(115200);
  //Setting up I2C comms
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

  servo1.attach(5);
  servo2.attach(6);
  servo3.attach(11);
  servo4.attach(10);

  //Set Default
  servo1.write(90);
  servo2.write(90);
  servo3.write(90);
  servo4.write(90);

  //Other setups
  state = HOVER;
  getInput();
  xPID.SetMode(AUTOMATIC);
  yPID.SetMode(AUTOMATIC);
  zPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-40, 40);
  yPID.SetOutputLimits(-40, 40);
  zPID.SetOutputLimits(-40, 40);
  xSetpoint = 0;
  ySetpoint = 0;
  zSetpoint = 0;
  
  prevTime = millis();
}

void loop() {
    //Tilt while lift off/hover/landing
    getInput();
    xPID.Compute();
    xRealOutput = xOutput + 90;
    yPID.Compute();
    yRealOutput = yOutput + 90;
    ServoController();
}

void ServoController(){
  servo1.write(xRealOutput);
  servo2.write(180-yRealOutput);
  servo3.write(180-xRealOutput);
  servo4.write(yRealOutput);
  Serial.print(yInput);
  Serial.print(",");
  Serial.print(yRealOutput);
  Serial.println("");
}
void getInput() {
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (fifoCount < packetSize) {
       fifoCount = mpu.getFIFOCount();
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            //yInput=ypr[0] * 180/M_PI;
            yInput=ypr[1] * 180/M_PI;
            xInput=ypr[2] * 180/M_PI;
  }
}

