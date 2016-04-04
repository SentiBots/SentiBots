#include <Servo.h>
#define TARGET_BOARD_IDENTIFIER SENTIBOTS_IDENTIFIER
#include "serial_msp.h"
#include <PID_v1.h>
#include <I2Cdev.h>
#include <Wire.h>
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

//Filter Variables
double prevTime;

//PID Variables
double xSetpoint, xInput, xOutput, xRealOutput;
double ySetpoint, yInput, yOutput, yRealOutput;
double zSetpoint, zInput, zOutput, zRealOutput;

double Kp=5, Ki=0, Kd=0;
PID xPID(&xInput, &xOutput, &xSetpoint, Kp, Ki, Kd, DIRECT);
PID yPID(&yInput, &yOutput, &ySetpoint, Kp, Ki, Kd, DIRECT);
PID zPID(&zInput, &zOutput, &zSetpoint, Kp, Ki, Kd, DIRECT);

bool negative = false;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

//unsigned int cycleDelay = 5;

Servo m1;
//Servo m2;
Servo s1;
Servo s2;
Servo s3;
Servo s4;



mspPort_t currentPort;

void dmpDataReady() {
    mpuInterrupt = true;
}

static void serialize8(uint8_t a) {
  Serial.write(a);
  currentPort.checksum ^= a;
}

static void serialize16(uint16_t a) {
  serialize8((uint8_t)(a >> 0));
  serialize8((uint8_t)(a >> 8));
}

static void serialize32(uint32_t a) {
  serialize16((uint16_t)(a >> 0));
  serialize16((uint16_t)(a >> 16));
}

static uint8_t read8(void) {
  return currentPort.inBuf[currentPort.indRX++] & 0xff;
}

static uint16_t read16(void) {
  uint16_t t = read8();
  t += (uint16_t)read8() << 8;
  return t;
}

static uint32_t read32(void) {
  uint32_t t = read16();
  t += (uint32_t)read16() << 16;
  return t;
}

static void headSerialResponse(uint8_t err, uint8_t responseBodySize) {
  serialize8('$');
  serialize8('M');
  serialize8(err ? '!' : '>');
  currentPort.checksum = 0;
  serialize8(responseBodySize);
  serialize8(currentPort.cmdMSP);
}

static void headSerialReply(uint8_t responseBodySize) {
  headSerialResponse(0, responseBodySize);
}

static void headSerialError(uint8_t responseBodySize) {
  headSerialResponse(1, responseBodySize);
}

bool processOutCommand(uint8_t cmdMSP) {
  switch (cmdMSP) {
    case 0:
      return false;
      break;
    default:
      return false;
  }
  return true;
}

static bool processInCommand(void) {
  switch (currentPort.cmdMSP) {
    case MSP_SET_RAW_COAX:
      m1.writeMicroseconds(read16());
      read16();
      xSetpoint = read16() - 90;
      read16();
      read16();
      ySetpoint = read16() - 90;
      break;
    default:
      return false;
  }
  headSerialReply(0);
  return true;
}

void tailSerialReply(void) {
  serialize8(currentPort.checksum);
}

void mspProcessReceivedCommand() {
  if (!(processOutCommand(currentPort.cmdMSP) || processInCommand())) {
    headSerialError(0);
  }
  tailSerialReply();
  currentPort.c_state = IDLE;
}

static bool mspProcessReceivedData(uint8_t c) {
  if (currentPort.c_state == IDLE) {
    if (c == '$') {
      currentPort.c_state = HEADER_START;
    } else {
      return false;
    }
  } else if (currentPort.c_state == HEADER_START) {
    currentPort.c_state = (c == 'M') ? HEADER_M : IDLE;
  } else if (currentPort.c_state == HEADER_M) {
    currentPort.c_state = (c == '<') ? HEADER_ARROW : IDLE;
  } else if (currentPort.c_state == HEADER_ARROW) {
    if (c > MSP_PORT_INBUF_SIZE) {
      currentPort.c_state = IDLE;
    } else {
      currentPort.dataSize = c;
      currentPort.offset = 0;
      currentPort.checksum = 0;
      currentPort.indRX = 0;
      currentPort.checksum ^= c;
      currentPort.c_state = HEADER_SIZE;
    }
  } else if (currentPort.c_state == HEADER_SIZE) {
    currentPort.cmdMSP = c;
    currentPort.checksum ^= c;
    currentPort.c_state = HEADER_CMD;
  } else if (currentPort.c_state == HEADER_CMD && currentPort.offset < currentPort.dataSize) {
    currentPort.checksum ^= c;
    currentPort.inBuf[currentPort.offset++] = c;
  } else if (currentPort.c_state == HEADER_CMD && currentPort.offset >= currentPort.dataSize) {
    //if (currentPort.checksum == c) {
    currentPort.c_state = COMMAND_RECEIVED;
    //} else {
    //  currentPort.c_state = IDLE;
    //}
  }
  return true;
}


void setup() {
  Serial.begin(115200);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        mpuIntStatus = mpu.getIntStatus();
        
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
    }
    // PD3 PD5 PD6 PB2 PB3
    // 3   5   6   10  11
    m1.attach(3);
    //m2.attach(10); // 9
    s1.attach(5);
    s2.attach(6);
    s3.attach(11);
    s4.attach(10);
    m1.writeMicroseconds(1000);
    //m2.writeMicroseconds(1000);
    s1.write(90);
    s2.write(90);
    s3.write(90);
    s4.write(90);
    //memset(currentPort, 0, sizeof(mspPort_t));
    currentPort.c_state = IDLE;

    //Other setups
    getInput();
    xPID.SetMode(AUTOMATIC);
    yPID.SetMode(AUTOMATIC);
    zPID.SetMode(AUTOMATIC);
    xPID.SetOutputLimits(-40, 40);
    yPID.SetOutputLimits(-40, 40);
    zPID.SetOutputLimits(-40, 40);
    xPID.SetControllerDirection(REVERSE);
    yPID.SetControllerDirection(REVERSE);
    zPID.SetControllerDirection(REVERSE);
    xSetpoint = 0;
    ySetpoint = 0;
    zSetpoint = 0;
  
    prevTime = millis();
}

void loop() {
  while (Serial.available() > 0) {

    uint8_t c = Serial.read();
    //bool consumed = mspProcessReceivedData(c);
    mspProcessReceivedData(c);

    if (currentPort.c_state == COMMAND_RECEIVED) {
      mspProcessReceivedCommand();
      break; // process one command at a time so as not to block.
    }

  }

  getInput();
  xPID.Compute();
  xRealOutput = xOutput + 90;
  yPID.Compute();
  yRealOutput = yOutput + 90;
  ServoController();
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
        //Serial.println(F("FIFO overflow!"));

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
            zInput=ypr[0] * 180/M_PI;
            yInput=ypr[1] * 180/M_PI;
            xInput=ypr[2] * 180/M_PI;
            xInput=xInput-2.5;
            yInput=yInput-3;
  }
}

void ServoController(){
  s1.write(xRealOutput);
  s2.write(yRealOutput);
  s3.write(180-xRealOutput);
  s4.write(180-yRealOutput);
  Serial.print(xInput);
  Serial.print(",");
  Serial.print(yInput);
  Serial.println("");
}
