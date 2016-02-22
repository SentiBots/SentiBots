#include <PID_v1.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <Servo.h>

MPU6050 mpu;

//Test Constants
#define IMU 0
#define SERVO 1
#define pid 2
#define MOTOR 3

//Test Mode
#define TEST pid

//Test Variables
#define PID_SPEED 1
#define MOTOR_SPEED 200
#define MOTOR_TIME 3 //Seconds

//IMU Offset
#define X_GYRO_OFFSET 220 //Default: 220
#define Y_GYRO_OFFSET 76 //Default: 76
#define Z_GYRO_OFFSET -85 //Default: -85
#define X_ACCEL_OFFSET 0 //Default: 0
#define Y_ACCEL_OFFSET 0 //Default: 0
#define Z_ACCEL_OFFSET 1788 //Default: 1788

//Servo Offset
const int SERVO_OFFSET[] = {90, 90, 90, 90};

//Servo PIN
const byte SERVO_PIN[] = {10, 11, 5, 6};

//Servo Axis
const byte SERVO_YAW[] = {0, 1};
const byte SERVO_PITCH[] = {2, 3};

//PID Constants
#define OUTPUT_LIMITS 30
#define PID_RANGE 10

//PID Variables
double yprSetpoint[3], yprInput[3], yprOutput[3];

double Kp=1.5, Ki=1, Kd=0;
PID yPID(&yprInput[0], &yprOutput[0], &yprSetpoint[0], Kp, Ki, Kd, DIRECT);
PID pPID(&yprInput[1], &yprOutput[1], &yprSetpoint[1], Kp, Ki, Kd, DIRECT);
PID rPID(&yprInput[2], &yprOutput[2], &yprSetpoint[2], Kp, Ki, Kd, DIRECT);

//Servo declaration
Servo servo[4];

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

// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Function declaration;
void IMU_setup();
void IMU_loop();
void servo_setup();
void servo_loop();
void PID_setup();
void PID_loop();
void motor_setup();
void motor_loop();

void setup() {
	Serial.begin(9600);
  servo_setup();
	switch (TEST) {
		case IMU:
			IMU_setup();
      break;
		case SERVO:
      break;
		case pid:
			PID_setup();
      break;
		case MOTOR:
			motor_setup();
      break;
		default:
			Serial.println("You screwed up somehow");
	}
}

void loop() {
	switch (TEST) {
		case IMU:
			IMU_loop();
      break;
		case SERVO:
			servo_loop();
      break;
		case pid:
			PID_loop();
      break;
		case MOTOR:
			motor_loop();
      break;
		default:
			Serial.println("STAPH");
	}
}

void IMU_setup() {
	//Setting up I2C comms
	Wire.begin();

  // join I2C bus (I2Cdev library doesn't do this automatically)
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
  mpu.setXGyroOffset(X_GYRO_OFFSET);
  mpu.setYGyroOffset(Y_GYRO_OFFSET);
  mpu.setZGyroOffset(Z_GYRO_OFFSET);
  mpu.setXAccelOffset(X_ACCEL_OFFSET);
  mpu.setYAccelOffset(Y_ACCEL_OFFSET);
  mpu.setZAccelOffset(Z_ACCEL_OFFSET); // 1688 factory default for my test chip
  
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
}

void IMU_loop() {
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
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
      
      //change then axis
      yprInput[0] = ypr[1] * 180/M_PI;
      yprInput[1] = ypr[2] * 180/M_PI;
      yprInput[2] = ypr[0] * 180/M_PI;

      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);/*
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.println(aaWorld.z);*/
  }
}

void servo_setup() {
  for (int i = 0; i < 4; i++) {
	servo[i].attach(SERVO_PIN[i]);
  }

  //Set Default/Offset
  for (int i = 0; i < 4; i++) {
	servo[i].write(SERVO_OFFSET[i]);
  }
}


int s[] = {SERVO_OFFSET[0], SERVO_OFFSET[1], SERVO_OFFSET[2], SERVO_OFFSET[3]};
const int speed = 1;
int d[] = {speed, 0, 0, 0};
int stage = 0;

void servo_loop() {
	for (int i = 0; i < 4; i++) {
		servo[i].write(s[i]);
	}
	for (int i = 0; i < 4; i++) {
		s[i] += d[i];
	}
	if (stage == 0) {
		for (int i = 0; i < 4; i++) {
			if (s[i] >= SERVO_OFFSET[i] + OUTPUT_LIMITS && d[i] == speed) {
				d[i] = -speed;
			}
			else if (s[i] <= SERVO_OFFSET[i] - OUTPUT_LIMITS && d[i] == -speed) {
				d[i] = speed;
			}
			else if (s[i] == SERVO_OFFSET[i] && d[i] == speed) {
				d[i] = 0;
				if (i < 3) {
					d[i + 1] = speed;
				}
				else {
					stage = 1;
					
					//Set valuex for Stage 1
					for (int i = 0; i < 4; i++) {
						d[i] = speed;
					}
				}
			}
		}
	}
	else if (stage == 1) {
		for (int i = 0; i < 4; i++) {
			if (s[i] >= SERVO_OFFSET[i] + OUTPUT_LIMITS && d[i] == speed) {
				d[i] = -speed;
			}
			else if (s[i] <= SERVO_OFFSET[i] - OUTPUT_LIMITS && d[i] == -speed) {
				d[i] = speed;
			}
			else if (s[i] == SERVO_OFFSET[i] && d[i] == speed) {
				d[i] = 0;
				stage = 0;

        //Set value for Stage 0
        d[0] = speed;
        for (int i = 1; i <= 3; i++) {
          d[i] = 0;
        }
			}
		}
	}
 delay(100);
}

double prevTime;

void PID_setup() {
	IMU_setup();
  prevTime = millis();
	
	yPID.SetMode(AUTOMATIC);
	pPID.SetMode(AUTOMATIC);
	rPID.SetMode(AUTOMATIC);
	yPID.SetOutputLimits(-OUTPUT_LIMITS, OUTPUT_LIMITS);
	pPID.SetOutputLimits(-OUTPUT_LIMITS, OUTPUT_LIMITS);
	rPID.SetOutputLimits(-OUTPUT_LIMITS, OUTPUT_LIMITS);
	for (int i = 0; i < 3; i++) {
    yprSetpoint[i] = 0;
	}
}

void PID_loop() {
  if (millis() - prevTime >= 100) {
    IMU_loop();
    
    if (yprInput[0] >= PID_RANGE || yprInput[0] <= -PID_RANGE) {
      yPID.Compute();
    }
    if (yprInput[1] >= PID_RANGE || yprInput[1] <= -PID_RANGE) {
      pPID.Compute();
    }
    
    if (yprOutput[0] > yprOutput[1]) {
      //Servo Act
      //Yaw PID
      servo[SERVO_YAW[0]].write(SERVO_OFFSET[SERVO_YAW[0]] + yprOutput[0]);
      servo[SERVO_YAW[1]].write(SERVO_OFFSET[SERVO_YAW[1]] - yprOutput[0]);
    }
    else {
      //Servo Act
      //Pitch PID
      servo[SERVO_PITCH[0]].write(SERVO_OFFSET[SERVO_PITCH[0]] + yprOutput[1]);
      servo[SERVO_PITCH[1]].write(SERVO_OFFSET[SERVO_PITCH[1]] - yprOutput[1]);
    }
    
    rPID.Compute();

    prevTime = millis();

  }
}

void motor_setup() {
}

void motor_loop() {
}
