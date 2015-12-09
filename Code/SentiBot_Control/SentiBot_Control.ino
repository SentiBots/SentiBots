
/* ============================================
I2Cdev device library code is placed under the MIT license
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/
#include <I2Cdev.h>
#include <MPU9250.h>
#include <Wire.h>
#include <Servo.h>

MPU9250 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;

float pitch=0;
float pitchAcc;
float P_CompCoeff= 0.98;


Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;

void setup() {
  Wire.begin();

  Serial.begin(19200);

  Serial.println("Initialising I2C devices");
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  
  servo1.attach(3);
  servo2.attach(5);
  servo3.attach(9);
  servo4.attach(10);
}

void loop() {
    accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);

    Serial.print("a/g/m:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.print(gz); Serial.print("\t");
    Serial.print(mx); Serial.print("\t");
    Serial.print(my); Serial.print("\t");
    Serial.println(mz);
}


void ComplementaryFilter(int ax,int ay,int az,int gy,int gz) {
 long squaresum=(long)ay*ay+(long)az*az;
 pitch+=((-gy/32.8f)*(delta_t/1000000.0f)); 
 pitchAcc =atan(ax/sqrt(squaresum))*RAD_TO_DEG;
 pitch =P_CompCoeff*pitch + (1.0f-P_CompCoeff)*pitchAcc;
}

void Pitch(int pos){
    servo1.write(90+pos);
    servo2.write(90-pos);
  }

void Roll(int pos){
    servo1.write(90+pos);
    servo2.write(90-pos);
  }

void Yaw(int pos){
     servo1.write(90+pos);
     servo2.write(90+pos);
     servo3.write(90+pos);
     servo4.write(90+pos);
  }



