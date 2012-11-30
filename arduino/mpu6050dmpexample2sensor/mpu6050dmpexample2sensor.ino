
// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
// 2012-06-20 - improved FIFO overflow handling and simplified read process
// 2012-06-19 - completely rearranged DMP initialization code and simplification
// 2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
// 2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
// 2012-06-05 - add gravity-compensated initial reference frame acceleration output
// - add 3D math helper file to DMP6 example sketch
// - add Euler output and Yaw/Pitch/Roll output formats
// 2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
// 2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
// 2012-05-30 - basic DMP initialization working

/* ============================================
 I2Cdev device library code is placed under the MIT license
 Copyright (c) 2012 Jeff Rowberg
 
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
// Libraries

#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"


MPU6050 mpu;
MPU6050 mpu2;

/* =========================================================================
 NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
 depends on the MPU-6050s' INT pins being connected to the Arduino's
 external interrupt #0 and #1 pins. On the Arduino Uno and Mega 2560, this is
 digital I/O pins 2 and 3.
 * ========================================================================= */



// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
bool dmp2Ready = false;
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t mpu2IntStatus; 
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint8_t devStatus2;
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint16_t fifoCount2;
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
VectorInt16 gyro;
VectorInt16 gyro2;
Quaternion q; // [w, x, y, z] quaternion container
Quaternion q2; // [w, x, y, z] quaternion container
VectorInt16 aa; // [x, y, z] accel sensor measurements
VectorInt16 aaReal; // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z] gravity vector
VectorFloat gravity2; // [x, y, z] gravity vector
float euler[3]; // [psi, theta, phi] Euler angle container
float euler2[3]; // [psi, theta, phi] Euler angle container
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
float ypr2[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

// Serial request var
int readTrigger=0;


// ================================================================
// === INTERRUPT DETECTION ROUTINE ===
// ================================================================

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
volatile bool mpu2Interrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}
void dmp2DataReady() {
  mpu2Interrupt = true;
}



// ================================================================
// === INITIAL SETUP ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(115200);
  while (!Serial.available());

  // initialize device
  Serial.println("Initializing I2C devices...");
  mpu=MPU6050(0x69);
  mpu2=MPU6050(0x68);

  mpu.initialize();
  mpu2.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // wait for ready
  Serial.println("\nSend any character to begin DMP programming and demo: ");
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()); // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure DMP1
  Serial.println("Initializing DMP1...");
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP1...");
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println("Enabling interrupt detection (Arduino external interrupt 0)...");
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP ready! Waiting for first interrupt...");
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP1 Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }


  // load and configure DMP2
  Serial.println("Initializing DMP2...");
  devStatus2 = mpu2.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus2 == 0) {
    // turn on the DMP, now that it's ready
    Serial.println("Enabling DMP2...");
    mpu2.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println("Enabling interrupt detection (Arduino external interrupt 1)...");
    attachInterrupt(1, dmp2DataReady, RISING);
    mpu2IntStatus = mpu2.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println("DMP2 ready! Waiting for first interrupt...");
    dmp2Ready = true;
  } 
  else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP2 Initialization failed (code ");
    Serial.print(devStatus2);
    Serial.println(")");
  }

}

// ================================================================
// === MAIN PROGRAM LOOP ===
// ================================================================

void loop() {


  // if programming failed, don't try to do anything
  if (!dmpReady || !dmp2Ready) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize && !mpu2Interrupt && fifoCount2 < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }
  // reset interrupt flags and get INT_STATUS bytes
  mpuInterrupt = false;
  mpu2Interrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  mpu2IntStatus = mpu2.getIntStatus();
  // get current FIFO count

  fifoCount = mpu.getFIFOCount();
  fifoCount2 = mpu2.getFIFOCount();
  // check for overflow (this should never happen unless our code is too inefficient)
  if (fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println("FIFO overflow!");
  }
  if (fifoCount2 == 1024) {
    // reset so we can continue cleanly
    mpu2.resetFIFO();
    Serial.println("FIFO2 overflow!");
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } 

  if(Serial.available()>0){
    readTrigger= Serial.read();
  }

  // wait for correct available data length, should be a VERY short wait
  while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);
  mpu.resetFIFO();

  if(readTrigger==1){
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    mpu.dmpGetGyro(&gyro, fifoBuffer);
    Serial.print("ypr1\t");
    Serial.print(ypr[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180/M_PI);
    Serial.print("\t gyro1\t");
    Serial.print(gyro.x);
    Serial.print("\t");
    Serial.print(gyro.y);
    Serial.print("\t");
    Serial.println(gyro.z);
    readTrigger=0;
  }

  while (fifoCount2 < packetSize) fifoCount2 = mpu2.getFIFOCount();

  // read a packet from FIFO

  mpu2.getFIFOBytes(fifoBuffer, packetSize);
  mpu2.resetFIFO();

  if(readTrigger==2){
    mpu2.dmpGetQuaternion(&q2, fifoBuffer);
    mpu2.dmpGetGravity(&gravity2, &q2);
    mpu2.dmpGetYawPitchRoll(ypr2, &q2, &gravity2);
    mpu2.dmpGetGyro(&gyro2, fifoBuffer);
    Serial.print("ypr2\t");
    Serial.print(ypr2[0] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr2[1] * 180/M_PI);
    Serial.print("\t");
    Serial.print(ypr2[2] * 180/M_PI);
    Serial.print("\t gyro2\t");
    Serial.print(gyro2.x);
    Serial.print("\t");
    Serial.print(gyro2.y);
    Serial.print("\t");
    Serial.println(gyro2.z);
    readTrigger=0;
  }
}








