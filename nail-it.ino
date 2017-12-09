#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#include <ArduinoLog.h>

#define INTERRUPT_PIN 2
#define LED_INDICATOR 7

// Led status
int bHigh = HIGH;

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ################################################################
// ###               INTERRUPT DETECTION ROUTINE                ###
// ################################################################
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  // Set monitor output BDU
  Serial.begin(115200);
  Log.begin(LOG_LEVEL_NOTICE, &Serial, true);

  // Set led inidicator for arduino stacking
  pinMode(LED_INDICATOR, OUTPUT);
  
  // Start connection and set speed to the Gyro sensor
  Wire.begin();
//  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Init the gyro
  mpu.initialize();
  // Set interrupt pin number on the arduino
  pinMode(INTERRUPT_PIN, INPUT);

  // Check the connection to the gyro
  if (mpu.testConnection()) {
      Log.notice(F("MPU6050 connection successful\n"));
  } else {
      Log.error(F("MPU6050 connection failed\n"));
  }

  // DMP init and Gyro/Accel offset *basic* reset
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(176);
  mpu.setYAccelOffset(-554);
  mpu.setZAccelOffset(2602);
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(-53);
  mpu.setZGyroOffset(111);

  if (devStatus == 0) {
      Log.trace("Enabling DMP...\n");
      mpu.setDMPEnabled(true);
      
      Log.trace("Enabling interrupt detection (Arduino external interrupt 0)...\n");
      attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();
  
      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      Log.trace("DMP ready! Waiting for first interrupt...\n");
      dmpReady = true;
  
      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Log.error("DMP Initialization failed (code %d)\n", devStatus);
  }
}

void loop() {
  // Loop led indicator change mode
  bHigh = (bHigh == HIGH) ? LOW : HIGH;
  digitalWrite(LED_INDICATOR, bHigh);
  
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  //while (!mpuInterrupt && fifoCount < packetSize) {
      // other program behavior stuff here
      // .
      // .
      // .
      // if you are really paranoid you can frequently test in between other
      // stuff to see if mpuInterrupt is true, and if so, "break;" from the
      // while() loop to immediately process the MPU data
      // .
      // .
      // .
  //}

  // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Log.error("FIFO overflow!");

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display real acceleration, adjusted to remove gravity
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetAccel(&aa, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        Log.trace("areal\t%d\t%d\t%d\n", aaReal.x, aaReal.y, aaReal.z);
        Log.trace("ypr\t%d\t%d\t%d\n", ypr[0] * 180/M_PI, ypr[1] * 180/M_PI, ypr[2] * 180/M_PI);
    }

    
}
