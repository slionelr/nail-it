#include <I2Cdev.h>
#include <Wire.h>
#include <ArduinoLog.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Adafruit_NeoPixel.h>

#define NEO_INDICATOR

// DEBUG
#define DEBUG // TODO: Delete(!) this in prod
// Gyroscope and Accelerometer
//#define PRINT_GYRO_MEASURS
//#define PRINT_GYRO_CALIB_PROCESS
//#define BYPASS_GYRO_CALIB
// Vibration
//#define PRINT_VIB

#define BDU         115200
#define WIRE_CLOCK  400000 

#define ALARM_TONE  37

#define INTERRUPT_PIN 2
#define SPEAKER_PIN   9
#define LED_INDICATOR 7
#define VIB_SENS_1    10

#ifdef NEO_INDICATOR
#define NEO_LED       8
#endif

// Led status
#define MAX_BRIGHT    32
#define MIN_BRIGHT    2
#define MAX_LOOP_NEO  5
#define RET_LOOP_NEO  (MAX_LOOP_NEO * 2)
enum LED_STATUS { RED, GREEN, BLUE };
LED_STATUS statusLed;
int brightStep = 1;
int currBright;
int loopCounter;
int intLed;
int detectIndiDelay;


// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Calibration
bool isCalibrated;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)
int16_t ax, ay, az,gx, gy, gz;

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Potentials
bool isMovingTo;

// Vibration
#define THRESH_MIN  1
#define THRESH_MAX  4
#define THRESH_START_CALIB_MIN  1500
#define THRESH_START_CALIB_MAX  2000
int preMeasur;
int measurement;

#ifdef NEO_INDICATOR
// FLORA NeoPixel status indicator
Adafruit_NeoPixel indi = Adafruit_NeoPixel(1, NEO_LED, NEO_GRB + NEO_KHZ800);
#endif

void setup() {
  // Set monitor output BDU
  Serial.begin(BDU);

  #ifdef DEBUG
  Log.begin(LOG_LEVEL_TRACE, &Serial, true);
  #else
  Log.begin(LOG_LEVEL_NOTICE, &Serial, true);
  #endif

  #ifdef NEO_INDICATOR
  // NeoPixel Indicator
  loopCounter = 0;
  currBright = MAX_BRIGHT;
  indi.begin();
  indi.setBrightness(currBright);
  #endif

  // Set led inidicator for arduino stacking
  pinMode(LED_INDICATOR, OUTPUT);
  pinMode(SPEAKER_PIN, OUTPUT);
  pinMode(VIB_SENS_1, INPUT);
  
  // Start connection and set speed to the Gyro sensor
  Wire.begin();
  Wire.setClock(WIRE_CLOCK); // 400kHz I2C clock. Comment this line if having compilation difficulties

  // Init the gyro
  mpu.initialize();

  // Check the connection to the gyro
  if (mpu.testConnection()) {
      Log.notice(F("MPU6050 connection successful\n"));
  } else {
      Log.error(F("MPU6050 connection failed\n"));
  }

  // DMP init and Gyro/Accel offset *basic* reset
  devStatus = mpu.dmpInitialize();

  
  #if defined(DEBUG) and defined(BYPASS_GYRO_CALIB)
  isCalibrated = true;
  #else
  isCalibrated = false;
  #endif

  ax_offset = -755;
  ay_offset = -396;
  az_offset = 1979;
  gx_offset = 69;
  gy_offset = -9;
  gz_offset = -39;
  
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
//    mpu.setXAccelOffset(0);
//    mpu.setYAccelOffset(0);
//    mpu.setZAccelOffset(0);
//    mpu.setXGyroOffset(0);
//    mpu.setYGyroOffset(0);
//    mpu.setZGyroOffset(0);

  if (devStatus == 0) {
      Log.trace("Enabling DMP...\n");
      mpu.setDMPEnabled(true);
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

  // Potentials setups
  isMovingTo = false;

  // Vibration setups
  preMeasur = 0;
  measurement = 0;
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) {
      return;
  }

  // Get vibration measurement first of all
  // This is for accurate pulse calculation (beacuse we are not going to use the pulseIn function)
  int vib = getVibration();
  if (vib > 0 ) { Log.notice("Vibration result: %d\n", vib); }
  if ((vib > THRESH_MIN) && (vib < THRESH_MAX)) {
    tone(SPEAKER_PIN, ALARM_TONE);
    detectIndiDelay = RET_LOOP_NEO;
    purple();
  } else {
    if (0 == --detectIndiDelay) {
      noTone(SPEAKER_PIN);
      blue();
    }
  }

  if (!isCalibrated) {
    int toCalib = getVibration();
    Log.notice(F("Vibrate hard to start calibration.\n"));
    while ((toCalib < THRESH_START_CALIB_MIN) || (toCalib > THRESH_START_CALIB_MAX)) {
      flashLed();
      if (toCalib > 0) { 
        Log.notice(F("Vibrate hard to start calibration. %d\n"), toCalib);
      }
      toCalib = getVibration();
    }
    red();
    Log.trace(F("Start calibration...\n"));
    calibrate();
    isCalibrated = true;
    blue();
  }

  flashLed();

  mpuIntStatus = mpu.getIntStatus();
  Log.verbose("MPT Status: %d\n", mpuIntStatus);

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    Log.error("FIFO overflow!\n");
    mpu.resetFIFO();

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

    float yaw = ypr[0] * 180/M_PI;
    float pitch = ypr[1] * 180/M_PI;
    float roll = ypr[2] * 180/M_PI;

    #ifdef defined(DEBUG) and defined(PRINT_GYRO_MEASURS)
    Log.trace("areal\t%d\t%d\t%d \t||| ", aaReal.x, aaReal.y, aaReal.z);
    Serial.print("YPR:\t ");
    Serial.print(yaw);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\n");
    #endif

//    Log.trace("Check hand position\n");
    if (checkPotentialPosition(yaw, pitch, roll)) {
      Log.trace("The hand is neer to the mouth!\n");

      // Check the sesmic sensor for ksisa of the nails
      tone(SPEAKER_PIN, ALARM_TONE);
    } else {
      noTone(SPEAKER_PIN);
    }
  }

  delay(15);
}

void flashLed() {
  // Loop led indicator change mode
  intLed = (intLed == LOW || intLed == 0) ? HIGH : LOW;
  digitalWrite(LED_INDICATOR, intLed);
}

/// Check if the hand position is in potenital direction to the mouth.
bool checkPotentialPosition(float yaw, float pitch, float roll) {
  bool isInDirection = false;
  if ((0.0 < yaw) && (45.0 > yaw)) {
    if ((15.0 < pitch) && (30.0 > pitch)) {
      if ((10.0 > roll) && (-10.0 < roll)) {
          isInDirection = true;
      }
    }
  }
  return isInDirection;
}

bool checkPotentialAccel(float x, float y, float z) {
    return true;
}

int getVibration() {
  int current = digitalRead(VIB_SENS_1);
  int ret = 0;
  if (current == LOW){
    if (current != preMeasur) {
      ret = measurement;
      measurement = 0;
      preMeasur = 0;
    }
  } else {
    measurement++;
  }
  preMeasur = current;
  #if defined(DEBUG) and defined(PRINT_VIB)
  if (ret > 0) { Log.trace("Vibration %d\n", ret); }
  #endif
  return ret;
}

void calibrate() {
  #ifdef DEBUG
  while(state < 3) {
  #else
  while(state < 2) {
  #endif
    if (state==0){
      Log.trace("Reading sensors for first time...\n");
      meansensors();
      state++;
    }
  
    if (state==1) {
      Log.trace("Calculating offsets...\n");
      calibration();
      state++;
    }

    #ifdef DEBUG
    if (state==2) {
      meansensors();
      Log.trace("FINISHED Calibration:\t");
      
      Serial.print(mean_ax); 
      Serial.print("\t");
      Serial.print(mean_ay); 
      Serial.print("\t");
      Serial.print(mean_az); 
      Serial.print("\t");
      Serial.print(mean_gx); 
      Serial.print("\t");
      Serial.print(mean_gy); 
      Serial.print("\t");
      Serial.println(mean_gz);
      Serial.print("Your offsets:\t");
      Serial.print(ax_offset); 
      Serial.print("\t");
      Serial.print(ay_offset); 
      Serial.print("\t");
      Serial.print(az_offset); 
      Serial.print("\t");
      Serial.print(gx_offset); 
      Serial.print("\t");
      Serial.print(gy_offset); 
      Serial.print("\t");
      Serial.println(gz_offset); 
      
      Log.trace("Data is printed as: acelX acelY acelZ giroX giroY giroZ\n");
      state++;
    }
    #endif
  }
}

void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    fade();
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    #if defined(DEBUG) and defined(PRINT_GYRO_MEASURS)
    Serial.print("getMotion6:");
    serialPrintGyro(ax, ay, az, gx, gy, gz);
    #endif
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
//  ax_offset=-mean_ax/8;
//  ay_offset=-mean_ay/8;
//  az_offset=(16384-mean_az)/8;
//
//  gx_offset=-mean_gx/4;
//  gy_offset=-mean_gy/4;
//  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    
    flashLed();
    fade();

    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);
    #if defined(DEBUG) and defined(PRINT_GYRO_CALIB_PROCESS)
    Log.trace("Gyro mean values from measur func:");
    serialPrintGyro(mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz);
    Log.trace("Gyro set offsets in calibration:");
    serialPrintGyro(ax_offset, ay_offset, ay_offset, gx_offset, gy_offset, gz_offset);
    #endif

    meansensors();
    Log.notice("In calibration...\n");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;

  }
}

#ifdef NEO_INDICATOR
// NeoPixel Indication function
void red() {
  indi.setPixelColor(0, indi.Color(255, 0, 0));
  indi.show();
}

void blue() {
  indi.setPixelColor(0, indi.Color(0, 0, 255));
  indi.show();
}

void purple() {
  indi.setPixelColor(0, indi.Color(255, 0, 255));
  indi.show();
}

void fade() {
  loopCounter++;
  if (loopCounter == MAX_LOOP_NEO) {
    loopCounter = 0;
    currBright = currBright + brightStep;
    if ((currBright > MAX_BRIGHT) || (currBright < MIN_BRIGHT)) {
      brightStep = brightStep * (-1);
    }
    indi.setBrightness(currBright);
    indi.show();
  }
}
#else
void red() {}
void blue() {}
void fade() {}
#endif

#ifdef DEBUG
void serialPrintGyro(int ax, int ay, int az, int gx, int gy, int gz) {
  Serial.print("\t");
  Serial.print(ax);Serial.print(" \t");
  Serial.print(ay);Serial.print(" \t");
  Serial.print(az);Serial.print(" \t");
  Serial.print(" || \t");
  Serial.print(gx);Serial.print(" \t");
  Serial.print(gy);Serial.print(" \t");
  Serial.print(gz);
  Serial.println("");
}
#endif
