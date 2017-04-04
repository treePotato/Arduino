#include <I2Cdev.h>
#include <MPU6050.h>

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

//Motor stuff
int motorLFwd = 5;
int motorLRev = 6;
int motorRFwd = 9;
int motorRRev = 10;
double motorValueL, motorValueR; //Delay in microseconds for each motor
double motorRateL = 10;
double motorRateR = 10;              //Higher value = greater delay = slower motor speed
int gravityValue = 15000;
int deadSpace = 200;

//Time calculations
unsigned long previousTimeL = 0;
unsigned long previousTimeR = 0;
unsigned long previousTimeDisp = 0;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 12
bool blinkState = false;

void setup() {
    
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    /*
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    accelgyro.setXGyroOffset(220);
    accelgyro.setYGyroOffset(76);
    accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    */
    accelgyro.setXAccelOffset(-500);
    accelgyro.setYAccelOffset(-3920);
    accelgyro.setZAccelOffset(1480);

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
    pinMode(motorLFwd, OUTPUT);
    pinMode(motorLRev, OUTPUT);
    pinMode(motorRFwd, OUTPUT);
    pinMode(motorRRev, OUTPUT);
}

void loop() {
    unsigned long currentTime = millis();
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // these methods (and a few others) are also available
    //accelgyro.getAcceleration(&ax, &ay, &az);
    //accelgyro.getRotation(&gx, &gy, &gz);
    
   if(currentTime - previousTimeDisp > 150)
   {
    #ifdef OUTPUT_READABLE_ACCELGYRO
        // display tab-separated accel/gyro x/y/z values
        Serial.print("a/g:\t");
        Serial.print(ax); Serial.print("\t");
        Serial.print(ay); Serial.print("\t");
        Serial.print(az); Serial.print("\t");
        Serial.print(gx); Serial.print("\t");
        Serial.print(gy); Serial.print("\t");
        Serial.println(gz);
    #endif
    previousTimeDisp = currentTime;
   }


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    /*
     * MOTOR STUFFFF
     */    
     if(ax > deadSpace || ax < -1*deadSpace)
     {
     motorValueL = map(abs(ax), 0, gravityValue, 0, 255);
     motorValueR = motorValueL;
     }

//LEFT MOTOR
     if(currentTime - previousTimeL >= motorValueL*motorRateL)
     {
        analogWrite(motorLRev, 0);
        analogWrite(motorLFwd, motorValueL*motorRateL);
        previousTimeL = currentTime;
     }
     if(currentTime - previousTimeL >= motorValueL*motorRateL)
     {
        analogWrite(motorLFwd, 0);
        analogWrite(motorLRev, motorValueL*motorRateL);
        previousTimeL = currentTime;
     }

//RIGHT MOTOR
     if(currentTime - previousTimeR >= motorValueR*motorRateR)
     {
        analogWrite(motorRRev, 0);
        analogWrite(motorRFwd, motorValueR*motorRateR);
        previousTimeR = currentTime;
     }
     if(currentTime - previousTimeR >= motorValueR*motorRateR)
     {
        analogWrite(motorRFwd, 0);
        analogWrite(motorRRev, motorValueR*motorRateR);
        previousTimeR = currentTime;
     }


    
}
