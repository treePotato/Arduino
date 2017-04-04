#include <I2Cdev.h>
#include <MPU6050.h>
#include <Servo.h>

Servo servoL, servoR; 

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

int ax, ay, az;
int gx, gy, gz;

//Servo stuff
int servoLPin = 5;
int servoRPin = 6;
double servoValueL, servoValueR = 90;
int servoDelay = 15;    //Servo delay in miliseconds
int gravityValue = 16500;
int deadSpace = 200;

//Time calculations
unsigned long previousTime = 0;
unsigned long previousTimeDisp = 0;

#define OUTPUT_READABLE_ACCELGYRO

#define LED_PIN 12
bool blinkState = false;

void setup() {
    servoL.attach(servoLPin);
    servoR.attach(servoRPin);
    
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
    
    Serial.println("Updating internal sensor offsets...");
    // -76  -2359 1688  0 0 0
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");
//    accelgyro.setXGyroOffset(160);
//    accelgyro.setYGyroOffset(100);
//    accelgyro.setZGyroOffset(30);
    accelgyro.setXAccelOffset(-500);
    accelgyro.setYAccelOffset(-3920);
    accelgyro.setZAccelOffset(1480);
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");

    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);
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
     * SERVO STUFFFF
     */     
     //servoValueL = map(gy,-32768,32767,0,180);
     //servoValueR = map(servoValueL,0,180,180,0);
     if(ax > deadSpace || ax < -1*deadSpace)
     {
     servoValueL = map(ax,-1*gravityValue, gravityValue, 0, 180);
     servoValueR = map(servoValueL, 0, 180, 180, 0);
     }
     
     if(currentTime - previousTime >= servoDelay)
     {
        servoL.write(servoValueL);
        servoR.write(servoValueR);
        previousTime = currentTime;
     }
    
}
