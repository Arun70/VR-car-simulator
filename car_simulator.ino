#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

   bool sendLeft = true;
   bool sendRight = true;
   bool sendJump = true;
   bool sendCenter = true;

bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 


Quaternion q;           
VectorInt16 aa;        
VectorInt16 aaReal;     
VectorInt16 aaWorld;    
VectorFloat gravity;    
float euler[3];         
float ypr[3];           
volatile bool mpuInterrupt = false;  

void setup() 
{
 
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; 
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(9600);  //For use with Arduino Uno
    Serial1.begin(9600); //For use with Leonardo

    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 
    if (devStatus == 0) {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
   
}


void loop() 
{
  
    if (!dmpReady) return;
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
        Serial.println(F("FIFO Overflow"));

    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) {
          fifoCount = mpu.getFIFOCount();
        }
        mpu.getFIFOBytes(fifoBuffer, packetSize);        
        fifoCount -= packetSize;
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        
        int x = ypr[0] * 180/M_PI;
        int y = ypr[1] * 180/M_PI;
        int z = ypr[2] * 180/M_PI;
        
   if (z >= 0){
      //backward
      if (sendJump){
      Serial1.write("b");
      Serial1.write(10);
      Serial.write("b");
      Serial.write(10);
      sendJump = false;
      }
      sendLeft = true;
      sendRight = true;
      sendCenter = true;
      
    } else if (y > 4){   //To make more sensitive change value to 4 or less
      //right
      if (sendRight){
      Serial1.write("r");
      Serial1.write(10);
      Serial.write("r");
      Serial.write(10);
      sendRight = false;
      }
      sendJump = true;
      sendLeft = true;
      sendCenter = true;
    } else if (y < -4){  //To make more sensitive change to -4 or greater 
      //left
      if (sendLeft){
      Serial1.write("l");
      Serial1.write(10);
      Serial.write("l");
      Serial.write(10);
      sendLeft = false;
      }
      sendJump = true;
      sendRight = true;
      sendCenter = true;

    } else if (y >= -3 || y <= 3){
      //center
      if (sendCenter == true){
          Serial1.write("c");
          Serial1.write(10);
          Serial.write("c");
          Serial.write(10);
          sendCenter = false;
      }
      
      sendJump = true;
      sendLeft = true;
      sendRight = true;
    }   
        
       
    }
}

void dmpDataReady() 
{
    mpuInterrupt = true;
}
