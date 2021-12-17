#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"


MPU6050 mpu;


#define INTERRUPT_PIN 2 // use pin 2 on Arduino Uno & most boards


bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;      
uint16_t packetSize;    
uint16_t fifoCount;     
uint8_t fifoBuffer[64]; 
int VRX = A0;
int VRY = A1;
int buttonState = 0;
int A;
int B;


Quaternion q;           
VectorFloat gravity;    
float ypr[3];           



volatile bool mpuInterrupt = false;  


void dmpDataReady() {
    mpuInterrupt = true;
}


void setup() {

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties


    Serial.begin(115200);
    
    Serial.println(F("Initializing I2C devices..."));
    

    mpu.initialize();
    pinMode(6, INPUT);

    pinMode(INTERRUPT_PIN, INPUT);
    pinMode(7,OUTPUT);

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
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
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

void loop() {


   if (!dmpReady) return;


   while (!mpuInterrupt && fifoCount < packetSize);


   mpuInterrupt = false;

   mpuIntStatus = mpu.getIntStatus();


   fifoCount = mpu.getFIFOCount();
    

   if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
       mpu.resetFIFO();
       Serial.println(F("FIFO overflow!"));    
   } else if (mpuIntStatus & 0x02) {
            
       while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();


       mpu.getFIFOBytes(fifoBuffer, packetSize);        
       fifoCount -= packetSize;


       mpu.dmpGetQuaternion(&q, fifoBuffer);
       mpu.dmpGetGravity(&gravity, &q);
       mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
       A = (ypr[0] * 180/M_PI); 
       B = (ypr[2] * 180/M_PI);


    }
    int val_x = analogRead(VRX);
    int val_y = analogRead(VRY);
    buttonState = digitalRead(6);
    if(Serial.available()){
      if(Serial.read() == 's'){
        Serial.write(val_x/4);
        Serial.write(val_y/4);
        Serial.write(buttonState);
        Serial.write(A);
        Serial.write(B);

        
        Serial.flush();
      }
      if(buttonState==0){
        analogWrite(3,200);
        analogWrite(7,200);
        
      }
      else{
        analogWrite(3,0);
        analogWrite(7,0);
      }
   }   
}
