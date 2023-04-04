
/*
 * gyro scope orientation KRSBI RSCUAD 
 * AHMAD DAHLAN UNIVERSITY INDONESA
 * Author : JIhad Rahmawan
 * Develop :Danu Andrean, muhammad annas,hamdandih
 */

//============== i2c inisialisasi======================= 
#include <Wire.h>

// ============= LCD module =============================
#include <Adafruit_SSD1306.h> 
//#include "Adafruit_SSD1306.h"
#define SCREEN_WIDTH   128
#define SCREEN_HEIGHT   32 
#define OLED_RESET    4 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

//===============gyro module==============================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

MPU6050     mpu;
bool        dmpReady = false;  
uint8_t     mpuIntStatus;  
uint8_t     devStatus;     
uint16_t    packetSize;   
uint16_t    fifoCount;     
uint8_t     fifoBuffer[64]; 
// orientation
Quaternion  q;            // [w, x, y, z]                   
VectorFloat gravity;      // [x, y, z]           
float       euler[3];   // [psi, theta, phi]    
float       ypr[3];     // [yaw, pitch, roll]  
int         yaws;
int         yaw;
int         zero_offset_yaw;

volatile bool   mpuInterrupt = false; 
bool            startsend=false;    


const int   led_13 = 13;
int         countSaveGyro = 0;


int         y,limit;
int         countY, yfrist,ysecond, plus;
int         j=0;
float       vol;
uint32_t    notsend;
bool        start;

//======================= button ==================================
const int   b1 = 6;
const int   b2 = 7;

int         nilai_b1;
int         nilai_b2;

int         action      = 0;        //  tidak melakukan apa apa
int         menu        = 1;        // 
bool        isActive    = false;  //


//======================= prototype function =======================
void Gyro();
void dmpDataReady();
void Calibration();
void MODE_1();
void MODE_2();
void MODE_3();
void MODE_4();
void Menu();
void Button();
void (* reset) (void)= 0;

void setup() 
{
  Serial.begin(9600);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) // alamat I2c 
  { 
      Serial.println(F("SSD1306 allocation failed"));
   //  for(;;); // Don't proceed, loop forever
    }
    pinMode(led_13,OUTPUT);
   
//  Serial.println("================= start ===================");

   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    mpu.initialize();
    devStatus = mpu.dmpInitialize();
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); 

    if (devStatus == 0) 
  {
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    } 
  
    countY  = 1;
    start = false;
    pinMode(b1, INPUT_PULLUP);
    pinMode(b2, INPUT_PULLUP);
}

void loop() 
{

  Button();   // algo button
  
  Gyro();     // get data gyro

  /*
    Send data
  */
  String data = String(yaws)+" "+String(action);
  Serial.println(data);   
 
}

void Gyro()
{
  if (!dmpReady) 
    return;

    mpuInterrupt = false;
    
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
  {
        mpu.resetFIFO();
    } 
  else if (mpuIntStatus & 0x02) 
  {
    while (fifoCount < packetSize) 
      fifoCount = mpu.getFIFOCount();
    
    countY++;
    if(countY > 10)
      countY =1;

    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    y=ypr[0]* 180/M_PI;
            
    if(countY==2){
      yfrist=y;
//      ysecond=ysecond;
      if(!start)
      {
                digitalWrite(led_13,HIGH);
                delay(10);
      }
    } 
    else if(countY==9)
    {
      digitalWrite(led_13,LOW);
      delay(10);
      
      ysecond=y;
//      yfrist=yfrist;
  
    }
               
              
    if(yfrist==ysecond)
    {
      plus++;
      if(!start)
      {
        Calibration();
//        Serial.print("pluss");
//        Serial.println(plus);
      }
       
      if(plus==45 && start==0)  //setelah falid take nilai limit untuk mencari nilia zero point
      {
        limit=y;
        start=true;
//        Serial.println("done");
//        Serial.println("\n");
      }
    }
    if(start)
    {
      if(limit>=0)
      
        yaw=y-limit;
      else
        yaw=y+limit;
      
      if(yaw<0)
        yaws=360+yaw;

      else
        yaws=yaw;
      
      if((yaws<=10 && yaws>=0) || (yaws<=360 && yaws>=350))
      {
        digitalWrite(led_13,HIGH);

        delay(10);
      }
      else
      {
        digitalWrite(led_13,LOW);
        delay(10);
      }   
                  
    }
    else
    {  
//      Serial.print("y second ");
//      Serial.println(ysecond);
      if (ysecond < 0 ) {
//        countSaveGyro++;
//        if(countSaveGyro > 5)
//        {
            reset();
//          Serial.print("reset \n");
            delay (800); 
//        }
      }
//      else
//        countSaveGyro = 0;   
      delay(100);
    }      
    }
}

void dmpDataReady() 
{
    mpuInterrupt = true;   
}

void Calibration()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE); 
  
  display.setCursor(42,0); 
  display.println("R-SCUAD"); 
  display.setTextSize(2);
  
  display.setCursor(12,10); 
  display.setTextColor(WHITE); 
  display.print("Kalibrasi  "); 
  display.display();
}

void MODE_1()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(42,0);
  display.println("R-SCUAD"); 
  display.setTextColor(WHITE);
  
  display.setCursor(5,8);
  if(isActive)
  {
    display.print("Gyro = "); 
    display.print(yaws); 
  }
  else
  {
    display.println("pilihan mode");
  }
  
  display.setTextSize(2); 
  display.setTextColor(WHITE);
  
  display.setTextColor(WHITE);
  display.setCursor(28,18);
  display.print("MODE 1"); 
  display.display();
}

void MODE_2()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(42,0);
  display.println("R-SCUAD"); 
  display.setTextColor(WHITE);
  
  display.setCursor(5,8);
  if(isActive)
  {
    display.print("Gyro = "); 
    display.print(yaws); 
  }
  else
  {
    display.println("pilihan mode");
  }
  
  display.setTextSize(2); 
  display.setTextColor(WHITE);
  display.setTextSize(2); 
  display.setTextColor(WHITE);
  display.setCursor(22,18);
  display.print("MODE 2"); 
  display.display();
}

void MODE_3()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(42,0);
  display.println("R-SCUAD"); 
  display.setTextColor(WHITE);
  
  display.setCursor(5,8);
  if(isActive)
  {
    display.print("Gyro = "); 
    display.print(yaws); 
  }
  else
  {
    display.println("pilihan mode");
  }
  
  display.setTextSize(2); 
  display.setTextColor(WHITE);
  
  display.setCursor(28,18);
  display.print("MODE 3"); 
  display.display();
}

void MODE_4()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(42,0);
  display.println("R-SCUAD"); 
  display.setTextColor(WHITE);
  
  display.setCursor(5,8);
  if(isActive)
  {
    display.print("Gyro = "); 
    display.print(yaws); 
  }
  else
  {
    display.println("pilihan mode");
  }
  
  display.setTextSize(2); 
  display.setTextColor(WHITE);
  
  display.setCursor(22,18);
  display.print("MODE 4"); 
  display.display();
}

void Menu()
{
    if (menu==1)
  {
    MODE_1();
  }
  else if (menu==2)
  {
    MODE_2();
  }
  else if (menu==3)
  {
    MODE_3();
  }
  else if (menu==4)
  {
    MODE_4();
  }
}

void Button()
{
  nilai_b1 = digitalRead(b1);
  nilai_b2 = digitalRead(b2);

  // only gyro stable
  if(start)
  {
    // active only hit reset or begining
    if(isActive == false)
    {
      if (nilai_b1 == LOW )
      {
        action = menu;
        isActive = true;  //  lock active
        delay(100); 
      }
      if (nilai_b2== LOW)
      {
        menu++;
        if(menu > 4)
        menu = 1;
        delay(200);
      }
    }
      
    //reset
    if (nilai_b1 == LOW && nilai_b2 == LOW)
    {
      isActive  = false;  // release lock
      menu      = 1;      //MODE 1 // default soccer
      action    = 0;      // robot stop
      delay(100); 
    }

    // menu selection
    if(action == 0) Menu();
    if(action == 1) MODE_1();
    if(action == 2) MODE_2();
    if(action == 3) MODE_3();
    if(action == 4) MODE_4();
    
    }
}
