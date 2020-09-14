#include <PS3BT.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#define enl 3     //first try with enl = 3. If it does not work, check each of the PWM pins one at a time.
#define enr 5
#define r1 7
#define r2 8
#define l1 2
#define l2 4

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch
int dly = 0;

void forward(int val)
{
    //Serial.println(F("\r\nF"));
    Serial.println(val);
    analogWrite(enl,val);
    analogWrite(enr,val);

    digitalWrite(l1,HIGH);
    digitalWrite(l2,LOW);
    digitalWrite(r1,HIGH);
    digitalWrite(r2,LOW);
}

void left(){
    //Serial.println(F("\r\nL"));
    analogWrite(enl,100);
    analogWrite(enr,100);
  
    digitalWrite(l1,LOW);
    digitalWrite(l2,HIGH);
    digitalWrite(r1,HIGH);
    digitalWrite(r2,LOW);
}

void back(int val)
  {
    //Serial.println(F("\r\nB"));
    Serial.println(val);  
    analogWrite(enl,val);
    analogWrite(enr,val);
  
    digitalWrite(l1,LOW);
    digitalWrite(l2,HIGH);
    digitalWrite(r1,LOW);
    digitalWrite(r2,HIGH);
  }
  
  void stopBot(){
   //Serial.println(F("\r\nS"));
    digitalWrite(l1,LOW);
    digitalWrite(l2,LOW);
    digitalWrite(r1,LOW);
    digitalWrite(r2,LOW);
}

void right()
{
  //Serial.println(F("\r\nR"));
    analogWrite(enl,100);
    analogWrite(enr,100);
 
    digitalWrite(l1,HIGH);
    digitalWrite(l2,LOW);
    digitalWrite(r1,LOW);
    digitalWrite(r2,HIGH);
}

void setup() {

  pinMode(13,OUTPUT);
  
    Serial.begin(115200);
    pinMode(enl,OUTPUT); pinMode(enr,OUTPUT); pinMode(l1,OUTPUT); pinMode(l2,OUTPUT); pinMode(r1,OUTPUT); pinMode(r2,OUTPUT);
       
    #if !defined(__MIPSEL__)
      while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
    #endif
      if (Usb.Init() == -1) {
        Serial.print(F("\r\nOSC did not start"));
        while (1); //halt
      }
      Serial.print(F("\r\nPS3 Bluetooth Library Started"));
}
void blinkl()
{
digitalWrite(13,HIGH);
delay(20);
digitalWrite(13,LOW);
delay(20);
}

void loop() 
{
      Usb.Task();
      if (PS3.PS3Connected || PS3.PS3NavigationConnected) 
      {  
        if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) 
        {
           if(PS3.getAnalogHat(RightHatX) > 137)
                right();
                     
           else if(PS3.getAnalogHat(RightHatX) < 117)
                left();
                
           else if(PS3.getAnalogHat(LeftHatY) < 117){
                forward(map(255 - PS3.getAnalogHat(LeftHatY),139,255,0,255));
                
                }
                
           else if(PS3.getAnalogHat(LeftHatY) > 137)
                back( map( PS3.getAnalogHat(LeftHatY) ,138 ,255 , 0 , 255 ));
      
/*Serial.print(F("\r\nLeftHatX: "));   Serial.print(PS3.getAnalogHat(LeftHatX));
            Serial.print(F("\tLeftHatY: "));     Serial.print(PS3.getAnalogHat(LeftHatY));
            Serial.print(F("\tRightHatX: "));    Serial.print(PS3.getAnalogHat(RightHatX));
            Serial.print(F("\tRightHatY: "));    Serial.print(PS3.getAnalogHat(RightHatY));*/
        }
        else
        {
          stopBot();
        }


        if (PS3.getButtonClick(PS)) 
        {
          Serial.print(F("\r\nPS"));
          PS3.disconnect();
        }
        else 
        {
          if (PS3.getButtonClick(CROSS))
              Serial.print(F("\r\nCross"));
        }
      }
}
