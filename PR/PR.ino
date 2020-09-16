#include <PS3BT.h>
#include <usbhub.h>

#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include<Servo.h>
//--------------------------------Gripper claws constraint---------------
#define MaxRightGripperClosingAngle  0                       /********************** Specify the angle here**********************************************/
#define MaxRightGripperReleasingAngle 40                      /********************** Specify the angle here***********************************************/ 
#define MaxLeftGripperClosingAngle 180
#define MaxLeftGripperReleasingAngle 140
//---------------------------------------------------------------------

//-------------------------------Vertical Flap constraints-----------------
#define MaxVerticalFlapClosingAngle 0
#define MaxVerticalFlapOpeningAngle 180
//-------------------------------------------------------------------------

//-------------------------------Horizontal Flap constraints-----------------
#define MaxHorizontalFlapClosingAngle 0
#define MaxHorizontalFlapOpeningAngle 180
//-------------------------------------------------------------------------

//----------------------------Gripper Lifting PMDC pin and default speed---------------------
#define GripperLiftingPMDC_pin1 2
#define GripperLiftingPMDC_pin2 3
#define GripperLiftingPMDC_PWM 8
#define GripperLiftingPMDC_defaultSpeed 150
//----------------------------------------------------------------------------------

//----------------------------Catapult Lifting PMDC pin and default speed---------------------
#define CatapultPMDC_pin1 9
#define CatapultPMDC_pin2 10
#define CatapultPMDC_PWM 11
#define CatapultPMDC_defaultSpeed 150
//----------------------------------------------------------------------------------

bool startPos = true;
int LeftgrippingAngle=MaxLeftGripperReleasingAngle,RightgrippingAngle=MaxRightGripperReleasingAngle,VerticalFlapAngle=MaxVerticalFlapOpeningAngle;
int HorizontalFlapAngle=MaxHorizontalFlapOpeningAngle;

int GripperPMDCflag,CatapultPMDCflag; 
Servo LeftGripperservo, RightGripperservo,VerticalFlapservo,HorizontalFlapservo;

USB Usb;
//USBHub Hub1(&Usb); // Some dongles have a hub inside

BTD Btd(&Usb); // You have to create the Bluetooth Dongle instance like so
/* You can create the instance of the class in two ways */
PS3BT PS3(&Btd); // This will just create the instance
//PS3BT PS3(&Btd, 0x00, 0x15, 0x83, 0x3D, 0x0A, 0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

void calculate(unsigned int xl,unsigned int yl,unsigned int xr,unsigned int yr)
{
    double angle,r;
    if(xr>=127)
    {
        xr=xr-127;
        Serial.print(xr);
        if(yr>127){
            //Quadrant 4
            yr=yr-127;
            Serial.print("Quadrant 4\t");
            angle = atan2((double)yr,(double)xr);
            angle = 4.7123 + (1.5708-angle);  //angle in radians
        }
        else{
            //Quadrant 1
           
            Serial.print("Quadrant 1\t");
            yr = 127-yr;  
            angle = atan2((double)yr,(double)xr);
        }
    }
    else{
        xr = 127-xr;
        if(yr>127){
            //Quadrant 3
            yr = yr-127;
            Serial.print("Quadrant 3\t");
            angle = atan2((double)yr,(double)xr);
            angle = angle + 3.1415;
        }
        else{
            //Quadrant 2
            yr = 127-yr;
            Serial.print("Quadrant 2\t");
            angle = atan2((double)yr,(double)xr);
            angle = 1.5708 + (1.5708-angle);
        }
    }

    Serial.print("xr: ");Serial.print(xr);Serial.print("\t");Serial.print("yr: ");Serial.print(yr); 

    Serial.print("\t");Serial.print(sq(xr));Serial.print("\t");Serial.print(sq(yr));
    r = sqrt(sq(xr)+sq(yr));
    Serial.print("\tr=");Serial.print(r);
    xr = map(r*0.8756*abs(cos(angle)-sin(angle)),0,127,0,255);
    yr = map(r*0.8756*abs(cos(angle)+sin(angle)),0,127,0,255);

    Serial.print("\t\txr: ");Serial.print(xr);Serial.print("\t");Serial.print("yr: ");Serial.print(yr); 
     
    analogWrite(set1PWM,xr);
    analogWrite(set2PWM,yr);

    if(angle>6.2831)
        angle=angle-6.2831;

    Serial.print("\tAngle: ");Serial.println(angle);

    if(angle>0.7853 && angle<=2.3561)
    {
        digitalWrite(wset11,HIGH);   //HIGH-LOW is positive direction & LOW-HIGH is negative direction
        digitalWrite(wset12,LOW);
        digitalWrite(wset21,HIGH);
        digitalWrite(wset22,LOW); 
    }
    else if(angle>2.3561 && angle<=3.9269)
    {
        digitalWrite(wset11,HIGH);
        digitalWrite(wset12,LOW);
        digitalWrite(wset21,LOW);
        digitalWrite(wset22,HIGH);
    }
    else if(angle>3.9269 && angle<=5.4977)
    {
        digitalWrite(wset11,LOW);
        digitalWrite(wset12,HIGH);
        digitalWrite(wset21,LOW);
        digitalWrite(wset22,HIGH); 
    }
    else
    {
        digitalWrite(wset11,LOW);
        digitalWrite(wset12,HIGH);
        digitalWrite(wset21,HIGH);
        digitalWrite(wset22,LOW); 
    }

    if(xl<100) //100 is a minimum limit value for xl to activate rotation
    {
        xl = 100-xl;
        map(xl,0,100,0,255);
        rotateLeft(xl);
    }
    if(xl>155)
    {
        xl = 255-xl;
        map(xl,0,100,0,255);
        rotateRight(xl);
    }    
}

void rotateLeft(int mag)
{
    analogWrite(set1PWM,mag);
    analogWrite(set2PWM,mag);
    digitalWrite(wset11,LOW);
    digitalWrite(wset12,HIGH);
    digitalWrite(wset21,HIGH);
    digitalWrite(wset22,LOW);
}

void rotateRight(int mag)
{
    analogWrite(set1PWM,mag);
    analogWrite(set2PWM,mag);    
    digitalWrite(wset11,HIGH);
    digitalWrite(wset12,LOW);
    digitalWrite(wset21,LOW);
    digitalWrite(wset22,HIGH);
}

void setup() {
  Serial.begin(115200);

#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print("OSC did not start");
    while (1); //halt
  }
  Serial.print("PS3 Bluetooth Library Started");
  LeftGripperservo.attach(4);
  RightGripperservo.attach(6);
  VerticalFlapservo.attach(7);
  HorizontalFlapservo.attach(5);
  
  LeftGripperservo.write(MaxLeftGripperReleasingAngle);
  RightGripperservo.write(MaxRightGripperReleasingAngle);
  VerticalFlapservo.write(MaxVerticalFlapOpeningAngle);  
  HorizontalFlapservo.write(MaxHorizontalFlapOpeningAngle);
  
  analogWrite(GripperLiftingPMDC_PWM,GripperLiftingPMDC_defaultSpeed);
  analogWrite(CatapultPMDC_PWM,CatapultPMDC_defaultSpeed);

}

void loop() {
  Usb.Task();

  if (PS3.PS3Connected || PS3.PS3NavigationConnected) {
    if (PS3.getAnalogHat(LeftHatX) > 137 || PS3.getAnalogHat(LeftHatX) < 117 || PS3.getAnalogHat(LeftHatY) > 137 || PS3.getAnalogHat(LeftHatY) < 117 || PS3.getAnalogHat(RightHatX) > 137 || PS3.getAnalogHat(RightHatX) < 117 || PS3.getAnalogHat(RightHatY) > 137 || PS3.getAnalogHat(RightHatY) < 117) 
    {
      xl = PS3.getAnalogHat(LeftHatX);
      yl = PS3.getAnalogHat(LeftHatY);

      if (PS3.PS3Connected) 
      { // The Navigation controller only have one joystick
          xr = PS3.getAnalogHat(RightHatX);
          yr = PS3.getAnalogHat(RightHatY);
      }

      calculate(xl,yl,xr,yr);
    }
    else{
      calculate(xl,yl,127,127);
    }
    
  if (PS3.getAnalogButton(L2)){
        //Serial.print(F("\r\nL2: "));
        //Serial.print(PS3.getAnalogButton(L2));
        GripperPMDCflag = 2;    //towardRack
        }
  if (PS3.getAnalogButton(R2)){
        //Serial.print(F("\r\nR2: "));
        //Serial.print(PS3.getAnalogButton(R2));
        openGripper();
        }
    

    if (PS3.getButtonClick(PS)) {
      Serial.print(F("\r\nPS"));
      PS3.disconnect();
    }
    else {
      if (PS3.getAnalogButton(TRIANGLE)) {
        startPositionForKick();
      }
      if (PS3.getAnalogButton(CIRCLE)) {
        Serial.print(F("\r\nCircle"));
        CatapultPMDCflag ==2;
      }
      if (PS3.getAnalogButton(CROSS)){
        kick();
        }
      if (PS3.getAnalogButton(SQUARE)){
        Serial.print(F("\r\nSquare"));
        CatapultPMDCflag ==1;
      }
      if (PS3.getAnalogButton(UP)) {
        Serial.print(F("\r\nUp"));
        verticalFlapOPEN();
      }
      if (PS3.getAnalogButton(RIGHT)) {
        Serial.print(F("\r\nRight"));
        horizontalFlapCLOSE();
      }
      if (PS3.getAnalogButton(DOWN)) {
        Serial.print(F("\r\nDown"));
        verticalFlapCLOSE();
      }
      if (PS3.getAnalogButton(LEFT)) {
        Serial.print(F("\r\nLeft"));
        horizontalFlapOPEN();
      }

      if (PS3.getAnalogButton(L1))
      {
        Serial.print(F("\r\nL1"));
        GripperPMDCflag = 1; //towardCatapult
      }
      if (PS3.getButtonClick(L3))
      { 
        Serial.print(F("\r\nL3"));
      }
      if (PS3.getAnalogButton  (R1)){
        Serial.print(F("\r\nR1"));
        closeGripper();
      }
      if (PS3.getButtonClick(R3))
        Serial.print(F("\r\nR3"));

      if (PS3.getButtonClick(SELECT)) {
        Serial.print(F("\r\nSelect - "));
        //PS3.printStatusString();
      }
      if (PS3.getButtonClick(START)) {
        Serial.print(F("\r\Autonomous Mode ON"));
          autonomous();
      }
    }
  }

  if(GripperPMDCflag == 1){
    towardsCatapult();
    GripperPMDCflag=0;
    }
  else if(GripperPMDCflag == 2){
    towardsRack();
    GripperPMDCflag=0;}
  else{
    stopGripperPMDC();  
  }

  if(CatapultPMDCflag ==1){
    CatapultThrow();
    CatapultPMDCflag ==0;
    }
   else if(CatapultPMDCflag ==2){
    CatapultBack();
    CatapultPMDCflag ==0;
   }
   else
     stopCatapult();
}

void autonomous()
{       Usb.Task(); 
  while(!PS3.getButtonClick(START))
  {   Usb.Task();
       Serial.print(F("\r\nAutonomous Mode ON"));
       /**********
       * WRITE CODE FOR AUTONOMOUS MODE HERE
       */
  }
       Serial.print(F("\r\nAutonomous Mode OFF"));
  }      


void kick()
{
  if(startPos){
  Serial.print(F("\r\nKick"));
  startPos = false;  
  }
}

void startPositionForKick()
{
  if(!startPos)
  {
    Serial.print(F("\r\nGet back to initial position"));  
   startPos = true;
  }
}

void openGripper()
{
  Serial.print(F("\r\nopenGripper"));
  LeftgrippingAngle--;                                            /***************Updation of angle depends on the setting of servo on the gripper****************/
  RightgrippingAngle++;
  
  if(LeftgrippingAngle < MaxLeftGripperReleasingAngle)
    LeftgrippingAngle = MaxLeftGripperReleasingAngle;
  if(RightgrippingAngle > MaxRightGripperReleasingAngle)
    RightgrippingAngle = MaxRightGripperReleasingAngle;  
Serial.print(F("LeftgrippingAngle-->"));
Serial.print(LeftgrippingAngle);
Serial.print(F("            RightgrippingAngle-->"));
Serial.println(RightgrippingAngle);  
  LeftGripperservo.write(LeftgrippingAngle);
  RightGripperservo.write(RightgrippingAngle);
}

void closeGripper()
{
  Serial.print(F("\r\ncloseGripper"));
  LeftgrippingAngle++;                        /***************Updation of angle depends on the setting of servo on the gripper****************/
  RightgrippingAngle--;
  if(LeftgrippingAngle > MaxLeftGripperClosingAngle)
    LeftgrippingAngle = MaxLeftGripperClosingAngle;
  if(RightgrippingAngle < MaxRightGripperClosingAngle)
    RightgrippingAngle = MaxRightGripperClosingAngle;
Serial.print(F("LeftgrippingAngle-->"));
Serial.print(LeftgrippingAngle);
Serial.print(F("            RightgrippingAngle-->"));
Serial.println(RightgrippingAngle);
  RightGripperservo.write(RightgrippingAngle);    
  LeftGripperservo.write(LeftgrippingAngle);
}

void towardsRack()
{
  Serial.print(F("\r\ntowardsRack"));
  digitalWrite(GripperLiftingPMDC_pin1,HIGH);
  digitalWrite(GripperLiftingPMDC_pin2,LOW);
}

void towardsCatapult()
{
  Serial.print(F("\r\ntowardsCatapult"));
  digitalWrite(GripperLiftingPMDC_pin1,LOW);
  digitalWrite(GripperLiftingPMDC_pin2,HIGH);  
}

void stopGripperPMDC()
{
  Serial.print(F("\r\nstopGripperPMDC"));
  digitalWrite(GripperLiftingPMDC_pin1,LOW);
  digitalWrite(GripperLiftingPMDC_pin2,LOW);    
}

void CatapultThrow()
{
  Serial.print(F("\r\nCatapultThrow"));
  digitalWrite(CatapultPMDC_pin1,LOW);
  digitalWrite(CatapultPMDC_pin2,HIGH);    
}

void CatapultBack()
{
  Serial.print(F("\r\nCatapultBack"));
  digitalWrite(CatapultPMDC_pin1,LOW);
  digitalWrite(CatapultPMDC_pin2,HIGH);    
}
void stopCatapult()
{
    Serial.print(F("\r\nstopCatapult"));
  digitalWrite(CatapultPMDC_pin1,LOW);
  digitalWrite(CatapultPMDC_pin2,LOW); 
}
void verticalFlapOPEN()
{
  Serial.print(F("\r\nverticalFlapOPEN"));
  VerticalFlapAngle++;                        /***************Updation of angle depends on the setting of servo on the gripper****************/

  if(VerticalFlapAngle > MaxVerticalFlapOpeningAngle)
    VerticalFlapAngle = MaxVerticalFlapOpeningAngle;

  Serial.print(F("VerticalFlapAngle-->"));
  Serial.print(VerticalFlapAngle);
    
  VerticalFlapservo.write(VerticalFlapAngle);  
}

void verticalFlapCLOSE()
{
  Serial.print(F("\r\nverticalFlapCLOSE"));
  VerticalFlapAngle--;                        /***************Updation of angle depends on the setting of servo on the gripper****************/

  if(VerticalFlapAngle < MaxVerticalFlapClosingAngle)
    VerticalFlapAngle = MaxVerticalFlapClosingAngle;

  Serial.print(F("VerticalFlapAngle-->"));
  Serial.print(VerticalFlapAngle);
    
  VerticalFlapservo.write(VerticalFlapAngle);   
}


void horizontalFlapCLOSE()
{
  Serial.print(F("\r\nhorizontalFlapCLOSE"));
  HorizontalFlapAngle--;                        /***************Updation of angle depends on the setting of servo on the gripper****************/

  if(HorizontalFlapAngle < MaxHorizontalFlapClosingAngle)
    HorizontalFlapAngle = MaxHorizontalFlapClosingAngle;

  Serial.print(F("HorizontalFlapAngle-->"));
  Serial.print(HorizontalFlapAngle);
    
  HorizontalFlapservo.write(HorizontalFlapAngle);    
}

void horizontalFlapOPEN()
{
  Serial.print(F("\r\horizontalFlapOPEN"));
  HorizontalFlapAngle++;                        /***************Updation of angle depends on the setting of servo on the gripper****************/

  if(HorizontalFlapAngle > MaxHorizontalFlapOpeningAngle)
    HorizontalFlapAngle = MaxHorizontalFlapOpeningAngle;

  Serial.print(F("HorizontalFlapAngle-->"));
  Serial.print(HorizontalFlapAngle);
    
  HorizontalFlapservo.write(HorizontalFlapAngle);    
}
