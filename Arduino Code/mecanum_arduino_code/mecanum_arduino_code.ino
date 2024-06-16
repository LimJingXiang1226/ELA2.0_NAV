#include <Arduino.h>
#include "CytronMotorDriver.h"

int ID = 0;
int DIR = 0;
int SPD = 0;
int spd = 0;

int capturingValues = 0; // Use int to track capture state
int capActivate = 0;
int motorSpd[4];
int counter = 0;

CytronMD motor2(PWM_DIR, 3, 7); //Front Right
CytronMD motor8(PWM_DIR, 11, 13); //Rear Right
CytronMD motor1(PWM_DIR, 9, 8); //Front Left
CytronMD motor4(PWM_DIR, 10, 12); //Rear Left

void setup() 
{
  Serial.begin(9600); // Replace 9600 with your baud rate if using Serial
  motorSpd[0] = 0;
  motorSpd[1] = 0;
  motorSpd[2] = 0;
  motorSpd[3] = 0;
}

void loop() 
{
  if (Serial.available() > 0) 
  {
//    Serial.println("loop started..........");
    byte data = Serial.read();
//    Serial.println("data:");
//    Serial.println(data);
//    Serial.print("capActivate:");
//    Serial.println(capActivate);
    if (capActivate == 0)
    {
      if (data == 126)
      {
        capturingValues = 1;
      } 
      else if (capturingValues > 0)
      {
        if (data == 1)
        {
          capturingValues = 2;
        } 
        else if (capturingValues > 1)
        {
          if (data == 4)
          {
            capturingValues = 3;
            capActivate = 1;
          } 
        }
      }
//      Serial.print("capturingValues:");
//      Serial.println(capturingValues);
    } 
    else if (capActivate == 1)
    {
      switch (capturingValues)
      {
          case 3: // Expected motor ID position
            ID= data ;
            capturingValues--; // Decrement for next value
            counter = 0;
            break;
          case 2: // Expected direction position
            DIR = data ;
            capturingValues--;
            counter = 0;
            break;
          case 1: // Expected speed position
            SPD = data ;
            capturingValues = 0; // Reset to 0 (not capturing)
            capActivate = 0;
            counter = 1;
            break;
      }
//      Serial.println("Motor ID: ");
//      Serial.println(ID);
//      Serial.print(", Direction: ");
//      Serial.println(DIR);
//      Serial.print(", Speed: ");
//      Serial.print(SPD);
      if (SPD < 34)
      {
        spd = SPD*10;
//        Serial.print(", Actual_speed:  ");
        Serial.println(spd);
//        spd = 20;
        if (DIR == 1)
        {
          switch (ID)
          {
            case 1:
              motorSpd[0] = spd;
//              motor1.setSpeed(spd);
              break;
            case 2:
              motorSpd[1] = -spd;
//              motor2.setSpeed(-spd);
              break;
            case 4:
              motorSpd[2] = spd;
//              motor4.setSpeed(spd);
              break;
            case 8:
              motorSpd[3] = -spd;
//              motor8.setSpeed(-spd);
              if (counter == 1 && ID==8)
              { 
                motor1.setSpeed(motorSpd[0]);
                motor2.setSpeed(motorSpd[1]);
                motor4.setSpeed(motorSpd[2]);
                motor8.setSpeed(motorSpd[3]); 
              }
              break;    
          }
        } 
        else 
        {
          switch (ID)
          {
            case 1:
              motorSpd[0] = -spd;
//              motor1.setSpeed(-spd);
              break;
            case 2:
              motorSpd[1] = spd;
//              motor2.setSpeed(spd);
              break;      
            case 4:
              motorSpd[2] = -spd;
//              motor4.setSpeed(-spd);
              break;
            case 8:
              motorSpd[3] = spd;
//              motor8.setSpeed(spd);
              if (counter == 1 && ID==8)
              { 
                motor1.setSpeed(motorSpd[0]);
                motor2.setSpeed(motorSpd[1]);
                motor4.setSpeed(motorSpd[2]);
                motor8.setSpeed(motorSpd[3]);
              }
              break;          
          }
        }
      }
    }
  }
}
