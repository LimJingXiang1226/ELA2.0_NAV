#include "CytronMotorDriver.h"


// Configure the motor driver.
CytronMD motor2(PWM_DIR, 3, 7); //Front Right
CytronMD motor8(PWM_DIR, 11, 13); //Rear Right
CytronMD motor1(PWM_DIR, 9, 8); //Front Left
CytronMD motor4(PWM_DIR, 10, 12); //Rear Left
int spd = 60;


// The setup routine runs once when you press reset.
void setup() 
{
  //Stop
//  motor1.setSpeed(0);
//  motor2.setSpeed(0);
//  motor4.setSpeed(0);
//  motor8.setSpeed(0); 
//  delay(10000);
  
  //Moving Foward
  motor1.setSpeed(spd);
  motor2.setSpeed(-spd);
  motor4.setSpeed(spd);
  motor8.setSpeed(-spd);  

//  //Moving Backward
//  motor1.setSpeed(-spd);
//  motor2.setSpeed(spd);
//  motor4.setSpeed(-spd);
//  motor8.setSpeed(spd);
//  
//  //Turning Right
//  motor1.setSpeed(-spd);
//  motor2.setSpeed(-spd);
//  motor4.setSpeed(-spd);
//  motor8.setSpeed(-spd);  
//
//  //Turning Left
//  motor1.setSpeed(-spd);
//  motor2.setSpeed(-spd);
//  motor4.setSpeed(-spd);
//  motor8.setSpeed(-spd); 

  delay(10000);
  //Stop
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor4.setSpeed(0);
  motor8.setSpeed(0); 
}


void loop() {
  
}
