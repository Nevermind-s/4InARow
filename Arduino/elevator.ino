/*
*  RosSerial BlinkM Example
*  This program shows how to control a blinkm
*  from an arduino using RosSerial
*/

#include <stdlib.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

#include <ros.h>
#include <std_msgs/UInt16.h>

void calcule_pas(void);
void programme_1(void);
void Programme_C(const std_msgs::UInt16 &msg);
void programme_BACKWARD_1(void);
void programme_FORWARD_1(void);
void programme_BACKWARD_FORWARD(char sens, int hauteur);
const int buttonPin_B = 2;     // the number of the pushbutton pin
const int buttonPin_H = 3;     // the number of the pushbutton pin
int buttonState = 0; // variables will change:
char uart_rd;

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// stepper moteur connecté à #2 (M2 and M3)
Adafruit_StepperMotor *myMotor = AFMS.getStepper(972, 2);

ros::NodeHandle nh;
ros::Subscriber<std_msgs::UInt16> sub("elevator" , &Programme_C);

void Programme_C(const std_msgs::UInt16 &msg)
{
    unsigned ope = msg.data >> 8;
    unsigned arg = msg.data & 0xFF;

    switch (ope) {
      case 1:
        programme_FORWARD_1();
        break;
      case 2:
        programme_RESET_BACKWARD_1();
        break;
      case 3:
        char sens = 'B';
        programme_BACKWARD_FORWARD(sens, arg);
        break;
    }
    
void setup()
{
    pinMode(buttonPin_B, INPUT);
    pinMode(buttonPin_H, INPUT);
    AFMS.begin();  // create with the default frequency 1.6KHz
    myMotor->setSpeed(255);  // vitesse à 10 tour par minutes
    nh.initNode();
    nh.subscribe(sub);
}

void loop()
{
   nh.spinOnce();
   delay(100);
}

void calcule_pas()
{
    int cont = 0;

    while (digitalRead(buttonPin_B) == HIGH) {
      // read the state of the pushbutton value:
      myMotor->step(1, BACKWARD, INTERLEAVE);
      //Serial.println(cont++, DEC);  // print as an ASCII-encoded decimal
  }
}

void programme_FORWARD_1()
{

  myMotor->step(100, BACKWARD, INTERLEAVE);//1480

  while (digitalRead(buttonPin_B) == HIGH)
  {
    myMotor->step(10, FORWARD, INTERLEAVE);
    // delay(3);
  }
  myMotor->step(0, FORWARD, INTERLEAVE);
}

void programme_BACKWARD_1()
{
  myMotor->step(100, FORWARD, INTERLEAVE);//1480

  while (digitalRead(buttonPin_H) == HIGH)
  {
    myMotor->step(10, BACKWARD, INTERLEAVE);
    // delay(3);
  }

  myMotor->step(0, FORWARD, INTERLEAVE);
}

void programme_RESET_BACKWARD_1()
{
  myMotor->step(100, FORWARD, INTERLEAVE);//1480

  while (digitalRead(buttonPin_H) == HIGH)
  {
    myMotor->step(10, BACKWARD, INTERLEAVE);
    // delay(3);
  }

  myMotor->step(0, FORWARD, INTERLEAVE);
}


void programme_BACKWARD_FORWARD(char sens, int hauteur) {
  
  int count = 0;
  
  // Serial.println(hauteur);

  if (sens == 'B') {
    while (( count != hauteur))
    {
      count++;
      if (digitalRead(buttonPin_B) == HIGH)
      {
        //Serial.println(count);
        myMotor->step(count, FORWARD, INTERLEAVE);
        // delay(3);
      }
      else
      {
        myMotor->step(0, FORWARD, INTERLEAVE);
        //Serial.println("ERROR_SWITCH");
        break;
      }
    }
  }
  if (sens == 'H') {
      while (( count != hauteur))
      {
        count++;
        if (digitalRead(buttonPin_H) == HIGH)
        {
          //Serial.println(count);
          myMotor->step(count, BACKWARD, INTERLEAVE);
          // delay(3);
        }
        else
        {
          myMotor->step(0, BACKWARD, INTERLEAVE);
          //Serial.println("ERROR_SWITCH");
          break;
        }
      }
    }
  }
