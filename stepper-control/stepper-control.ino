// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#include <AccelStepper.h>
#include <ButtonDebounce.h>
#include <Wire.h>
#include <Adafruit_LPS2X.h>
#include <Adafruit_Sensor.h>
#include "iptester.h"

#define btn_abort 18
#define btn_test 19
ButtonDebounce button_test(btn_test, 200);
ButtonDebounce button_abort(btn_abort, 20);

#define lps_int2 12
#define lps_int1 11
#define m_enable 8
#define m_slp 9
#define m_rst 10

#define LED1R 20
#define LED2R 4

Adafruit_LPS22 lps;
Adafruit_LPS22 lps2;

//Temporary holding for the two lps22hb pressure sensors
sensors_event_t temp;
sensors_event_t pressure;
sensors_event_t temp2;
sensors_event_t pressure2;

bool LED2R_state = true;

//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper x_axis(1, 13, 5); // pin 3 = step, pin 6 = direction
testing_state test_state = iptest_idle; 


void button_test(int state){
  digitalWrite(LED1R, LOW);
}

void button_abort(int state){
  digitalWrite(LED2R, LOW);
}

void setup()
{  
    Serial.begin(115200);
  // while (!Serial) delay(10);     // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Adafruit LPS22 test!");

  // Try to initialize!
  if (!lps.begin_I2C(0x5D)) {
    Serial.println("Failed to find LPS22 chip");
    while (1) { delay(10); }
  }
  if (!lps2.begin_I2C(0x5C)) {
    Serial.println("Failed to find LPS22-2 chip");
    while (1) { delay(10); }
  }
  Serial.println("Both LPS22 Found!");

  lps.setDataRate(LPS22_RATE_10_HZ);
  lps2.setDataRate(LPS22_RATE_25_HZ);
    
  pinMode(m_enable, OUTPUT);
  pinMode(m_slp, OUTPUT);
  pinMode(m_rst, OUTPUT);

  pinMode(LED1R, OUTPUT);
  digitalWrite(LED1R, HIGH); //HIGH = off
  pinMode(LED2R, OUTPUT);
  digitalWrite(LED2R, HIGH); //HIGH = off

  pinMode(lps_int1, INPUT_PULLUP);
  pinMode(lps_int2, INPUT_PULLUP);
  
  x_axis.setMaxSpeed(1000);
  x_axis.setSpeed(500);	
  x_axis.setAcceleration(50);
  digitalWrite(m_enable, LOW);
  digitalWrite(m_slp, HIGH); //HIGH = NOT sleep
  digitalWrite(m_rst, HIGH); //HIGH = NOT reset

  button_test.setCallback(button_test);
  button_abort.setCallback(button_abort);
}

void loop()
{  
  x_axis.moveTo(1000);
  //Basic running states

  x_axis.run();
  read_lps();

  if(test_state == idle)
  {
    button_test.update();
    button_abort.update();

  }
  else if (test_stat == initialization)
  {
    button_test.update();
    button_abort.update();
  }
  else
}

/**
 * @brief reads the lps sensors. Because the sensors are interrupt driven but are not on interruptable pins
 * then they must be read manually. The interrupt will clear after reading. Reading is blocking on arduino
 * so don't expect great speed
 * 
 */
void read_lps()
{
  if(digitalRead(lps_int1) ==0)
  {
    lps.getEvent(&pressure, &temp);// get pressure
    Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.print(" degrees C,");
    Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa,");
  }
  if(digitalRead(lps_int2) == 0)
  {
  lps2.getEvent(&pressure2, &temp2);// get pressure
  Serial.print("Temperatur2: ");Serial.print(temp2.temperature);Serial.print(" degrees C,");
  Serial.print("Pressur2: ");Serial.print(pressure2.pressure);Serial.println(" hPa");
  }
}

