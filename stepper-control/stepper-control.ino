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

#define PRESSURE_STABLE 5.0f
//Temporary holding for the two lps22hb pressure sensors
sensors_event_t temp;
sensors_event_t pressure;
sensors_event_t temp2;
sensors_event_t pressure2;

float pressure_max[2] = {0.0f, 0.0f};
float pressure_min[2] = {1200.0f,1200.0f};
float pressure_current[2] = {0.0f, 0.0f};
bool pressure_fail[2] = {0,0};
bool test_init = 0;

bool LED2R_state = true;

bool button_test_state = 0;
bool button_abort_state = 0;
bool stepper_sleep = 1;
//AccelStepper stepper; // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5
AccelStepper x_axis(1, 13, 5); // pin 3 = step, pin 6 = direction
#define stepper_distance 2000

testing_state test_state = idle; 
rgb_state led_state[2] = {off,off};


void button_processor_test(int state){
  button_test_state = 1;
}

void button_processor_abort(int state){
  button_abort_state = 1;
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
  x_axis.setSpeed(800);	
  x_axis.setAcceleration(200);
  digitalWrite(m_enable, LOW);

  digitalWrite(m_slp, HIGH); //HIGH = NOT sleep
  digitalWrite(m_rst, HIGH); //HIGH = NOT reset

  button_test.setCallback(button_processor_test);
  button_abort.setCallback(button_processor_abort);
}

void loop()
{  
  //Basic running states
  if(test_state == idle)
  {
    button_procesor();
    stepper_process(false);
  }
  else if (test_state == initialization)
  {
    stepper_process(true);
    button_procesor();
    x_axis.moveTo(stepper_distance);
    x_axis.run();
    if(x_axis.currentPosition() == stepper_distance)
    {
      pressure_reset();
      test_state = test;
    }
  }
  else if (test_state == test)
  {
    read_lps();
    button_procesor();
    if(pressure_fail[0] && pressure_fail[1])
    {
      test_state = test_abort;
      pressure_reset();
    }
  }
  else if (test_state == test_abort)
  {
    x_axis.moveTo(0);
    x_axis.run();
    if(x_axis.currentPosition() == 0)
    {
      test_state = idle;
    }
  }
}

/**
 * @brief 
 * 
 */
void button_procesor()
{
  button_test.update();
  button_abort.update();
  if(button_abort_state == 1)
  {
    test_state = test_abort;
    pressure_reset();
  }

  if(test_state == idle & button_test_state == 1)
  {
    test_state = initialization;
    Serial.println("start test");
  } 

  //Reset the button states
  button_test_state = 0;
  button_abort_state = 0;
}


/**
 * @brief The drv8825 used is loud and annoying, so shutting this down helps
 * 
 */
void stepper_process(bool sleep_command)
{
  if (stepper_sleep != sleep_command)
  {
    if (stepper_sleep)
    {
      digitalWrite(m_slp, LOW); 
    } else
    {
      digitalWrite(m_slp, HIGH);
    }
    stepper_sleep = sleep_command;
  }

}

void pressure_reset()
{
  pressure_max[0] = 0.0f;
  pressure_max[1] = 0.0f;
  pressure_min[0] = 1200.0f;
  pressure_min[1] = 1200.0f;
  pressure_fail[0] = 0;
  pressure_fail[1] = 0;
  test_init = 1;
}

void led_process(rgb_state newstate, bool channel)
{
  
}

void pressure_stable_process(float sample_pressure, bool channel)
{
    //The reset pressures are too different and it'll be hard to know what they should be
    //unless it's changed to a target pressure (might work?), so wait until both pressures are set then go to stability testing
    pressure_current[channel] = sample_pressure;
    if (test_init)
    {
      pressure_max[channel] = sample_pressure;
      pressure_min[channel] = sample_pressure;
      if ((pressure_max[0] == pressure_min[0]) && (pressure_max[1] == pressure_min[1]))
      {
        test_init = 0;
      }
    }
    else //not test init
    {
      if (sample_pressure > pressure_max[channel])
      {
          pressure_max[channel] = sample_pressure;
      }

      if (sample_pressure < pressure_min[channel])
      {
          pressure_min[channel] = sample_pressure;
      }
      // Serial.print("P1Max: ");Serial.print(pressure_max[0]);Serial.print(" P2Max: ");Serial.print(pressure_max[1]);Serial.print(" hPa");
      // Serial.print(" P1Min: ");Serial.print(pressure_min[0]);Serial.print(" P2Min: ");Serial.print(pressure_min[1]);Serial.println(" hPa");
      Serial.print(" P1: ");Serial.print(pressure_current[0]);Serial.print(" P2: ");Serial.print(pressure_current[1]);Serial.println(" hPa");
      for(uint8_t i = 0; i < 2; i++)
      {
          if((pressure_max[i]-pressure_min[i]) > PRESSURE_STABLE)
          {
            pressure_fail[i] = 1;
            Serial.print("Pressure fail in:");
            Serial.println(i);
          }
      }
    }
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
    // Serial.print("Temperature: ");Serial.print(temp.temperature);Serial.print(" degrees C,");
    // Serial.print("Pressure: ");Serial.print(pressure.pressure);Serial.println(" hPa,");
    pressure_stable_process(pressure.pressure,0);
  }
  if(digitalRead(lps_int2) == 0)
  {
  lps2.getEvent(&pressure2, &temp2);// get pressure
  // Serial.print("Temperatur2: ");Serial.print(temp2.temperature);Serial.print(" degrees C,");
  // Serial.print("Pressur2: ");Serial.print(pressure2.pressure);Serial.println(" hPa");
  pressure_stable_process(pressure2.pressure,1);
  }
}

