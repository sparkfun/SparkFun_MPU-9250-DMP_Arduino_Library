/************************************************************
MPU9250_Wake_On_Motion
 Wake-on-Motion interrupt sketch for MPU-9250 DMP Arduino Library 
Oles Yaremenko @ home
original creation date: July 17, 2019
https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

This example sketch demonstrates how to itialize the 
MPU-9250 to implement Wake-On-Motion interrupt.

Development environment specifics:
Arduino IDE 1.8.9
ESP32 DevKit v.2 (interrupt on pin 4)
STM32L072CZ  (interrupt on pin 4)
Supported Platforms:
- ATSAMD21 (Arduino Zero, SparkFun SAMD21 Breakouts)
- ESP32 Arduino Core
- STM32L0 with https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
*************************************************************/
#ifdef defined(SAMD)
 #define SerialPort SerialUSB
#else
  #define SerialPort Serial
#endif

#include <SparkFunMPU9250-DMP.h>
#include <MPU9250_RegisterMap.h>
//Include the Invensense MPU9250 driver and DMP keys:
extern "C" {
#include "util/inv_mpu.h"
#include "util/inv_mpu_dmp_motion_driver.h"
}
// undef magnetometer if you use DMP only
#define MPU6500  //  Used instead of #define MPU9250 to #undef AK8963_SECONDARY

#define INTERRUPT_PIN 35   // Choose WOM interrupt pin

static void imuISR(void);  // Wake-on-Motion ISR

volatile bool imuWoke;     // ISR Woke-on-Motion Flag

// Create an instance of the MPU9250_DMP class
MPU9250_DMP imu;

void setup() 
{
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  SerialPort.begin(115200);
  SerialPort.println("MPU9250_Wake_On_Motion Example");
  
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      SerialPort.println("Unable to communicate with MPU-9250");
      SerialPort.println("Check connections, and try again.");
      SerialPort.println();
      delay(5000);
    }
  }

  // Power Down Gyro and Compass
  imu.setSensors(INV_XYZ_ACCEL);

  // The interrupt level can either be active-high or low. Configure as active-low.
  // Options are INT_ACTIVE_LOW or INT_ACTIVE_HIGH
  imu.setIntLevel(INT_ACTIVE_LOW);

  // disable Auxiliary I2C Master I/F
  mpu_set_bypass(0);

  // Enabe Wake On Motion low power mode with a threshold of 40 mg and
  // an accelerometer data rate of 15.63 Hz. 
  // Only accelerometer is enabled in LP mode
  // The interrupt is 50us pulse.
  if (mpu_lp_motion_interrupt(40,0,2) != INV_SUCCESS) {    
    // Failed to initialize MPU-9250, report somehow
    Serial.println(F("IMU set up failed. Please check installed IMU IC."));    
  }

  // Attach Interupt Service Routine to be called on device motion (to check angles for example)
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), imuISR, FALLING);


  // Do not forget to detach interrupt if you stop WOM mode
  //detachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN)); 
  imuWoke = false;
}

void loop() 
{
  // Check if the interrupt was fired
  if ( imuWoke )
  {
    // Do something.
    SerialPort.println("Can't touch this!");
    // Reset WOM Flag
    imuWoke = false;
  }
}

void imuISR() {
  // Set Wake-on-Motion Flag only
  imuWoke = true;
}