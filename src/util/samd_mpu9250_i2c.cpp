#include "samd_mpu9250_i2c.h"
#include <Arduino.h>
#include <Wire.h>

int arduino_i2c_write(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	Wire.beginTransmission(slave_addr);
	Wire.write(reg_addr);
	for (unsigned char i = 0; i < length; i++)
	{
		Wire.write(data[i]);
		//SerialUSB.println("Wrote 0x" + String(data[i], HEX) + " to " + String(reg_addr + i) + " [0x" + String(slave_addr, HEX) + "]");
	}
	Wire.endTransmission(true);
	
	return 0;
}
int arduino_i2c_read(unsigned char slave_addr, unsigned char reg_addr,
                       unsigned char length, unsigned char * data)
{
	Wire.beginTransmission(slave_addr);
	Wire.write(reg_addr);
	Wire.endTransmission(false);
	Wire.requestFrom(slave_addr, length);
	for (unsigned char i = 0; i < length; i++)
	{
		data[i] = Wire.read();
	}
	Wire.endTransmission(true);
	
	/*for (int j = 0;  j < length; j++)
	{
		SerialUSB.println("Read 0x" + String(data[j], HEX) + " from " + String(reg_addr + j) + " [0x" + String(slave_addr, HEX) + "]");
	}*/
	
	return 0;
}
