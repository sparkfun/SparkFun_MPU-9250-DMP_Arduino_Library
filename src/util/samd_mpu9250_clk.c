#include "samd_mpu9250_clk.h"
#include <Arduino.h>

int samd21_get_clock_ms(unsigned long *count)
{
	*count = millis();
	return 0;
}

int samd21_delay_ms(unsigned long num_ms)
{
	delay(num_ms);
	return 0;
}