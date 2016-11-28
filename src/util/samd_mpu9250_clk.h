
#ifndef _SAMD_MPU9250_CLK_H_
#define _SAMD_MPU9250_CLK_H_

int samd21_get_clock_ms(unsigned long *count);
int samd21_delay_ms(unsigned long num_ms);

#endif // _SAMD_MPU9250_CLK_H_