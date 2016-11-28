#include "samd_mpu9250_log.h"
#include <Arduino.h>
#include <stdarg.h>

// Based on log_stm32.c from Invensense motion_driver_6.12

#define BUF_SIZE        (256)
#define PACKET_LENGTH   (23)

#define PACKET_DEBUG    (1)
#define PACKET_QUAT     (2)
#define PACKET_DATA     (3)

void logString(char * string) 
{
	SerialUSB.println(string);
}

int _MLPrintLog (int priority, const char* tag, const char* fmt, ...)
{
    va_list args;
    int length, ii, i;
    char buf[BUF_SIZE], out[PACKET_LENGTH], this_length;
	
	// Can be modified to act on specific priorities
    /*switch (priority) {
    case MPL_LOG_UNKNOWN:
    case MPL_LOG_DEFAULT:
    case MPL_LOG_VERBOSE:
    case MPL_LOG_DEBUG:
    case MPL_LOG_INFO:
    case MPL_LOG_WARN:
    case MPL_LOG_ERROR:
    case MPL_LOG_SILENT:
        break;
    default:
        return 0;
    }*/
	
    va_start(args, fmt);

    length = vsprintf(buf, fmt, args);
	SerialUSB.println("length = " + String(length));
    if (length <= 0) {
        va_end(args);
        return length;
    }

    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DEBUG;
    out[2] = priority;
    out[21] = '\r';
    out[22] = '\n';
    for (ii = 0; ii < length; ii += (PACKET_LENGTH-5)) {
#define min(a,b) ((a < b) ? a : b)
        this_length = min(length-ii, PACKET_LENGTH-5);
        memset(out+3, 0, 18);
        memcpy(out+3, buf+ii, this_length);
        for (i=0; i<PACKET_LENGTH; i++) {
          //fputc(out[i]);
		  //SerialUSB.write((char)out[i]);
        }
    }
	
    va_end(args);

    return 0;
}

void eMPL_send_quat(long *quat)
{
    int i;
    if (!quat)
        return;
	SerialUSB.println("Quat: " + String(quat[0]) + 
		", " + String(quat[1]) + ", " + String(quat[2]) +
		", " + String(quat[3]));
    /*
    char out[PACKET_LENGTH];
	memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_QUAT;
    out[3] = (char)(quat[0] >> 24);
    out[4] = (char)(quat[0] >> 16);
    out[5] = (char)(quat[0] >> 8);
    out[6] = (char)quat[0];
    out[7] = (char)(quat[1] >> 24);
    out[8] = (char)(quat[1] >> 16);
    out[9] = (char)(quat[1] >> 8);
    out[10] = (char)quat[1];
    out[11] = (char)(quat[2] >> 24);
    out[12] = (char)(quat[2] >> 16);
    out[13] = (char)(quat[2] >> 8);
    out[14] = (char)quat[2];
    out[15] = (char)(quat[3] >> 24);
    out[16] = (char)(quat[3] >> 16);
    out[17] = (char)(quat[3] >> 8);
    out[18] = (char)quat[3];
    out[21] = '\r';
    out[22] = '\n';*/
}

void eMPL_send_data(unsigned char type, long *data)
{
	switch (type)
	{
	case PACKET_DATA_ACCEL:
		SerialUSB.println("A " + String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]));
		break;
    case PACKET_DATA_GYRO:
		SerialUSB.println("G " + String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]));
		break;
    case PACKET_DATA_COMPASS:
		SerialUSB.println("C " + String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]));
		break;
    case PACKET_DATA_EULER:
		SerialUSB.println("E " + String(data[0]) + ", " + String(data[1]) + ", " + String(data[2]));
		break;
	case PACKET_DATA_HEADING:
		SerialUSB.println("H " + String(data[0]));
		break;
	case PACKET_DATA_QUAT:
		SerialUSB.println("Q " + String(data[3]));
		break;
	default:
		break;
	}
	
    /*char out[PACKET_LENGTH];
    int i;
    if (!data)
        return;
    memset(out, 0, PACKET_LENGTH);
    out[0] = '$';
    out[1] = PACKET_DATA;
    out[2] = type;
    out[21] = '\r';
    out[22] = '\n';
    switch (type) {
    // Two bytes per-element.
    case PACKET_DATA_ROT:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[1] >> 24);
        out[6] = (char)(data[1] >> 16);
        out[7] = (char)(data[2] >> 24);
        out[8] = (char)(data[2] >> 16);
        out[9] = (char)(data[3] >> 24);
        out[10] = (char)(data[3] >> 16);
        out[11] = (char)(data[4] >> 24);
        out[12] = (char)(data[4] >> 16);
        out[13] = (char)(data[5] >> 24);
        out[14] = (char)(data[5] >> 16);
        out[15] = (char)(data[6] >> 24);
        out[16] = (char)(data[6] >> 16);
        out[17] = (char)(data[7] >> 24);
        out[18] = (char)(data[7] >> 16);
        out[19] = (char)(data[8] >> 24);
        out[20] = (char)(data[8] >> 16);
        break;
    // Four bytes per-element. 
    /Four elements.
    case PACKET_DATA_QUAT:
        out[15] = (char)(data[3] >> 24);
        out[16] = (char)(data[3] >> 16);
        out[17] = (char)(data[3] >> 8);
        out[18] = (char)data[3];
    // Three elements.
    case PACKET_DATA_ACCEL:
    case PACKET_DATA_GYRO:
    case PACKET_DATA_COMPASS:
    case PACKET_DATA_EULER:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        out[7] = (char)(data[1] >> 24);
        out[8] = (char)(data[1] >> 16);
        out[9] = (char)(data[1] >> 8);
        out[10] = (char)data[1];
        out[11] = (char)(data[2] >> 24);
        out[12] = (char)(data[2] >> 16);
        out[13] = (char)(data[2] >> 8);
        out[14] = (char)data[2];
        break;
    case PACKET_DATA_HEADING:
        out[3] = (char)(data[0] >> 24);
        out[4] = (char)(data[0] >> 16);
        out[5] = (char)(data[0] >> 8);
        out[6] = (char)data[0];
        break;
    default:
        return;
    }
    for (i=0; i<PACKET_LENGTH; i++) {
      fputc(out[i]);
    }*/
}