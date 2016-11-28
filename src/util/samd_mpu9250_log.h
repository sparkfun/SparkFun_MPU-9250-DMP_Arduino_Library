
#ifndef _SAMD_MPU9250_LOG_H_
#define _SAMD_MPU9250_LOG_H_


//#define MPL_LOGI(fmt, ...) MPL_LOG(LOG_INFO, MPL_LOG_TAG, fmt, ##__VA_ARGS__)

#define MPL_LOG_UNKNOWN		(0)
#define MPL_LOG_DEFAULT		(1)
#define MPL_LOG_VERBOSE		(2)
#define MPL_LOG_DEBUG		(3)
#define MPL_LOG_INFO		(4)
#define MPL_LOG_WARN		(5)
#define MPL_LOG_ERROR		(6)
#define MPL_LOG_SILENT		(8)

typedef enum {
    PACKET_DATA_ACCEL = 0,
    PACKET_DATA_GYRO,
    PACKET_DATA_COMPASS,
    PACKET_DATA_QUAT,
    PACKET_DATA_EULER,
    PACKET_DATA_ROT,
    PACKET_DATA_HEADING,
    PACKET_DATA_LINEAR_ACCEL,
    NUM_DATA_PACKETS
} eMPL_packet_e;

#if defined(__cplusplus) 
extern "C" {
#endif

#include <stdarg.h>

void logString(char * string);
int _MLPrintLog (int priority, const char* tag, const char* fmt, ...);
void eMPL_send_quat(long *quat);
void eMPL_send_data(unsigned char type, long *data);

#if defined(__cplusplus) 
}
#endif


#endif // _SAMD_MPU9250_LOG_H_