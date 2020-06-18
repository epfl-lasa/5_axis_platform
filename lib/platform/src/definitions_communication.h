#ifndef __DEFINITIONS_COMMUNICATION_H
#define __DEFINITIONS_COMMUNICATION_H

//#include "main.h"
//#include "definitions.h"

#include <stdarg.h>
#include "definitions_main.h"

/** @defgroup Communication Main / Communication
  * @brief Control the communication with the computer.
  *
  * This module controls all the communication logic between the board and the
  * computer. It uses a specific communication protocol between the
  * computer and the board, with a system of messages. Thanks to this, the
  * the MATLAB application is able to get the value of selected variables on the
  * STM, and is even capable of modifiying them remotely.
  *
  * Make sure that the files communication.h/.c are up-to-date with the Excel
  * spreadsheet "Protocol description.xlsx".
  *
  * Call comm_Init() to setup this module. Its interrupt function will be called
  * automatically when a message arrives, or periodically when the data
  * streaming is enabled.
  *
  * @addtogroup Communication
  * @{
  */

#include <stdbool.h>

// Message IDs sent by host (PC) to the device (STM).
typedef enum {
  PC_MESSAGE_DO_NOTHING = 0, ///< Do nothing.
  PC_MESSAGE_PING, ///< Request the board an answer, to check the connection
                   /// status.
  PC_MESSAGE_GET_VARS_LIST,    ///< Request the SyncVars list.
  PC_MESSAGE_SET_STREAMED_VAR, ///< Set the variables to be streamed
                               /// continuously.
  PC_MESSAGE_GET_VAR, ///< Request the device to send the selected value.
  PC_MESSAGE_SET_VAR  ///< Set the selected variable.
} comm_PcMessage;

// Message IDs sent by device (STM) to the device (PC).
typedef enum {
  STM_MESSAGE_PINGBACK = 0,     ///< Response to a ping request.
  STM_MESSAGE_VAR,              ///< Variable state.
  STM_MESSAGE_STREAMING_PACKET, ///< Streaming packet.
  STM_MESSAGE_DEBUG_TEXT,       ///< Debug text message.
  STM_MESSAGE_VARS_LIST,        ///< Monitored variables list.
  STM_MESSAGE_START_INFO ///< Notification that the board has (re)started.
} comm_StmMessage;

// SyncVar.
#define N_SYNCVARS_MAX 255 // Maximum number of SyncVars.
#define SYNCVAR_NAME_SIZE                                                      \
  50 // Max size of a SyncVar name, including the '\0' trailing character.

typedef enum { READONLY = 0, WRITEONLY, READWRITE } comm_VarAccess;

typedef enum {
  BOOL = 0,
  UINT8,
  INT8,
  UINT16,
  INT16,
  UINT32,
  INT32,
  UINT64,
  INT64,
  FLOAT32,
  FLOAT64
} comm_VarType;

// UART baudrate [b/s].
//#define UART_BAUDRATE 1843200

typedef struct
{
    char name[SYNCVAR_NAME_SIZE];
    void *address;
    comm_VarType type;
    uint8_t size;
    comm_VarAccess access;
    bool usesVarAddress;
    void (*getFunc)(void);
    void (*setFunc)(void);
} comm_SyncVar;


void comm_Init(void);
void comm_Step(void);

void comm_NotifyReady(void);

void comm_monitorVar(const char name[], void *address, comm_VarType type,
                     uint8_t size, comm_VarAccess access);
void comm_monitorVarFunc(const char name[], comm_VarType type, uint8_t size,
                         void (*getFunc)(void), void (*setFunc)(void));
                     
void comm_monitorBool(const char name[], bool *address,
                      comm_VarAccess access);
void comm_monitorUint8(const char name[], uint8_t *address,
                       comm_VarAccess access);
void comm_monitorInt8(const char name[], int8_t *address,
                      comm_VarAccess access);
void comm_monitorUint16(const char name[], uint16_t *address,
                        comm_VarAccess access);
void comm_monitorInt16(const char name[], int16_t *address,
                       comm_VarAccess access);
void comm_monitorUint32(const char name[], uint32_t *address,
                        comm_VarAccess access);
void comm_monitorInt32(const char name[], int32_t *address,
                       comm_VarAccess access);
void comm_monitorUint64(const char name[], uint64_t *address,
                        comm_VarAccess access);
void comm_monitorInt64(const char name[], int64_t *address,
                       comm_VarAccess access);
void comm_monitorFloat(const char name[], float *address,
                       comm_VarAccess access);
void comm_monitorDouble(const char name[], double *address,
                        comm_VarAccess access);
                        
void comm_monitorBoolFunc(const char name[],
                          bool (*getFunc)(void), void (*setFunc)(bool));
void comm_monitorUint8Func(const char name[],
                           uint8_t (*getFunc)(void), void (*setFunc)(uint8_t));
void comm_monitorInt8Func(const char name[],
                          int8_t (*getFunc)(void), void (*setFunc)(int8_t));
void comm_monitorUint16Func(const char name[],
                            uint16_t (*getFunc)(void),
                            void (*setFunc)(uint16_t));
void comm_monitorInt16Func(const char name[],
                           int16_t (*getFunc)(void), void (*setFunc)(int16_t));
void comm_monitorUint32Func(const char name[],
                            uint32_t (*getFunc)(void),
                            void (*setFunc)(uint32_t));
void comm_monitorInt32Func(const char name[],
                           int32_t (*getFunc)(void), void (*setFunc)(int32_t));
void comm_monitorUint64Func(const char name[],
                            uint64_t (*getFunc)(void),
                            void (*setFunc)(uint64_t));
void comm_monitorInt64Func(const char name[],
                           int64_t (*getFunc)(void), void (*setFunc)(int64_t));
void comm_monitorFloatFunc(const char name[],
                           float (*getFunc)(void), void (*setFunc)(float));
void comm_monitorDoubleFunc(const char name[],
                            double (*getFunc)(void), void (*setFunc)(double));

void comm_LockSyncVarsList(void);

void comm_SendDebugMessage(const char *format, ...);

/**
 * @brief Sends a debug message to the computer, with decimation.
 * This macro is useful to print human-readable text, in a fast loop, to avoid
 * overloading the communication bus, or the computer.
 * @param decimation this macro will actually print once out of decimation, and
 * do nothing otherwise.
 * @param format format string. See the printf() documentation for format
 * specification.
 * @param ... variables to be printed in the format string.
 */ 
#define comm_SendDebugMessageDecimated(decimation, format, ...) \
do \
{ \
    static int comm_decim##__COUNTER__ = 0; \
    if(comm_decim##__COUNTER__++ % decimation == 0) \
    { \
        comm_SendDebugMessage(format, ##__VA_ARGS__); \
    } \
} while(0)

/**
  * @}
  */


#endif
