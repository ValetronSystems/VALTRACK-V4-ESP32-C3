/***************************************************************************
* File name :   CircularBuffer.h                                           *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/

#ifndef CIRCULARBUFFER_H
#define CIRCULARBUFFER_H

#include "SCI.h"

#define IDLE_EVENT_CODE                     0x00
#define ASIC_LINK_EVENT_CODE                0x01
#define INT_AUD_AMP_EVENT_CODE              0x02
#define INT_WIRELESS_EVENT_CODE             0x03
#define IR_DATA_EVENT_CODE                  0x04
#define IR_DATA_REPEAT_EVENT_CODE           0x05
#define TEMPERATURE_EVENT_CODE              0x06
#define _10_MILLISECOND_TIMER_EVENT_CODE    0x07

#define MUTE_EVENT_CODE                     0x08
#define RECEIVER_UPDATE_EVENT               74
#define VOLUME_EVENT_CODE                   0x0A


#define EVENT_NULL                  0   // NULL event (nothing happened)

#define FIFO_OVERRUN_OCCURED          0x28
#define NO_EVENT_TO_SERVICE           0x29
  
#define EVENT_QUEUE 1
#define PING_QUEUE  2

#define POST_SUCCESS 1
#define GET_SUCCESS  2
#define POST_FAILED  3
#define GET_FAILED   4

#define EEPROM_FIFO
//
// Circular buffer functions
//
extern unsigned short HeadIndex;
extern unsigned short TailIndex;
extern unsigned char GPacketCacheIndex;  
#ifdef EEPROM_FIFO



#define EE_EVENTS_INDEX_MASK 0x3FF//0x7FF  // Note must be (power of 2) minus 1
#define EE_EVENTS_INDEX_MASK_HALF 0x1FF//0x3FF 


#define EE_HEADINDEX_ADDR 11
#define EE_TAILINDEX_ADDR 13

#define EE_PARAMS_ADDR 50


#define EE_PARAMS_LENGTH 550+100 // 100 for client ID

typedef union EEPARAMS
{
    
    unsigned char Bytes[EE_PARAMS_LENGTH];
    struct 
    {
        unsigned int PingInterval;
        char WorkingMode[5];
        char MotionAlertMode[5];
        unsigned char MotionThreshold;
        char HTTPURL[150];
        char HTTPKey[100];
        char APNName[20];
        char APNUsername[20];
        char APNPassword[20];
        char Band[30];
        char rxNumber[16];
        char MQTTHost[30];
        char MQTTPort[10];
        char MQTTClientID[120];
        char MQTTTopic[30];
        char MQTTProtocolName[10];
        unsigned char MQTTLVL;
        unsigned char MQTTFlags;
        unsigned int MQTTKeepAlive;
        char MQTTUsername[30];
        char MQTTPassword[35];
        
        
    }Fields;
}ParamsType;

extern ParamsType Params;

extern unsigned short EEHeadIndex;
extern unsigned short EETailIndex;
extern unsigned short EEEventOverRunCounter;
extern unsigned short EENoEventToProcessCounter;
extern unsigned long EEPacketAddr;//,tEEAddr;
extern unsigned char sEEAddress;
//
#endif
typedef enum QueueTypes
{
    RAMQueue=0,
    ROMQueue
}QueueType;
 

//*****************************************************************
//
// Function     :   PostEvent
// Purpose      :   Post HW event to circular buffer for later
//                  processing by background SW code
//
// Called with  :   Pointer to Hardware Event type structure
// Returns      :   SUCCESS if OK, of FIFO_OVERRUN_OCCURRED
//                  if not enough room to store the event
//
//*****************************************************************
unsigned char PostEvent(HWEventDataType *pHardwareEvent);

//*****************************************************************
//
// Function     :   PostEvent
// Purpose      :   Post HW event to circular buffer for later
//                  processing by background SW code
//
// Called with  :   Pointer to Hardware Event type structure
// Returns      :   SUCCESS if event copied from FIFO
//                  NO_EVENT_TO_SERVICE if FIFO is empty
//                  In either case copies data into structure
//                  via the pointer passed to it in the call
//                  to the function.
//                  
//            
//*****************************************************************
unsigned char GetEvent(HWEventDataType *pHardwareEvent, unsigned char Queue);




//*****************************************************************
//
// Function     :   PostEEEvent
// Purpose      :   Post HW event to circular buffer for later
//                  processing by background SW code
//
// Called with  :   Pointer to Hardware Event type structure
// Returns      :   SUCCESS if OK, of FIFO_OVERRUN_OCCURRED
//                  if not enough room to store the event
//
//*****************************************************************
unsigned char PostEEEvent(HWEventDataType *pHardwareEvent);

//*****************************************************************
//
// Function     :   GetEEEvent
// Purpose      :   Get HW event from circular buffer for later
//                  processing by background SW code
//
// Called with  :   Pointer to Hardware Event type structure
// Returns      :   SUCCESS if event copied from FIFO
//                  NO_EVENT_TO_SERVICE if FIFO is empty
//                  In either case copies data into structure
//                  via the pointer passed to it in the call
//                  to the function.
//                  
//            
//*****************************************************************
unsigned char GetEEEvent(HWEventDataType *pHardwareEvent); 
void GetEEWord(unsigned short *pData, unsigned short EEAddress);
void StoreEEWord(unsigned short *pData, unsigned short EEAddress);    
void ReadEEIndexes(void);

void GetEEParams(void);

void StoreEEParams(void);

unsigned char IsQueueEmpty(QueueType Queue);
void BackupPackets(void);
void RestorePackets(void);
void EraseEEPROMPackets(void);

void SaveToEventCache(HWEventDataType *pHardwareEvent);
void RestoreEventCache(void);
void ClearEventCache(void);
#endif

// EOF CircularBuffer.h
