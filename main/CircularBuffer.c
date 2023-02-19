/***************************************************************************
* File name :   CircularBuffer.c                                           *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/

#include <string.h>
#include <stdio.h>
#include "SCI.h"
#include "at24c.h"
#include "CircularBuffer.h"
#define EVENTS_INDEX_MASK 0x7FF//0x1F  // Note must be (power of 2) minus 1 0x3E=62,0x7C=124,0xF8=248,0x1F0=496
#define EVENTS_INDEX_MASK_2 0x1F  // Note must be (power of 2) minus 1






unsigned short HeadIndex=0;
unsigned short TailIndex=0;
unsigned short HeadIndex2=0;
unsigned short TailIndex2=0;

//
// Queue can accomodate N events where N is INDEX mask
// plus one and number of events is a power of 2
//.
HWEventDataType EventQueue[EVENTS_INDEX_MASK+1];
//HWEventDataType EventQueue2[EVENTS_INDEX_MASK_2+1];
HWEventDataType GPacketCache[PACKET_COUNT];
unsigned char GPacketCacheIndex=0;        
//
// Circular buffer statistic counters
// AKB Changed from RESET to 0 as these are counters
// and RESET is an bit or function action
//
unsigned short NoEventToProcessCounter = 0;
unsigned short EventOverRunCounter     = 0;

unsigned short NoEventToProcessCounter2 = 0;
unsigned short EventOverRunCounter2     = 0;

//extern double fLat,fLong;
extern unsigned char Speed[];
extern unsigned char fuel[];
void GetPicture(void);
void GetFuelValue( HWEventDataType *pHardwareEvent);
unsigned char IsQueueEmpty(QueueType Queue)
{
    switch(Queue)
    {
        case RAMQueue:
            if(HeadIndex == TailIndex) 
            {
                return 0;
            }
            else 
            {
                return 1;
            }
        case ROMQueue:
//            if(HeadIndex2 == TailIndex2) 
//            {
//                return 0;
//            }
//            else 
//            {
                return 1;
//            }
        //break;
        default:
            return 1;
        //break;
        
    }
}

void SaveToEventCache(HWEventDataType *pHardwareEvent)
{
    
    if(GPacketCacheIndex<PACKET_COUNT) 
    {
        memcpy(GPacketCache[GPacketCacheIndex].Bytes,pHardwareEvent->Bytes,sizeof(HWEventDataType));
        GPacketCacheIndex++;
        //GPacketCacheIndex = 0;
    }
}
void RestoreEventCache(void)
{    
    for(int i=0;i<GPacketCacheIndex;i++)
    {
        PostEvent(&GPacketCache[i]);       
    }
    GPacketCacheIndex = 0;
}
void ClearEventCache(void)
{    
    for(int i=0;i<PACKET_COUNT;i++)
    {
        memset(GPacketCache[i].Bytes,0,sizeof(HWEventDataType));
    }
    GPacketCacheIndex = 0;
}
void BackupPackets(void)
{
    unsigned char i=0;
    HWEventDataType TPacket;
    RestoreEventCache(); // Save cache data
    while(GetEvent(&TPacket,EVENT_QUEUE) == GET_SUCCESS)
    {
        //TBD//PostEEEvent(&TPacket);
        i++;
        if(i>100)
            break;
    }
}
extern unsigned short LPUARTTimer;
void RestorePackets(void)
{
    //TBD
    // unsigned char i=0;
    // HWEventDataType TPacket;
    // while(GetEEEvent(&TPacket) == GET_SUCCESS)
    // {
    //     LPUARTTimer=0; // Clear the timer so that it doesnt reset
    //     PostEvent(&TPacket);
    //     i++;
    //     if(i>100)
    //         break;
    // }
}
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
unsigned char PostEvent(HWEventDataType *pHardwareEvent)
{
    unsigned char i;
    unsigned char Queue;
    unsigned short TempIndex=0; // For Stupid ESP ERROR
    
    //if((pHardwareEvent->GEvent.FunctionCode[0] == '3') && (pHardwareEvent->GEvent.FunctionCode[1] == 'B'))
    //{
    //    GetPicture();
    //    return POST_SUCCESS;  // No need to post just return    
    //}
    if(
          ((pHardwareEvent->GEvent.EventType == 0x23))
       || ((pHardwareEvent->GEvent.EventType == 0x24))
      )
    {
        Queue = EVENT_QUEUE;
    }
    else
    {
        Queue = EVENT_QUEUE;
    }
    
    
    if(Queue == EVENT_QUEUE)
    {
        //
        // Test if  posting this event would cause
        // an overrun to occur    
        if( TailIndex == ((HeadIndex-1) & EVENTS_INDEX_MASK))
        {
            //PostEEEvent(pHardwareEvent);
            EventOverRunCounter++;
            for(i=0;i<HW_EVENT_LENGTH;i++)
            {
                EventQueue[TailIndex].Bytes[i]=SYSTEM_EVENT_FIFO_OVERRUN;
            }
            return FIFO_OVERRUN_OCCURED;        
        
        }
        
            ////if((pHardwareEvent->GEvent.EventType[0] == '0') && ((pHardwareEvent->GEvent.EventType[1] == '2') || (pHardwareEvent->GEvent.EventType[1] == '3')))
            
            //if((pHardwareEvent->GEvent.FunctionCode[0] == '3') && (pHardwareEvent->GEvent.FunctionCode[1] == 'B'))
            //{
            //        GetPicture();
            //}
        //
        // Room to store event, copy data to circular buffer
        //
        for(i=0;i<HW_EVENT_LENGTH;i++)
        {
            EventQueue[TailIndex].Bytes[i]=pHardwareEvent->Bytes[i];
        }
//        EventQueue[TailIndex].GEvent.Lat = fLat;
//        EventQueue[TailIndex].GEvent.Long = fLong;
        //for(i=0;i<10;i++)
        {
            EventQueue[TailIndex].GEvent.Speed= fSpeed;
        }
				
        //EventQueue[TailIndex].GEvent.Speed[9] = '\0';
        TempIndex=((++TailIndex) & EVENTS_INDEX_MASK); // Increment TailIndex;
        TailIndex = TempIndex;
        return POST_SUCCESS;
    }
    /*else if(Queue == PING_QUEUE)
    {
        //
        // Test if  posting this event would cause
        // an overrun to occur    
        if( TailIndex2 == ((HeadIndex2-1) & EVENTS_INDEX_MASK_2))
        {
            EventOverRunCounter2++;
            for(i=0;i<HW_EVENT_LENGTH;i++)
            {
                EventQueue2[TailIndex2].Bytes[i]=SYSTEM_EVENT_FIFO_OVERRUN;
            }
            return FIFO_OVERRUN_OCCURED;        
        
        }
        
            ////if((pHardwareEvent->GEvent.EventType[0] == '0') && ((pHardwareEvent->GEvent.EventType[1] == '2') || (pHardwareEvent->GEvent.EventType[1] == '3')))
            
            //if((pHardwareEvent->GEvent.FunctionCode[0] == '3') && (pHardwareEvent->GEvent.FunctionCode[1] == 'B'))
            //{
            //        GetPicture();
            //}
        //
        // Room to store event, copy data to circular buffer
        //
        for(i=0;i<HW_EVENT_LENGTH;i++)
        {
            EventQueue2[TailIndex2].Bytes[i]=pHardwareEvent->Bytes[i];
        }
        EventQueue2[TailIndex2].GEvent.fLat = fLat;
        EventQueue2[TailIndex2].GEvent.fLong = fLong;
        for(i=0;i<10;i++)
        {
            EventQueue2[TailIndex2].GEvent.Speed[i]= Speed[i];
        }
				
				
        GetFuelValue(pHardwareEvent);
						
        
        EventQueue2[TailIndex2].GEvent.Speed[9] = '\0';
        TailIndex2=((++TailIndex2) & EVENTS_INDEX_MASK_2); // Increment TailIndex;
        
        return POST_SUCCESS;
    }*/
    else
        return POST_FAILED;
    
}

//*****************************************************************
//
// Function     :   GetEvent
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
unsigned char GetEvent(HWEventDataType *pHardwareEvent,unsigned char Queue) 
{
    unsigned char i;
    unsigned short TempIndex=0; // For Stupid ESP ERROR
    if(Queue == EVENT_QUEUE)
    {    
        if(HeadIndex==TailIndex)      // If there is no event Posted then
        {                             // return an Error code along with 
                                      // a flag being set.
            
            NoEventToProcessCounter++;
            for(i=0;i<HW_EVENT_LENGTH;i++)
            {
                pHardwareEvent->Bytes[i]=EVENT_NULL;
            }    
            return NO_EVENT_TO_SERVICE;
            
        }
        
        else
        {
            for(i=0;i<HW_EVENT_LENGTH;i++)
            {
                pHardwareEvent->Bytes[i]=EventQueue[HeadIndex].Bytes[i];
            }
            TempIndex=((++HeadIndex) & EVENTS_INDEX_MASK); // Increment HeadIndex;
            HeadIndex = TempIndex;
            SaveToEventCache(pHardwareEvent);
            return GET_SUCCESS;
            
        }
    }
    /*else if(Queue == PING_QUEUE)
    {    
        if(HeadIndex2==TailIndex2)      // If there is no event Posted then
        {                             // return an Error code along with 
                                      // a flag being set.
            
            NoEventToProcessCounter2++;
            for(i=0;i<HW_EVENT_LENGTH;i++)
            {
                pHardwareEvent->Bytes[i]=EVENT_NULL;
            }    
            return NO_EVENT_TO_SERVICE;
            
        }
        
        else
        {
            for(i=0;i<HW_EVENT_LENGTH;i++)
            {
                pHardwareEvent->Bytes[i]=EventQueue2[HeadIndex2].Bytes[i];
            }
            HeadIndex2=((++HeadIndex2) & EVENTS_INDEX_MASK_2); // Increment HeadIndex;
            
            return GET_SUCCESS;
            
        }
    }*/
    else
        return GET_FAILED;
    
}// EOF GetEvent().




#ifdef EEPROM_FIFO




unsigned short EEHeadIndex=0;
unsigned short EETailIndex=0;
unsigned short EEEventOverRunCounter = 0;
unsigned short EENoEventToProcessCounter = 0;
unsigned long EEPacketAddr;//,tEEAddr;
unsigned char sEEAddress;



ParamsType Params;
ParamsType *pEEParams;

//
// GPS Log
//



void GetEEParams(void)
{
    // unsigned short j;
    // sEEAddress  = 0xA0; // Store in first page
    // for(j=0;j<EE_PARAMS_LENGTH;j++)
    // {
    //     ReadRom(EEPROM_t * dev, uint16_t data_addr, uint8_t * data)
    //     HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EE_PARAMS_ADDR+j, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[j], 1, 10000);
    //     //osDelay(5);
    // }
    
    // //HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EE_PARAMS_ADDR, I2C_MEMADD_SIZE_16BIT,(uint8_t*) Params.Bytes, sizeof(ParamsType), 10000);
    // //osDelay(100);
    // /*HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EE_PARAMS_ADDR, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[0], 200, 10000);
    // osDelay(100);
    // HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EE_PARAMS_ADDR+200, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[200], 200, 10000);
    // osDelay(100);*/
    
}
void StoreEEParams(void)
{
    // unsigned short j;
    // sEEAddress  = 0xA0; // Store in first page
    // for(j=0;j<EE_PARAMS_LENGTH;j++)
    // {
    //     HAL_I2C_Mem_Write(&hi2c1, sEEAddress, EE_PARAMS_ADDR+j, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[j], 1, 10000);
    //     osDelay(5);
    // }
   
    // /*osDelay(100);
    // HAL_I2C_Mem_Write(&hi2c1, sEEAddress, EE_PARAMS_ADDR+200, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[200], 200, 10000);
    // osDelay(100);*/
    
    // GetEEParams();
    // /*HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EE_PARAMS_ADDR, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[0], 200, 10000);
    // osDelay(100);
    // HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EE_PARAMS_ADDR+200, I2C_MEMADD_SIZE_16BIT,(uint8_t*) &Params.Bytes[200], 200, 10000);
    // osDelay(100);*/
}

void GetEEWord(unsigned short *pData, unsigned short EEAddress)
{
    // uint16_t nBR; 
    
    // // Initialize the I2C EEPROM driver ----------------------------------------//
    // //sEE_Init();  
    // // Wait for EEPROM standby state 
    // //sEE_WaitEepromStandbyState();  
    
    // sEEAddress  = 0xA0; // Store in first page
    // nBR = 2;
    // //sEE_ReadBuffer((uint8_t*)pData, EEAddress, &nBR); 
    // HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EEAddress, I2C_MEMADD_SIZE_16BIT,(uint8_t*) pData, nBR, 10000);
    // osDelay(100);
    // //sEE_DeInit();   
    
}
void StoreEEWord(unsigned short *pData, unsigned short EEAddress)
{
    // uint16_t nBR; 
    
    // // Initialize the I2C EEPROM driver ----------------------------------------//
    // //sEE_Init();  
    // // Wait for EEPROM standby state 
    // //sEE_WaitEepromStandbyState();  
    
    // sEEAddress  = 0xA0; // Store in first page
    // nBR = 2;
    
    // //sEE_WriteBuffer((uint8_t*)pData, EEAddress, nBR);
    // HAL_I2C_Mem_Write(&hi2c1, sEEAddress, EEAddress, I2C_MEMADD_SIZE_16BIT,(uint8_t*) pData, nBR, 10000);
    // nBR = 2;
    // osDelay(100);
    // //sEE_ReadBuffer((uint8_t*)pData, EEAddress, &nBR); 
    // HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EEAddress, I2C_MEMADD_SIZE_16BIT,(uint8_t*) pData, nBR, 10000);
    // osDelay(100);
    // //sEE_DeInit();   
}
void ReadEEIndexes(void)
{
    GetEEWord(&EEHeadIndex,EE_HEADINDEX_ADDR);
    GetEEWord(&EETailIndex,EE_TAILINDEX_ADDR);
    
    if(EEHeadIndex == 65535)
    {
        EEHeadIndex = 0;
        StoreEEWord(&EEHeadIndex,EE_HEADINDEX_ADDR);
    }
    if(EETailIndex == 65535)
    {
        EETailIndex = 0;
        StoreEEWord(&EETailIndex,EE_TAILINDEX_ADDR);
    }
}

HWEventDataType ReadEvent;
unsigned char DBGString[25];
//void ConvertToJSON(HWEventDataType *pPacket,unsigned short *pDataLength, unsigned char *pStr);
void StoreEEPacket(HWEventDataType *pHardwareEvent)
{
    // //unsigned char PrintString[250];
    // //uint16_t nBR; 
    // unsigned short SearchIndex=0,SearchIndex2=0,i=0;//,DataLength;
    // unsigned char LEraseByte=0xFF;
    // for(SearchIndex=1500;SearchIndex<=60000;SearchIndex+=sizeof(HWEventDataType))
    // {
    //     HAL_I2C_Mem_Read(&hi2c1, 0xA0, SearchIndex, I2C_MEMADD_SIZE_16BIT,(uint8_t*)ReadEvent.Bytes, sizeof(HWEventDataType), 10000);
        
    //     if(ReadEvent.GEvent.Hours==0xFF)
    //     {
    //         for(i=0;i<sizeof(HWEventDataType);i++)
    //         {
    //             HAL_I2C_Mem_Write(&hi2c1, 0xA0, SearchIndex+i, I2C_MEMADD_SIZE_16BIT,(uint8_t*)&pHardwareEvent->Bytes[i], 1, 10000);
    //             //osDelay(10);
    //             osDelay(5);
    //         }
    //         HAL_I2C_Mem_Read(&hi2c1, 0xA0, SearchIndex, I2C_MEMADD_SIZE_16BIT,(uint8_t*)ReadEvent.Bytes, sizeof(HWEventDataType), 10000);
    //         sprintf((void*)DBGString,"Saved %d-%d\r\n",SearchIndex,SearchIndex/sizeof(HWEventDataType));
    //         //Print1(DBGString);
    //         //ConvertToJSON(&ReadEvent,&DataLength,PrintString);
    //         //Print1((void*)"Saved n Read back:\r\n");
    //         //Print1(PrintString);
    //         //Print1((void*)"\r\n");
    //         if(SearchIndex/sizeof(HWEventDataType) == 512) // If reached half mark, clear end half
    //         {
    //             for(SearchIndex2=SearchIndex+sizeof(HWEventDataType);SearchIndex2<=60000;SearchIndex2+=sizeof(HWEventDataType))
    //             {                    
    //                 for(i=0;i<sizeof(HWEventDataType);i++)
    //                 {                        
    //                    HAL_I2C_Mem_Write(&hi2c1, 0xA0, SearchIndex2+i, I2C_MEMADD_SIZE_16BIT,(void*)&LEraseByte,1, 10000); 
    //                    osDelay(5);
    //                 }                    
                                        
    //             }
    //         }
    //         if(SearchIndex/sizeof(HWEventDataType) == 974) // If reached end mark, clear first half 58500/60bytes (60000-1500)
    //         {
    //             for(SearchIndex2=1500;SearchIndex2<=1500+(sizeof(HWEventDataType)*512);SearchIndex2+=sizeof(HWEventDataType))
    //             {                    
    //                 for(i=0;i<sizeof(HWEventDataType);i++)
    //                 {                        
    //                    HAL_I2C_Mem_Write(&hi2c1, 0xA0, SearchIndex2+i, I2C_MEMADD_SIZE_16BIT,(void*)&LEraseByte,1, 10000); 
    //                    osDelay(5);
    //                 }
    //             }
    //         }
    //         return;
    //     }
    // }
    
    
    // for(SearchIndex=1500;SearchIndex<=60000;SearchIndex+=sizeof(HWEventDataType))
    // {
    //      HAL_I2C_Mem_Read(&hi2c1, 0xA0, SearchIndex, I2C_MEMADD_SIZE_16BIT,(uint8_t*)ReadEvent.Bytes, sizeof(HWEventDataType), 10000);
        
    //     if(ReadEvent.Bytes[0]==0xFF)
    //     {
    //         for(i=0;i<sizeof(HWEventDataType);i++)
    //         {
                
    //            HAL_I2C_Mem_Write(&hi2c1, 0xA0, SearchIndex+i, I2C_MEMADD_SIZE_16BIT,(void*)&LEraseByte,1, 10000); 
    //            osDelay(10);
    //         }
    //         return;
    //     }
        
    // }
    
   
}
void EraseEEPROMPackets(void)
{
    // //unsigned char PrintString[250];
    // //uint16_t nBR; 
    // unsigned short SearchIndex=0,i=0;
    // unsigned char LEraseByte=0xFF;
    // for(SearchIndex=1500;SearchIndex<=65000;SearchIndex++)
    // {
    //     sprintf((void*)DBGString,"Erasing EEPROM: %d\r\n",SearchIndex);
    //     HAL_I2C_Mem_Write(&hi2c1, 0xA0, SearchIndex+i, I2C_MEMADD_SIZE_16BIT,(void*)&LEraseByte,1, 10000); 
    //     osDelay(5);                  
    // }
   
}
void StoreEEPacket_lib(HWEventDataType *pHardwareEvent)
{
    // //uint16_t nBR; 

    // EEPacketAddr=((sizeof(HWEventDataType) * (EETailIndex % (EE_EVENTS_INDEX_MASK_HALF))))+1000;
    // // Initialize the I2C EEPROM driver ----------------------------------------//
    // //sEE_Init();  
    // // Wait for EEPROM standby state 
    // //sEE_WaitEepromStandbyState();
    // //__disable_irq();            
    
    // if(EETailIndex < EE_EVENTS_INDEX_MASK_HALF) sEEAddress = 0xA0;
    // if(EETailIndex >= EE_EVENTS_INDEX_MASK_HALF) sEEAddress = 0xA2;
    
    // // Write on I2C EEPROM from sEE_WRITE_ADDRESS2             
    // //sEE_WriteBuffer((uint8_t*)pHardwareEvent->Bytes, EEPacketAddr, sizeof(HWEventDataType)); 
    // HAL_I2C_Mem_Write(&hi2c1, sEEAddress, EEPacketAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)pHardwareEvent->Bytes, sizeof(HWEventDataType), 10000);
    
    // osDelay(100);
    
    // memset((uint8_t*)pHardwareEvent->Bytes,0,sizeof(HWEventDataType));
    // //nBR = sizeof(HWEventDataType);
    // //sEE_ReadBuffer((uint8_t*)pHardwareEvent->Bytes, EEPacketAddr, &nBR); 
    // HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EEPacketAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)pHardwareEvent->Bytes, sizeof(HWEventDataType), 10000);
    // osDelay(100);
    // //sEE_DeInit();  
 
}
void ReadEEPacket(HWEventDataType *pHardwareEvent)
{
    // //uint16_t nBR;
    // //unsigned char str[50];
    // //unsigned char i,data;
    // //unsigned char FilenameBuff[50];
    // //unsigned char *pToken;
    // //unsigned long j;
    
    // EEPacketAddr=((sizeof(HWEventDataType) * (EEHeadIndex % (EE_EVENTS_INDEX_MASK_HALF))))+1000;
    // // Initialize the I2C EEPROM driver ----------------------------------------//
    // //sEE_Init();  
    // // Wait for EEPROM standby state 
    // //sEE_WaitEepromStandbyState();
    // //__disable_irq();            
    
    // if(EEHeadIndex < EE_EVENTS_INDEX_MASK_HALF) sEEAddress = 0xA0;
    // if(EEHeadIndex >= EE_EVENTS_INDEX_MASK_HALF) sEEAddress = 0xA2;
    
    
    
    // //nBR = sizeof(HWEventDataType);
    // //sEE_ReadBuffer((uint8_t*)pHardwareEvent->Bytes, EEPacketAddr, &nBR); 
    // HAL_I2C_Mem_Read(&hi2c1, sEEAddress, EEPacketAddr, I2C_MEMADD_SIZE_16BIT,(uint8_t*)pHardwareEvent->Bytes, sizeof(HWEventDataType), 10000);
    
    // //ysEE_DeInit();   
     
}

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
HWEventDataType *pHardwareEventView;
unsigned char PostEEEvent(HWEventDataType *pHardwareEvent)
{
    //unsigned char i;
    //unsigned char Queue;
//    pHardwareEventView=pHardwareEvent;
//    
//    //GetEEWord(&EEHeadIndex,EE_HEADINDEX_ADDR);
//    //GetEEWord(&EETailIndex,EE_TAILINDEX_ADDR);
//    ReadEEIndexes();
//    
//    //
//    // Test if  posting this event would cause
//    // an overrun to occur    
//    if( EETailIndex == ((EEHeadIndex-1) & EE_EVENTS_INDEX_MASK))
//    {
//        EEEventOverRunCounter++;
//        /*for(i=0;i<HW_EVENT_LENGTH;i++)
//        {
//            EventQueue[EETailIndex].Bytes[i]=SYSTEM_EVENT_FIFO_OVERRUN;
//        }*/
//        
//        return FIFO_OVERRUN_OCCURED;        
//    
//    }
//    
    
    //
    // Room to store event, copy data to circular buffer
    //
    StoreEEPacket(pHardwareEvent);
    /*for(i=0;i<HW_EVENT_LENGTH;i++)
    {
        EventQueue[EETailIndex].Bytes[i]=pHardwareEvent->Bytes[i];
    }*/
   
            
    
//    EETailIndex=((++EETailIndex) & EE_EVENTS_INDEX_MASK); // Increment TailIndex;
//    
//    StoreEEWord(&EETailIndex,EE_TAILINDEX_ADDR);
    
    return POST_SUCCESS;
    
    
    
}

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
HWEventDataType GReadEvent;
unsigned char GDBGString[25];
unsigned char GetEEEvent(HWEventDataType *pHardwareEvent) 
{
    // //unsigned char PrintString[250];
    // unsigned short SearchIndex=0;//,DataLength;
    // unsigned char LEraseByte=0xFF;
    // for(SearchIndex=1500;SearchIndex<=60000;SearchIndex+=sizeof(HWEventDataType))
    // {
    //      HAL_I2C_Mem_Read(&hi2c1, 0xA0, SearchIndex, I2C_MEMADD_SIZE_16BIT,(uint8_t*)GReadEvent.Bytes, sizeof(HWEventDataType), 10000);
        
    //     if(GReadEvent.Bytes[0]!=0xFF)
    //     {
    //         memcpy(pHardwareEvent->Bytes,GReadEvent.Bytes,sizeof(HWEventDataType));
    //         sprintf((void*)GDBGString,"Got %d-%d\r\n",SearchIndex,SearchIndex/sizeof(HWEventDataType));
    //         //Print1(GDBGString);
            
    //         //ConvertToJSON(&GReadEvent,&DataLength,PrintString);
    //         //Print1((void*)"Got back:\r\n");
    //         //Print1(PrintString);
    //         //Print1((void*)"\r\n");
            
     
    //         //for(i=0;i<sizeof(HWEventDataType);i++)
    //         {
    //            HAL_I2C_Mem_Write(&hi2c1, 0xA0, SearchIndex, I2C_MEMADD_SIZE_16BIT,(void*)&LEraseByte,1, 10000); 
    //            osDelay(10);
    //         }
        
        
    
    //         return GET_SUCCESS;
    //     }
    // }
     return NO_EVENT_TO_SERVICE;
}

unsigned char GetEEEvent_Lib(HWEventDataType *pHardwareEvent) 
{
    //unsigned char i;
    unsigned short TempIndex=0; // For Stupid ESP ERROR
    
    //GetEEWord(&EEHeadIndex,EE_HEADINDEX_ADDR);
    //GetEEWord(&EETailIndex,EE_TAILINDEX_ADDR);
    ReadEEIndexes();
    
    if(EEHeadIndex==EETailIndex)      // If there is no event Posted then
    {                             // return an Error code along with 
                                  // a flag being set.
        
        EENoEventToProcessCounter++;
        /*for(i=0;i<HW_EVENT_LENGTH;i++)
        {
            pHardwareEvent->Bytes[i]=EVENT_NULL;
        }*/    
        return NO_EVENT_TO_SERVICE;
        
    }    
    else
    {
        /*for(i=0;i<HW_EVENT_LENGTH;i++)
        {
            pHardwareEvent->Bytes[i]=EventQueue[EEHeadIndex].Bytes[i];
        }*/
        ReadEEPacket(pHardwareEvent);
        
        TempIndex=((++EEHeadIndex) & EE_EVENTS_INDEX_MASK); // Increment HeadIndex;
        EEHeadIndex = TempIndex;
        StoreEEWord(&EEHeadIndex,EE_HEADINDEX_ADDR);
        
        return GET_SUCCESS;
        
    }

    
}// EOF GetEvent().

#endif

// EOF CircularBuffer.c
