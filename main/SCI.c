/***************************************************************************
* File name :   SCI.c                                                      *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/

#include <string.h>
#include <stdio.h>
#include "esp_log.h"
#include "SCI.h"
#include "FrontPanel.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/uart.h"


unsigned char DebugPrintEnabled=0;
extern unsigned char WatchTimer;
char Buff[BUFF_SIZE],Buff2[BUFF2_SIZE];
unsigned short BuffIndex = 0,Buff2Index = 0;
//unsigned char ProcessPacket = 0;
unsigned long dCount;

unsigned char Byte1,Byte2,Byte3,Byte4,Byte5,Byte6;

char* pTokenStat;
    
unsigned char HeaderReceived;
unsigned char Header;
char Lat[10],Long[10];
char Altitude[10];
//http://maps.google.com/maps?z=18&q=10.8061,106.7130

char Link[100];
char D1,D2,GPSStatus='V';
float fLat,fLong,pfLat,pfLong,fSpeed,fAltitude;     
char*pData;
unsigned char CommaCount;


char GPSTime[20],GPSDate[20],GPSSpeed[20];
unsigned char GPSHours,GPSMinutes,GPSSeconds,GPSDay,GPSMonth,GPSYear;


extern unsigned short LPUARTTimer;
unsigned char AnswerCall = 0;
extern unsigned char NeedBTAttention;

unsigned char RingCount;

int dada;
extern char tBuff[];
unsigned char f;

static QueueHandle_t uart0_queue;
void osDelay(unsigned int Value)
{
    vTaskDelay(Value / portTICK_PERIOD_MS);
}
char sBuff[500];
static void uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    unsigned char Data=0;
    
    int i;
    uint8_t* dtmp = (uint8_t*) malloc(RD_BUF_SIZE);
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
            bzero(dtmp, RD_BUF_SIZE);
            //ESP_LOGI(TAG, "uart[%d] event:", UART_PORT_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    //ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
                    uart_read_bytes(UART_PORT_NUM, dtmp, event.size, portMAX_DELAY);
                    //ESP_LOGI(TAG, "[DATA EVT]:");
                    LPUARTTimer = 0;
                    for( i =0;i< event.size;i++)
                    {
                        Data = dtmp[i];
                        if(Data != 0x00)
                        {  Buff2[Buff2Index]=Data;    }//USART_ReceiveData(USART2);
                        else
                        { return;}
                            //if( (Buff[BuffIndex-1] == 0x0D)&&(Buff[BuffIndex] == 0x0A) )
                            //if( (Buff[BuffIndex-2] == 'R')&&(Buff[BuffIndex-1] == 'M')&&(Buff[BuffIndex] == 'C') )
                    
                        if( 
                            (Buff2[Buff2Index-3] == 'C')&&
                            (Buff2[Buff2Index-2] == 'L')&&
                            (Buff2[Buff2Index-1] == 'I')&&
                            (Buff2[Buff2Index] == 'P') )
                        {
                            if(++RingCount>=3)
                            {
                                //DDelay();
                                RingCount=0;
                                //WakeUp();
                                AnswerCall = 1;//Print("ATA\r\n");
                            }
                            ResetBuffer();
                        }
                        

                        if(NeedBTAttention == 1)
                        {
                            if(Buff2[Buff2Index] == '$')//if(MapForward(Buff2,BUFF2_SIZE,(char*)"+BTSPPDATA",10) != NULL)
                            {                
                                Buff2Index=0;//ResetBuffer();
                                Buff2[Buff2Index] = '$';
                            }
                            if(Buff2[Buff2Index] == '+')//if(MapForward(Buff2,BUFF2_SIZE,(char*)"+BTSPPDATA",10) != NULL)
                            {                
                                Buff2Index=0;//ResetBuffer();
                                Buff2[Buff2Index] = '+';
                            }
                        }
                    
                        Buff2Index++;
                        if(Buff2Index >= BUFF2_SIZE-1)
                            Buff2Index = 0;
                        //uart_write_bytes(UART_PORT_NUM, (const char*) dtmp, event.size);
                        sBuff[i]=Data;
                    }
                    sBuff[i]=0;
                    ESP_LOGI(TAG, "%s",sBuff); 
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(UART_PORT_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGI(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGI(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGI(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    uart_get_buffered_data_len(UART_PORT_NUM, &buffered_size);
                    int pos = uart_pattern_pop_pos(UART_PORT_NUM);
                    ESP_LOGI(TAG, "[UART PATTERN DETECTED] pos: %d, buffered size: %d", pos, buffered_size);
                    if (pos == -1) {
                        // There used to be a UART_PATTERN_DET event, but the pattern position queue is full so that it can not
                        // record the position. We should set a larger queue size.
                        // As an example, we directly flush the rx buffer here.
                        uart_flush_input(UART_PORT_NUM);
                    } else {
                        uart_read_bytes(UART_PORT_NUM, dtmp, pos, 100 / portTICK_PERIOD_MS);
                        uint8_t pat[PATTERN_CHR_NUM + 1];
                        memset(pat, 0, sizeof(pat));
                        uart_read_bytes(UART_PORT_NUM, pat, PATTERN_CHR_NUM, 100 / portTICK_PERIOD_MS);
                        ESP_LOGI(TAG, "read data: %s", dtmp);
                        ESP_LOGI(TAG, "read pat : %s", pat);
                    }
                    break;
                //Others
                default:
                    ESP_LOGI(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(dtmp);
    dtmp = NULL;
    vTaskDelete(NULL);
}

void InitUART(void)
{
    

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(UART_PORT_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &uart0_queue, 0);
    uart_param_config(UART_PORT_NUM, &uart_config);

    //Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(UART_PORT_NUM, ECHO_TEST_TXD, ECHO_TEST_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    //Set uart pattern detect function.
    uart_enable_pattern_det_baud_intr(UART_PORT_NUM, '+', PATTERN_CHR_NUM, 9, 0, 0);
    //Reset the pattern queue length to record at most 20 pattern positions.
    uart_pattern_queue_reset(UART_PORT_NUM, 20);

    //Create a task to handler UART event from ISR
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}



void  Binary2Ascii(unsigned long HexValue)
{  
    Byte1=0; 
	Byte2=0;
	Byte3=0;
    Byte4=0;
    Byte5=0;
    Byte6=0;

    while(HexValue >= 0x186A0 )
	{
	    ++(Byte6);
		HexValue=HexValue - 0x186A0;
    }
    while(HexValue >= 0x2710 )
	{
	    ++(Byte5);
		HexValue=HexValue - 0x2710;
    }
    while(HexValue >= 0x3E8 )
	{
	    ++(Byte4);
		HexValue=HexValue - 0x3E8;
    }
    while(HexValue >= 0x64 )
	{
	    ++(Byte3);
		HexValue=HexValue - 0x64;
    }

	while(HexValue >= 0x0A)
	{
	    ++(Byte2);
		HexValue=HexValue - 0x0A;
	}

	Byte1=HexValue;	   // Remainder is the lower decimal.

	Byte1=Byte1|0x30;  // Convert decimal to Ascii.
	Byte2=Byte2|0x30;  // Convert decimal to Ascii.
	Byte3=Byte3|0x30;  // Convert decimal to Ascii.
    Byte4=Byte4|0x30;  // Convert decimal to Ascii.
    Byte5=Byte5|0x30;  // Convert decimal to Ascii.
    Byte6=Byte6|0x30;  // Convert decimal to Ascii.
	
}
void RTOSDelay(unsigned int millis)
{
    unsigned int prevTickValue;
    prevTickValue = xTaskGetTickCount();
    
    while(xTaskGetTickCount() - prevTickValue < millis);
}

//void DelayProc(unsigned long Count)
//{
//    dCount = Count;
//    //IWDG_ReloadCounter();
//    while(dCount > 0) 
//    {
//        dCount--;
//        //WatchTimer = 0;
//        
//    }
//    
//}
void DelayProc(unsigned long Count)
{
    dCount = Count;
    
    osDelay(dCount/500);
    
}
void DelayProc1(unsigned long Count)
{
    dCount = Count;
    //IWDG_ReloadCounter();
    while(dCount > 0) 
    {
        dCount--;
        //WatchTimer = 0;
        
    }
    
}
unsigned char GetAscii(unsigned char Data)
{
	Data |= 0x30;
	if((Data & 0x0F) > 9)
	{
		Data += 7;
	}
	return Data;
}
unsigned char GetCheckSum(char*pData,unsigned char len)
{  unsigned char lrc,gc1; 
   lrc=0;
   for(gc1=0;gc1<len;gc1++)
   {
     lrc+=pData[gc1];
     
   }
   lrc^=0xff;lrc+=1;
   return lrc;
     
 }



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void SCIError_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

/*****************************************************************************
* Function Name : puchar                                                     *
*                                                                            *
* Description   : This function is used to print data onto the serial        *
*                 communications interface of STM8 type MCU                  *
*                                                                            *
* Arguments     : 1) Data to be printed                                      *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void putcchar(unsigned char Data)
{
  
    uart_write_bytes(UART_PORT_NUM, &Data,1);    
}
void putcchar1(unsigned char Data)
{    
    // unsigned long Count;
    // //HAL_UART_Transmit(&huart1, (uint8_t*)&Data, 1,10000);
    // USART1->ICR = USART_ICR_TCCF; /* Clear transfer complete flag */
    // USART1->TDR = Data;
    // while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)
    // {
    //     Count++;
    //     if(Count>150000)
    //         break;
    // }
    
    //USART_SendData(USART1, Data);
    //while (!((USART1->USART_STATUS_REG & USART_FLAG_TXE));       
}
/*****************************************************************************
* Function Name : Print                                                      *
*                                                                            *
* Description   : This function is used to print data onto the serial        *
*                 communications interface of STM8 type MCU                  *
*                                                                            *
* Arguments     : 1) Pointer to the data to be printed                       *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void Print(char *pData)
{
    
	while(*pData != '\0')
    {
        putcchar(*pData);        
        //osDelay(10);        
        pData++;
    }
    
}
void PrintPacket(char*pData,unsigned short Length)
{
    
	while(Length>0)
    {
        putcchar(*pData);        
        //osDelay(10);
        pData++;
        Length--;
    }
    
}
void Print1(char *pData)
{
    
	while(*pData != '\0')
    {
        putcchar1(*pData);        
        osDelay(10);        
        pData++;
    }
}
#ifdef DEBUG_PRINT
void DebugPrint(char*pData)
{
    if(DebugPrintEnabled==1)
    {
        Print1(pData);
        osDelay(500);
    }
}
#endif
/*****************************************************************************
* Function Name : ResetBuffer                                                *
*                                                                            *
* Description   : This function is used to reset the Input UART buffer       *
*                                                                            *
* Arguments     : None                                                       *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void ResetBuffer(void)
{
    unsigned short i;
    
    for(i = 0; i < BUFF2_SIZE; i++){ Buff2[i] = 0; }
    Buff2Index = 0;

}
/*****************************************************************************
* Function Name : ResetBuffer                                                *
*                                                                            *
* Description   : This function is used to reset the Input UART buffer       *
*                                                                            *
* Arguments     : None                                                       *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void ResetBuffer1(void)
{
    unsigned short i;
    
    for(i = 0; i < BUFF_SIZE; i++){ Buff[i] = 0; }
    BuffIndex = 0;
  
}
/*******************************************************************************
* Funtion name : MapForward                                                    *
*                                                                              *
* Description  : This function performs forward mapping and returns pointer    *
*                to the start of the found data or else returns NULL pointer   *
*                                                                              *
* Arguments    : 1) Poshorter to the data in which mapping is to be made       *
*                2) Length of the data in which mapping is to be made          *
*                3) Poshorter to the data points to be mapped                  *
*                4) Length of the data points to be mapped                     *
*                                                                              *
* Returns      : Pointer in which result is returned                           *
*******************************************************************************/
char * MapForward
(
	char *pMapData,
	unsigned short   MapDataLength,
	char *pMapPoints,
	unsigned short   MapPointsLength	
)
{
    unsigned short DataIndex;
    unsigned short MapPointIndex;

    for(DataIndex = 0; DataIndex < MapDataLength - MapPointsLength + 1; DataIndex++)
    {
        
        for(MapPointIndex = 0; MapPointIndex < MapPointsLength; MapPointIndex++)
        {
            if( pMapData[DataIndex + MapPointIndex] != pMapPoints[MapPointIndex])
            {
                goto PICK_NEXT_FMAPDATA;
            }
        }

        return(& pMapData[DataIndex]); 
        
    PICK_NEXT_FMAPDATA:;

    }
    return(NULL);
}

unsigned char ReadUART2(void)
{
    unsigned char Data=0;
    
     return Data;
    
}
void WriteUART2(unsigned char Data)
{
    putcchar(Data);
}
unsigned char tmp,tmp1;

extern unsigned short ADCVal;
/*****************************************************************************
* Function Name : PrintNumber                                                *
*                                                                            *
* Description   : This function is used to print data onto the serial        *
*                 communications interface of 8051 type MCU                  *
*                                                                            *
* Arguments     : 1) Number to be printed                                    *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void PrintNumber(unsigned short Data)
{
    /*tmp = ( (ADCVal >> 12) & 0x0F ) ;
    GetAscii(tmp);	
	WriteUART2(tmp);
    //while( PIR1bits.TXIF == 0);
    
    tmp = ( (ADCVal >> 8) & 0x0F ) ;
    GetAscii(tmp);	
	WriteUART2(tmp);
    //while( PIR1bits.TXIF == 0);
    
    tmp = ( (ADCVal >> 4) & 0x0F ) ;
    GetAscii(tmp);	
	WriteUART2(tmp);
    //while( PIR1bits.TXIF == 0);
    
    tmp = ( (ADCVal >> 0) & 0x0F ) ;
    GetAscii(tmp);	
	WriteUART2(tmp);
    //while( PIR1bits.TXIF == 0);
    */
    /*//TI = 0;
	WriteUART1(GetAscii((unsigned char)(ADCVal >> 8)& 0x0F));
    //while( PIR1bits.TXIF == 0);
    //TI = 0;
	WriteUART1(GetAscii((unsigned char)(ADCVal >> 4)& 0x0F));
    //while( PIR1bits.TXIF == 0);
    //TI = 0;
	WriteUART1(GetAscii((unsigned char)(ADCVal >> 0) & 0x0F));
    //while( PIR1bits.TXIF == 0);
    //TI = 0;
    */
    
}
void WakeUp(void);
unsigned char RingCount;

int dada;
extern char tBuff[];
unsigned char f;
void ResetTBuffer(void)
{
    unsigned short i;
    
    for(i = 0; i < TBUFF_SIZE; i++){ tBuff[i] = 0; }
    f = 0;
    
}

double GPRMC2Degrees(double Value)
{
    return ((int)(Value / 100) + (Value - ((int)(Value / 100) * 100)) / 60);
}

unsigned char u1,u2;
extern char GPSTime[],GPSDate[],GPSSpeed[],Speed[];
unsigned char prevGPSStatus;
void HandleGPSData(void)
 {

}

void HandleGPSINFData(unsigned char Byte)
{
    unsigned char Data;
    u1++;
  
    
    {
        Data = Byte;//(uint8_t)(USART1->RDR); /* Receive data, clear flag */
              
      
        Buff[BuffIndex]=Data;//USART_ReceiveData(USART1);
        tBuff[f] = Buff[BuffIndex];//I2C_ReceiveData(I2C1);
        if(Buff[BuffIndex] == '+')BuffIndex=0;    
        if( (tBuff[f-1] == 0x0D)&&(tBuff[f] == 0x0A) )
        {
                f=-1;
                //ResetTBuffer();
        }
        f++;
        if( (tBuff[f-3] == 'F') && (tBuff[f-2] == 'O') && (tBuff[f-1] == ':') )
        {
            HeaderReceived = 1;
            CommaCount = 0;
            //USART_ITConfig(USART1,USART_IT_RXNE, ENABLE);
            return;
            //i = 0;
        }				
            
        if(HeaderReceived == 1)
        {	            
            if(tBuff[f-1] == '+')
                HeaderReceived = 0;
            if(tBuff[f-1] == ' ')
                pData = Lat;
                
            if( (tBuff[f-1] == ',') )
            {	
                CommaCount++;
                switch(CommaCount)
                {
					
                    case 0:
                        pData = Lat;
                    break;
                    case 1:
                        pData = &D1;
                    break;
                    case 2:
                        pData = Long;
                    break;
                    case 3:
                        pData = &D2;
                    break;
                    case 4:
                        pData = GPSDate;
                    break;
                    case 5:
                        pData = GPSTime;
                    break;
                    case 6:
                        pData = Altitude;
                    break;
                    case 7:
                        pData = Speed;
                    break;
                 
                    case 8:
//                    pData = tBuff;
//                    break;
//                    case 9:
//                        pData = GPSDate;
//                    break;
//                    case 10:
                        
                        if(GPSStatus!='A')
                        {
                            memset(Lat,0,sizeof(Lat));
                            memset(Long,0,sizeof(Long));
                        }
                        pData = tBuff;
                        HeaderReceived = 0;
                        CommaCount = 0;
                        f = 0;						
                        sscanf( (void*)Lat, "%f", &fLat);
                        sscanf( (void*)Long, "%f", &fLong);
                        sscanf( (void*)Altitude, "%f", &fAltitude);
                        sscanf( (void*)Speed, "%f", &fSpeed);
                        sscanf( (void*)GPSDate, "%2d%2d%2d", (int*)&GPSDay,(int*)&GPSMonth,(int*)&GPSYear);
                        sscanf( (void*)GPSTime, "%2d%2d%2d", (int*)&GPSHours,(int*)&GPSMinutes,(int*)&GPSSeconds);
                        if(D1 == 'S')fLat *= -1;
                        if(D2 == 'W')fLong *= -1;
                        fLat = GPRMC2Degrees(fLat);
                        fLong = GPRMC2Degrees(fLong); 
                        sprintf
                        (
                            (void*)Link,
                            "http://maps.google.com/maps?z=18&q=%f,%f",
                            fLat,fLong
                        );
                            
                        if(GPSStatus == 'A')
                        {
                            pfLat = fLat;
                            pfLong = fLong;
                        }
                        if(GPSStatus != prevGPSStatus)
                        {
                            UpdateLocation(GPSStatus);
                            prevGPSStatus = GPSStatus;
                        }
                    
                    
                    return;
                    
                }
            }
            else
            {
                
                *pData = tBuff[f-1];
                pData++;
            }         	
        }
        if(f>250) f=0;		
        BuffIndex++;
        if(BuffIndex >= BUFF_SIZE-1)
            BuffIndex = 0;
        
				
    }
    
}
/*****************************************************************************
* Function Name : UART1_RX_IRQHandler                                        *
*                                                                            *
* Description   : This ISR  handles the UART Rx interrupt                    *
*                                                                            *
* Arguments     : None                                                       *
*                                                                            *
* Returns       : Nothing                                                    *
*****************************************************************************/
void USART1_IRQHandler(void)
{

}


///*****************************************************************************
//* Function Name : UART2_RX_IRQHandler                                        *
//*                                                                            *
//* Description   : This ISR  handles the UART Rx interrupt                    *
//*                                                                            *
//* Arguments     : None                                                       *
//*                                                                            *
//* Returns       : Nothing                                                    *
//*****************************************************************************/
/**
  * @brief This function handles LPUART1 global interrupt.
  */
void HandleGSMData(void)
{

}

void LPUART1_IRQHandler(void)
{

}

