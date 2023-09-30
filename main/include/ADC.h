/***************************************************************************
* File name :   ADC.h                                                      *
*                                                                          *
* Author    :   Ravi Y. Pujar                                              *
*                                                                          *
* Owner     :   Copyright (c) 2023 Valetron Systems Pvt Ltd,               *
*                all rights reserved                                       *
***************************************************************************/
#ifndef _ADC_H
#define _ADC_H

extern float ADCBatteryVoltage;
extern unsigned short BatteryADCCount;
extern int TADCReading;
//extern uint32_t voltage;
void ADCTask(void *arg);
void AverageADCSamples(void);
#endif