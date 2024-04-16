/******************************************************************************
 *
 *  This file is part of ESS: Repeatable Evaluation of Energy Harvesting Subsystems for Industry-Grade IoT Platforms.
 *
 *  Copyright 2021-2023 Elina Ghasemi, NTNU
 *
 *****************************************************************************/

#include <jtag_lowlevel.h>
#include "lynsyn_main.h"
#include "arm.h"
#include "jtag.h"
#include "jtag_lowlevel.h"

#include <stdio.h>
#include <string.h>
#include <em_usart.h>
#include <em_prs.h>
#include <stdlib.h>
#include <config.h>

#define SHUNT_RESISRANCE 10 /*/10 OHOM */
#define VCC 3.0 /*/3V OHOM */
#define DEEP_SLEEP_POWER 0.9e-6 
#define IDLE_POWER 7.5e-6 
#define ACTIVE_THRESHOLD 1300.0e-6
#define BLE_THRESHOLD 8.1e-3
#define DEEP_SLEEP_CURRENT 0.3e-6
#define IDLE_SLEEP_CURRENT 2.5e-6
#define BACKGROUND_CURRENT 0.0
#define BACKGROUND_POWER ( 3 * BACKGROUND_CURRENT )
#define BLE_END 0.5


#define LYNSYN_REF_VOLTAGE 2.5
#define LYNSYN_MAX_SENSOR_VALUE 32768
#define CURRENT_SENSOR_GAIN_V3_1 20

#define LYNSYN_FREQ 48000000

#define PRS_CHANNEL 1
#define PRS_CHANNEL_MASK 2

#ifndef _linux_
#define USE_USART
#endif

extern volatile int ADC_value;
double ADC_send, ADC_send_compensate;
extern double harvestingCurrent;
extern int Counter_index;
double E_consuming;
double E_harvesting, E_harvesting_ble, E_consuming_ble =0;
volatile int PES_microcontroller_state = 0; //0 off , 1 on 
int64_t oldTime, newTime;
double deltaTiming,deltaTiming2;
volatile int StartSamplingTimeState =-1;
double offset = 0;
double gain = 1;

double Instantaneous_energy;
double primaryEnergy = 0.005;
double resumeEnergy = 0.004;
double backupThreshold = 0.0002;
double offThreshold = 0.0;
double CapacityEnergy = 20000;
#define SIMULATOR

#ifdef SIMULATOR
	#define scaling_time  550			
#else			
	#define scaling_time  1			
#endif

#define scaled_Power (IDLE_POWER + BACKGROUND_POWER) * scaling_time
#define scaled_Power2 (IDLE_POWER + BACKGROUND_POWER) * (scaling_time -1)
#define real_Power IDLE_POWER  * scaling_time
#define SLEEP_BACKGROUND_CURRENT (IDLE_SLEEP_CURRENT + BACKGROUND_CURRENT ) *(scaling_time -1)

static inline void jtagPinWrite2(bool clk, bool tdi, bool tms) {
#ifndef _linux_
  if(tdi) GPIO_PinOutSet(TDI_PORT, TDI_BIT);
  else GPIO_PinOutClear(TDI_PORT, TDI_BIT);

  if(tms) GPIO_PinOutSet(JTAG_PORT, JTAG_TMS_BIT);
  else GPIO_PinOutClear(JTAG_PORT, JTAG_TMS_BIT);

  if(clk) GPIO_PinOutSet(JTAG_PORT, JTAG_TCK_BIT);
  else GPIO_PinOutClear(JTAG_PORT, JTAG_TCK_BIT);
#endif
}

void delay(int sec )
{
	volatile int64_t currentTime = calculateTime();
	volatile int64_t currentTimeOffset = currentTime + LYNSYN_FREQ * sec ; 
	while( calculateTime() < ( currentTimeOffset )) 
	{
	}
}
void delay_ms(int ms_sec )
{
	volatile int64_t currentTime = calculateTime();
	volatile int64_t currentTimeOffset = currentTime + LYNSYN_FREQ /1000 * ms_sec ; 
	while( calculateTime() < ( currentTimeOffset )) 
	{
	}
}

void delay_us(int us_sec )
{
	volatile int64_t currentTime = calculateTime();
	volatile int64_t currentTimeOffset = currentTime + LYNSYN_FREQ /1000000 * us_sec ; 
	while( calculateTime() < ( currentTimeOffset )) 
	{
	}
}

static inline void turn_PES_OFF()//tdi
{
	GPIO_PinOutClear(TDI_PORT, TDI_BIT);
	delay_us(400) ;
	GPIO_PinOutSet(TDI_PORT, TDI_BIT);
}
static inline void turn_PES_ON()//tms
{
  GPIO_PinOutClear(JTAG_PORT, JTAG_TMS_BIT);
	delay_us(100) ;
  GPIO_PinOutSet(JTAG_PORT, JTAG_TMS_BIT);
}

void test_on_off()
{
	GPIO_PinOutSet(JTAG_PORT, JTAG_TMS_BIT);
	GPIO_PinOutSet(TDI_PORT, TDI_BIT);
	return;
	delay(20) ;
	while(true){
	turn_PES_OFF();
	delay(20) ;
	turn_PES_ON();
	delay(20) ;
	}
}

extern int64_t calculateTime();
extern double getDouble(char *id);
volatile int ble_state = 0;
int64_t TimeS,TimeStartBle =0; 
double compensate_energy_consuming =0;
double compensate_energy_H =0;
double consuming_energy(void)
{
	double v = (((double)ADC_value-offset) * (double)LYNSYN_REF_VOLTAGE / (double)LYNSYN_MAX_SENSOR_VALUE) * gain;
	double vs = v / CURRENT_SENSOR_GAIN_V3_1;
	double ADC_current = vs / SHUNT_RESISRANCE;
	ADC_current = ADC_current - 0.000113; //offset
	
	if (PES_microcontroller_state == 0){
		ADC_send  = DEEP_SLEEP_CURRENT* scaling_time + 0.000113 ;
		E_consuming = DEEP_SLEEP_POWER * deltaTiming * scaling_time;
	}
	else{
		#ifdef SIMULATOR
		if ((ble_state==0) && (ADC_current>BLE_THRESHOLD)){
			ble_state=1;
		}
		else if ((ble_state==1) && (ADC_current>BLE_THRESHOLD)){
			ble_state=2;
			TimeS = newTime;
			//delta_time_ble =0;
			TimeStartBle = newTime ;
		}
		else if ((ble_state==1) && (ADC_current <= BLE_THRESHOLD)){
			ble_state=0;		
		}
		if ((ble_state==2) && (ADC_current>BLE_THRESHOLD)){
			TimeS = newTime;		
		}

		else if ((ble_state==2) && (ADC_current<BLE_THRESHOLD)){
			deltaTiming2 = (double)(newTime - TimeS) / (double)LYNSYN_FREQ;
			if (deltaTiming2 > BLE_END){
				ble_state=0;
				deltaTiming2 =0;
				TimeStartBle =0;
				compensate_energy_H = (scaling_time -1) *E_harvesting_ble;
				E_harvesting_ble =0;
				compensate_energy_consuming = (scaling_time -1) * E_consuming_ble;
				E_consuming_ble = 0;
				ADC_send_compensate = compensate_energy_consuming / VCC / deltaTiming;
			}
		}
		#endif
		if (ADC_current < ACTIVE_THRESHOLD ){
			#ifdef SIMULATOR
			if(ble_state==2){
				ADC_send = IDLE_SLEEP_CURRENT  + 0.000113;
				E_consuming = IDLE_POWER *deltaTiming;
			}
			else{		
					ADC_send = (IDLE_SLEEP_CURRENT + BACKGROUND_CURRENT) * scaling_time + 0.000113;
					E_consuming = scaled_Power *deltaTiming;
					E_consuming = E_consuming + compensate_energy_consuming;
					compensate_energy_consuming =0 ;
					ADC_send = ADC_send + ADC_send_compensate;
					ADC_send_compensate =0;
			}
				#else			
					ADC_send = IDLE_SLEEP_CURRENT * scaling_time + 0.000113;
					E_consuming = real_Power *deltaTiming;
				#endif
			
		}
		else{
			#ifdef SIMULATOR		
				ADC_send = ADC_current + 0.000113;
				E_consuming = (VCC - (SHUNT_RESISRANCE * ADC_current))* ADC_current * deltaTiming;
				if(ble_state!=2){
					E_consuming = E_consuming + scaled_Power2 * deltaTiming;
					ADC_send = ADC_send + SLEEP_BACKGROUND_CURRENT;
				}		
			#else					
				ADC_send = ADC_current + 0.000113;
				E_consuming = (VCC - (SHUNT_RESISRANCE * ADC_current))* ADC_current * deltaTiming;
			#endif
			}
		#ifdef SIMULATOR		
			if(ble_state==2){
				E_consuming_ble = E_consuming_ble + IDLE_POWER *deltaTiming;	
			}
		#endif
	}							
	return E_consuming;
}

double harvesting_energy(void)
{
	#ifdef SIMULATOR
		if((ble_state==2) && (PES_microcontroller_state==1)){
			E_harvesting = harvestingCurrent * deltaTiming;
			
			E_harvesting_ble = E_harvesting_ble + E_harvesting;
		}
		else{
			E_harvesting = harvestingCurrent * deltaTiming *scaling_time;
			E_harvesting = E_harvesting + compensate_energy_H ;
			compensate_energy_H = 0 ;		
		}	
	
	#else 
		E_harvesting = harvestingCurrent * deltaTiming *scaling_time;
	#endif	
	return E_harvesting;
}

void Intermittent_init(void){
	StartSamplingTimeState =1;
	Instantaneous_energy = primaryEnergy;
}
void energy_spectrum(void)
{
	if (StartSamplingTimeState ==-1) return;
	if (StartSamplingTimeState ==1){
		deltaTiming = 0;
		StartSamplingTimeState =0;
		oldTime = calculateTime();
		newTime = oldTime;
		offset = getDouble("offc00");
		gain = getDouble("gainc00");		
			
		if (Instantaneous_energy > backupThreshold){
			turn_PES_ON();
			PES_microcontroller_state =1;			
		}
		else{
			turn_PES_OFF();
			PES_microcontroller_state =0;							
		}
	}
	else{
		newTime = calculateTime() ;
		deltaTiming = (double)(newTime - oldTime) / (double)LYNSYN_FREQ;
		oldTime = newTime;
	}
	Instantaneous_energy = Instantaneous_energy - consuming_energy() + harvesting_energy() ;	
	if (Instantaneous_energy > CapacityEnergy){
		Instantaneous_energy = CapacityEnergy;
	}
	
	if (Instantaneous_energy < 0){
		Instantaneous_energy =0;
	}
	if (PES_microcontroller_state ==0){
		if(Instantaneous_energy> resumeEnergy){
			PES_microcontroller_state =1;	
			turn_PES_ON();
			TimeS = newTime + 1 * LYNSYN_FREQ;
		}
	}
	else{
		if(Instantaneous_energy< backupThreshold){
			PES_microcontroller_state =0;
			turn_PES_OFF();				
		}
	}
}
























