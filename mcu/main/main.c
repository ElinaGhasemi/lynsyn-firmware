/******************************************************************************
 *
 *  This file is part of the Lynsyn PMU firmware.
 *
 *  Copyright 2018-2019 Asbjørn Djupdal, NTNU
 *
 *****************************************************************************/

#include "lynsyn_main.h"

#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <inttypes.h>

#include <em_device.h>
#include <em_chip.h>
#include <em_cmu.h>
#include <em_gpio.h>
#include <em_emu.h>

#include "../common/swo.h"
#include "../common/usbprotocol.h"

#include "arm.h"
#include "jtag_lowlevel.h"
#include "adc.h"
#include "config.h"
#include "usb.h"
#include "jtag.h"
#ifdef VERSION2
#include "fpga.h"
#endif

volatile bool sampleMode;
volatile bool samplePc;
volatile bool useStopBp;
volatile int64_t sampleStop;

volatile uint32_t lastLowWord = 0;
volatile uint32_t highWord = 0;

static struct SampleReplyPacket sampleBuf1[MAX_SAMPLES] __attribute__((__aligned__(4)));
static struct SampleReplyPacket sampleBuf2[MAX_SAMPLES] __attribute__((__aligned__(4)));

static int16_t continuousCurrent[CHANNELS];
uint64_t continuousCurrentAcc[CHANNELS];
int continuousSamplesSinceLast[CHANNELS];
int16_t continuousCurrentInstant[CHANNELS];
int16_t continuousCurrentAvg[CHANNELS];

///////////////////////////////////////////////////////////////////////////////

void panic(const char *fmt, ...) {
  va_list args;
  va_start(args, fmt);
  printf("\nPanic: ");
  vprintf(fmt, args);
  printf("\n");
  va_end(args);
  while(true);
}
//////////////////////////////////////////////////////////////////////////////
extern void energy_spectrum(void);
extern double harvestingCurrent;
volatile int ADC_value;
extern double ADC_send;
extern double E_consuming;
extern double E_harvesting;
extern double Instantaneous_energy;
extern int PES_microcontroller_state;
extern int ble_state;
extern double offset, gain;
extern void Intermittent_init(void);
extern void test_on_off();
extern int64_t newTime;
///////////////////////////////////////////////////////////////////////////////

int main(void) {
  sampleMode = false;

#ifndef __linux__
  // setup clocks
  CMU_ClockEnable(cmuClock_HFPER, true);
  CMU_ClockEnable(cmuClock_USB, true);
  CMU_ClockEnable(cmuClock_ADC0, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
#endif

  printf("Lynsyn initializing...\n");

#ifndef __linux__
  // Enable cycle counter
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= 0x01;
  DWT->CYCCNT = 0;
#endif

  configInit();

  printf("Hardware V%" PRIx32 ".%" PRIx32 " Firmware %s\n", (getUint32("hwver") & 0xf0) >> 4, getUint32("hwver") & 0xf, SW_VERSION_STRING);

  //jtagInitLowLevel();
  GPIO_PinModeSet(TDI_PORT, TDI_BIT, gpioModePushPull, 0);
  GPIO_PinModeSet(JTAG_PORT, JTAG_TMS_BIT, gpioModePushPull, 0);
  adcInit();
  usbInit();
  
  clearLed(0);

  printf("Ready.\n");

  int samples = 0;

  unsigned currentSample = 0;

  struct SampleReplyPacket *sampleBuf = sampleBuf1;

test_on_off();

  // main loop
  while(true) {
#ifdef __linux__
    linux_processUsb();
#endif

    if(sampleMode) {
      int64_t currentTime = calculateTime();

      struct SampleReplyPacket *samplePtr = &sampleBuf[currentSample];

      samplePtr->flags = 0;

      bool halted = false;
      bool sampleOk = true;

      {
        adcScan(samplePtr->channel);
        if(samplePc) {
          sampleOk = coreReadPcsrFast(samplePtr->pc, &halted);
          if(!sampleOk) halted = false;
          if(sampleOk && halted) {
            for(int i = 0; i < numCores; i++) {
              uint64_t pc;
              if(!readPc(i, &pc)) {
                printf("Warning, can't read PC\n");
              }
              if(pc == markBp) {
                clearBp(MARK_BP);
                coresResume();
                halted = false;
                samplePtr->flags = SAMPLE_REPLY_FLAG_MARK;
                setBp(MARK_BP, markBp);
                break;
              }
            }

          }
        } else {

        }

        if(!useStopBp) {
          halted = currentTime >= sampleStop;
        }

	adcScanWait();
        ADC_value = samplePtr->channel[4];
        energy_spectrum();
        samplePtr->channel[4] = ADC_value;
            samplePtr->pc[0] = E_harvesting *1e12; //nj
            samplePtr->pc[1] = E_consuming *1e12;  // nj
            samplePtr->pc[2] = Instantaneous_energy *1e12; //nj
            samplePtr->pc[3] = ADC_send * 1e9;
            samplePtr->channel[4] = PES_microcontroller_state *200;
            samplePtr->channel[2] = ble_state *200;
            //samplePtr->pc[3] = PES_microcontroller_state;
            //samplePtr->pc[3] = ADC_value;
            //samplePtr->pc[2] = offset*1e6 ; 
            //samplePtr->pc[3] = gain*1e6;
            samplePtr->time = newTime;
      }

      if(halted) {
        samplePtr->time = -1;
        sampleMode = false;
        sampleOk = true;

        if(useStopBp) {
          clearBp(STOP_BP);
          coresResume();
        }

#ifdef VERSION2
        jtagExt();
#endif
        clearLed(0);

        printf("Exiting sample mode, %d samples\n", samples);

        samples = 0;
      }

      {
        if(sampleOk) {
          samples++;
          currentSample++;

          if(samplePtr->flags & SAMPLE_REPLY_FLAG_MARK) samplePtr->pc[0] = calculateTime();

          if((currentSample >= MAX_SAMPLES) || halted) {
            sendSamples(sampleBuf, currentSample);
            if(sampleBuf == sampleBuf1) sampleBuf = sampleBuf2;
            else sampleBuf = sampleBuf1;
            currentSample = 0;
          }
        }
      }
    } else {
#ifndef __linux__
      __disable_irq();
#endif

      adcScan(continuousCurrent);
      adcScanWait();

      for(int i = 0; i < CHANNELS; i++) {
        continuousCurrentAcc[i] += continuousCurrent[i];
        continuousSamplesSinceLast[i]++;
        continuousCurrentInstant[i] = continuousCurrent[i];
      }

#ifndef __linux__
      __enable_irq();
#endif
    }
  }
}

void wait(unsigned cycles) {
#ifndef __linux__
  DWT->CYCCNT = 0;
  while(DWT->CYCCNT < cycles);
#endif
}

int64_t calculateTime() {
#ifdef __linux__
  return 0;
#else
  uint32_t lowWord = DWT->CYCCNT;
  __disable_irq();
  if(lowWord < lastLowWord) highWord++;
  lastLowWord = lowWord;
  __enable_irq();
  return ((uint64_t)highWord << 32) | lowWord;
#endif
}

int _write(int fd, char *str, int len) {
#ifndef __linux__
  for (int i = 0; i < len; i++) {
    ITM_SendChar(str[i]);
  }
#endif
  return len;
}

void getCurrentAvg(int16_t *sampleBuffer) {
  for(int i = 0; i < CHANNELS; i++) {
    sampleBuffer[i] = continuousCurrentAcc[i] / continuousSamplesSinceLast[i];
    continuousCurrentAcc[i] = 0;
    continuousSamplesSinceLast[i] = 0;
  }
}

void clearLog(void) {
  logReply.buf[0] = 0;
  logReply.size = 1;
}

void addLogLine(const char *fmt, ...) {
  if(logReply.size < MAX_LOG_SIZE) {
    va_list args;
    va_start(args, fmt);
    logReply.size += vsnprintf(logReply.buf + logReply.size - 1, MAX_LOG_SIZE - logReply.size, fmt, args);
    va_end(args);
  }
}
