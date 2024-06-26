/******************************************************************************
 *
 *  This file is part of the Lynsyn PMU firmware.
 *
 *  Copyright 2018-2019 Asbjørn Djupdal, NTNU
 *
 *****************************************************************************/

#ifndef VERSION2

#include "lynsyn_main.h"
#include "arm.h"
#include "jtag.h"
#include "jtag_lowlevel.h"

#include <stdio.h>
#include <string.h>
#include <em_usart.h>
#include <em_prs.h>
#include <stdlib.h>

#define PRS_CHANNEL 1
#define PRS_CHANNEL_MASK 2

#ifndef __linux__
#define USE_USART
#endif

//#define PRINTBIT
//#define DUMP_PINS

inline void jtagPinWrite(bool clk, bool tdi, bool tms) {
#ifndef __linux__
  if(tdi) GPIO_PinOutSet(TDI_PORT, TDI_BIT);
  else GPIO_PinOutClear(TDI_PORT, TDI_BIT);

  if(tms) GPIO_PinOutSet(JTAG_PORT, JTAG_TMS_BIT);
  else GPIO_PinOutClear(JTAG_PORT, JTAG_TMS_BIT);

  if(clk) GPIO_PinOutSet(JTAG_PORT, JTAG_TCK_BIT);
  else GPIO_PinOutClear(JTAG_PORT, JTAG_TCK_BIT);
#endif
}

static inline void usartOn(void) {
#ifdef USE_USART
  jtagPinWrite(false, false, false);

  JTAG_USART->ROUTE = (JTAG_USART->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | JTAG_USART_LOC
    | USART_ROUTE_TXPEN | USART_ROUTE_RXPEN | USART_ROUTE_CLKPEN;

  TDI_USART->ROUTE = (TDI_USART->ROUTE & ~_USART_ROUTE_LOCATION_MASK) | TDI_USART_LOC
    | USART_ROUTE_TXPEN;
#endif
}

static inline void usartOff(void) {
#ifdef USE_USART
  JTAG_USART->ROUTE = 0;
  TDI_USART->ROUTE = 0;
#endif
}

static inline unsigned readBit(void) {
#ifdef __linux__
  return linux_jtagReadBit();
#else
  unsigned bit = GPIO_PinInGet(JTAG_PORT, JTAG_TDO_BIT);
#ifdef PRINTBIT
  printf("%d, ", bit);
#endif
  return bit;
#endif
}

static inline void writeBit(bool tdi, bool tms) {
  jtagPinWrite(false, tdi, tms);
  jtagPinWrite(true, tdi, tms);
}

void readWriteSeq(unsigned size, uint8_t *tdiData, uint8_t *tmsData, uint8_t *tdoData) {
  int bytes = size / 8;
  int leftovers = size % 8;

  if(bytes) {
    for(int byte = 0; byte < bytes; byte++) {
#ifdef DUMP_PINS
      printf("%x %x = ", *tdiData, *tmsData);
#endif
#ifdef USE_USART
      JTAG_USART->CMD = USART_CMD_TXDIS;
      TDI_USART->CMD = USART_CMD_TXDIS;

      uint8_t txDataTms = *tmsData++;
      JTAG_USART->TXDATA = (uint32_t)txDataTms;

      uint8_t txDataTdi = *tdiData++;
      TDI_USART->TXDATA = (uint32_t)txDataTdi;

      PRS_PulseTrigger(PRS_CHANNEL_MASK);

      while(!(JTAG_USART->STATUS & USART_STATUS_RXDATAV));
      uint8_t val = (uint8_t)JTAG_USART->RXDATA;
#ifdef PRINTBIT
      for(int i = 0; i < 8; i++) {
        printf("%d, ", (val >> i) & 0x1);
      }
#endif
#else
      uint8_t val = 0;

      for(int readPos = 0; readPos < 8; readPos++) {
        writeBit((*tdiData >> readPos) & 1, (*tmsData >> readPos) & 1);
        val |= readBit() << readPos;
      }
      tdiData++;
      tmsData++;

#endif
#ifdef DUMP_PINS
      printf("%x\n", val);
#endif
      if(tdoData) *tdoData++ = val;
    }
  }

  if(leftovers) {
    usartOff();

    uint8_t val = 0;
    for(int readPos = 0; readPos < leftovers; readPos++) {
      writeBit((*tdiData >> readPos) & 1, (*tmsData >> readPos) & 1);
      val |= readBit() << readPos;
    }

    if(tdoData) *tdoData = val;

    usartOn();
  }
}

static inline void readWriteSeqSpi(unsigned bytes, uint8_t *tdiData, uint8_t *tmsData, uint8_t *tdoData) {
  for(int byte = 0; byte < bytes; byte++) {
#ifdef DUMP_PINS
    printf("%x %x = ", *tdiData, *tmsData);
#endif

#ifdef __linux__
    uint8_t val = 0;
    for(int i = 0; i < 8; i++) {
      val |= (linux_jtagReadBit() << i);
    }

#else
    JTAG_USART->CMD = USART_CMD_TXDIS;
    TDI_USART->CMD = USART_CMD_TXDIS;

    uint8_t txDataTms = *tmsData++;
    JTAG_USART->TXDATA = (uint32_t)txDataTms;

    uint8_t txDataTdi = *tdiData++;
    TDI_USART->TXDATA = (uint32_t)txDataTdi;

    PRS_PulseTrigger(PRS_CHANNEL_MASK);

    while(!(JTAG_USART->STATUS & USART_STATUS_RXDATAV));
    uint8_t val = (uint8_t)JTAG_USART->RXDATA;
#endif
#ifdef PRINTBIT
    for(int i = 0; i < 8; i++) {
      printf("%d, ", (val >> i) & 0x1);
    }
#endif

#ifdef DUMP_PINS
    printf("%x\n", val);
#endif
    *tdoData++ = val;
  }
}

static uint8_t *storedTdi = NULL;
static uint8_t *storedTms = NULL;

static unsigned progSize;
static uint32_t *words = NULL;
static unsigned *ackPos = NULL;
static bool *doRead = NULL;

static unsigned *initSize = NULL;
static uint8_t **initTdi = NULL;
static uint8_t **initTms = NULL;

static unsigned *loopSize = NULL;
static uint8_t **loopTdi = NULL;
static uint8_t **loopTms = NULL;

void storeSeq(uint16_t size, uint8_t *tdiData, uint8_t *tmsData) {
  if(storedTdi) free(storedTdi);
  storedTdi = malloc(size);

  if(storedTms) free(storedTms);
  storedTms = malloc(size);

  if(!storedTdi || !storedTms) panic("Out of memory (storeSeq)");

  memcpy(storedTdi, tdiData, size);
  memcpy(storedTms, tmsData, size);
}

void storeProg(unsigned size, uint8_t *read, uint16_t *initPos, uint16_t *loopPos, uint16_t *ap, uint16_t *endPos) {
  progSize = size;

  if(ackPos) free(ackPos);
  ackPos = malloc(size * sizeof(unsigned));

  if(doRead) free(doRead);
  doRead = malloc(size * sizeof(bool));

  if(initSize) free(initSize);
  initSize = malloc(size * sizeof(unsigned));
  if(initTdi) free(initTdi);
  initTdi = malloc(size * sizeof(uint8_t*));
  if(initTms) free(initTms);
  initTms = malloc(size * sizeof(uint8_t*));

  if(loopSize) free(loopSize);
  loopSize = malloc(size * sizeof(unsigned));
  if(loopTdi) free(loopTdi);
  loopTdi = malloc(size * sizeof(uint8_t*));
  if(loopTms) free(loopTms);
  loopTms = malloc(size * sizeof(uint8_t*));

  if(words) free(words);
  words = malloc(size * sizeof(uint32_t));

  if(!ackPos || !doRead || !initSize || !initTdi || !initTms || !loopSize || !loopTdi || !loopTms || !words) panic("Out of memory");

  for(int command = 0; command < size; command++) {
    doRead[command] = read[command];

    initSize[command] = (loopPos[command] - initPos[command]) / 8;
    int initByte = initPos[command] / 8;

    loopSize[command] = (endPos[command] - loopPos[command]) / 8;
    int loopByte = loopPos[command] / 8;

    ackPos[command] = ap[command] - loopPos[command];

    initTdi[command] = storedTdi + initByte;
    initTms[command] = storedTms + initByte;

    loopTdi[command] = storedTdi + loopByte;
    loopTms[command] = storedTms + loopByte;
  }
}

bool executeSeq(void) {
  int wordCounter = 0;

  for(int command = 0; command < progSize; command++) {
    uint8_t ack;
    uint8_t tdo[32];

    readWriteSeqSpi(initSize[command], initTdi[command], initTms[command], tdo);

    do {
      readWriteSeqSpi(loopSize[command], loopTdi[command], loopTms[command], tdo);
      unsigned pos = ackPos[command];
      ack = extractAck(&pos, tdo);
      if(ack != ARM_ACK_WAIT) {
        if(doRead[command]) {
          uint32_t word = extractWord(&pos, tdo);
          words[wordCounter++] = word;
        }
      }

    } while(ack == ARM_ACK_WAIT);

    if(ack != ARM_ACK_OK) return false;
  }

  return true;
}

uint32_t *getWords(void) {
  return words;
}

void setTrst(bool on) {
#ifndef __linux__
  if(on) GPIO_PinOutClear(JTAG_PORT, JTAG_TRST_BIT);
  else GPIO_PinOutSet(JTAG_PORT, JTAG_TRST_BIT);
#else
  printf("TRST %s\n", on ? "on" : "off");
#endif
}

///////////////////////////////////////////////////////////////////////////////

uint32_t jtagSetBaudrate(uint32_t baudrate) {
#ifndef __linux__
  USART_BaudrateSyncSet(JTAG_USART, 0, baudrate);
  USART_BaudrateSyncSet(TDI_USART, 0, baudrate);
  return USART_BaudrateGet(JTAG_USART);
#else
  return baudrate;
#endif
}

bool jtagInitLowLevel(void) {
#ifndef __linux__
  GPIO_PinModeSet(TDI_PORT, TDI_BIT, gpioModePushPull, 0);
  GPIO_PinModeSet(JTAG_PORT, JTAG_TCK_BIT, gpioModePushPull, 0);
  GPIO_PinModeSet(JTAG_PORT, JTAG_TDO_BIT, gpioModeInput, 0);
  GPIO_PinModeSet(JTAG_PORT, JTAG_TMS_BIT, gpioModePushPull, 0);
  GPIO_PinModeSet(JTAG_PORT, JTAG_TRST_BIT, gpioModePushPull, 1);
#endif

#ifdef USE_USART
  ///////////////////////////////////////////////////////////////////////////////
  // USART
  
  USART_Reset(JTAG_USART);
  USART_Reset(TDI_USART);

  // enable clock
  CMU_ClockEnable(JTAG_USART_CLK, true);
  CMU_ClockEnable(TDI_USART_CLK, true);
  CMU_ClockEnable(cmuClock_PRS, true);;

  // configure
  USART_InitSync_TypeDef init = USART_INITSYNC_DEFAULT;
  init.baudrate = 10000000;
  init.msbf     = false;
  USART_InitSync(JTAG_USART, &init);
  USART_InitSync(TDI_USART, &init);

  USART_PrsTriggerInit_TypeDef prsInit = USART_INITPRSTRIGGER_DEFAULT;
  prsInit.autoTxTriggerEnable = false;
  prsInit.rxTriggerEnable = true;
  prsInit.txTriggerEnable = true;
  prsInit.prsTriggerChannel = PRS_CHANNEL;
  USART_InitPrsTrigger(JTAG_USART, &prsInit);
  USART_InitPrsTrigger(TDI_USART, &prsInit);

  usartOn();
#endif

  return true;
}

#endif

