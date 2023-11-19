/*
  xdrv135_uvr_can.ino - CAN bus support for TA UVR

  Copyright (C) 2023

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_SPI
#ifdef USE_UVRCAN
#ifdef USE_MCP2515 || USE_CANSNIFFER
#undef USE_MCP2515
#warning **** USE_MCP2515 and USE_CANSNIFFER disabled in favour of USE_UVRCAN ****
#endif
/*********************************************************************************************\
 * CAN sniffer using MCP2515 - Microchip CAN controller
 *
 * Connections:
 * MCP2515  ESP32           Tasmota
 * -------  --------------  ----------
 *  INT     GPIO35          MCP2515_INT
 *  SCK     GPIO14          SPI CLK
 *  SI      GPIO13          SPI MOSI
 *  SO      GPIO12          SPI MISO
 *  CS      GPIO15          MCP2515
 *  Gnd     Gnd
 *  VCC     Vin/5V
\*********************************************************************************************/

#define XDRV_135              135

#ifndef UVRCAN_BITRATE
  #define UVRCAN_BITRATE    CAN_50KBPS
#endif

#ifndef UVRCAN_CLOCK
  #define UVRCAN_CLOCK      MCP_8MHZ
#endif

#ifndef UVRCAN_MAX_FRAMES
  #define UVRCAN_MAX_FRAMES 8
#endif

#ifndef CAN_KEEP_ALIVE_SECS
  #define CAN_KEEP_ALIVE_SECS 300
#endif

#ifndef UVRCAN_TIMEOUT
  #define UVRCAN_TIMEOUT 10
#endif

#define D_PRFX_CAN "Can"
//#define D_CMND_CANSEND "Send"

void UVRCAN_ISR();

//const char kCanCommands[] PROGMEM =  D_PRFX_CAN "|" "|" D_CMND_CANSEND ;
//void (* const CanCommand[])(void) PROGMEM = { &CmndCan, &CmndCanSend};

#include "mcp2515.h"

struct UVRCAN_Struct {
  uint32_t lastFrameRecv = 0;
  int8_t init_status = 0;
  unsigned char flagRecv = 0;
} Mcp2515;

struct can_frame canFrame;

MCP2515 *mcp2515 = nullptr;

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

// void CmndCanSend(void) {
//   JsonParser parser(XdrvMailbox.data);
//   JsonParserObject root = parser.getRootObject();

//   uint16_t id = root.getUInt(PSTR("ID"), 0);   // case insensitive
//   uint16_t len = root.getUInt(PSTR("LEN"), 0);   // case insensitive
//   JsonParserArray data = root[PSTR("DATA")];

//   struct can_frame canMsg;

//   AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: CanSend (%d)->%d"), id,len);
//   canMsg.can_id =id;
//   canMsg.can_dlc=len;
//   for (uint8_t i=0;i<len;i++) {
//     canMsg.data[i]=data[i].getUInt();
//     AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: CanSend data[%d]=%d"),i,data[i].getUInt());
//   }
//   mcp2515->sendMessage(&canMsg);
// //  delay(100);
//   ResponseCmndChar_P(PSTR("OK"));
// }

char c2h(char c) {
  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}

void UVRCAN_FrameSizeError(uint8_t len, uint32_t id) {
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Unexpected length (%d) for ID 0x%x"), len, id);
}

void UVRCAN_Init(void) {
  if (PinUsed(GPIO_MCP2515_CS, GPIO_ANY) && PinUsed(GPIO_MCP2515_INT, GPIO_ANY) && TasmotaGlobal.spi_enabled) {
    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Init"));
    
    SPI._spi_num = HSPI;        // hack in SPI.h: class SPIClass --> int8_t _spi_num must be public to be set to HSPI
    SPI.setFrequency(1000000);

    mcp2515 = new MCP2515(Pin(GPIO_MCP2515_CS));
    

    attachInterrupt(digitalPinToInterrupt(Pin(GPIO_MCP2515_INT)), UVRCAN_ISR, FALLING); // start interrupt

    if (MCP2515::ERROR_OK != mcp2515->reset()) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to reset module"));
      return;
    }
    delay(10);
    if (MCP2515::ERROR_OK != mcp2515->setBitrate(UVRCAN_BITRATE, UVRCAN_CLOCK)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set module bitrate"));
      return;
    }
    delay(10);
    if (MCP2515::ERROR_OK != mcp2515->setNormalMode()) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set normal mode"));
      return;
    }
    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Initialized"));
    Mcp2515.init_status = 1;
  }
}

void UVRCAN_Write() {
  uint16_t id = 5;
  uint16_t len = 8;
  uint8_t  data[8];
  data[0] = 0x01;
  data[1] = 0x02;
  data[2] = 0x03;
  data[3] = 0x04;
  data[4] = 0x05;
  data[5] = 0x06;
  data[6] = 0x07;
  data[7] = 0x08;

  struct can_frame canMsg;

  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: CanSend (%d)->%d"), id,len);

  canMsg.can_id =id;
  canMsg.can_dlc=len;
  for (uint8_t i=0;i<len;i++) {
    canMsg.data[i]=data[i];
    //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: CanSend data[%d]=%d"),i,data[i]);
  }
  mcp2515->sendMessage(&canMsg);

}

void UVRCAN_Read() {
  uint8_t nCounter = 0;
  bool checkRcv;
  char mqtt_data[128];

  Mcp2515.flagRecv = 0;
  //checkRcv = mcp2515->checkReceive();
  checkRcv = true;

  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv"));

  while (checkRcv && nCounter <= UVRCAN_MAX_FRAMES) {
    mcp2515->checkReceive();
    nCounter++;
    if (mcp2515->readMessage(&canFrame) == MCP2515::ERROR_OK) {
      Serial.println(F("UVRCAN: Frame Rcv"));

      Mcp2515.lastFrameRecv = TasmotaGlobal.uptime;

        char canMsg[17];
        canMsg[0] = 0;
        for (int i = 0; i < canFrame.can_dlc; i++) {
          canMsg[i*2] = c2h(canFrame.data[i]>>4);
          canMsg[i*2+1] = c2h(canFrame.data[i]);
        }

        if (canFrame.can_dlc > 0) {
          canMsg[(canFrame.can_dlc - 1) * 2 + 2] = 0;
        }
//          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received message 0x%s from ID 0x%x"), canMsg, (uint32_t)canFrame.can_id);

//          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received: ID: %d"), (uint32_t)canFrame.can_id);
//          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received: LEN: %d"), (uint32_t)canFrame.can_dlc);
//          for (int i = 0; i < canFrame.can_dlc; i++) {
//            AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received: DATA[%d]: %d"), i,canFrame.data[i]);
//            }
        Response_P(PSTR("{\"%s\":%d,\"%s\":%d"),
          "ID",(uint32_t)canFrame.can_id,
          "LEN",(uint32_t)canFrame.can_dlc
          );
        for (int i = 0; i < canFrame.can_dlc; i++) { ResponseAppend_P(PSTR(",\"D%d\":%d"),i,canFrame.data[i]); }
        ResponseJsonEnd();


        MqttPublishPrefixTopic_P(STAT, "CAN");
        ResponseClear();


    } else if (mcp2515->checkError()) {
      uint8_t errFlags = mcp2515->getErrorFlags();
        mcp2515->clearRXnOVRFlags();
        AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received error %d"), errFlags);
        break;
    }
  }
}


void UVRCAN_ISR() {
    Mcp2515.flagRecv = 1;
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv135(uint32_t function) {
  bool result = false;

  if (FUNC_INIT == function) {
    UVRCAN_Init();
  }
  else if (Mcp2515.init_status) {
    switch (function) {
      case FUNC_EVERY_50_MSECOND:
        if(Mcp2515.flagRecv) UVRCAN_Read();
        break;
      case FUNC_COMMAND:
        
        break;
      case FUNC_EVERY_SECOND:
        UVRCAN_Write();
        break;

      case FUNC_JSON_APPEND:
//        UVRCAN_Show(1);
        break;
        #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
//        UVRCAN_Show(0);
        break;
      #endif  // USE_WEBSERVER
          }
  }
  return result;
}

#endif  // USE_UVRCAN
#endif  // USE_SPI