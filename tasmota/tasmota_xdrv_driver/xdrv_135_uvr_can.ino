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

#define UVR_CAN_DEBUG       // comment to disable debug messages over uart


#define XDRV_135              135

#ifndef UVRCAN_BITRATE
  #define UVRCAN_BITRATE      CAN_125KBPS
#endif

#ifndef UVRCAN_CLOCK
  #define UVRCAN_CLOCK        MCP_8MHZ
#endif

#ifndef UVRCAN_MAX_FRAMES
  #define UVRCAN_MAX_FRAMES   8
#endif

#ifndef CAN_KEEP_ALIVE_SECS
  #define CAN_KEEP_ALIVE_SECS 300
#endif

#ifndef UVRCAN_TIMEOUT
  #define UVRCAN_TIMEOUT      10
#endif

#define UVRCAN_MAXID          62          // Id / Knoten 1...62 allowed

// CAN Senden
// #define CAN_KNOTEN_ID           5                                 // Knotennummer dieses Geraets
// #define CAN_SEND_ID_DIGITAL     (CAN_KNOTEN_ID | 0x180)           // Digitalwerte 1...16
// #define CAN_SEND_ID_ANALOG_1    (CAN_KNOTEN_ID | 0x200)           // Analogwerte  1...4
// #define CAN_SEND_ID_ANALOG_2    (CAN_KNOTEN_ID | 0x280)           // Analogwerte  5...8
// #define CAN_SEND_ID_ANALOG_3    (CAN_KNOTEN_ID | 0x300)           // Analogwerte  9...12
// #define CAN_SEND_ID_ANALOG_4    (CAN_KNOTEN_ID | 0x380)           // Analogwerte 13...16
#define CAN_SEND_ID_DIGITAL     0x180           // Digitalwerte 1...16
#define CAN_SEND_ID_ANALOG_1    0x200           // Analogwerte  1...4
#define CAN_SEND_ID_ANALOG_2    0x280           // Analogwerte  5...8
#define CAN_SEND_ID_ANALOG_3    0x300           // Analogwerte  9...12
#define CAN_SEND_ID_ANALOG_4    0x380           // Analogwerte 13...16

// CAN Empfangen
// #define CAN_KNOTEN_ID_RECV_1    1                                 // Kontennummer Hauptsteuerung
// #define CAN_RECV_ID_DIGITAL_1   (0x180 | CAN_KNOTEN_ID_RECV_1)    // Digitalwerte 1...16
// #define CAN_RECV_ID_ANALOG_1    (0x200 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte  1...4
// #define CAN_RECV_ID_ANALOG_2    (0x280 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte  5...8
// #define CAN_RECV_ID_ANALOG_3    (0x300 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte  9...12 
// #define CAN_RECV_ID_ANALOG_4    (0x380 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte 13...16
#define CAN_RECV_ID_DIGITAL_1   0x180           // Digitalwerte 1...16
#define CAN_RECV_ID_ANALOG_1    0x200           // Analogwerte  1...4
#define CAN_RECV_ID_ANALOG_2    0x280           // Analogwerte  5...8
#define CAN_RECV_ID_ANALOG_3    0x300           // Analogwerte  9...12 
#define CAN_RECV_ID_ANALOG_4    0x380           // Analogwerte 13...16


#define D_PRFX_UVRCAN "UvrCan"
#define D_CMD_UVRCAN_DATASET "UvrCanDataset"
#define D_CMD_UVRCAN_SENDID  "UvrCanSendId"
#define D_CMD_UVRCAN_RECVID  "UvrCanRecvId"


void UVRCAN_ISR();

const char kUvrCanCommands[] PROGMEM = "|" D_CMD_UVRCAN_DATASET 
                                       "|" D_CMD_UVRCAN_SENDID
                                       "|" D_CMD_UVRCAN_RECVID;
void (* const UvrCanCommand[])(void) PROGMEM = { &CmndUvrCanDataset, &CmndUvrCanSendId, &CmndUvrCanRecvId };

#include "mcp2515.h"

struct UVRCAN_Struct {
  uint32_t lastFrameRecv = 0;
  int8_t   init_status = 0;
  unsigned char flagRecv = 0;  
} Mcp2515;

struct can_frame canFrame;

MCP2515 *mcp2515 = nullptr;

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndUvrCanDataset (void) {
  if ((XdrvMailbox.payload >= 1) && (XdrvMailbox.payload <= 2)) {
    Settings->UvrCanDataset = XdrvMailbox.payload;
  }
  ResponseCmndIdxNumber(Settings->UvrCanDataset);
  
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Dataset %d"), Settings->UvrCanDataset);
}

void CmndUvrCanSendId (void) {
  if ((XdrvMailbox.payload >= 1) && (XdrvMailbox.payload <= UVRCAN_MAXID)) {
    Settings->UvrCanSendId = XdrvMailbox.payload;
  }
  ResponseCmndIdxNumber(Settings->UvrCanSendId);
  
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Send ID %d"), Settings->UvrCanSendId);
}

void CmndUvrCanRecvId (void) {
  if ((XdrvMailbox.payload >= 1) && (XdrvMailbox.payload <= UVRCAN_MAXID)) {
    Settings->UvrCanRecvId = XdrvMailbox.payload;
    UVRCAN_SetFilter((uint8_t) Settings->UvrCanRecvId);
  }
  ResponseCmndIdxNumber(Settings->UvrCanRecvId);
  
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv ID %d"), Settings->UvrCanRecvId);
}


char c2h(char c) {
  return "0123456789ABCDEF"[0x0F & (unsigned char)c];
}


void UVRCAN_FrameSizeError(uint8_t len, uint32_t id) {
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Unexpected length (%d) for ID 0x%x"), len, id);
}


void UVRCAN_SetFilter(uint8_t RecvId) {
    /*
        set filter 0 ... 5
    */
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF0, false, ((uint32_t) RecvId | CAN_RECV_ID_ANALOG_1) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF0"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF1, false, ((uint32_t)RecvId | CAN_RECV_ID_DIGITAL_1) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF1"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF2, false, ((uint32_t)RecvId | CAN_RECV_ID_ANALOG_1) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF2"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF3, false, ((uint32_t)RecvId | CAN_RECV_ID_DIGITAL_1) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF3"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF4, false, ((uint32_t)RecvId | CAN_RECV_ID_ANALOG_1) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF4"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF5, false, ((uint32_t)RecvId | CAN_RECV_ID_DIGITAL_1) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF5"));
      return;
    }
}


void UVRCAN_Init(void) {
  if (PinUsed(GPIO_MCP2515_CS, GPIO_ANY) && PinUsed(GPIO_MCP2515_INT, GPIO_ANY) && TasmotaGlobal.spi_enabled) {
    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Init"));
    
    SPI._spi_num = HSPI;        // hack in SPI.h: class SPIClass --> int8_t _spi_num must be public to be set to HSPI
                                       // hack in SPI.h: class SPIClass --> uint8_t pinSet must be public to be set to HSPI
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

    // check settings
    if (Settings->UvrCanDataset < 1) Settings->UvrCanDataset = 1;
    if (Settings->UvrCanRecvId < 1)  Settings->UvrCanRecvId = 1;
    if (Settings->UvrCanRecvId > UVRCAN_MAXID) Settings->UvrCanRecvId = UVRCAN_MAXID;
    if (Settings->UvrCanSendId < 1)  Settings->UvrCanSendId = 1;
    if (Settings->UvrCanSendId > UVRCAN_MAXID) Settings->UvrCanSendId = UVRCAN_MAXID;

    /*
        set mask, set both the mask to 0x3ff
    */
    if (MCP2515::ERROR_OK != mcp2515->setFilterMask(MCP2515::MASK0, false, 0x3ff)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set Filter Mask 0"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilterMask(MCP2515::MASK1, false, 0x3ff)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set Filter Mask 1"));
      return;
    }

    // set filter id
    UVRCAN_SetFilter((uint32_t) Settings->UvrCanRecvId);
    
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
  static int messagecnt = 0;
  struct can_frame canMsg;
  
  canMsg.can_dlc = 8;

  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("UVRCAN: Send CAN"));

  // collect payload
  if(Settings->UvrCanDataset == 1) {
    UVRCan_Dataset_1_Send(&canMsg, messagecnt);
    messagecnt++;
    if (messagecnt>3) messagecnt = 0;
  }
  else if(Settings->UvrCanDataset == 2) {
    UVRCan_Dataset_2_Send(&canMsg, messagecnt);
    messagecnt++;
    if (messagecnt>2) messagecnt = 1;
  }
     
  mcp2515->sendMessage(&canMsg);

#ifdef UVR_CAN_DEBUG
  //Serial.print("Send CAN msg: "); Serial.println(messagecnt, DEC);
#endif

}


void UVRCAN_Read() {
  uint8_t nCounter = 0;
  bool checkRcv;
  char mqtt_data[128];
  unsigned int intval = 0;

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
        
        if(Settings->UvrCanDataset == 1) {
          if(canFrame.can_id == (Settings->UvrCanRecvId | CAN_RECV_ID_DIGITAL_1)) UVRCan_Dataset_1_Recv(&canFrame, CAN_RECV_ID_DIGITAL_1);
          else if(canFrame.can_id == (Settings->UvrCanRecvId | CAN_RECV_ID_ANALOG_1)) UVRCan_Dataset_1_Recv(&canFrame, CAN_RECV_ID_ANALOG_1);
        }
        else if(Settings->UvrCanDataset == 2) {
          //if(canFrame.can_id == (Settings->UvrCanRecvId | CAN_RECV_ID_DIGITAL_1)) UVRCan_Dataset_2_Recv(&canFrame, CAN_RECV_ID_DIGITAL_1);
          //else if(canFrame.can_id == (Settings->UvrCanRecvId | CAN_RECV_ID_ANALOG_1)) UVRCan_Dataset_2_Recv(&canFrame, CAN_RECV_ID_ANALOG_1);
        }

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
        result = DecodeCommand(kUvrCanCommands, UvrCanCommand);
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


void UVRCan_Dataset_1_Send (struct can_frame *canMsg, uint8_t message_nr) {
  int intval = 0;
  
  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("UVRCAN: Dataset 1 - %d"), message_nr);

  switch (message_nr) {
    case 0: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_DIGITAL);

            intval = 0x0000;
            if (DcomMbLt.circ_pump_run) intval | 0x0001;
            else intval & ~0x0001;
            if (DcomMbLt.compressor_run) intval | 0x0002;
            else intval & ~0x0002;      
            if (DcomMbLt.booster_heat_run) intval | 0x0004;
            else intval & ~0x0004;   
            if (DcomMbLt.desinfection_op) intval | 0x0008;
            else intval & ~0x0008;
            if (DcomMbLt.defrost_startup) intval | 0x0010;
            else intval & ~0x0010;
            if (DcomMbLt.hot_start) intval | 0x0020;
            else intval & ~0x0020;    
            if (DcomMbLt.valve_3way) intval | 0x0040;
            else intval & ~0x0040;
            if (DcomMbLt.op_mode = 1) intval | 0x0080;
            else intval & ~0x0080;
            if (DcomMbLt.op_mode = 2) intval | 0x0100;
            else intval & ~0x0100;
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;    // unused
            canMsg->data[5] = 0x00;    // unused

            canMsg->data[6] = 0x00;    // unused
            canMsg->data[7] = 0x00;    // unused

            break;

    case 1: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_1);
    
            canMsg->data[0] = (uint8_t) (DcomMbLt.unit_error & 0xFF);
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;

            break;

    case 2: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_2);
    
            intval = (int) (DcomMbLt.leaving_water_PHE_temp * 10);
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.leaving_water_BHU_temp * 10);
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.return_water_temp * 10);
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.dom_hot_water_temp * 10);
            canMsg->data[6] = (uint8_t) (intval & 0xFF);
            canMsg->data[7] = (uint8_t) (intval >> 8 & 0xFF);

            break;

    case 3: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_3);    
            intval = (int) (DcomMbLt.outside_air_temp * 10);
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.liquid_refrig_temp * 10);
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) DcomMbLt.flow_rate;
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.room_temp * 10);
            canMsg->data[6] = (uint8_t) (intval & 0xFF);
            canMsg->data[7] = (uint8_t) (intval >> 8 & 0xFF);
    
            break;

    case 4: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_4);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;    

    default: 
            break;

  }
}


void UVRCan_Dataset_2_Send (struct can_frame *canMsg, uint8_t message_nr) {
  int intval = 0;
  switch (message_nr) {
    case 0: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_DIGITAL);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 1: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_1);
            // Val1: balanced power of all three phases [W]
            // Val2: power phase 1 [W]
            // Val3: power phase 2 [W]
            // Val4: power phase 3 [W]
            intval = (int) (Energy->active_power[0] + Energy->active_power[1] + Energy->active_power[2]);
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) Energy->active_power[0];
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) Energy->active_power[1];
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) Energy->active_power[2];
            canMsg->data[6] = (uint8_t) (intval & 0xFF);
            canMsg->data[7] = (uint8_t) (intval >> 8 & 0xFF);
            break;

    case 2: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_2);
            // Val1: import energy today [0.1 kWh]
            // Val2: export energy today [0.1 kWh]
            // Val3: daily energy balanced [0.1 kWh]
            // Val4: -
            intval = (int) (Energy->daily_sum_import_balanced * 10);
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (Energy->daily_sum_export_balanced * 10);
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) ((Energy->daily_kWh[0] + Energy->daily_kWh[1] + Energy->daily_kWh[2]) * 10);
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 3: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_3);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 4: canMsg->can_id = ((uint32_t)Settings->UvrCanSendId | CAN_SEND_ID_ANALOG_4);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    default: 
            break;
  }
}


void UVRCan_Dataset_1_Recv (struct can_frame *canMsg, uint32_t message_id) {
  unsigned int intval = 0;

  switch (message_id) {
    case CAN_RECV_ID_DIGITAL_1:
          // Space Heating/Cooling On/Off         	int16	  Auto/Heat/Cool        M1, Bit 0 Heating, M1, Bit 1 Cooling
          // Space Heating/Cooling On/Off         	int16	  0:OFF 1:ON	          M1, Bit 2
          // Quiet Mode Operation	                  int16	  0:OFF 1:ON	          M1, Bit 3
          // DHW Booster Mode On/Off                int16	  0:OFF 1:ON	          M1, Bit 4

          // Dies wird dann in die ersten 4 bytes gesteckt, die Reihenfolge ist so: (1. byte, 2. byte usw.)
          // 8 7 6 5 4 3 2 1 16 15 14 13 12 11 10 9 24 23 22 21 20 19 18 17 32 31 30 29 28 27 26 25
          // Die Zahlen steht für die jeweilge Ausgangsnummer.

          // Operation Mode - Auto/Heat/Cool - Heating has prio
          if (canMsg->data[0] & 0x01) intval = 1;
          else if (canMsg->data[0] & 0x02) intval = 2;
          else intval = 0;
          DcomMbLt.target_opmode = (uint16_t) intval;
          Serial.print("Operation Mode: "); Serial.println(intval, DEC);

          // Space Heating/Cooling On/Off
          if (canMsg->data[0] & 0x04) intval = 1;
          else intval = 0;
          DcomMbLt.target_spaceheatcool = (uint16_t) intval;
          Serial.print("Space Heating/Cooling: "); Serial.println(intval, DEC);

          // Quiet Mode Operation
          if (canMsg->data[0] & 0x08) intval = 1;
          else intval = 0;
          DcomMbLt.target_quietmode = (uint16_t) intval;
          Serial.print("Quiet Mode Operation: "); Serial.println(intval, DEC);

          // DHW Booster Mode On/Off
          if (canMsg->data[0] & 0x10) intval = 1;
          else intval = 0;
          DcomMbLt.target_dhwbooster = (uint16_t) intval;
          Serial.print("DHW Booster Mode On/Off: "); Serial.println(intval, DEC);

          break;

    case CAN_RECV_ID_ANALOG_1:
          // Leaving Water Main Heating Setpoint    int16	  25 .. 55ºC	            M0, Byte0..1

          // Leaving Water Main Heating Setpoint
          intval = ((unsigned int) canMsg->data[1] << 8) + (unsigned int) canMsg->data[0];
          if (intval > 550) intval = 55;
          else if (intval < 250) intval = 25;
          DcomMbLt.target_leavingwaterheattemp = (uint16_t) intval / 10;
          Serial.print("Leaving Water Main Heating Setpoint: "); Serial.println(DcomMbLt.target_leavingwaterheattemp, DEC);
          
          break;

    default: break;
  }
}


#endif  // USE_UVRCAN
#endif  // USE_SPI