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
  #define UVRCAN_BITRATE    CAN_125KBPS
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

// CAN Senden
#define CAN_KNOTEN_ID           5                                 // Knotennummer dieses Geraets
#define CAN_SEND_ID_DIGITAL     (CAN_KNOTEN_ID | 0x180)           // Digitalwerte 1...16
#define CAN_SEND_ID_ANALOG_1    (CAN_KNOTEN_ID | 0x200)           // Analogwerte  1...4
#define CAN_SEND_ID_ANALOG_2    (CAN_KNOTEN_ID | 0x280)           // Analogwerte  5...8
#define CAN_SEND_ID_ANALOG_3    (CAN_KNOTEN_ID | 0x300)           // Analogwerte  9...12
#define CAN_SEND_ID_ANALOG_4    (CAN_KNOTEN_ID | 0x380)           // Analogwerte 13...16

// CAN Empfangen
#define CAN_KNOTEN_ID_RECV_1    1                                 // Kontennummer Hauptsteuerung
#define CAN_RECV_ID_DIGITAL_1   (0x180 | CAN_KNOTEN_ID_RECV_1)    // Digitalwerte 1...16
#define CAN_RECV_ID_ANALOG_1    (0x200 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte  1...4
#define CAN_RECV_ID_ANALOG_2    (0x280 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte  5...8
#define CAN_RECV_ID_ANALOG_3    (0x300 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte  9...12 
#define CAN_RECV_ID_ANALOG_4    (0x380 | CAN_KNOTEN_ID_RECV_1)    // Analogwerte 13...16



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


    /*
        set filter 0 ... 5
    */
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF0, false, CAN_RECV_ID_ANALOG_1)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF0"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF1, false, CAN_RECV_ID_DIGITAL_1)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF1"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF2, false, CAN_RECV_ID_ANALOG_1)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF2"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF3, false, CAN_RECV_ID_DIGITAL_1)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF3"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF4, false, CAN_RECV_ID_ANALOG_1)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF4"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF5, false, CAN_RECV_ID_DIGITAL_1)) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF5"));
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
  static int messagecnt = 0;
  int intval = 0;
  struct can_frame canMsg;

  // uint16_t id = 5;
  // uint16_t len = 8;
  // uint8_t  data[8];
  // data[0] = 0x01;
  // data[1] = 0x02;
  // data[2] = 0x03;
  // data[3] = 0x04;
  // data[4] = 0x05;
  // data[5] = 0x06;
  // data[6] = 0x07;
  // data[7] = 0x08;

  // struct can_frame canMsg;

  // AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: CanSend (%d)->%d"), id,len);

  // canMsg.can_id =id;
  // canMsg.can_dlc=len;
  // for (uint8_t i=0;i<len;i++) {
  //   canMsg.data[i]=data[i];
  //   //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: CanSend data[%d]=%d"),i,data[i]);
  // }
  // mcp2515->sendMessage(&canMsg);

  // Serial.println(DcomMbLt.return_water_temp, DEC);

  // uint16_t id = CAN_SEND_ID;
  // uint16_t len = 8;
  // uint8_t  data[8];
  
  canMsg.can_dlc = 8;

  switch (messagecnt) {
    case 0: canMsg.can_id = CAN_SEND_ID_ANALOG_1;
    
            canMsg.data[0] = (uint8_t) (DcomMbLt.unit_error & 0xFF);
            canMsg.data[1] = 0x00;

            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;

            canMsg.data[4] = 0x00;
            canMsg.data[5] = 0x00;

            canMsg.data[6] = 0x00;
            canMsg.data[7] = 0x00;
            messagecnt++;
            break;

    case 1: canMsg.can_id = CAN_SEND_ID_DIGITAL;

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
            canMsg.data[0] = (uint8_t) (intval & 0xFF);
            canMsg.data[1] = (uint8_t) (intval >> 8 & 0xFF);

            canMsg.data[2] = 0x00;
            canMsg.data[3] = 0x00;

            canMsg.data[4] = 0x00;    // unused
            canMsg.data[5] = 0x00;    // unused

            canMsg.data[6] = 0x00;    // unused
            canMsg.data[7] = 0x00;    // unused
            messagecnt++;
            break;

    case 2: canMsg.can_id = CAN_SEND_ID_ANALOG_2;
    
            intval = (int) (DcomMbLt.leaving_water_PHE_temp * 10);
            canMsg.data[0] = (uint8_t) (intval & 0xFF);
            canMsg.data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.leaving_water_BHU_temp * 10);
            canMsg.data[2] = (uint8_t) (intval & 0xFF);
            canMsg.data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.return_water_temp * 10);
            canMsg.data[4] = (uint8_t) (intval & 0xFF);
            canMsg.data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.dom_hot_water_temp * 10);
            canMsg.data[6] = (uint8_t) (intval & 0xFF);
            canMsg.data[7] = (uint8_t) (intval >> 8 & 0xFF);
            messagecnt++;
            break;

    case 3: canMsg.can_id = CAN_SEND_ID_ANALOG_3;
    
            intval = (int) (DcomMbLt.outside_air_temp * 10);
            canMsg.data[0] = (uint8_t) (intval & 0xFF);
            canMsg.data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.liquid_refrig_temp * 10);
            canMsg.data[2] = (uint8_t) (intval & 0xFF);
            canMsg.data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) DcomMbLt.flow_rate;
            canMsg.data[4] = (uint8_t) (intval & 0xFF);
            canMsg.data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (DcomMbLt.room_temp * 10);
            canMsg.data[6] = (uint8_t) (intval & 0xFF);
            canMsg.data[7] = (uint8_t) (intval >> 8 & 0xFF);
            messagecnt = 0;
            break;

    case 4: messagecnt = 0;
            break;

    default: messagecnt = 0;
            break;
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
//          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received message 0x%s from ID 0x%x"), canMsg, (uint32_t)canFrame.can_id);

//          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received: ID: %d"), (uint32_t)canFrame.can_id);
//          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received: LEN: %d"), (uint32_t)canFrame.can_dlc);
//          for (int i = 0; i < canFrame.can_dlc; i++) {
//            AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received: DATA[%d]: %d"), i,canFrame.data[i]);
//            }

        // Response_P(PSTR("{\"%s\":%d,\"%s\":%d"),
        //   "ID",(uint32_t)canFrame.can_id,
        //   "LEN",(uint32_t)canFrame.can_dlc
        //   );
        // for (int i = 0; i < canFrame.can_dlc; i++) { ResponseAppend_P(PSTR(",\"D%d\":%d"),i,canFrame.data[i]); }
        // ResponseJsonEnd();


        // MqttPublishPrefixTopic_P(STAT, "CAN");
        // ResponseClear();


    

        

        switch(canFrame.can_id) {
          case CAN_RECV_ID_DIGITAL_1:
              // Space Heating/Cooling On/Off         	int16	  Auto/Heat/Cool        M1, Bit 0 Heating, M1, Bit 1 Cooling
              // Space Heating/Cooling On/Off         	int16	  0:OFF 1:ON	          M1, Bit 2
              // Quiet Mode Operation	                  int16	  0:OFF 1:ON	          M1, Bit 3
              // DHW Booster Mode On/Off                int16	  0:OFF 1:ON	          M1, Bit 4

              // Dies wird dann in die ersten 4 bytes gesteckt, die Reihenfolge ist so: (1. byte, 2. byte usw.)
              // 8 7 6 5 4 3 2 1 16 15 14 13 12 11 10 9 24 23 22 21 20 19 18 17 32 31 30 29 28 27 26 25
              // Die Zahlen steht für die jeweilge Ausgangsnummer.

              // Operation Mode - Auto/Heat/Cool - Heating has prio
              if (canFrame.data[0] & 0x01) intval = 1;
              else if (canFrame.data[0] & 0x02) intval = 2;
              else intval = 0;
              DcomMbLt.target_opmode = (uint16_t) intval;
              Serial.print("Operation Mode: "); Serial.println(intval, DEC);

              // Space Heating/Cooling On/Off
              if (canFrame.data[0] & 0x04) intval = 1;
              else intval = 0;
              DcomMbLt.target_spaceheatcool = (uint16_t) intval;
              Serial.print("Space Heating/Cooling: "); Serial.println(intval, DEC);

              // Quiet Mode Operation
              if (canFrame.data[0] & 0x08) intval = 1;
              else intval = 0;
              DcomMbLt.target_quietmode = (uint16_t) intval;
              Serial.print("Quiet Mode Operation: "); Serial.println(intval, DEC);

              // DHW Booster Mode On/Off
              if (canFrame.data[0] & 0x10) intval = 1;
              else intval = 0;
              DcomMbLt.target_dhwbooster = (uint16_t) intval;
              Serial.print("DHW Booster Mode On/Off: "); Serial.println(intval, DEC);
              
              break;

          case CAN_RECV_ID_ANALOG_1:
              // Leaving Water Main Heating Setpoint    int16	  25 .. 55ºC	            M0, Byte0..1

              // Leaving Water Main Heating Setpoint
              intval = ((unsigned int) canFrame.data[1] << 8) + (unsigned int) canFrame.data[0];
              if (intval > 550) intval = 55;
              else if (intval < 250) intval = 25;
              DcomMbLt.target_leavingwaterheattemp = (uint16_t) intval / 10;
              Serial.print("Leaving Water Main Heating Setpoint: "); Serial.println(DcomMbLt.target_leavingwaterheattemp, DEC);

              break;
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