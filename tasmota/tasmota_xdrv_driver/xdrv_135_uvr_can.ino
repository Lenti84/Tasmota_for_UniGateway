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
#if defined(USE_MCP2515) || defined(USE_CANSNIFFER)
#undef USE_MCP2515
#warning **** USE_MCP2515 and USE_CANSNIFFER disabled in favour of USE_UVRCAN ****
#endif
/*********************************************************************************************\
 * CAN interface using MCP2515 - Microchip CAN controller
 *
 * Connections:
 * MCP2515  ESP32           Tasmota
 * -------  --------------  ----------
 *  INT     GPIO35          MCP2515_INT
 *  SCK     GPIO14          SPI CLK 1
 *  SI      GPIO13          SPI MOSI 1
 *  SO      GPIO12          SPI MISO 1
 *  CS      GPIO15          MCP2515_CS
 *  Gnd     Gnd
 *  VCC     Vin/5V
\*********************************************************************************************/

#define UVR_CAN_DEBUG       // comment to disable debug messages over uart


#define XDRV_135              135

#ifdef USE_SDM630_MULTI
// Implemented by xsns_122_sdm630_multi.ino.
float Sdm630MultiGetData(uint8_t index);
#endif

#ifdef USE_DCOM_LT_MB
#define UvrCanDcom DcomMbLt
#else
// Temporary internal adapters for compile testing without the UniGateway
// DCOM driver. Measurements are zero; setters only update RAM.
#warning UVRCAN uses an internal DCOM dummy without USE_DCOM_LT_MB
static struct {
  bool circ_pump_run = false;
  bool compressor_run = false;
  bool booster_heat_run = false;
  bool desinfection_op = false;
  bool defrost_startup = false;
  bool hot_start = false;
  bool valve_3way = false;
  uint16_t op_mode = 0;
  uint16_t unit_error = 0;
  float leaving_water_PHE_temp = 0;
  float leaving_water_BHU_temp = 0;
  float return_water_temp = 0;
  float dom_hot_water_temp = 0;
  float outside_air_temp = 0;
  float liquid_refrig_temp = 0;
  float flow_rate = 0;
  float room_temp = 0;
  uint16_t target_opmode = 0;
  uint16_t target_spaceheatcool = 0;
  uint16_t target_quietmode = 0;
  uint16_t target_dhwbooster = 0;
  uint16_t target_leavingwaterheattemp = 0;
} UvrCanDcom;
#endif

// static bool UvrCanDummySolisManual = false;
// static int UvrCanDummySolisPower = 0;

// void UvrCanDummySolisSetMode(bool manual) {
//   UvrCanDummySolisManual = manual;
// }

// void UvrCanDummySolisSetPower(int power) {
//   UvrCanDummySolisPower = power;
// }

// #ifdef USE_SDM72_SDM230
// float Sdm72Sdm230GetData(uint8_t index);

// int UvrCanSdm72Sdm230GetPower(uint8_t index) {
//   float power = Sdm72Sdm230GetData(index);
//   return isfinite(power) ? (int)power : 0;
// }
// #endif

// Dataset 3 requires meter values not provided by the combined SDM72/SDM230 sensor.
// float UvrCanDummySdmGetData(uint32_t index) {
//   (void)index;
//   return 0.0f;
// }

#include "mcp2515.h"

const CAN_SPEED kUvrCanBitrates[] = {
  CAN_10KBPS, CAN_20KBPS, CAN_50KBPS, CAN_125KBPS, CAN_250KBPS, CAN_500KBPS
};
const uint16_t kUvrCanBitrateKbps[] = { 10, 20, 50, 125, 250, 500 };

bool UvrCanValidBitrate(uint32_t bitrate) {
  for (uint32_t i = 0; i < sizeof(kUvrCanBitrates) / sizeof(kUvrCanBitrates[0]); i++) {
    if (bitrate == kUvrCanBitrates[i]) { return true; }
  }
  return false;
}

#ifndef UVRCAN_BITRATE
  #define UVRCAN_BITRATE      CAN_50KBPS
#endif

#ifndef UVRCAN_CLOCK
  #define UVRCAN_CLOCK        MCP_8MHZ
#endif

#ifndef UVRCAN_MAX_FRAMES
  #define UVRCAN_MAX_FRAMES   2
#endif

#ifndef CAN_KEEP_ALIVE_SECS
  #define CAN_KEEP_ALIVE_SECS 300
#endif

#ifndef UVRCAN_TIMEOUT
  #define UVRCAN_TIMEOUT      10
#endif

#define UVRCAN_MAXID          62          // Id / Knoten 1...62 allowed

// Driver-local settings, persisted through Tasmota's JSON settings API.
#define UVRCAN_SETTINGS_KEY "drvset135"
struct {
  uint32_t dataset = 1;
  uint32_t send_id = 1;
  uint32_t recv_id = 1;
  uint32_t bitrate = UVRCAN_BITRATE;
} UvrCanSettings;
uint32_t UvrCanSettingsCrc = 0;

void UVRCAN_SettingsLoad(bool erase) {
  UvrCanSettings.dataset = 1;
  UvrCanSettings.send_id = 1;
  UvrCanSettings.recv_id = 1;
  UvrCanSettings.bitrate = UvrCanValidBitrate(UVRCAN_BITRATE) ? UVRCAN_BITRATE : CAN_50KBPS;
  UvrCanSettingsCrc = 0;
#ifdef USE_UFILESYS
  char key[] = UVRCAN_SETTINGS_KEY;
  if (erase) {
    UfsJsonSettingsDelete(key);
    return;
  }
  String json = UfsJsonSettingsRead(key);
  if (!json.length()) { return; }
  JsonParser parser((char*)json.c_str());
  JsonParserObject root = parser.getRootObject();
  if (!root) { return; }

  uint32_t dataset = root.getUInt(PSTR("Dataset"), 1);
  uint32_t send_id = root.getUInt(PSTR("SendId"), 1);
  uint32_t recv_id = root.getUInt(PSTR("RecvId"), 1);
  uint32_t bitrate = root.getUInt(PSTR("Bitrate"), UvrCanSettings.bitrate);
  if (UvrCanValidBitrate(bitrate)) { UvrCanSettings.bitrate = bitrate; }
  if ((dataset >= 1) && (dataset <= 3)) { UvrCanSettings.dataset = dataset; }
  if ((send_id >= 1) && (send_id <= UVRCAN_MAXID)) { UvrCanSettings.send_id = send_id; }
  if ((recv_id >= 1) && (recv_id <= UVRCAN_MAXID)) { UvrCanSettings.recv_id = recv_id; }
  // Rewrite invalid values on the next save.
  if ((dataset == UvrCanSettings.dataset) && (send_id == UvrCanSettings.send_id) &&
      (recv_id == UvrCanSettings.recv_id) && (bitrate == UvrCanSettings.bitrate)) {
    UvrCanSettingsCrc = GetCfgCrc32((uint8_t*)&UvrCanSettings, sizeof(UvrCanSettings));
  }
#else
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Settings are not persisted without USE_UFILESYS"));
#endif
}

void UVRCAN_SettingsSave(void) {
#ifdef USE_UFILESYS
  uint32_t crc = GetCfgCrc32((uint8_t*)&UvrCanSettings, sizeof(UvrCanSettings));
  if (crc == UvrCanSettingsCrc) { return; }
  Response_P(PSTR("{\"" UVRCAN_SETTINGS_KEY "\":{\"Dataset\":%u,\"SendId\":%u,\"RecvId\":%u,\"Bitrate\":%u}}"),
             UvrCanSettings.dataset, UvrCanSettings.send_id, UvrCanSettings.recv_id, UvrCanSettings.bitrate);
  if (UfsJsonSettingsWrite(ResponseData())) {
    UvrCanSettingsCrc = crc;
  } else {
    // Keep the old CRC so a failed write is retried on the next save.
    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Unable to save settings"));
  }
#endif
}

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
#define CAN_RECV_ID_ANALOG_NEW  0x1C0           // Analogwerte - neues Datenformat, alle Analogwerte in Botschaft 0x1Cx (x = Knoten)


#define D_PRFX_UVRCAN "UvrCan"
#define D_CMD_UVRCAN_DATASET "UvrCanDataset"
#define D_CMD_UVRCAN_SENDID  "UvrCanSendId"
#define D_CMD_UVRCAN_RECVID  "UvrCanRecvId"


void UVRCAN_ISR();

const char kUvrCanCommands[] PROGMEM = "|" D_CMD_UVRCAN_DATASET 
                                       "|" D_CMD_UVRCAN_SENDID
                                       "|" D_CMD_UVRCAN_RECVID;
void (* const UvrCanCommand[])(void) PROGMEM = { &CmndUvrCanDataset, &CmndUvrCanSendId, &CmndUvrCanRecvId };


struct UVRCAN_Struct {
  uint32_t lastFrameRecv = 0;
  int8_t   init_status = 0;
  unsigned char flagRecv = 0;
  uint8_t  errors = 0; 
  uint8_t framecnt = 0;
} Mcp2515;


//struct can_frame canFrameData[16];
//struct can_frame *canFrame = nullptr;
struct can_frame canFrame[16];

MCP2515 *mcp2515 = nullptr;

/*********************************************************************************************\
 * Commands
\*********************************************************************************************/

void CmndUvrCanDataset (void) {
  if ((XdrvMailbox.payload >= 1) && (XdrvMailbox.payload <= 3)) {
    UvrCanSettings.dataset = XdrvMailbox.payload;
  }
  ResponseCmndIdxNumber(UvrCanSettings.dataset);
  
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Dataset %d"), UvrCanSettings.dataset);
}

void CmndUvrCanSendId (void) {
  if ((XdrvMailbox.payload >= 1) && (XdrvMailbox.payload <= UVRCAN_MAXID)) {
    UvrCanSettings.send_id = XdrvMailbox.payload;
  }
  ResponseCmndIdxNumber(UvrCanSettings.send_id);
  
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Send ID %d"), UvrCanSettings.send_id);
}

void CmndUvrCanRecvId (void) {
  if ((XdrvMailbox.payload >= 1) && (XdrvMailbox.payload <= UVRCAN_MAXID)) {
    UvrCanSettings.recv_id = XdrvMailbox.payload;
    UVRCAN_SetFilter((uint8_t) UvrCanSettings.recv_id);
  }
  ResponseCmndIdxNumber(UvrCanSettings.recv_id);
  
  AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv ID %d"), UvrCanSettings.recv_id);
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
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF0, false, ((uint32_t)RecvId | CAN_RECV_ID_ANALOG_NEW) )) {
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
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF3, false, ((uint32_t)RecvId | CAN_RECV_ID_ANALOG_2) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF3"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF4, false, ((uint32_t)RecvId | CAN_RECV_ID_ANALOG_3) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF4"));
      return;
    }
    if (MCP2515::ERROR_OK != mcp2515->setFilter(MCP2515::RXF5, false, ((uint32_t)RecvId | CAN_RECV_ID_ANALOG_4) )) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set setFilter RXF5"));
      return;
    }
}


void UVRCAN_Init(void) {
  //if (nullptr == SpiBegin(1)) { return; }

  if (PinUsed(GPIO_MCP2515_CS, GPIO_ANY) && PinUsed(GPIO_MCP2515_INT, GPIO_ANY) && TasmotaGlobal.spi_enabled) {
    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Init"));
    
    // MCP2515 uses the global SPI instance; initialize it with Tasmota's bus 1 pins.
    // we have to use HSPI
    //if (nullptr == SpiBegin(1)) { return; }
    SPI._spi_num = HSPI;        // hack in SPI.h: class SPIClass --> int8_t _spi_num must be public to be set to HSPI
                                 // hack in SPI.h: class SPIClass --> uint8_t pinSet must be public to be set to HSPI


    SPI.setFrequency(1000000);

    mcp2515 = new MCP2515(Pin(GPIO_MCP2515_CS));    

    //attachInterrupt(digitalPinToInterrupt(Pin(GPIO_MCP2515_INT)), UVRCAN_ISR, FALLING); // start interrupt
    delay(1);

    if (MCP2515::ERROR_OK != mcp2515->reset()) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to reset module"));
      return;
    }
    delay(10);

    for (int x=0;x<5;x++) {
      if (x == 4) {
        AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set module bitrate finally"));
        return;
      }
      if (MCP2515::ERROR_OK != mcp2515->setBitrate((CAN_SPEED)UvrCanSettings.bitrate, UVRCAN_CLOCK)) {
        AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set module bitrate"));
        // return;
        delay(10);
      }
      else {
        AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Set module bitrate okay"));
        x = 5;
      }       
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

    // set filter id
    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Set Recv Id %d"), (uint8_t) UvrCanSettings.recv_id);
    UVRCAN_SetFilter((uint8_t) UvrCanSettings.recv_id);
    
    delay(10);
    if (MCP2515::ERROR_OK != mcp2515->setNormalMode()) {
      AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Failed to set normal mode"));
      return;
    }

    attachInterrupt(digitalPinToInterrupt(Pin(GPIO_MCP2515_INT)), UVRCAN_ISR, FALLING); // start interrupt

    AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Initialized"));
    Mcp2515.init_status = 1;
    Mcp2515.flagRecv = 0;
  }
  else AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: could not init"));
}


void UVRCAN_Write() {
  static int messagecnt = 0;
  struct can_frame canMsg;
  
  canMsg.can_dlc = 8;

  AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("UVRCAN: Send CAN"));

  // collect payload
  if(UvrCanSettings.dataset == 1) {
    UVRCan_Dataset_1_Send(&canMsg, messagecnt);
    messagecnt++;
    if (messagecnt>4) messagecnt = 0;
  }
  else if(UvrCanSettings.dataset == 2) {
    UVRCan_Dataset_2_Send(&canMsg, messagecnt);
    messagecnt++;
    if (messagecnt>2) messagecnt = 1;
  }
  else if(UvrCanSettings.dataset == 3) {
    UVRCan_Dataset_3_Send(&canMsg, messagecnt);
    messagecnt++;
    if (messagecnt>3) messagecnt = 1;
  }
     
  mcp2515->sendMessage(&canMsg);

#ifdef UVR_CAN_DEBUG
  //Serial.print("Send CAN msg: "); Serial.println(messagecnt, DEC);
#endif

}


void UVRCAN_Read() {
    
    while (Mcp2515.framecnt > 0) {        
      AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Frame Read"));
        
      if(UvrCanSettings.dataset == 1) {
        //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 1"));        
        if(canFrame[Mcp2515.framecnt-1].can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_DIGITAL_1)) UVRCan_Dataset_1_Recv(&canFrame[Mcp2515.framecnt-1], CAN_RECV_ID_DIGITAL_1);
        else if(canFrame[Mcp2515.framecnt-1].can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_4)) UVRCan_Dataset_1_Recv(&canFrame[Mcp2515.framecnt-1], CAN_RECV_ID_ANALOG_4);
        else AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Recv Dataset 1 - unknown Recv Id %03X"), (uint16_t) canFrame[Mcp2515.framecnt-1].can_id);
      }
      else if(UvrCanSettings.dataset == 2) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Recv Dataset 2 - nothing defined"));
        //if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_DIGITAL_1)) UVRCan_Dataset_2_Recv(&canFrame[Mcp2515.framecnt], CAN_RECV_ID_DIGITAL_1);
        //else if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_4)) UVRCan_Dataset_2_Recv(&canFrame[Mcp2515.framecnt], CAN_RECV_ID_ANALOG_4);
      }
      else if(UvrCanSettings.dataset == 3) {
        //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 3 - Recv Id %03X"), (uint16_t) canFrame[Mcp2515.framecnt-1].can_id);
        if(canFrame[Mcp2515.framecnt-1].can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_1)) UVRCan_Dataset_3_Recv(&canFrame[Mcp2515.framecnt-1], CAN_RECV_ID_ANALOG_1);
        else if(canFrame[Mcp2515.framecnt-1].can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_2)) UVRCan_Dataset_3_Recv(&canFrame[Mcp2515.framecnt-1], CAN_RECV_ID_ANALOG_2);
        else if(canFrame[Mcp2515.framecnt-1].can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_NEW)) UVRCan_Dataset_3_Recv(&canFrame[Mcp2515.framecnt-1], CAN_RECV_ID_ANALOG_NEW);          
        else AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Recv Dataset 3 - unknown Recv Id %03X"), (uint16_t) canFrame[Mcp2515.framecnt-1].can_id);
      }   

      Mcp2515.framecnt--;
    }
    Mcp2515.flagRecv = 0;
  

}

// void UVRCAN_Read() {
//   uint8_t nCounter = 0;
//   bool checkRcv;
//   char mqtt_data[128];
//   unsigned int intval = 0;

//   Mcp2515.flagRecv = 0;
//   //checkRcv = mcp2515->checkReceive();
//   checkRcv = true;

//   AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv"));

//   while (checkRcv && nCounter <= UVRCAN_MAX_FRAMES) {
//     mcp2515->checkReceive();
//     nCounter++;
//     if (mcp2515->readMessage(&canFrame) == MCP2515::ERROR_OK) {
//       //Serial.println(F("UVRCAN: Frame Rcv"));
//       AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Frame Rcv"));

//       Mcp2515.lastFrameRecv = TasmotaGlobal.uptime;

//         char canMsg[17];
//         canMsg[0] = 0;
//         for (int i = 0; i < canFrame.can_dlc; i++) {
//           canMsg[i*2] = c2h(canFrame.data[i]>>4);
//           canMsg[i*2+1] = c2h(canFrame.data[i]);
//         }

//         if (canFrame.can_dlc > 0) {
//           canMsg[(canFrame.can_dlc - 1) * 2 + 2] = 0;
//         }
        
//         if(UvrCanSettings.dataset == 1) {
//           AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 1"));
//           if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_DIGITAL_1)) UVRCan_Dataset_1_Recv(&canFrame, CAN_RECV_ID_DIGITAL_1);
//           else if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_4)) UVRCan_Dataset_1_Recv(&canFrame, CAN_RECV_ID_ANALOG_4);
//         }
//         else if(UvrCanSettings.dataset == 2) {
//           AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 2"));
//           //if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_DIGITAL_1)) UVRCan_Dataset_2_Recv(&canFrame, CAN_RECV_ID_DIGITAL_1);
//           //else if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_4)) UVRCan_Dataset_2_Recv(&canFrame, CAN_RECV_ID_ANALOG_4);
//         }
//         else if(UvrCanSettings.dataset == 3) {
//           AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Set3 - Recv Id %d"), (uint8_t) UvrCanSettings.recv_id);
//           if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_1)) UVRCan_Dataset_3_Recv(&canFrame, CAN_RECV_ID_ANALOG_1);
//           else if(canFrame.can_id == (UvrCanSettings.recv_id | CAN_RECV_ID_ANALOG_NEW)) UVRCan_Dataset_3_Recv(&canFrame, CAN_RECV_ID_ANALOG_NEW);          
//         }

//     } else if (mcp2515->checkError()) {
//       uint8_t errFlags = mcp2515->getErrorFlags();
//       Mcp2515.errors = errFlags;
//       mcp2515->clearRXnOVRFlags();
//       AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received error %d"), errFlags);
//       break;
//     }
//   }
// }


void UVRCAN_ISR() {
  uint8_t nCounter = 0;
  
  //AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: ISR"));

  while (nCounter < UVRCAN_MAX_FRAMES) {
    mcp2515->checkReceive();
    nCounter++;    
    if (mcp2515->readMessage(&canFrame[Mcp2515.framecnt]) == MCP2515::ERROR_OK) {           
      //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Frame Rcv"));
      Mcp2515.framecnt++;
      Mcp2515.flagRecv = 1;
    }      
    else if (mcp2515->checkError()) {
      uint8_t errFlags = mcp2515->getErrorFlags();
      Mcp2515.errors = errFlags;
      mcp2515->clearRXnOVRFlags();
      //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Received error %d"), errFlags);
      break;
    }
  }
  
}



void UVRCAN_Show(bool json) {  
  if (Mcp2515.init_status == 1) {
    if (json) {
      // none
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_P(PSTR("{s}UVR CAN Module{m}{e}"));
      WSContentSend_PD("{s}CAN Recv ID{m}%u{e}", UvrCanSettings.recv_id);
      WSContentSend_PD("{s}CAN Send ID{m}%u{e}", UvrCanSettings.send_id);
        for (uint32_t i = 0; i < sizeof(kUvrCanBitrates) / sizeof(kUvrCanBitrates[0]); i++) {
          if (UvrCanSettings.bitrate == kUvrCanBitrates[i]) {
            WSContentSend_PD("{s}CAN Bitrate{m}%u kbit/s{e}", (uint32_t)kUvrCanBitrateKbps[i]);
            break;
          }
        }
      WSContentSend_PD("{s}Dataset{m}%u{e}", UvrCanSettings.dataset);
      WSContentSend_PD("{s}Error Status{m}%u{e}", Mcp2515.errors);
      WSContentSend_P(PSTR("{s} {m} {e}"));      
#endif  // USE_WEBSERVER
    }
  }
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#ifdef USE_WEBSERVER
bool UvrCanWebValue(const char* name, uint32_t maximum, uint32_t* value) {
  String arg = Webserver->arg(name);
  if (!arg.length() || arg.length() > 2) { return false; }
  uint32_t number = 0;
  for (uint32_t i = 0; i < arg.length(); i++) {
    if (arg[i] < '0' || arg[i] > '9') { return false; }
    number = number * 10 + arg[i] - '0';
  }
  if (number < 1 || number > maximum) { return false; }
  *value = number;
  return true;
}

void HandleUvrCanConfiguration(void) {
  if (!HttpCheckPriviledgedAccess()) { return; }
  const char* message = nullptr;
  if (Webserver->method() == HTTP_POST && Webserver->hasArg(F("save"))) {
    uint32_t dataset, send_id, recv_id, bitrate;
    if (UvrCanWebValue("dataset", 3, &dataset) &&
        UvrCanWebValue("send_id", UVRCAN_MAXID, &send_id) &&
        UvrCanWebValue("recv_id", UVRCAN_MAXID, &recv_id) &&
        UvrCanWebValue("bitrate", CAN_500KBPS, &bitrate) && UvrCanValidBitrate(bitrate)) {
      bool filter_changed = recv_id != UvrCanSettings.recv_id;
      bool bitrate_changed = bitrate != UvrCanSettings.bitrate;
      UvrCanSettings.dataset = dataset;
      UvrCanSettings.send_id = send_id;
      UvrCanSettings.recv_id = recv_id;
      UvrCanSettings.bitrate = bitrate;
      UVRCAN_SettingsSave();
      if (filter_changed && Mcp2515.init_status) { UVRCAN_SetFilter(recv_id); }
#ifdef USE_UFILESYS
      bool saved = UvrCanSettingsCrc == GetCfgCrc32((uint8_t*)&UvrCanSettings, sizeof(UvrCanSettings));
      if (saved && bitrate_changed) {
        WebRestart(1);
        return;
      }
      message = saved
        ? "Einstellungen gespeichert."
        : "Einstellungen uebernommen, aber Speichern fehlgeschlagen.";
#else
      message = "Einstellungen uebernommen. Ohne Dateisystem gehen sie beim Neustart verloren.";
#endif
    } else {
      message = "Ungueltige Eingabe: Datensatz 1 bis 3, IDs 1 bis 62 und eine angebotene Bitrate waehlen. Keine Aenderung uebernommen.";
    }
  }

  WSContentStart_P(PSTR("UVR CAN"));
  WSContentSendStyle();
  if (message) { WSContentSend_P(PSTR("<p>%s</p>"), message); }
  WSContentSend_P(PSTR("<form method='post' action='uvrcan'><fieldset><legend>UVR CAN</legend>"
    "<p><label for='dataset'>Datensatz</label><select id='dataset' name='dataset'>"));
  const char* labels[] = { "1 - DCOM", "2 - Energie", "3 - SDM / Solis" };
  for (uint32_t i = 1; i <= 3; i++) {
    WSContentSend_P(PSTR("<option value='%u'%s>%s</option>"), i,
      (i == UvrCanSettings.dataset) ? " selected" : "", labels[i - 1]);
  }
  WSContentSend_P(PSTR("</select></p><p><label for='bitrate'>CAN-Bitrate (Neustart)</label>"
    "<select id='bitrate' name='bitrate'>"));
  for (uint32_t i = 0; i < sizeof(kUvrCanBitrates) / sizeof(kUvrCanBitrates[0]); i++) {
    WSContentSend_P(PSTR("<option value='%u'%s>%u kbit/s</option>"), (uint32_t)kUvrCanBitrates[i],
      (UvrCanSettings.bitrate == kUvrCanBitrates[i]) ? " selected" : "", (uint32_t)kUvrCanBitrateKbps[i]);
  }
    WSContentSend_P(PSTR("</select></p>"
    "<p><label for='send_id'>Sende-ID (1-62)</label>"
    "<input id='send_id' name='send_id' type='number' min='1' max='62' required value='%u'></p>"
    "<p><label for='recv_id'>Empfangs-ID (1-62)</label>"
    "<input id='recv_id' name='recv_id' type='number' min='1' max='62' required value='%u'></p>"
    "</fieldset><p><button name='save' value='1' type='submit'>" D_SAVE "</button></p></form>"),
    UvrCanSettings.send_id, UvrCanSettings.recv_id);
  WSContentSpaceButton(BUTTON_CONFIGURATION);
  WSContentStop();
}
#endif  // USE_WEBSERVER

bool Xdrv135(uint32_t function) {
  bool result = false;

#ifdef USE_WEBSERVER
  // Configuration must also be reachable before CAN hardware is configured.
  if (FUNC_WEB_ADD_BUTTON == function) {
    WSContentSend_P(HTTP_FORM_BUTTON, PSTR("uvrcan"), PSTR("UVR CAN"));
    return false;
  }
  if (FUNC_WEB_ADD_HANDLER == function) {
    WebServer_on(PSTR("/uvrcan"), HandleUvrCanConfiguration);
    return false;
  }
#endif

  if (FUNC_PRE_INIT == function) {
    UVRCAN_SettingsLoad(false);
  }
  else if (FUNC_RESET_SETTINGS == function) {
    UVRCAN_SettingsLoad(true);
    if (Mcp2515.init_status) { UVRCAN_SetFilter(UvrCanSettings.recv_id); }
  }
  else if (FUNC_SAVE_SETTINGS == function) {
    UVRCAN_SettingsSave();
  }
  else if (FUNC_INIT == function) {
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
        Mcp2515.flagRecv = 1;       // read CAN messages stored without INT generation by MCP2515
        break;

      case FUNC_JSON_APPEND:
//        UVRCAN_Show(1);
        break;
        #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        UVRCAN_Show(0);
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
    case 0: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_DIGITAL);

            intval = 0x0000;
            if (UvrCanDcom.circ_pump_run) intval |= 0x0001;
            else intval & ~0x0001;
            if (UvrCanDcom.compressor_run) intval |= 0x0002;
            else intval & ~0x0002;      
            if (UvrCanDcom.booster_heat_run) intval |= 0x0004;
            else intval & ~0x0004;   
            if (UvrCanDcom.desinfection_op) intval |= 0x0008;
            else intval & ~0x0008;
            if (UvrCanDcom.defrost_startup) intval |= 0x0010;
            else intval & ~0x0010;
            if (UvrCanDcom.hot_start) intval |= 0x0020;
            else intval & ~0x0020;    
            if (UvrCanDcom.valve_3way) intval |= 0x0040;
            else intval & ~0x0040;
            if (UvrCanDcom.op_mode == 1) intval |= 0x0080;
            else intval & ~0x0080;
            if (UvrCanDcom.op_mode == 2) intval |= 0x0100;
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

    case 1: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_1);
    
            canMsg->data[0] = (uint8_t) (UvrCanDcom.unit_error & 0xFF);
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;

            break;

    case 2: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_2);
    
            intval = (int) (UvrCanDcom.leaving_water_PHE_temp * 10);
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (UvrCanDcom.leaving_water_BHU_temp * 10);
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (UvrCanDcom.return_water_temp * 10);
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (UvrCanDcom.dom_hot_water_temp * 10);
            canMsg->data[6] = (uint8_t) (intval & 0xFF);
            canMsg->data[7] = (uint8_t) (intval >> 8 & 0xFF);

            break;

    case 3: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_3);    
            intval = (int) (UvrCanDcom.outside_air_temp * 10);
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (UvrCanDcom.liquid_refrig_temp * 10);
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) UvrCanDcom.flow_rate;
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (UvrCanDcom.room_temp * 10);
            canMsg->data[6] = (uint8_t) (intval & 0xFF);
            canMsg->data[7] = (uint8_t) (intval >> 8 & 0xFF);
    
            break;

    case 4: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_4);

            #ifdef USE_SDM72_SDM230
              //intval = UvrCanSdm72Sdm230GetPower(1);
              intval = Sdm72Sdm230GetData(1);
              canMsg->data[0] = (uint8_t) (intval & 0xFF);
              canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

              //intval = UvrCanSdm72Sdm230GetPower(2);
              intval = Sdm72Sdm230GetData(2);
              canMsg->data[2] = (uint8_t) (intval & 0xFF);
              canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

              canMsg->data[4] = 0x00;
              canMsg->data[5] = 0x00;

              canMsg->data[6] = 0x00;
              canMsg->data[7] = 0x00;
            #else
              canMsg->data[0] = 0x00;
              canMsg->data[1] = 0x00;

              canMsg->data[2] = 0x00;
              canMsg->data[3] = 0x00;

              canMsg->data[4] = 0x00;
              canMsg->data[5] = 0x00;

              canMsg->data[6] = 0x00;
              canMsg->data[7] = 0x00;
            #endif  // USE_SDM72_SDM230
            break;    

    default: 
            break;

  }
}


void UVRCan_Dataset_2_Send (struct can_frame *canMsg, uint8_t message_nr) {
  int intval = 0;
  switch (message_nr) {
    case 0: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_DIGITAL);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 1: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_1);
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

    case 2: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_2);
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

    case 3: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_3);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 4: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_4);
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

// smart meter messages
// total power, pv total power, consumption total
void UVRCan_Dataset_3_Send (struct can_frame *canMsg, uint8_t message_nr) {
  int intval = 0;
  switch (message_nr) {
    case 0: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_DIGITAL);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 1: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_1);
            // Val1: total grid power [W]
            // Val2: total pv power [W]
            // Val3: total consumption power [W]
            // Val4: total battery power [W]
            intval = (int) (Sdm630MultiGetData(10));
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (Sdm630MultiGetData(11));
            canMsg->data[2] = (uint8_t) (intval & 0xFF);
            canMsg->data[3] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (Sdm630MultiGetData(12));
            canMsg->data[4] = (uint8_t) (intval & 0xFF);
            canMsg->data[5] = (uint8_t) (intval >> 8 & 0xFF);

            intval = (int) (Sdm630MultiGetData(13));
            canMsg->data[6] = (uint8_t) (intval & 0xFF);
            canMsg->data[7] = (uint8_t) (intval >> 8 & 0xFF);
            break;

    case 2: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_2);
            // Val1: Meter 2 - Power Phase 2 [W]
            // Val2: none
            // Val3: none
            // Val4: none
            intval = (int) (Sdm630MultiGetData(8));
            canMsg->data[0] = (uint8_t) (intval & 0xFF);
            canMsg->data[1] = (uint8_t) (intval >> 8 & 0xFF);

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 3: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_3);
            canMsg->data[0] = 0x00;
            canMsg->data[1] = 0x00;

            canMsg->data[2] = 0x00;
            canMsg->data[3] = 0x00;

            canMsg->data[4] = 0x00;
            canMsg->data[5] = 0x00;

            canMsg->data[6] = 0x00;
            canMsg->data[7] = 0x00;
            break;

    case 4: canMsg->can_id = ((uint32_t)UvrCanSettings.send_id | CAN_SEND_ID_ANALOG_4);
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

  AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Recv Dataset 1 - Recv Id %03X"), (uint16_t) canFrame[Mcp2515.framecnt-1].can_id);

  switch (message_id) {
    case CAN_RECV_ID_DIGITAL_1:
          // UVR 9/10     Mode Heat/Cool                      	int16	  Auto/Heat/Cool        M1, Bit 8 Heating, M1, Bit 9 Cooling
          // UVR 11       Space Heating/Cooling On/Off         	int16	  0:OFF 1:ON	          M1, Bit 10
          // UVR 12       Quiet Mode Operation	                int16	  0:OFF 1:ON	          M1, Bit 11
          // UVR 13       DHW Booster Mode On/Off               int16	  0:OFF 1:ON	          M1, Bit 12

          // Dies wird dann in die ersten 4 bytes gesteckt, die Reihenfolge ist so: (1. byte, 2. byte usw.)
          // 8 7 6 5 4 3 2 1 16 15 14 13 12 11 10 9 24 23 22 21 20 19 18 17 32 31 30 29 28 27 26 25
          // Die Zahlen steht für die jeweilge Ausgangsnummer.

          // data[0] - Digital Out  1...8
          // data[1] - Digital Out  8...16
          // data[2] - Digital Out 17...24
          // data[3] - Digital Out 25...32

          // Operation Mode - Auto/Heat/Cool - Heating has prio
          if (canMsg->data[1] & 0x01) intval = 1;
          else if (canMsg->data[1] & 0x02) intval = 2;
          else intval = 0;
          UvrCanDcom.target_opmode = (uint16_t) intval;
          //Serial.print("Operation Mode: "); Serial.println(intval, DEC);
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Operation Mode: %d"), intval);

          // Space Heating/Cooling On/Off
          if (canMsg->data[1] & 0x04) intval = 1;
          else intval = 0;
          UvrCanDcom.target_spaceheatcool = (uint16_t) intval;
          //Serial.print("Space Heating/Cooling: "); Serial.println(intval, DEC);
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Space Heating/Cooling: %d"), intval);

          // Quiet Mode Operation
          if (canMsg->data[1] & 0x08) intval = 1;
          else intval = 0;
          UvrCanDcom.target_quietmode = (uint16_t) intval;
          //Serial.print("Quiet Mode Operation: "); Serial.println(intval, DEC);
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Quiet Mode Operation: %d"), intval);

          // DHW Booster Mode On/Off
          if (canMsg->data[1] & 0x10) intval = 1;
          else intval = 0;
          UvrCanDcom.target_dhwbooster = (uint16_t) intval;
          //Serial.print("DHW Booster Mode On/Off: "); Serial.println(intval, DEC);
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: DHW Booster Mode On/Off: %d"), intval);

          break;

    case CAN_RECV_ID_ANALOG_4:    // CAN Analog Out 13 ... 16
          // Leaving Water Main Heating Setpoint    int16	  25 .. 55ºC	            M0, Byte0..1

          // CAN Analog Out 13
          // Leaving Water Main Heating Setpoint
          intval = ((unsigned int) canMsg->data[1] << 8) + (unsigned int) canMsg->data[0];
          if (intval > 550) intval = 550;
          else if (intval < 250) intval = 250;
          UvrCanDcom.target_leavingwaterheattemp = (uint16_t) intval;
          //Serial.print("Leaving Water Main Heating Setpoint 0.1°C: "); Serial.println(UvrCanDcom.target_leavingwaterheattemp, DEC);
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Leaving Water Main Heating Setpoint 0.1°C: %d"), UvrCanDcom.target_leavingwaterheattemp);
          
          break;

    default: break;
  }
}


void UVRCan_Dataset_3_Recv (struct can_frame *canMsg, uint32_t message_id) {
  uint16_t intval  = 0;
  int16_t  sintval = 0;

  AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Recv Dataset 3 - Recv Id %03X"), (uint16_t) canFrame[Mcp2515.framecnt-1].can_id);
  //AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 3"));
  //AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: message_id: %u"), (message_id));
  //AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: message_id entry: %u"), (message_id & ~0x1C0));

  switch (message_id) {
    case CAN_RECV_ID_DIGITAL_1:
          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 3 - Digital 1 - no data set"));
          // Dies wird dann in die ersten 4 bytes gesteckt, die Reihenfolge ist so: (1. byte, 2. byte usw.)
          // 8 7 6 5 4 3 2 1 16 15 14 13 12 11 10 9 24 23 22 21 20 19 18 17 32 31 30 29 28 27 26 25
          // Die Zahlen steht für die jeweilge Ausgangsnummer.

          // data[0] - Digital Out  1...8
          // data[1] - Digital Out  8...16
          // data[2] - Digital Out 17...24
          // data[3] - Digital Out 25...32
          break;

    case CAN_RECV_ID_ANALOG_1:    // CAN Analog Out 1 ... 4
          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 3 - Analog 1"));

          // Battery Power Mode        int16	  0 - Auto, 1 - Manual            M0, Byte0..1
          // Battery Power Setpoint    int16	  -10000 ... +10000 W	            M1, Byte2..3

          // CAN Analog Out 3
          // Battery Power Mode: 0 - Auto, 1 - Manual
          intval = ((unsigned int) canMsg->data[5] << 8) + (unsigned int) canMsg->data[4];
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Mode: %d"), intval);         
          if (intval == 0) {                        
            SolisMeterSetMode(false);
          }
          else {                        
            SolisMeterSetMode(true);
          }
      
          // CAN Analog Out 4
          // Battery Power Setpoint: -10000 ... +10000 W
          sintval = (int16_t) ((canMsg->data[7] << 8) | canMsg->data[6]);
          if (sintval > 10000) sintval = 10000;
          else if (sintval < -10000) sintval = -10000;
          SolisMeterSetPower((int) sintval);          
          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Setpoint  W: %d"), sintval);
          
          break;

    case CAN_RECV_ID_ANALOG_2:    // CAN Analog Out 5 ... 8
          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 3 - Analog 2 - no data set"));
          break;

    case CAN_RECV_ID_ANALOG_NEW:    // CAN Analog Out 1 ... x - neues Format
          AddLog(LOG_LEVEL_INFO, PSTR("UVRCAN: Recv Dataset 3 - Analog New"));
          // Battery Power Mode        int16	  0 - Auto, 1 - Manual            Output 5
          // Battery Power Setpoint    int16	  -10000 ... +10000 W	            Output 6
          //AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: 3 New Data: %u"), (canMsg->data[1]));

          switch ((unsigned int) canMsg->data[1]) {
            
            // CAN Analog Out 3
            // Battery Power Mode: 0 - Auto, 1 - Manual
            case 0x02:  intval = ((uint16_t) canMsg->data[5] << 8) + (uint16_t) canMsg->data[4];
                        AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: CAN 3 Out = %u"), (intval));
                        if (intval > 1) intval = 1;
                        else if (intval < 0) intval = 0; 

                        if (intval == 0) {
                          SolisMeterSetMode(false);
                          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Mode: %s"), "auto");
                        }
                        else {
                          SolisMeterSetMode(true);
                          AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Mode: %s"), "manual");
                        }
                        break;

            // CAN Analog Out 4
            // Battery Power Setpoint: -10000 ... +10000 W
            case 0x03:  sintval = (int16_t) ((canMsg->data[5] << 8) | canMsg->data[4]);
            //case 0x05:  sintval = (int) (((unsigned int) canMsg->data[5] << 8) + (unsigned int) canMsg->data[4]);
            //case 0x05:  sintval = (int) (((int) canMsg->data[5] << 8) + (int) canMsg->data[4]);
                        AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: %d"), (sintval));
                        if (sintval > 10000) sintval = 10000;
                        else if (sintval < -10000) sintval = -10000;

                        SolisMeterSetPower(sintval);          
                        AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Setpoint  W: %d"), sintval);
                        break;

          }

          break;

    default: break;
  }
}

#endif  // USE_UVRCAN
#endif  // USE_SPI
