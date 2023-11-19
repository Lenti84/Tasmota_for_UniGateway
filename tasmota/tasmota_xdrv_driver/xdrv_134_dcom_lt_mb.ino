/*
  xdrv_134_dcom_lt_mb.ino - Daikin DCOM LT MB driver

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

#ifdef USE_DCOM_LT_MB
/*********************************************************************************************\
 * Daikin DCOM LT MB driver
 *
\*********************************************************************************************/

#define XDRV_134             134


#define D_DCOM_NAME         "Daikin LT MB"
#define D_DCOM_RETURN_TEMP  "Auslass Temperatur"
#define D_DCOM_COMP_RUN     "Kompressor"

#define DCOM_LT_MB_SPEED            9600      // default DCOM LT MB baudrate
#define DCOM_LT_MB_ADDR             0x00      // default DCOM LT MB Modbus address

#define DCOM_REGISTER_UNSUPPORTED   32767     // Device does not support requested register
#define DCOM_REGISTER_UNAVAILABLE   32766     // Requested register is not available in current configuration
#define DCOM_REGISTER_WAITFORVALUE  32765     // Requested register value is not loaded


#include <TasmotaModbus.h>
#include "../../SoftwareSerial-8.0.3/src/SoftwareSerial.h"
EspSoftwareSerial::UART DcomSwSerial;


uint16_t MBCalculateCRC(uint8_t *frame, uint8_t num);

uint8_t  DCOMInit = 0;

// read all default registers
// DCOM LT MB
const uint16_t dcom_lt_mb_start_addresses[] {
  21, // 0 - Unit Error               int16   0:No Error, 1: Fault, 2: Warning
  22, // 1 - Unit Error Code          text16  2 Ascii Characters
  23, // 2 - Unit Error Sub Code      int16   If No error 32766, If Unit Error 0 .. 99
  30, // 3 - Circulation Pump Running int16   0:OFF 1:ON
  31, // 4 - Compressor Run           int16   0:OFF 1:ON
  32, // 5 - Booster Heater Run       int16   0:OFF 1:ON
  33, // 6 - Disinfection Operation   int16   0:OFF 1:ON
  35, // 7 - Defrost/Startup          int16   0:OFF 1:ON
  36, // 8 - Hot Start                int16   0:OFF 1:ON
  37, // 9 - 3-Way Valve              int16   0: Space Heating, 1: DHW
  38, // 10 - Operation Mode           int16   1: Heating, 2: Cooling
  40, // 11 - Leaving Water Temperature pre PHE    temp16  -1.00..100.00ºC
  41, // 12 - Leaving Water Temperature pre BUH    temp16  -100.00..100.00ºC
  42, // 13 - Return Water Temperature             temp16  -100.00..100.00ºC
  43, // 14 - Domestic Hot Water Temperature       temp16  -100.00..100.00ºC
  44, // 15 - Outside Air Temperature              temp16  -100.00..100.00ºC
  45, // 16 - Liquid Refrigerant Temperature       temp16  -100.00..100.00ºC
  49, // 17 - Flow Rate                            int16   litres/minute x 100 
  50  // 18 - Remocon room temperature             temp16  -100.00..100.00ºC
};

// DCOM LT IO - sequencer mode
const uint16_t dcom_lt_io_start_addresses[] {
  21, // 0  - Unit Error                            int16   0:No Error, 1: Fault, 2: Warning
  22, // 1  - Unit Error Code                       text16  2 Ascii Characters
  23, // 2  - Leaving Water Temperature pre BUH     temp16  -100.00..100.00ºC
  36, // 3  - Unit Error Sub Code                   int16   If No error 32766, If Unit Error 0 .. 99
  37, // 4  - 3-Way Valve                           int16   0: Space Heating, 1: DHW
  38, // 5  - Operation Mode                        int16   1: Heating, 2: Cooling
  40, // 6  - Leaving Water Temperature pre PHE     temp16  -1.00..100.00ºC
  45, // 7  - Liquid Refrigerant Temperature        temp16  -100.00..100.00ºC
  49, // 8  - Flow Rate                             int16   litres/minute x 100 
  50, // 9  - Remocon room temperature              temp16  -100.00..100.00ºC
  70, // 10 - Space Heating/Cooling On/Off          int16   0:OFF 1:ON
  71, // 11 - Circulation Pump Running              int16   0:OFF 1:ON
  72, // 12 - Compressor Run                        int16   0:OFF 1:ON
  74, // 13 - Disinfection Operation                int16   0:OFF 1:ON
  76, // 14 - Defrost/Startup                       int16   0:OFF 1:ON
  77, // 15 - DHW Reheat On/Off                     int16   0:OFF 1:ON
  78, // 16 - Booster Heater Run                    int16   0:OFF 1:ON
  122,// 17 - Unit Error Code                       text16  2 Ascii Characters
  123,// 18 - Leaving Water Temperature pre BUH     temp16  -100.00..100.00ºC
  136,// 19 - Unit Error Sub Code                   int16   If No error 32766, If Unit Error 0 .. 99
  131,// 20 - Return Water Temperature              temp16  -100.00..100.00ºC
  132,// 21 - Domestic Hot Water Temperature        temp16  -100.00..100.00ºC
  133 // 22 - Outside Air Temperature               temp16  -100.00..100.00ºC
};

struct DCOM_MB_LT {
  uint8_t   read_state = 0;
  uint8_t   send_retry = 0;

  uint16_t  unit_error = 0;
  char      unit_error_code[3] = "";
  uint16_t  unit_error_subcode = 0;
  uint16_t  circ_pump_run = 0;
  uint16_t  compressor_run = 0;
  uint16_t  booster_heat_run = 0;
  uint16_t  desinfection_op = 0;
  uint16_t  defrost_startup = 0;
  uint16_t  hot_start = 0;
  uint16_t  valve_3way = 0;
  uint16_t  op_mode = 0;
  float     leaving_water_PHE_temp = 0.0;
  float     leaving_water_BHU_temp = 0.0;
  float     return_water_temp = 0.0;
  float     dom_hot_water_temp = 0.0;
  float     outside_air_temp = 0.0;
  float     liquid_refrig_temp = 0.0;
  float     flow_rate = 0.0;
  float     room_temp = 0.0;
} DcomMbLt;


/*********************************************************************************************/

#ifdef USE_WEBSERVER
const char HTTP_DRV_DCOM_DATA[] PROGMEM =
  //"{s}%s " D_VOLTAGE "{m}%s " D_UNIT_VOLT "{e}"
  //"{s}%s " D_CURRENT "{m}%s " D_UNIT_AMPERE "{e}"
  "{s}%s " D_DCOM_RETURN_TEMP "{m}%s " D_UNIT_DEGREE D_UNIT_CELSIUS "{e}"
  "{s}%s " D_DCOM_COMP_RUN "{m}%s {e}";
#endif  // USE_WEBSERVER

void DCOMShow(bool json)
{
  if(DCOMInit) {
    
    char returnwater[8];
    dtostrfd(DcomMbLt.return_water_temp, 1, returnwater);

    char comprun[4];    
    if (DcomMbLt.compressor_run) snprintf_P(comprun, sizeof(comprun), PSTR("%s"), "on");
    else snprintf_P(comprun, sizeof(comprun), PSTR("%s"), "off");
    
    char name[16];    
    snprintf_P(name, sizeof(name), PSTR("%s"), D_DCOM_NAME);

    // if (json) {
    //   ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_USAGE "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s}"),
    //                    name, 1, heaterpercent, netpower);

#ifdef USE_WEBSERVER
    //} else {
      WSContentSend_PD(HTTP_DRV_DCOM_DATA, name, returnwater, name, comprun);

#endif  // USE_WEBSERVER
    //}
  }
}

/*********************************************************************************************/

void DCOMEvery100ms(void)
{
  uint16_t data_len = DcomSwSerial.available();
  uint16_t crc;

  if (DCOMInit) {

    if (data_len) {
      uint8_t buffer[144];  // At least 

      //uint32_t error = Sdm630Modbus->ReceiveBuffer(buffer, 2);
      //AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, Sdm630Modbus->ReceiveCount());

      uint32_t error = DcomSwSerial.read(buffer, data_len);

      if (error == 0) {
        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM: SDM630 error %d"), error);
      } else {
        Serial.println(F("DCOM: recv data"));
        bool found = false;

        Serial.print("DCOM: RX: ");
        for (uint16_t y = 0;y < data_len;y++) {
          Serial.print(buffer[y], HEX);
          Serial.print(", ");
        }
        Serial.println("");

        // modbus check
        if (buffer[0] == DCOM_LT_MB_ADDR && buffer[1] == 0x03) {
          //if (buffer[2] == 0 && buffer[3] == dcom_start_addresses[DcomMbLt.read_state]) {
          if (buffer[2] == 2) {
            crc = MBCalculateCRC(&buffer[0], data_len-2);
            // Serial.println(crc, HEX);
            // Serial.println(buffer[data_len-1], HEX);
            // Serial.println((uint8_t (crc >> 8)), HEX);
            // Serial.println(buffer[data_len-2], HEX);
            // Serial.println((uint8_t (0x00FF & crc)), HEX);
            if (buffer[data_len-1] == (uint8_t (crc >> 8)) && buffer[data_len-2] == (uint8_t (0x00FF & crc))) {            
              found = true;
              Serial.println("DCOM: crc ok");
            }
          }
        }

        if (found) {
          int tempint;
          
          // prepare int value and use only when needed
          tempint = (((int) (buffer[3]) << 8) && ((int) (buffer[4]) & 0x00FF));

          switch(DcomMbLt.read_state) {
            case 0:
              DcomMbLt.unit_error = (uint16_t) ((buffer[4]) & 0x00FF);
              break;

            case 1:
              Serial.print("DCOM: 1 ");
              DcomMbLt.unit_error_code[0] = (char) (buffer[3]);
              DcomMbLt.unit_error_code[1] = (char) (buffer[4]);
              DcomMbLt.unit_error_code[2] = '\0';
              Serial.println(DcomMbLt.unit_error_code);
              break;

            case 2:
              
              break;

            case 3:
              
              break;

            case 4:
              
              break;

            case 5:
              
              break;

            case 6:
              
              break;

            case 7:
              
              break;

            case 8:
              
              break;

            case 9:
              
              break;

            case 10:
              
              break;

            case 11:              
              Serial.print("DCOM: 11 ");
              //tempint = (((int) (buffer[3]) << 8) && ((int) (buffer[4]) & 0x00FF));
              DcomMbLt.leaving_water_PHE_temp = (float) tempint;

              Serial.println(DcomMbLt.leaving_water_PHE_temp, DEC);
              break;

            case 12:
              
              break;

            case 13:
              
              break;

            case 14:
              
              break;

            case 15:
              
              break;

            case 16:
              
              break;

            case 17:
              
              break;

            case 18:
              
              break;

            case 19:
              Serial.print("DCOM: 19 ");              
              //DcomMbLt.return_water_temp = (float) tempint;
              #warning just for test
              DcomMbLt.return_water_temp = 12.5;
              Serial.println(DcomMbLt.return_water_temp, DEC);
              break;

            case 20:
              
              break;

            case 21:
              Serial.print("DCOM: 21 ");              
              DcomMbLt.outside_air_temp = (float) tempint;
              Serial.println(DcomMbLt.outside_air_temp, DEC);
              break;

            default:

              break;
          }
        }


        DcomMbLt.read_state++;
        if (sizeof(dcom_lt_io_start_addresses)/2 == DcomMbLt.read_state) {
          DcomMbLt.read_state = 0;
        }
      }
    } // end data ready

    if (0 == DcomMbLt.send_retry || data_len) {
      DcomMbLt.send_retry = 5;
      //Sdm630Modbus->Send(SDM630_ADDR, 0x04, sdm630_start_addresses[Sdm630.read_state], 2);

      uint8_t sendbuf[10];

      // construct request
      sendbuf[0] = DCOM_LT_MB_ADDR;
      sendbuf[1] = 0x03;                // Input register
      sendbuf[2] = 0;                   // adress MSB
      sendbuf[3] = dcom_lt_io_start_addresses[DcomMbLt.read_state];    // adress LSB
      sendbuf[4] = 0;                   // register count MSB
      sendbuf[5] = 1;                   // register count LSB
      
      crc = MBCalculateCRC(&sendbuf[0], 6);     // calculate CRC
      
      sendbuf[6] = (uint8_t) (0x00FF & crc);    // CRC LSB
      sendbuf[7] = (uint8_t) (crc >> 8);        // CRC MSB

      Serial.print("DCOM TX: ");
      for (uint16_t y = 0;y < 8;y++) {
        Serial.print(sendbuf[y], HEX);
        Serial.print(", ");
      }
      Serial.println("");
      
      DcomSwSerial.write(&sendbuf[0],8);      

    } else {
      DcomMbLt.send_retry--;
    }

  }
}

void DCOMSnsInit(void)
{
  if (PinUsed(GPIO_DCOM_MB_LT_TX) && PinUsed(GPIO_DCOM_MB_LT_RX) && PinUsed(GPIO_DCOM_MB_LT_ENA)) {
    Serial.println(F("DCOM MB LT SnsInit"));  

    DcomSwSerial.begin(DCOM_LT_MB_SPEED, SWSERIAL_8N1, Pin(GPIO_DCOM_MB_LT_RX), Pin(GPIO_DCOM_MB_LT_TX));
    DcomSwSerial.setTransmitEnablePin(Pin(GPIO_DCOM_MB_LT_ENA));
  
    uint8_t result  = 2;

    if (result) {
      if (2 == result) { 
        AddLog(LOG_LEVEL_INFO, PSTR("DCOM MB LT listen: init okay%d"), result);
        Serial.print(F("DCOM MB LT: init okay - "));   Serial.println(result, DEC);
        DCOMInit = 1;
      }
      else {
        Serial.print(F("DCOM MB LT: init not okay - "));   Serial.println(result, DEC);
      }
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("DCOM MB LT: error init %d"), result);  
      Serial.println(F("DCOM MB LT: init error"));  
    }
  }
}

void DCOMDrvInit(void)
{  
  if (PinUsed(GPIO_DCOM_MB_LT_TX) && PinUsed(GPIO_DCOM_MB_LT_RX) && PinUsed(GPIO_DCOM_MB_LT_ENA)) {
    Serial.println(F("DCOM MB LT DrvInit"));  
  
    //Serial.println(F("TasmotaGlobal.energy_driver = XNRG_25"));  
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv134(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_100_MSECOND:
      DCOMEvery100ms();
      break;
    case FUNC_INIT:
      DCOMSnsInit();
      break;
    case FUNC_PRE_INIT:
      DCOMDrvInit();
      break;
  #ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      DCOMShow(0);
      break;   
  #endif  // USE_WEBSERVER
  }
  return result;
}



uint16_t MBCalculateCRC(uint8_t *frame, uint8_t num)
{
  uint16_t crc = 0xFFFF;

  for (uint8_t i = 0; i < num; i++) {
    crc ^= frame[i];
    for (uint8_t j = 8; j; j--) {
      if ((crc & 0x0001) != 0) {        // If the LSB is set
        crc >>= 1;                      // Shift right and XOR 0xA001
        crc ^= 0xA001;
      } else {                          // Else LSB is not set
        crc >>= 1;                      // Just shift right
      }
    }
  }
  return crc;
}



#endif  // USE_DCOM_LT_MB
