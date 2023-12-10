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

#define DCOM_LT_DEBUG       // comment to disable debug messages over uart

#define XDRV_134             134


#define D_DCOM_NAME         "Daikin LT MB"
#define D_DCOM_RETURN_TEMP  "Auslass Temperatur"
#define D_DCOM_COMP_RUN     "Kompressor"

#define DCOM_LT_MB_SPEED            9600      // default DCOM LT MB baudrate
#define DCOM_LT_MB_ADDR             0x00      // default DCOM LT MB Modbus address

#define DCOM_REGISTER_UNSUPPORTED   32767     // Device does not support requested register
#define DCOM_REGISTER_UNAVAILABLE   32766     // Requested register is not available in current configuration
#define DCOM_REGISTER_WAITFORVALUE  32765     // Requested register value is not loaded

#define DCOM_MODBUS_INPUT_REG       0x04      // input registers
#define DCOM_MODBUS_HOLD_REG        0x03      // holding registers
#define DCOM_MODBUS_HOLD_WRITE_REG  0x06      // single holding register write


#include <TasmotaModbus.h>
#include "../../SoftwareSerial-8.0.3/src/SoftwareSerial.h"
EspSoftwareSerial::UART DcomSwSerial;


uint16_t MBCalculateCRC(uint8_t *frame, uint8_t num);

uint8_t  DCOMInit = 0;

// DCOM LT MB input registers
const uint16_t dcom_lt_mb_start_addresses[] {
  21, // 0 - Unit Error                            int16   0:No Error, 1: Fault, 2: Warning
  22, // 1 - Unit Error Code                       text16  2 Ascii Characters
  23, // 2 - Unit Error Sub Code                   int16   If No error 32766, If Unit Error 0 .. 99
  30, // 3 - Circulation Pump Running              int16   0:OFF 1:ON
  31, // 4 - Compressor Run                        int16   0:OFF 1:ON
  32, // 5 - Booster Heater Run                    int16   0:OFF 1:ON
  33, // 6 - Disinfection Operation                int16   0:OFF 1:ON
  35, // 7 - Defrost/Startup                       int16   0:OFF 1:ON
  36, // 8 - Hot Start                             int16   0:OFF 1:ON
  37, // 9 - 3-Way Valve                           int16   0: Space Heating, 1: DHW
  38, // 10 - Operation Mode                       int16   1: Heating, 2: Cooling
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
// input registers
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

// target registers
// DCOM LT MB holding registers
const uint16_t dcom_lt_mb_target_start_addresses[] { 
  1,  // Leaving Water Main Heating Setpoint        int16	  25 .. 55ºC
  2,  // Leaving Water Main Cooling Setpoint      	int16	  5 .. 22ºC
  3,  // Operation Mode	                            int16	  0: Auto, 1:Heating, 2:Cooling
  4,  // Space Heating/Cooling On/Off             	int16	  0:OFF 1:ON
  6,  // Room Thermostat Control Heating Setpoint  	int16	  12 .. 30ºC
  7,  // Room Thermostat Control Cooling Setpoint 	int16	  15 .. 35ºC
  9,  // Quiet Mode Operation	                      int16	  0:OFF 1:ON
  10, // DHW Reheat Setpoint	                      int16	  30 .. 60ºC
  12, // DHW Reheat On/Off	                        int16	  0:OFF 1:ON
  13,	// DHW Booster Mode On/Off              	    int16	  0:OFF 1:ON
  53, // Weather Dependent Mode                     int16	  0:Fixed, 1: Weather Dependent, 2: Fixed+Scheduled, 3: Weather Dependent+Scheduled
  54, // Weather Dependent Mode LWT Heating Setpoint Offset 	int16	  -10 .. 10ºC
  55  // Weather Dependent Mode LWT Cooling Setpoint Offset 	int16	  -10 .. 10ºC
};

struct DCOM_MB_LT {
  // driver
  uint8_t   read_state = 0;
  uint8_t   send_state = 0;
  uint8_t   send_retry = 0;
  uint8_t   send_active = 0;    // semaphore for send / recv control - send sequence in progress
  uint8_t   recv_active = 1;    // semaphore for send / recv control - receive sequence in progress
  uint8_t   writebuffer[4];     // write buffer to check if writing was successful

  // input registers - measurement values
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

  // holding registers - inputs
  uint16_t  target_leavingwaterheattemp = 25;     // Leaving Water Main Heating Setpoint
  uint16_t  target_leavingwatercooltemp = 22;     // Leaving Water Main Cooling Setpoint
  uint16_t  target_opmode = 0;                    // Operation Mode
  uint16_t  target_spaceheatcool = 0;             // Space Heating/Cooling On/Off
  uint16_t  target_roomheatsetp = 12;             // Room Thermostat Control Heating Setpoint
  uint16_t  target_roomcoolsetp = 15;             // Room Thermostat Control Cooling Setpoint
  uint16_t  target_quietmode = 0;                 // Quiet Mode Operation
  uint16_t  target_reheatsetp = 30;               // DHW Reheat Setpoint
  uint16_t  target_dhwreheat = 0;                 // DHW Reheat On/Off
  uint16_t  target_dhwbooster = 0;                // DHW Booster Mode On/Off
  uint16_t  target_weathermode = 0;               // Weather Dependent Mode
  uint16_t  target_weatherheatoffset = 0;         // Weather Dependent Mode LWT Heating Setpoint Offset
  uint16_t  target_weathercooloffset = 0;         // Weather Dependent Mode LWT Cooling Setpoint Offset

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
  uint8_t sendbuf[12];
  bool found = false;

  if (DCOMInit) {

    if (data_len) {
      uint8_t buffer[20];  // At least 

      //AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, Sdm630Modbus->ReceiveCount());

      uint32_t error = DcomSwSerial.read(&buffer[0], data_len);

      if (error == 0) {
        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM: SDM630 error %d"), error);
      } else {
        Serial.println(F("DCOM: recv data"));

        Serial.print("DCOM: RX: ");
        for (uint16_t y = 0;y < data_len;y++) {
          Serial.print(buffer[y], HEX);
          Serial.print(", ");
        }
        Serial.println("");

        // modbus check
        if (buffer[0] == DCOM_LT_MB_ADDR && (buffer[1] == DCOM_MODBUS_INPUT_REG || buffer[1] == DCOM_MODBUS_HOLD_WRITE_REG)) {
          crc = MBCalculateCRC(&buffer[0], data_len-2);
          // Serial.println(crc, HEX);
          // Serial.println(buffer[data_len-1], HEX);
          // Serial.println((uint8_t (crc >> 8)), HEX);
          // Serial.println(buffer[data_len-2], HEX);
          // Serial.println((uint8_t (0x00FF & crc)), HEX);

          //if (buffer[2] == 0 && buffer[3] == dcom_start_addresses[DcomMbLt.read_state]) {
          if (DcomMbLt.recv_active && buffer[2] == 2) {
            
            if (buffer[data_len-1] == (uint8_t (crc >> 8)) && buffer[data_len-2] == (uint8_t (0x00FF & crc))) {            
              found = true;
              Serial.println("DCOM: crc ok");
            }
          }
          else if ( DcomMbLt.send_active && \
                    buffer[2] == DcomMbLt.writebuffer[0] && \
                    buffer[3] == DcomMbLt.writebuffer[1] && \
                    buffer[4] == DcomMbLt.writebuffer[2] && \
                    buffer[5] == DcomMbLt.writebuffer[3] ) 
          {
            crc = MBCalculateCRC(&buffer[0], data_len-2);
            if (buffer[data_len-1] == (uint8_t (crc >> 8)) && buffer[data_len-2] == (uint8_t (0x00FF & crc))) {            
              found = true;
              Serial.println("DCOM: crc ok");
            }
          }
        }
        else Serial.println("DCOM: package does not match");

        if (found) {
          DcomMbLt.send_retry = 0; 
          int tempint;

          // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++          
          if(DcomMbLt.send_active) {
            //DcomMbLt.send_active = 0;
            Serial.println("Data packet with write holding");

            // switch(DcomMbLt.send_state) {
            //   case 0:                
            //     break;
            // }
            DcomMbLt.send_state++;

            if(DcomMbLt.send_state >= 4) {
              Serial.println("DCOM: Switch to measurement values");
              DcomMbLt.send_state = 0;
              DcomMbLt.send_active = 0;
              DcomMbLt.recv_active = 1;
            }

          }

          // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
          else if(DcomMbLt.recv_active) {
            //DcomMbLt.recv_active = 0;
            Serial.println("Data packet with recv input");
          
            // prepare int value and use only when needed
            tempint = (((int) (buffer[3]) << 8) && ((int) (buffer[4]) & 0x00FF));

            switch(DcomMbLt.read_state) {
              case 0:
                Serial.print("DCOM: 0 ");
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

            DcomMbLt.read_state++;
            if (sizeof(dcom_lt_io_start_addresses)/2 == DcomMbLt.read_state) {
              Serial.println("DCOM: Switch to target values");
              DcomMbLt.read_state = 0;
              DcomMbLt.recv_active = 0;
              DcomMbLt.send_active = 1;
            }

          }
        }



      }
    } // end data ready


    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    if (DcomMbLt.send_active) {
      
      if(0 == DcomMbLt.send_retry || found) {
        
        DcomMbLt.send_retry = 5;
        
          // construct message
        sendbuf[0] = DCOM_LT_MB_ADDR;
        sendbuf[1] = DCOM_MODBUS_HOLD_WRITE_REG;   // write single holding register
        sendbuf[2] = 0;                            // adress MSB
                
        // fetch data
        switch(DcomMbLt.send_state) {
          case 0:   // Leaving Water Main Heating Setpoint
                    sendbuf[3] = 1;                                         // adress LSB
                    sendbuf[4] = 0x00;                                      // data Hi
                    sendbuf[5] = (uint8_t) DcomMbLt.target_leavingwaterheattemp;     // data Lo
                    break;

          case 1:   // Operation mode
                    sendbuf[3] = 3;                                         // adress LSB
                    sendbuf[4] = 0x00;                                      // data Hi
                    sendbuf[5] = (uint8_t) DcomMbLt.target_opmode;                   // data Lo
                    break;

          case 2:   // Quiet Mode Operation
                    sendbuf[3] = 9;                                         // adress LSB
                    sendbuf[4] = 0x00;                                      // data Hi
                    sendbuf[5] = (uint8_t) DcomMbLt.target_quietmode;                // data Lo
                    break;

          case 3:   // DHW Booster Mode On/Off
                    sendbuf[3] = 13;                                        // adress LSB
                    sendbuf[4] = 0x00;                                      // data Hi
                    sendbuf[5] = (uint8_t) DcomMbLt.target_dhwbooster;               // data Lo
                    break;
          
          case 4:   //DcomMbLt.send_state = 0; 
                    break;

          default:  //DcomMbLt.send_state = 0; 
                    break;
        }

        // end modbus message
        crc = MBCalculateCRC(&sendbuf[0], 6);     // calculate CRC
            
        sendbuf[6] = (uint8_t) (0x00FF & crc);    // CRC LSB
        sendbuf[7] = (uint8_t) (crc >> 8);        // CRC MSB

        DcomSwSerial.write(&sendbuf[0],8);
        DcomMbLt.writebuffer[0] = sendbuf[2];
        DcomMbLt.writebuffer[1] = sendbuf[3];
        DcomMbLt.writebuffer[2] = sendbuf[4];
        DcomMbLt.writebuffer[3] = sendbuf[5];

        #ifdef DCOM_LT_DEBUG
          Serial.print("DCOM TX hold: ");
          for (uint16_t y = 0;y < 8;y++) {
            Serial.print(sendbuf[y], HEX);
            Serial.print(", ");
          }
          Serial.println("");
        #endif

      } else {
        DcomMbLt.send_retry--;
      }
    }
    
    // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    else if (DcomMbLt.recv_active) {
      Serial.print("send_retry "); Serial.println(DcomMbLt.send_retry, DEC);
      Serial.print("read state "); Serial.println(DcomMbLt.read_state, DEC);
      if(0 == DcomMbLt.send_retry || found) {
        //DcomMbLt.recv_active = 1;

        DcomMbLt.send_retry = 5;        

        //uint8_t sendbuf[10];

        // construct request
        sendbuf[0] = DCOM_LT_MB_ADDR;
        sendbuf[1] = DCOM_MODBUS_INPUT_REG; // input registers
        sendbuf[2] = 0;                     // adress MSB
        sendbuf[3] = dcom_lt_io_start_addresses[DcomMbLt.read_state];    // adress LSB
        sendbuf[4] = 0;                     // register count MSB
        sendbuf[5] = 1;                     // register count LSB
        
        crc = MBCalculateCRC(&sendbuf[0], 6);     // calculate CRC
        
        sendbuf[6] = (uint8_t) (0x00FF & crc);    // CRC LSB
        sendbuf[7] = (uint8_t) (crc >> 8);        // CRC MSB

        DcomSwSerial.write(&sendbuf[0],8);

        #ifdef DCOM_LT_DEBUG
          Serial.print("DCOM TX input: ");
          for (uint16_t y = 0;y < 8;y++) {
            Serial.print(sendbuf[y], HEX);
            Serial.print(", ");
          }
          Serial.println("");
        #endif      
        
      } else {
        DcomMbLt.send_retry--;
      }
    }

  }

  // test
  DcomMbLt.leaving_water_PHE_temp = 1.2;
  DcomMbLt.leaving_water_BHU_temp = 2.3;
  DcomMbLt.return_water_temp = 3.4;
  DcomMbLt.dom_hot_water_temp = 4.5;

}

// send routine
void DCOMEvery250ms(void)
{
  uint8_t sendbuf[10];
  uint16_t crc;

  if (DCOMInit) {
    if (DcomMbLt.send_active) {
      //DcomMbLt.send_active = 1;

      // construct message
      sendbuf[0] = DCOM_LT_MB_ADDR;
      sendbuf[1] = DCOM_MODBUS_HOLD_WRITE_REG;   // write single holding register
      sendbuf[2] = 0;                            // adress MSB
      
        
      // fetch data
      switch(DcomMbLt.send_state) {
        case 0:   // Leaving Water Main Heating Setpoint
                  sendbuf[3] = 1;                                         // adress LSB
                  sendbuf[4] = 0x00;                                      // data Hi
                  sendbuf[5] = (uint8_t) DcomMbLt.target_leavingwaterheattemp;     // data Lo
                  break;

        case 1:   // Operation mode
                  sendbuf[3] = 3;                                         // adress LSB
                  sendbuf[4] = 0x00;                                      // data Hi
                  sendbuf[5] = (uint8_t) DcomMbLt.target_opmode;                   // data Lo
                  break;

        case 2:   // Quiet Mode Operation
                  sendbuf[3] = 9;                                         // adress LSB
                  sendbuf[4] = 0x00;                                      // data Hi
                  sendbuf[5] = (uint8_t) DcomMbLt.target_quietmode;                // data Lo
                  break;

        case 3:   // DHW Booster Mode On/Off
                  sendbuf[3] = 13;                                        // adress LSB
                  sendbuf[4] = 0x00;                                      // data Hi
                  sendbuf[5] = (uint8_t) DcomMbLt.target_dhwbooster;               // data Lo
                  break;
        
        case 4:   //DcomMbLt.send_state = 0; 
                  break;

        default:  //DcomMbLt.send_state = 0; 
                  break;
      }

      //DcomMbLt.send_state++;

      // end modbus message
      crc = MBCalculateCRC(&sendbuf[0], 6);     // calculate CRC
          
      sendbuf[6] = (uint8_t) (0x00FF & crc);    // CRC LSB
      sendbuf[7] = (uint8_t) (crc >> 8);        // CRC MSB

    #ifdef DCOM_LT_DEBUG
      Serial.print("DCOM TX Set: ");
      for (uint16_t y = 0;y < 8;y++) {
        Serial.print(sendbuf[y], HEX);
        Serial.print(", ");
      }
      Serial.println("");
    #endif

      DcomSwSerial.write(&sendbuf[0],8);   
      


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
      //DCOMEvery100ms();
      break;
    case FUNC_EVERY_250_MSECOND:
      //DCOMEvery250ms();
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
