/*
  xdrv_132_deye_meter.ino - Deye Smart Meter Bridge

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

#ifdef USE_DEYE_METER
/*********************************************************************************************\
 * Deye Inverter Smart Meter Bridge
 * 
 * simulates a Smart Meter for Deye Solar Inverters
 *
\*********************************************************************************************/

#include <RingBuf.h>
RingBuf<uint8_t, 1000> DeyeRingBuffer;

#define DEYE_METER_DEBUG        // comment to disable debug messages over uart

#define XDRV_132                132


#define DEYE_METER_NAME         "DEYE inverter"
#define DEYE_METER_STATUS       "connection"

#define DEYE_METER_SPEED        9600      // default baudrate


#include "../../SoftwareSerial-8.0.3/src/SoftwareSerial.h"
EspSoftwareSerial::UART DeyeMeterSwSerial;
// in order to use RX2/TX2 esp32.json must be changed
// "extra_flags": "-DARDUINO_ESP32_DEV -DBOARD_HAS_PSRAM -DESP32_4M",
// to "extra_flags": "-DARDUINO_ESP32_DEV -DESP32_4M",
// if board has no PSRAM and therefore GPIO16 and GPIO17 can be used for UART


uint16_t DeyeMBCalculateCRC(uint8_t *frame, uint8_t num);

struct DEYE_METER {
  uint8_t   init = 0;
  uint8_t   conn_ready = 0;

  uint8_t   read_state = 0;
  uint8_t   send_state = 0;
  uint8_t   send_retry = 0;
  uint8_t   send_active = 0;    // semaphore for send / recv control - send sequence in progress
  uint8_t   recv_active = 1;    // semaphore for send / recv control - receive sequence in progress
  uint8_t   writebuffer[4];     // write buffer to check if writing was successful
} DeyeMeter;


const uint8_t Deye_modbus_match[] {
  0x01,     // MODBUS address
  0x03,     // MODBUS function code  
};

const uint8_t Deye_power_match[] {
  0x20,     // MODBUS register offset MSB
  0x14,     // MODBUS register offset LSB
  0x00,     // MODBUS register count MSB
  0x06,     // MODBUS register count LSB
};

const uint8_t Deye_energy_match[] {
  0x10,     // MODBUS register offset MSB
  0x1E,     // MODBUS register offset LSB
  0x00,     // MODBUS register count MSB
  0x0C,     // MODBUS register count LSB
};

/*********************************************************************************************/

#ifdef USE_WEBSERVER
const char HTTP_DRV_DEYE_METER_DATA[] PROGMEM =
  "{s}%s " DEYE_METER_STATUS "{m}%s {e}";
#endif  // USE_WEBSERVER

void DeyeMeterShow(bool json)
{
  if(DeyeMeter.init) {
    
    char status[8];    
    if (DeyeMeter.conn_ready) snprintf_P(status, sizeof(status), PSTR("%s"), "okay");
    else snprintf_P(status, sizeof(status), PSTR("%s"), "failed");
    
    char name[16];    
    snprintf_P(name, sizeof(name), PSTR("%s"), DEYE_METER_NAME);

    // if (json) {
    //   ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_USAGE "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s}"),
    //                    name, 1, heaterpercent, netpower);

#ifdef USE_WEBSERVER
    //} else {
      WSContentSend_PD(HTTP_DRV_DEYE_METER_DATA, name, status);

#endif  // USE_WEBSERVER
    //}
  }
}

/*********************************************************************************************/

void DeyeMeterEvery100ms(void)
{
  static uint8_t resetcnt = 0;

  uint8_t voidbuf;
  uint16_t bufsize;
  uint8_t buffer;
  uint32_t res;

  uint8_t found = 0;
  uint8_t databuf[4];
  uint16_t mycrc16;
  float value;

  if(DeyeMeter.init) {

    if (DeyeMeter.conn_ready) DeyeMeter.conn_ready--;
    
    // fetch data from Serial Interface if available and put in a ringbuffer 
    do {
      //res = DeyeMeterSwSerial->GetByte(&buffer);
      res = DeyeMeterSwSerial.available();
      buffer = (uint8_t)DeyeMeterSwSerial.read();

      if (res) {
        DeyeRingBuffer.push(buffer);
        //Serial.println("DEYE METER: some bytes");
      }
    } while (res);

    // check for wanted strings
    bufsize = DeyeRingBuffer.size();
    if (bufsize >= 8) {   // check if at least a request is in the buffer
      //Serial.println("DEYE METER: Buffer > 6");

      for (uint16_t x = 0;x<bufsize;x++) {
        // check modbus frame
        if(DeyeRingBuffer[x] == Deye_modbus_match[0]) {
          if(DeyeRingBuffer[x+1] == Deye_modbus_match[1]) {
            //Serial.println("DEYE METER: Match 1");
            
              // check specific adresses and length
              if(DeyeRingBuffer[x+2] == Deye_power_match[0] && DeyeRingBuffer[x+3] == Deye_power_match[1]) {      // adress 
                if(x > 0 && bufsize == 8) {
                  Serial.println("DEYE METER: break 1");
                  DeyeRingBuffer.pop(voidbuf);  // remove one byte
                  break;    // try new evaluation with more data
                }
                else if(DeyeRingBuffer[x+4] == Deye_power_match[2]) {    // length MSB
                  if(DeyeRingBuffer[x+5] == Deye_power_match[3]) {  // length LSB
                    found = 1;    // power request found
                  }
                }
              }

              else if(DeyeRingBuffer[x+2] == Deye_energy_match[0] && DeyeRingBuffer[x+3] == Deye_energy_match[1]) {     // adress 
                if(x > 0 && bufsize == 8) {
                  Serial.println("DEYE METER: break 2");
                  DeyeRingBuffer.pop(voidbuf);  // remove one byte
                  break;    // try new evaluation with more data
                }
                else if(DeyeRingBuffer[x+4] == Deye_energy_match[2]) {   // length MSB
                  if(DeyeRingBuffer[x+5] == Deye_energy_match[3]) { // length LSB
                    found = 2;    // energy request found
                  }
                }
              }

              if (found) {              
                //Serial.print("DEYE METER: Match 2 - "); Serial.println(found, DEC);
                mycrc16 = DeyeMBCalculateCRC(&DeyeRingBuffer[x],6);
                //Serial.print(mycrc16,HEX); Serial.println("");
                
                // checksum correct?
                if (mycrc16 == ((uint16_t) (DeyeRingBuffer[x+7] << 8) | ((uint16_t) (DeyeRingBuffer[x+6]) & 0x00FF))) {
                  //Serial.println("DEYE METER: crc req correct");

                  // // delete all evaluated data from ringbuffer
                  // for (uint16_t y = 0;y < 8;y++) {
                  //   DeyeRingBuffer.pop(voidbuf);
                  // }

                  //Serial.print("DEYE METER: found "); Serial.println(found, DEC);
                  //Serial.print("DEYE METER: bufsize "); Serial.println(bufsize, DEC);
                }
                else {
                  found = 0;
                  Serial.println("DEYE METER: crc req incorrect");
                  Serial.print("DEYE METER: bufsize "); Serial.println(bufsize, DEC);
                  #ifdef DEYE_METER_DEBUG
                    Serial.print("DEYE METER: RX: ");
                    for (uint16_t y = 0;y < bufsize;y++) {
                      Serial.print(DeyeRingBuffer[y], HEX);
                      Serial.print(", ");
                    }
                    Serial.println("");
                  #endif
                }

                // delete all evaluated data from ringbuffer
                for (uint16_t y = 0;y < x+7;y++) {
                  DeyeRingBuffer.pop(voidbuf);
                }
              }
              else {
                Serial.print("DEYE METER: Match 2 fail");
              }
            
          }
        }
      }
    }

    if(!found) {
      resetcnt++;
    }

    if (resetcnt > 50) {
      resetcnt = 0;
      DeyeRingBuffer.clear();
      Serial.println("DEYE METER: reset ringbuffer");
    }

    if(found) {
      // test
      //Energy->active_power[0] = 144.55;

      resetcnt = 0;
      uint8_t sendbuf[40];
      uint8_t sendlen = 0;

      DeyeMeter.conn_ready = 100;

      Serial.println("DEYE METER: found data packet");
            
      // construct message
      sendbuf[0] = Deye_modbus_match[0];
      sendbuf[1] = Deye_modbus_match[1];    // write single holding register
          
      // power request
      if(found == 1) {
        sendlen = Deye_power_match[3] * 2;   // length in bytes - 12
        sendbuf[2] = sendlen;
        // phase A
        value = Energy->active_power[0] * 10;
        sendbuf[3]  = ((uint8_t*)&value)[3];
        sendbuf[4]  = ((uint8_t*)&value)[2];
        sendbuf[5]  = ((uint8_t*)&value)[1];
        sendbuf[6]  = ((uint8_t*)&value)[0];
        // phase B
        value = Energy->active_power[1] * 10;
        sendbuf[7]  = ((uint8_t*)&value)[3];
        sendbuf[8]  = ((uint8_t*)&value)[2];
        sendbuf[9]  = ((uint8_t*)&value)[1];
        sendbuf[10] = ((uint8_t*)&value)[0];
        // phase C
        value = Energy->active_power[2] * 10;
        sendbuf[11] = ((uint8_t*)&value)[3];
        sendbuf[12] = ((uint8_t*)&value)[2];
        sendbuf[13] = ((uint8_t*)&value)[1];
        sendbuf[14] = ((uint8_t*)&value)[0];
      }

      // energy request
      else if(found == 2) {
        sendlen = Deye_energy_match[3] * 2;   // length in bytes - 24
        sendbuf[2] = sendlen;
        // import energy
        sendbuf[3]  = ((uint8_t*)&Energy->import_active[0])[3];
        sendbuf[4]  = ((uint8_t*)&Energy->import_active[0])[2];
        sendbuf[5]  = ((uint8_t*)&Energy->import_active[0])[1];
        sendbuf[6]  = ((uint8_t*)&Energy->import_active[0])[0];
        // ImpEpA (current) A Forward active energy (kWh)
        sendbuf[7]  = 0;
        sendbuf[8]  = 0;
        sendbuf[9]  = 0;
        sendbuf[10] = 0;
        // ImpEpB (current) B Forward active energy (kWh)
        sendbuf[11] = 0;
        sendbuf[12] = 0;
        sendbuf[13] = 0;
        sendbuf[14] = 0;
        // ImpEpC (current) C Forward active energy (kWh)
        sendbuf[15] = 0;
        sendbuf[16] = 0;
        sendbuf[17] = 0;
        sendbuf[18] = 0;
        // NetImpEp (current) Net Forward active energy (kWh)
        sendbuf[19] = 0;
        sendbuf[20] = 0;
        sendbuf[21] = 0;
        sendbuf[22] = 0;
        // export energy
        sendbuf[23] = ((uint8_t*)&Energy->export_active[0])[3];
        sendbuf[24] = ((uint8_t*)&Energy->export_active[0])[2];
        sendbuf[25] = ((uint8_t*)&Energy->export_active[0])[1];
        sendbuf[26] = ((uint8_t*)&Energy->export_active[0])[0];
      }


      // end modbus message
      mycrc16 = DeyeMBCalculateCRC(&sendbuf[0], sendlen + 3);     // calculate CRC
          
      sendbuf[sendlen + 3 + 0] = (uint8_t) (0x00FF & mycrc16);    // CRC LSB
      sendbuf[sendlen + 3 + 1] = (uint8_t) (mycrc16 >> 8);        // CRC MSB

      DeyeMeterSwSerial.write(&sendbuf[0],sendlen + 5);


      #ifdef DEYE_METER_DEBUG
        Serial.print("DEYE METER: TX: ");
        for (uint16_t y = 0;y < sendlen+5;y++) {
          Serial.print(sendbuf[y], HEX);
          Serial.print(", ");
        }
        Serial.println("");
      #endif
    }

  }
}


void DeyeMeterSnsInit(void)
{
  if (PinUsed(GPIO_DEYE_METER_TX) && PinUsed(GPIO_DEYE_METER_RX) && PinUsed(GPIO_DEYE_METER_ENA)) {
    Serial.println(F("DEYE METER: SnsInit"));  

    DeyeMeterSwSerial.begin(DEYE_METER_SPEED, SWSERIAL_8N1, Pin(GPIO_DEYE_METER_RX), Pin(GPIO_DEYE_METER_TX));
    DeyeMeterSwSerial.setTransmitEnablePin(Pin(GPIO_DEYE_METER_ENA));
  
    uint8_t result  = 2;

    if (result) {
      if (2 == result) { 
        AddLog(LOG_LEVEL_INFO, PSTR("DEYE METER: init okay%d"), result);
        Serial.print(F("DEYE METER: init okay - "));   Serial.println(result, DEC);
        DeyeMeter.init = 1;
      }
      else {
        Serial.print(F("DEYE METER: init not okay - "));   Serial.println(result, DEC);
      }
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("DEYE METER: error init %d"), result);  
      Serial.println(F("DEYE METER: init error"));  
    }
  }
}

void DeyeMeterDrvInit(void)
{  
  if (PinUsed(GPIO_DEYE_METER_TX) && PinUsed(GPIO_DEYE_METER_RX) && PinUsed(GPIO_DEYE_METER_ENA)) {
    Serial.println(F("DEYE METER: DrvInit"));  
  
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv132(uint32_t function)
{
  bool result = false;

  switch (function) {
    //case FUNC_EVERY_100_MSECOND:
    case FUNC_LOOP:
      DeyeMeterEvery100ms();
      break;
    case FUNC_EVERY_250_MSECOND:
      break;
    case FUNC_INIT:
      DeyeMeterSnsInit();
      break;
    case FUNC_PRE_INIT:
      DeyeMeterDrvInit();
      break;
  #ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      DeyeMeterShow(0);
      break;   
  #endif  // USE_WEBSERVER
  }
  return result;
}



uint16_t DeyeMBCalculateCRC(uint8_t *frame, uint8_t num)
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



#endif  // USE_DEYE_METER
