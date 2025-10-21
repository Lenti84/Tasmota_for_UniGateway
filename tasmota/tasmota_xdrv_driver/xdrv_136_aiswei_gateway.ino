/*
  xdrv_136_aiswei_gateway.ino - 

  Copyright (C) 2024

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

#ifdef USE_AISWEI_GATEWAY
/*********************************************************************************************\
 *
 * 
 * 
 *
\*********************************************************************************************/

#include <RingBuf.h>
RingBuf<uint8_t, 1000> AisweiRingBuffer;

//#define AISWEI_GATEWAY_DEBUG        // comment to disable debug messages over uart

#define XDRV_136                136


#define AISWEI_GATEWAY_NAME         "DEYE inverter"
#define AISWEI_GATEWAY_STATUS       "connection"

#define AISWEI_GATEWAY_SPEED        9600      // default baudrate


#include "../../SoftwareSerial-8.0.3/src/SoftwareSerial.h"
EspSoftwareSerial::UART AisweiGatewaySwSerial;
// in order to use RX2/TX2 esp32.json must be changed
// "extra_flags": "-DARDUINO_ESP32_DEV -DBOARD_HAS_PSRAM -DESP32_4M",
// to "extra_flags": "-DARDUINO_ESP32_DEV -DESP32_4M",
// if board has no PSRAM and therefore GPIO16 and GPIO17 can be used for UART


HardwareSerial Serial485(2);
// IO21 - RX
// IO22 - TX
// IO19 - SHUTDOWN
// IO17 - ENABLE


uint16_t AisweiMBCalculateCRC(uint8_t *frame, uint8_t num);

struct AISWEI_METER {
  uint8_t   init = 0;
  uint8_t   conn_ready = 0;

  uint8_t   read_state = 0;
  uint8_t   send_state = 0;
  uint8_t   send_retry = 0;
  uint8_t   send_active = 0;    // semaphore for send / recv control - send sequence in progress
  uint8_t   recv_active = 1;    // semaphore for send / recv control - receive sequence in progress
  uint8_t   writebuffer[4];     // write buffer to check if writing was successful
} AisweiGateway;


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
const char HTTP_DRV_AISWEI_GATEWAY_DATA[] PROGMEM =
  "{s}%s " AISWEI_GATEWAY_STATUS "{m}%s {e}";
#endif  // USE_WEBSERVER

void AisweiGatewayShow(bool json)
{
  if(AisweiGateway.init) {
    
    char status[8];    
    if (AisweiGateway.conn_ready) snprintf_P(status, sizeof(status), PSTR("%s"), "okay");
    else snprintf_P(status, sizeof(status), PSTR("%s"), "failed");
    
    char name[16];    
    snprintf_P(name, sizeof(name), PSTR("%s"), AISWEI_GATEWAY_NAME);

    // if (json) {
    //   ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_USAGE "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s}"),
    //                    name, 1, heaterpercent, netpower);

#ifdef USE_WEBSERVER
    //} else {
      WSContentSend_PD(HTTP_DRV_AISWEI_GATEWAY_DATA, name, status);

#endif  // USE_WEBSERVER
    //}
  }
}

/*********************************************************************************************/

void AisweiGatewayEvery100ms(void)
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

  if(AisweiGateway.init) {

    if (AisweiGateway.conn_ready) AisweiGateway.conn_ready--;
    
    // fetch data from Serial Interface if available and put in a ringbuffer 
    do {
      //res = AisweiGatewaySwSerial->GetByte(&buffer);
      res = AisweiGatewaySwSerial.available();
      buffer = (uint8_t)AisweiGatewaySwSerial.read();

      if (res) {
        AisweiRingBuffer.push(buffer);
        //Serial.println("AISWEI GW: some bytes");
      }
    } while (res);

    // check for wanted strings
    bufsize = AisweiRingBuffer.size();
    if (bufsize >= 8) {   // check if at least a request is in the buffer
      //Serial.println("AISWEI GW: Buffer > 6");

      for (uint16_t x = 0;x<bufsize;x++) {
        // check modbus frame
        if(AisweiRingBuffer[x] == Deye_modbus_match[0]) {
          if(AisweiRingBuffer[x+1] == Deye_modbus_match[1]) {
            //Serial.println("AISWEI GW: Match 1");
            
              // check specific adresses and length
              if(AisweiRingBuffer[x+2] == Deye_power_match[0] && AisweiRingBuffer[x+3] == Deye_power_match[1]) {      // adress 
                if(x > 0 && bufsize == 8) {
                  //Serial.println("AISWEI GW: break 1");
                  AisweiRingBuffer.pop(voidbuf);  // remove one byte
                  break;    // try new evaluation with more data
                }
                else if(AisweiRingBuffer[x+4] == Deye_power_match[2]) {    // length MSB
                  if(AisweiRingBuffer[x+5] == Deye_power_match[3]) {  // length LSB
                    found = 1;    // power request found
                  }
                }
              }

              else if(AisweiRingBuffer[x+2] == Deye_energy_match[0] && AisweiRingBuffer[x+3] == Deye_energy_match[1]) {     // adress 
                if(x > 0 && bufsize == 8) {
                  //Serial.println("AISWEI GW: break 2");
                  AisweiRingBuffer.pop(voidbuf);  // remove one byte
                  break;    // try new evaluation with more data
                }
                else if(AisweiRingBuffer[x+4] == Deye_energy_match[2]) {   // length MSB
                  if(AisweiRingBuffer[x+5] == Deye_energy_match[3]) { // length LSB
                    found = 2;    // energy request found
                  }
                }
              }

              if (found) {              
                //Serial.print("AISWEI GW: Match 2 - "); Serial.println(found, DEC);
                mycrc16 = DeyeMBCalculateCRC(&AisweiRingBuffer[x],6);
                //Serial.print(mycrc16,HEX); Serial.println("");
                
                // checksum correct?
                if (mycrc16 == ((uint16_t) (AisweiRingBuffer[x+7] << 8) | ((uint16_t) (AisweiRingBuffer[x+6]) & 0x00FF))) {
                  //Serial.println("AISWEI GW: crc req correct");

                  // // delete all evaluated data from ringbuffer
                  // for (uint16_t y = 0;y < 8;y++) {
                  //   AisweiRingBuffer.pop(voidbuf);
                  // }

                  //Serial.print("AISWEI GW: found "); Serial.println(found, DEC);
                  //Serial.print("AISWEI GW: bufsize "); Serial.println(bufsize, DEC);
                }
                else {
                  found = 0;
                  //Serial.println("AISWEI GW: crc req incorrect");
                  //Serial.print("AISWEI GW: bufsize "); Serial.println(bufsize, DEC);
                  #ifdef AISWEI_GATEWAY_DEBUG
                    Serial.print("AISWEI GW: RX: ");
                    for (uint16_t y = 0;y < bufsize;y++) {
                      Serial.print(AisweiRingBuffer[y], HEX);
                      Serial.print(", ");
                    }
                    Serial.println("");
                  #endif
                }

                // delete all evaluated data from ringbuffer
                for (uint16_t y = 0;y < x+7;y++) {
                  AisweiRingBuffer.pop(voidbuf);
                }
              }
              else {
                //Serial.print("AISWEI GW: Match 2 fail");
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
      AisweiRingBuffer.clear();
      //Serial.println("AISWEI GW: reset ringbuffer");
    }

    if(found) {
      // test
      //Energy->active_power[0] = 144.55;

      resetcnt = 0;
      uint8_t sendbuf[40];
      uint8_t sendlen = 0;

      AisweiGateway.conn_ready = 100;

      //Serial.println("AISWEI GW: found data packet");
            
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

      AisweiGatewaySwSerial.write(&sendbuf[0],sendlen + 5);


      #ifdef AISWEI_GATEWAY_DEBUG
        Serial.print("AISWEI GW: TX: ");
        for (uint16_t y = 0;y < sendlen+5;y++) {
          Serial.print(sendbuf[y], HEX);
          Serial.print(", ");
        }
        Serial.println("");
      #endif
    }

  }
}


void AisweiGatewaySnsInit(void)
{
  if (PinUsed(GPIO_AISWEI_GATEWAY_TX) && PinUsed(GPIO_AISWEI_GATEWAY_RX) && PinUsed(GPIO_AISWEI_GATEWAY_ENA)) {
    //Serial.println(F("AISWEI GW: SnsInit"));  

    Serial485.begin(9600, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);


    AisweiGatewaySwSerial.begin(AISWEI_GATEWAY_SPEED, SWSERIAL_8N1, Pin(GPIO_AISWEI_GATEWAY_RX), Pin(GPIO_AISWEI_GATEWAY_TX));
    AisweiGatewaySwSerial.setTransmitEnablePin(Pin(GPIO_AISWEI_GATEWAY_ENA));
  
    uint8_t result  = 2;

    if (result) {
      if (2 == result) { 
        AddLog(LOG_LEVEL_INFO, PSTR("AISWEI GW: init okay %d"), result);
        //Serial.print(F("AISWEI GW: init okay - "));   Serial.println(result, DEC);
        AisweiGateway.init = 1;
      }
      else {
        AddLog(LOG_LEVEL_INFO, PSTR("AISWEI GW: init not okay %d"), result);
        //Serial.print(F("AISWEI GW: init not okay - "));   Serial.println(result, DEC);
      }
    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("AISWEI GW: error init %d"), result);  
      //Serial.println(F("AISWEI GW: init error"));  
    }
  }
}

void AisweiGatewayDrvInit(void)
{  
  if (PinUsed(GPIO_AISWEI_GATEWAY_TX) && PinUsed(GPIO_AISWEI_GATEWAY_RX) && PinUsed(GPIO_AISWEI_GATEWAY_ENA)) {
    //Serial.println(F("AISWEI GW: DrvInit"));  
  
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv136(uint32_t function)
{
  bool result = false;

  switch (function) {
    //case FUNC_EVERY_100_MSECOND:
    case FUNC_LOOP:
      AisweiGatewayEvery100ms();
      break;
    case FUNC_EVERY_250_MSECOND:
      break;
    case FUNC_INIT:
      AisweiGatewaySnsInit();
      break;
    case FUNC_PRE_INIT:
      AisweiGatewayDrvInit();
      break;
  #ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      AisweiGatewayShow(0);
      break;   
  #endif  // USE_WEBSERVER
  }
  return result;
}



uint16_t AisweiMBCalculateCRC(uint8_t *frame, uint8_t num)
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



#endif  // USE_AISWEI_GATEWAY
