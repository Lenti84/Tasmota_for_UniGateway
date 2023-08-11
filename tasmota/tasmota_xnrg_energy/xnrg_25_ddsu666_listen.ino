/*
  xnrg_25_ddsu666_listen.ino - Chint DDSU666-Modbus energy meter in listen mode support for Tasmota

  Copyright (C) 2021  Pablo Zerón, Theo Arends, Lenti84

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as psblished by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifdef USE_ENERGY_SENSOR
#ifdef USE_DDSU666_LISTEN
/*********************************************************************************************\
 * Chint DDSU666 Modbus energy meter - listen mode - when a MODBUS master is already existing
 * this module is tested with HUAWEI SUN2000 10KTL-M1
\*********************************************************************************************/

#include <RingBuf.h>
RingBuf<uint8_t, 1000> MyRingBuffer;

#define XNRG_25             25

// can be user defined in my_user_config.h
#ifndef DDSU666_LISTEN_SPEED
  #define DDSU666_LISTEN_SPEED      9600    // default DDSU66 Modbus address
#endif
// can be user defined in my_user_config.h
#ifndef DDSU666_LISTEN_ADDR
  #define DDSU666_LISTEN_ADDR       11      // default DDSU66 Modbus address
#endif

#include "../../SoftwareSerial-8.0.3/src/SoftwareSerial.h"
EspSoftwareSerial::UART SwSerial;

const uint16_t Ddsu666_listen_start_addresses[] {
  0x2000,   // DDSU666_VOLTAGE             [V]
  0x2002,   // DDSU666_CURRENT             [A]
  0x2004,   // DDSU666_POWER               [KW]
  0x2006,   // DDSU666_REACTIVE_POWER      [KVAR]
  0x200A,   // DDSU666_POWER_FACTOR
  0x200E,   // DDSU666_FREQUENCY           [Hz]
  0X4000,   // DDSU666_IMPORT_ACTIVE       [kWh]
  0X400A,   // DDSU666_EXPORT_ACTIVE       [kWh]
};


// const uint8_t Ddsu666_energy_match[] {
//   0x0B,     // MODBUS address
//   0x03,     // MODBUS function code
//   0x20,     // MODBUS register offset MSB
//   0x12,     // MODBUS register offset LSB
//   0x00,     // MODBUS register count MSB
//   0x02,     // MODBUS register count LSB
// };

const uint8_t Ddsu666_energy_match[] {
  0x0B,     // MODBUS address
  0x03,     // MODBUS function code
  0x08,     // MODBUS register offset MSB
  0x36,     // MODBUS register offset LSB
  0x00,     // MODBUS register count MSB
  0x50,     // MODBUS register count LSB
};

struct DDSU666_Listen {
  uint8_t read_state = 0;
  uint8_t send_retry = 0;
} Ddsu666_Listen;

/*********************************************************************************************/


uint16_t DDSU666CalculateCRC(uint8_t *frame, uint8_t num)
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




void DDSU666ListenEvery250ms(void)
{
  static uint8_t resetcnt = 0;

  uint8_t voidbuf;
  uint16_t bufsize;
  uint8_t buffer;
  uint32_t res;

  uint8_t found = 0;
  uint8_t databuf[4];
  
  // fetch data from Serial Interface if available and put in a ringbuffer 
  do {
    //res = Ddsu666ListenModbus->GetByte(&buffer);
    res = SwSerial.available();
    buffer = (uint8_t)SwSerial.read();

    if (res) {
      MyRingBuffer.push(buffer);
    }
  } while (res);

  // check for wanted strings
  bufsize = MyRingBuffer.size();
  if (bufsize >= 6) {   // check if at least a request is in the buffer
    //Serial.println("Buffer > 6");

    for (uint16_t x = 0;x<bufsize;x++) {
      if(MyRingBuffer[x] == Ddsu666_energy_match[0]) {
        if(MyRingBuffer[x+1] == Ddsu666_energy_match[1]) {
          //Serial.println("Match 1");

          if(MyRingBuffer[x+2] == Ddsu666_energy_match[2]) {
            if(MyRingBuffer[x+3] == Ddsu666_energy_match[3]) {
              if(MyRingBuffer[x+4] == Ddsu666_energy_match[4]) {
                if(MyRingBuffer[x+5] == Ddsu666_energy_match[5]) {
                  uint16_t mycrc16;
                  //Serial.println("Match 2");
                  mycrc16 = DDSU666CalculateCRC(&MyRingBuffer[x],6);
                  //Serial.print(mycrc16,HEX); Serial.println("");
                  
                  // checksum correct?
                  if (mycrc16 == ((uint16_t) (MyRingBuffer[x+7] << 8) | ((uint16_t) (MyRingBuffer[x+6]) & 0x00FF))) {
                    Serial.println("crc req correct");
                    
                    if (bufsize >= 200) {   // check if answer is available already
                      // check answer from DDSU
                      if(MyRingBuffer[x+8] == Ddsu666_energy_match[0]) {        // address same?
                        if(MyRingBuffer[x+9] == Ddsu666_energy_match[1]) {      // function code same?
                          Serial.println("some answer there");
                          
                          uint8_t datalen;
                          datalen = MyRingBuffer[x+10];
                          Serial.print("datalen"); Serial.print(datalen,DEC); Serial.println("");

                          Serial.print("Buffersize:"); Serial.print(bufsize,DEC); Serial.println("");

                          if (bufsize >= datalen+3+2) {   // check if complete answer is available
                            mycrc16 = DDSU666CalculateCRC(&MyRingBuffer[x+8],datalen+3);
                            Serial.print(mycrc16,HEX); Serial.println("");
                            
                            Serial.print("CRC data: "); Serial.print(MyRingBuffer[x+8+datalen+4],HEX); Serial.print(MyRingBuffer[x+8+datalen+3],HEX); Serial.println("");


                            Serial.print("first 4 byte: "); 
                            Serial.print(MyRingBuffer[x+8+3],HEX);
                            Serial.print(MyRingBuffer[x+8+4],HEX);
                            Serial.print(MyRingBuffer[+x+8+5],HEX);
                            Serial.println(MyRingBuffer[x+8+6],HEX);

                            // checksum answer correct?
                            if (mycrc16 == ((uint16_t) (MyRingBuffer[x+8+datalen+4] << 8) | ((uint16_t) (MyRingBuffer[x+8+datalen+3]) & 0x00FF))) {
                              Serial.println("crc answer correct");
                              uint8_t dataoffset = 4 * 12;    // 12th place is combined power 
                              databuf[0] = MyRingBuffer[dataoffset+x+8+3];
                              databuf[1] = MyRingBuffer[dataoffset+x+8+4];
                              databuf[2] = MyRingBuffer[dataoffset+x+8+5];
                              databuf[3] = MyRingBuffer[dataoffset+x+8+6];

                              found = 1;

                              // float value2;
                              // ((uint8_t*)&value2)[3] = databuf[0];   // Get float values
                              // ((uint8_t*)&value2)[2] = databuf[1];
                              // ((uint8_t*)&value2)[1] = databuf[2];
                              // ((uint8_t*)&value2)[0] = databuf[3];                        

                              //Serial.print(value2,DEC); Serial.println("");

                              //Serial.print("Buffersize:"); Serial.print(bufsize,DEC); Serial.println("");

                              // delete all evaluated data from ringbuffer
                              for (uint16_t y = 0;y < x+8+datalen+4;y++) {
                                MyRingBuffer.pop(voidbuf);
                              }
                            }
                            else Serial.println("crc answer incorrect");
                          }

                          //  for (uint8_t y = 0;y < datalen+3+2;y++) {
                          //       MyRingBuffer.pop(voidbuf);
                          //     }


                        }
                      }
                    }
                    else {
                      // clear junk in buffer up to answer
                      // for (uint8_t y = 0;y < x;y++) {
                      //   MyRingBuffer.pop(voidbuf);
                      // } 
                      // break;
                    }
                  }
                }
              }           
            }
          }


        }

      }

    }


  }

  if(!found) {
    resetcnt++;
  }

  if (resetcnt > 20) {
    resetcnt = 0;
    MyRingBuffer.clear();
    Serial.println("reset ringbuffer");
  }

  if(found) {
      Serial.println("found data packet");

      Energy->data_valid[0] = 0;

      float value;
      ((uint8_t*)&value)[3] = databuf[0];   // Get float values
      ((uint8_t*)&value)[2] = databuf[1];
      ((uint8_t*)&value)[1] = databuf[2];
      ((uint8_t*)&value)[0] = databuf[3];

      //Serial.print(value,DEC); Serial.println("");

      Energy->active_power[0] = value;     // -196.3 W
      
      //EnergyUpdateTotal();

      // switch(Ddsu666_Listen.read_state) {
      //   case 0:
      //     Energy->voltage[0] = value;          // 230.2 V
      //     break;

      //   case 1:
      //     Energy->current[0]  = value;         // 1.260 A
      //     break;

      //   case 2:
      //     Energy->active_power[0] = value * 1000;     // -196.3 W
      //     break;

      //   case 3:
      //     Energy->reactive_power[0] = value * 1000;   // 92.2
      //     break;

      //   case 4:
      //     Energy->power_factor[0] = value;     // 0.91
      //     break;

      //   case 5:
      //     Energy->frequency[0] = value;        // 50.0 Hz
      //     break;

      //   case 6:
      //     Energy->import_active[0] = value;    // 478.492 kWh
      //     break;

      //   case 7:
      //     Energy->export_active[0] = value;    // 6.216 kWh
      //     break;
      // }

      // Ddsu666_Listen.read_state++;

      // if (Ddsu666_Listen.read_state == 8) {
      //   Ddsu666_Listen.read_state = 0;
      //   //EnergyUpdateTotal();  // 484.708 kWh
      // }
    
  } // end found

}

void Ddsu666ListenSnsInit(void)
{
  if (PinUsed(GPIO_DDSU666_LISTEN_RX)) {
    Serial.println(F("Ddsu666ListenSnsInit"));  
    //Ddsu666ListenModbus = new TasmotaModbusSwUART(Pin(GPIO_DDSU666_LISTEN_RX), Pin(GPIO_DDSU666_LISTEN_TX), Pin(GPIO_NRG_MBS_TX_ENA));
    //uint8_t result = Ddsu666ListenModbus->Begin(DDSU666_LISTEN_SPEED);

    // init software serial in rx only mode
    //SwSerial.begin(DDSU666_LISTEN_SPEED, SWSERIAL_8N1, 4, -1, false);
    SwSerial.begin(DDSU666_LISTEN_SPEED, SWSERIAL_8N1, Pin(GPIO_DDSU666_LISTEN_RX));
  uint8_t result  = 2;

    if (result) {
      if (2 == result) { 
        //ClaimSerial(); 
        AddLog(LOG_LEVEL_INFO, PSTR("DDSU listen: init okay%d"), result);
        Serial.print(F("DDSU listen: init okay - "));   Serial.println(result, DEC);
        //Ddsu666ListenModbus->Send(0x01, 0x04, 0, 8);

        Energy->phase_count = 1;
      }
      else {
        Serial.print(F("DDSU listen: init not okay - "));   Serial.println(result, DEC);
      }
    } else {
      TasmotaGlobal.energy_driver = ENERGY_NONE;
      AddLog(LOG_LEVEL_INFO, PSTR("DDSU listen: error init %d"), result);  
      Serial.println(F("DDSU listen: init error"));  
    }
  }
}

void Ddsu666ListenDrvInit(void)
{
  Serial.println(F("Ddsu666ListenDrvInit"));  
  if (PinUsed(GPIO_DDSU666_LISTEN_RX)) {
    TasmotaGlobal.energy_driver = XNRG_25;
    Energy->voltage_available = false;
    Energy->current_available = false;
    Energy->energy_available = false;
    Serial.println(F("TasmotaGlobal.energy_driver = XNRG_25"));  
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg25(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_250_MSECOND:
      DDSU666ListenEvery250ms();
      break;
    case FUNC_INIT:
      Ddsu666ListenSnsInit();
      break;
    case FUNC_PRE_INIT:
      Ddsu666ListenDrvInit();
      break;
  }
  return result;
}

#endif  // USE_DDSU666_LISTEN
#endif  // USE_ENERGY_SENSOR
