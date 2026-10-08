/*
  xdrv_137_solis_meter.ino - Solis Smart Meter Bridge

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

#ifdef USE_SOLIS_METER
/*********************************************************************************************\
 * Solis Inverter Smart Meter Bridge
 * 
 * simulates a Smart Meter for Solis Solar Inverters
 * SDM630MCD
 *
\*********************************************************************************************/

#include <TasmotaSerial.h>

#ifndef USE_SDM630_MULTI
#error "USE_SOLIS_METER requires USE_SDM630_MULTI"
#endif

float Sdm630MultiGetData(uint8_t index);
uint8_t SolisMeterRequest[8];
uint8_t SolisMeterRequestLength = 0;
uint32_t SolisMeterLastByte = 0;
uint32_t SolisMeterLastRequest = 0;

//#define SOLIS_METER_DEBUG        // comment to disable debug messages over uart

#define XDRV_137                137


#define SOLIS_METER_NAME         "SOLIS inverter"
#define SOLIS_METER_STATUS       "connection"

#ifndef SOLIS_METER_SPEED
#define SOLIS_METER_SPEED        9600      // default baudrate
#endif


TasmotaSerial *SolisMeterSwSerial = nullptr;

uint16_t SolisMBCalculateCRC(uint8_t *frame, uint8_t num);

struct SOLIS_METER {
  uint8_t   init = 0;
  uint8_t   conn_ready = 0;

  uint8_t   read_state = 0;
  uint8_t   send_state = 0;
  uint8_t   send_retry = 0;
  uint8_t   send_active = 0;    // semaphore for send / recv control - send sequence in progress
  uint8_t   recv_active = 1;    // semaphore for send / recv control - receive sequence in progress
  uint8_t   writebuffer[4];     // write buffer to check if writing was successful
} SolisMeter;

bool    SolisMeterPowerMode     = false;
int16_t SolisMeterPowerSetpoint = 0;

const uint8_t Solis_modbus_match[] {
  0x01,     // MODBUS address
  0x04,     // MODBUS function code  
};

const uint8_t Solis_power_match[] {
  0x00,     // MODBUS register offset MSB
  0x00,     // MODBUS register offset LSB
  0x00,     // MODBUS register count MSB
  0x4C,     // MODBUS register count LSB
};

const uint8_t Solis_energy_match[] {
  0x01,     // MODBUS register offset MSB
  0x56,     // MODBUS register offset LSB
  0x00,     // MODBUS register count MSB
  0x02,     // MODBUS register count LSB
};

/*********************************************************************************************/

#ifdef USE_WEBSERVER
const char HTTP_DRV_SOLIS_METER_DATA[] PROGMEM =
  "{s}%s " SOLIS_METER_STATUS "{m}%s {e}";
#endif  // USE_WEBSERVER


void SolisMeterSetMode(bool enable_manual)
{
  SolisMeterPowerMode = enable_manual;
}

void SolisMeterSetPower(int16_t manual_powerset_point)
{
  //AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Setpoint 2 W: %d"), manual_powerset_point);
  if (manual_powerset_point > 10000) SolisMeterPowerSetpoint = 10000;
  else if (manual_powerset_point < -10000) SolisMeterPowerSetpoint = -10000;
  else SolisMeterPowerSetpoint = manual_powerset_point;
  //AddLog(LOG_LEVEL_DEBUG, PSTR("UVRCAN: Battery Power Setpoint 3 W: %d"), SolisMeterPowerSetpoint);
}

void SolisMeterShow(bool json)
{
  if(SolisMeter.init) {
    
    char status[8];    
    if (SolisMeter.conn_ready) snprintf_P(status, sizeof(status), PSTR("%s"), "okay");
    else snprintf_P(status, sizeof(status), PSTR("%s"), "failed");
    
    char name[16];    
    snprintf_P(name, sizeof(name), PSTR("%s"), SOLIS_METER_NAME);

    // if (json) {
    //   ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_USAGE "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s}"),
    //                    name, 1, heaterpercent, netpower);

    //WSContentSend_PD(HTTP_DRV_SOLIS_METER_DATA, name, status);

#ifdef USE_WEBSERVER
    //} else {
      WSContentSend_P(PSTR("{s}SOLIS inverter{m}{e}"));      
      WSContentSend_PD("{s}connection{m}%s{e}", status);      
      
      if (!SolisMeterPowerMode) WSContentSend_P("{s}Set Mode{m}auto{e}");
      else WSContentSend_P("{s}Set Mode{m}manual{e}");

      WSContentSend_PD("{s}Set Power {m}%d{e}", SolisMeterPowerSetpoint);


#endif  // USE_WEBSERVER
    //}
  }
}

/*********************************************************************************************/

void SolisMeterEvery100ms(void)
{
  uint8_t found = 0;
  uint16_t mycrc16;
  float value;

  if (SolisMeter.init) {
    SolisMeter.conn_ready = (millis() - SolisMeterLastRequest < 10000 && SolisMeterLastRequest != 0);
    if (SolisMeterRequestLength && millis() - SolisMeterLastByte > 100) {
      SolisMeterRequestLength = 0;
    }
    while (SolisMeterSwSerial->available() && !found) {
      SolisMeterRequest[SolisMeterRequestLength++] = SolisMeterSwSerial->read();
      SolisMeterLastByte = millis();
      if (SolisMeterRequestLength < sizeof(SolisMeterRequest)) { continue; }
      mycrc16 = SolisMBCalculateCRC(SolisMeterRequest, 6);
      if (SolisMeterRequest[0] == 1 && SolisMeterRequest[1] == 4 &&
          mycrc16 == (SolisMeterRequest[6] | (uint16_t(SolisMeterRequest[7]) << 8))) {
        if (!memcmp(SolisMeterRequest + 2, Solis_power_match, 4)) { found = 1; }
        else if (!memcmp(SolisMeterRequest + 2, Solis_energy_match, 4)) { found = 2; }
        SolisMeterRequestLength = 0;
      } else {
        memmove(SolisMeterRequest, SolisMeterRequest + 1, 7);
        SolisMeterRequestLength = 7;
      }
    }

    if(found) {
      // test
      //Energy->active_power[0] = 144.55;

      SolisMeterLastRequest = millis();
      uint8_t sendbuf[160];   // very long answer
      uint8_t sendlen = 0;

      SolisMeter.conn_ready = 100;

      //Serial.println("SOLIS METER: found data packet");
            
      // construct message
      sendbuf[0] = Solis_modbus_match[0];
      sendbuf[1] = Solis_modbus_match[1];    // write single holding register
          
      // power request
      if(found == 1) {
        AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("SOLIS: Power request"));
        sendlen = Solis_power_match[3] * 2;   // length in bytes - 152
        sendbuf[2] = sendlen;

        // clear all buffer bytes
        for(uint16_t x = 3;x<159;x++) {
          sendbuf[x] = 0x00;
        }

        // PhaseA voltage
        value = 230.0;
        sendbuf[0x0000*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x0000*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x0000*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x0000*2+2+4] = ((uint8_t*)&value)[0];

        // PhaseB voltage
        value = 230.0;
        sendbuf[0x0002*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x0002*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x0002*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x0002*2+2+4] = ((uint8_t*)&value)[0];

        // PhaseC voltage
        value = 230.0;
        sendbuf[0x0004*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x0004*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x0004*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x0004*2+2+4] = ((uint8_t*)&value)[0];

        // PhaseA power - phase 1 power
        //value = Energy->active_power[0] * 10;
        value = -1 * Sdm630MultiGetData(1);
        sendbuf[0x000C*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x000C*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x000C*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x000C*2+2+4] = ((uint8_t*)&value)[0];
        // PhaseB power - phase 2 power
        //value = Energy->active_power[1] * 10;
        value = -1 * Sdm630MultiGetData(2);
        sendbuf[0x000E*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x000E*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x000E*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x000E*2+2+4] = ((uint8_t*)&value)[0];
        // PhaseC power - phase 3 power
        //value = Energy->active_power[2] * 10;
        value = -1 * Sdm630MultiGetData(3);
        sendbuf[0x0010*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x0010*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x0010*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x0010*2+2+4] = ((uint8_t*)&value)[0];


        // total system power
        if (SolisMeterPowerMode) {
          value = -1 * Sdm630MultiGetData(10) + SolisMeterPowerSetpoint;
        }
        else {
          value = -1 * Sdm630MultiGetData(10);
        }
        
        sendbuf[0x0034*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x0034*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x0034*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x0034*2+2+4] = ((uint8_t*)&value)[0];


        // input energy - Total Import kWh
        //value = Energy->active_power[2] * 10;
        value = 4444.0;
        sendbuf[0x0048*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x0048*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x0048*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x0048*2+2+4] = ((uint8_t*)&value)[0];
        // output energy - Total Export kWh
        //value = Energy->active_power[2] * 10;
        value = 5555.0;
        sendbuf[0x004A*2+2+1] = ((uint8_t*)&value)[3];
        sendbuf[0x004A*2+2+2] = ((uint8_t*)&value)[2];
        sendbuf[0x004A*2+2+3] = ((uint8_t*)&value)[1];
        sendbuf[0x004A*2+2+4] = ((uint8_t*)&value)[0];
      }

      // energy request
      else if(found == 2) {
        AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("SOLIS: Energy request"));
        sendlen = Solis_energy_match[3] * 2;   // length in bytes - 4
        sendbuf[2] = sendlen;
        float testval = 1234.0;
        // Total kwh
        // sendbuf[3]  = ((uint8_t*)&Energy->import_active[0])[3];
        // sendbuf[4]  = ((uint8_t*)&Energy->import_active[0])[2];
        // sendbuf[5]  = ((uint8_t*)&Energy->import_active[0])[1];
        // sendbuf[6]  = ((uint8_t*)&Energy->import_active[0])[0];
        sendbuf[3]  = ((uint8_t*)&testval)[3];
        sendbuf[4]  = ((uint8_t*)&testval)[2];
        sendbuf[5]  = ((uint8_t*)&testval)[1];
        sendbuf[6]  = ((uint8_t*)&testval)[0];         
      }


      // end modbus message
      mycrc16 = SolisMBCalculateCRC(&sendbuf[0], sendlen + 3);     // calculate CRC
          
      sendbuf[sendlen + 3 + 0] = (uint8_t) (0x00FF & mycrc16);    // CRC LSB
      sendbuf[sendlen + 3 + 1] = (uint8_t) (mycrc16 >> 8);        // CRC MSB

      digitalWrite(Pin(GPIO_SOLIS_METER_ENA), HIGH);
      SolisMeterSwSerial->write(&sendbuf[0],sendlen + 5);
      SolisMeterSwSerial->flush();
      digitalWrite(Pin(GPIO_SOLIS_METER_ENA), LOW);


      #ifdef SOLIS_METER_DEBUG
        Serial.print("SOLIS METER: TX: ");
        for (uint16_t y = 0;y < sendlen+5;y++) {
          Serial.print(sendbuf[y], HEX);
          Serial.print(", ");
        }
        Serial.println("");
      #endif
    }

  }
}


void SolisMeterSnsInit(void)
{
  if (PinUsed(GPIO_SOLIS_METER_TX) && PinUsed(GPIO_SOLIS_METER_RX) && PinUsed(GPIO_SOLIS_METER_ENA)) {
    if (SolisMeterSwSerial) { return; }
    SolisMeterSwSerial = new TasmotaSerial(Pin(GPIO_SOLIS_METER_RX), Pin(GPIO_SOLIS_METER_TX), 1);
    // Drive DE for the whole RTU frame, rather than toggling it per byte.
    if (SolisMeterSwSerial->begin(SOLIS_METER_SPEED, SERIAL_8N1)) {
      if (SolisMeterSwSerial->hardwareSerial()) { ClaimSerial(); }
      SolisMeter.init = 1;
      AddLog(LOG_LEVEL_INFO, PSTR("SOLIS: Serial initialized"));
    } else {
      delete SolisMeterSwSerial;
      SolisMeterSwSerial = nullptr;
      AddLog(LOG_LEVEL_ERROR, PSTR("SOLIS: Serial initialization failed"));
    }
  }
}

void SolisMeterDrvInit(void)
{  
  if (PinUsed(GPIO_SOLIS_METER_TX) && PinUsed(GPIO_SOLIS_METER_RX) && PinUsed(GPIO_SOLIS_METER_ENA)) {
    pinMode(Pin(GPIO_SOLIS_METER_ENA), OUTPUT);
    digitalWrite(Pin(GPIO_SOLIS_METER_ENA), LOW);  
  
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xdrv137(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_ACTIVE:
      return SolisMeter.init;
    //case FUNC_EVERY_100_MSECOND:
    case FUNC_LOOP:
      SolisMeterEvery100ms();
      break;
    case FUNC_EVERY_250_MSECOND:
      break;
    case FUNC_INIT:
      SolisMeterSnsInit();
      break;
    case FUNC_PRE_INIT:
      SolisMeterDrvInit();
      break;
  #ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      SolisMeterShow(0);
      break;   
  #endif  // USE_WEBSERVER
  }
  return result;
}



uint16_t SolisMBCalculateCRC(uint8_t *frame, uint8_t num)
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



#endif  // USE_SOLIS_METER
