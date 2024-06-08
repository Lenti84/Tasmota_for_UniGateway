/*
  xnrg_27_sdm72_sdm230.ino - SDM72 / SDM230 combined energy meter support for Tasmota

  Copyright (C) 2021  Gennaro Tortone, Theo Arends and Norbert Richter

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

#ifdef USE_SDM72_SDM230
/*********************************************************************************************\
 * Eastron SDM72 and SDM230 Modbus energy meter
\*********************************************************************************************/

#define XSNS_111             111

// can be user defined in my_user_config.h
#ifndef SDM72_SDM230_SPEED
  #define SDM72_SDM230_SPEED       9600    // default SDM72/SDM230 Modbus address
#endif
// can be user defined in my_user_config.h
#ifndef SDM72230_72_ADDR
  #define SDM72230_72_ADDR        1       // default SDM72 Modbus address
#endif
#ifndef SDM72230_230_ADDR
  #define SDM72230_230_ADDR       2       // default SDM230 Modbus address
#endif

#include <TasmotaModbus.h>
TasmotaModbus *Sdm72Sdm230Modbus;

uint8_t unused1;

struct SDM72_SDM230 {
  float total_active_SDM72 = NAN;
  float total_active_SDM230 = NAN;
  float import_power_SDM72 = NAN;
  float import_power_SDM230 = NAN;
  uint8_t read_state = 0;
  uint8_t send_retry = 0;
  uint8_t init = 0;
} Sdm72Sdm230;

// 2D-array: address, modbus adress, target mem adress (register count)
const uint32_t sdm72sdm230_register[][4] {
  {0x0034, SDM72230_72_ADDR, (uint32_t) &Sdm72Sdm230.total_active_SDM72},   // SDM72 total system power     [W]
  {0x0048, SDM72230_72_ADDR, (uint32_t) &Sdm72Sdm230.import_power_SDM72},   // SDM72 total import energy    [kWh]
  {0x000C, SDM72230_230_ADDR, (uint32_t) &Sdm72Sdm230.total_active_SDM230}, // SDM230 power                 [W]
  {0x0048, SDM72230_230_ADDR, (uint32_t) &Sdm72Sdm230.import_power_SDM230}  // SDM230 Import active energy  [kWh]
};

/*********************************************************************************************/

float Sdm72Sdm230GetData(uint8_t index) {
  if(index == 1) return Sdm72Sdm230.total_active_SDM72;
  else if(index == 2) return Sdm72Sdm230.total_active_SDM230;
  else return 0.0;
}

/*********************************************************************************************/

void Sdm72Sdm230Every250ms(void)
{
  if (Sdm72Sdm230.init == 1) {
    bool data_ready = Sdm72Sdm230Modbus->ReceiveReady();
    //Serial.print("recv: ");
    //Serial.println(Sdm72Sdm230Modbus->ReceiveCount(), DEC);

    if (data_ready) {
      uint8_t buffer[14];  // At least 5 + (2 * 2) = 9

      uint32_t error = Sdm72Sdm230Modbus->ReceiveBuffer(buffer, 2);
      AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, Sdm72Sdm230Modbus->ReceiveCount());

      if (error) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("SDM72230: error %d"), error);
      } else {

        float value;
        float *fptr;
        ((uint8_t*)&value)[3] = buffer[3];   // Get float values
        ((uint8_t*)&value)[2] = buffer[4];
        ((uint8_t*)&value)[1] = buffer[5];
        ((uint8_t*)&value)[0] = buffer[6];

        fptr = (float*) sdm72sdm230_register[Sdm72Sdm230.read_state][2];
        *fptr = value;

        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM72230: val %f"), value);
        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM72230: total_active_SDM72 %f"), Sdm72Sdm230.total_active_SDM72);
        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM72230: total_active_SDM230 %f"), Sdm72Sdm230.total_active_SDM230);

        ++Sdm72Sdm230.read_state %= nitems(sdm72sdm230_register);
        // if (0 == Sdm72Sdm230.read_state && !isnan(Sdm72Sdm230.total_active)) {
        //   Energy->import_active[0] = Sdm72Sdm230.total_active;
        //   EnergyUpdateTotal();
        // }
      }
    } // end data ready

    if (0 == Sdm72Sdm230.send_retry || data_ready) {
      Sdm72Sdm230.send_retry = 5;
      Sdm72Sdm230Modbus->Send((uint8_t) (sdm72sdm230_register[Sdm72Sdm230.read_state][1]), 0x04, (uint16_t) (sdm72sdm230_register[Sdm72Sdm230.read_state][0]), 2);
    } else {
      Sdm72Sdm230.send_retry--;
    }

  }
}

void Sdm72Sdm230SnsInit(void)
{ 
  if (PinUsed(GPIO_SDM72_SDM230_RX) && PinUsed(GPIO_SDM72_SDM230_TX) && PinUsed(GPIO_NRG_MBS_TX_ENA)) {
    AddLog(LOG_LEVEL_INFO, PSTR("SDM72230: Sns Init"));
    Sdm72Sdm230Modbus = new TasmotaModbus(Pin(GPIO_SDM72_SDM230_RX), Pin(GPIO_SDM72_SDM230_TX), Pin(GPIO_NRG_MBS_TX_ENA));
    uint8_t result = Sdm72Sdm230Modbus->Begin(SDM72_SDM230_SPEED);
    if (result) {
      if (1 == result) {
          //ClaimSerial();
          Sdm72Sdm230.init = 1;
          AddLog(LOG_LEVEL_INFO, PSTR("SDM72230: Sns Init successful"));
      }
      else AddLog(LOG_LEVEL_INFO, PSTR("SDM72230: Sns Init result != 1"));

    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("SDM72230: Sns Init Error"));
    }
  }
}

void Sdm72Sdm230DrvInit(void)
{
  if (PinUsed(GPIO_SDM72_SDM230_RX) && PinUsed(GPIO_SDM72_SDM230_TX) && PinUsed(GPIO_NRG_MBS_TX_ENA)) {
    AddLog(LOG_LEVEL_INFO, PSTR("SDM72230: Drv Init"));
  }
}

void Sdm72Sdm230Show(bool json) {  
  if (Sdm72Sdm230.init == 1) {

    //if (isnan(Sdm72Sdm230.total_active_SDM72)) { return; }

    if (json) {
      //ResponseAppend_P(PSTR(",\"" D_JSON_EXPORT_POWER "\":%s"), EnergyFmt(&Sdm72Sdm230.export_power, Settings->flag2.wattage_resolution));
      //ResponseAppend_P(PSTR(",\"" D_JSON_IMPORT_POWER "\":%s"), EnergyFmt(&Sdm72Sdm230.import_power, Settings->flag2.wattage_resolution));
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_P(PSTR("{s}SDM72 SDM230 Meters{m}{e}"));
      WSContentSend_PD("{s}SDM72 Leistung{m}%.0f W{e}", Sdm72Sdm230.total_active_SDM72);
      WSContentSend_PD("{s}SDM72 Verbrauch{m}%.3f kWh{e}", Sdm72Sdm230.import_power_SDM72);
      WSContentSend_PD("{s}SDM230 Leistung{m}%.0f W{e}", Sdm72Sdm230.total_active_SDM230);
      WSContentSend_PD("{s}SDM230 Verbrauch{m}%.3f kWh{e}", Sdm72Sdm230.import_power_SDM230);
      WSContentSend_P(PSTR("{s} {m} {e}"));      
#endif  // USE_WEBSERVER
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns111(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_250_MSECOND:
      Sdm72Sdm230Every250ms();
      break;
    case FUNC_JSON_APPEND:
      Sdm72Sdm230Show(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      Sdm72Sdm230Show(0);
      break;
#endif  // USE_WEBSERVER
    case FUNC_INIT:
      Sdm72Sdm230SnsInit();
      break;
    case FUNC_PRE_INIT:
      Sdm72Sdm230DrvInit();
      break;
  }
  return result;
}

#endif  // USE_SDM72S_DM230
