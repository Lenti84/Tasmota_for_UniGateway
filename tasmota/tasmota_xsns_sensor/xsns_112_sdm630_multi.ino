/*
  xnrg_112_sdm630_multi.ino - SDM630 multi energy meter support for Tasmota

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

#ifdef USE_SDM630_MULTI
/*********************************************************************************************\
 * Eastron SDM630 Multi meter support
\*********************************************************************************************/

#define XSNS_112             112

// can be user defined in my_user_config.h
#ifndef SDM630_MULTI_SPEED
  #define SDM630_MULTI_SPEED       19200    // default SDM630 multi Modbus speed
#endif
// can be user defined in my_user_config.h
#ifndef SDM630_1_ADDR
  #define SDM630_1_ADDR       2       // default SDM630 #1 Modbus address
#endif
#ifndef SDM630_2_ADDR
  #define SDM630_2_ADDR       3       // default SDM630 #2 Modbus address
#endif
#ifndef SDM630_3_ADDR
  #define SDM630_3_ADDR       4       // default SDM630 #3 Modbus address
#endif

#include <TasmotaModbus.h>
TasmotaModbus *Sdm630MultiModbus;

typedef struct SDM630_METER {
  uint32_t address = 0;
  float power_phase1 = NAN;
  float power_phase2 = NAN;
  float power_phase3 = NAN;
} Sdm630_MeterData;

struct SDM630_MULTI {
  // float power_meter1_phase1 = NAN;
  // float power_meter1_phase2 = NAN;
  // float power_meter1_phase3 = NAN;
  // float power_meter2_phase1 = NAN;
  // float power_meter2_phase2 = NAN;
  // float power_meter2_phase3 = NAN;
  // float power_meter3_phase1 = NAN;
  // float power_meter3_phase2 = NAN;
  // float power_meter3_phase3 = NAN;
  Sdm630_MeterData meter[3];
  float total_grid_power = 0;
  float total_pv_power = 0;
  float total_load_power = 0;
  float total_battery_power = 0;
  uint8_t read_state = 0;
  uint8_t send_retry = 0;
  uint8_t init = 0;
  uint8_t metercount = 0;
} Sdm630_Multi;


// 2D-array: address, modbus adress, target mem adress (register count)
const uint32_t sdm630_multi_register[][9] {
  // {0x000C, SDM630_1_ADDR, (Sdm630_MeterData) &Sdm630_Multi.meter1},   // SDM630 Phase 1...3 power     [W]
  // {0x000C, SDM630_2_ADDR, (Sdm630_MeterData) &Sdm630_Multi.meter2},   // SDM630 Phase 1...3 power     [W]
  // {0x000C, SDM630_3_ADDR, (Sdm630_MeterData) &Sdm630_Multi.meter3},   // SDM630 Phase 1...3 power     [W]
  // // {0x000E, SDM630_1_ADDR, (uint32_t) &Sdm630_Multi.power_meter1_phase2},   // SDM630 Phase 2 power     [W]
  // {0x0010, SDM630_1_ADDR, (uint32_t) &Sdm630_Multi.power_meter1_phase3},   // SDM630 Phase 3 power     [W]
  // {0x000C, SDM630_2_ADDR, (uint32_t) &Sdm630_Multi.power_meter2_phase1},   // SDM630 Phase 1 power     [W]
  // {0x000E, SDM630_2_ADDR, (uint32_t) &Sdm630_Multi.power_meter2_phase2},   // SDM630 Phase 2 power     [W]
  // {0x0010, SDM630_2_ADDR, (uint32_t) &Sdm630_Multi.power_meter2_phase3},   // SDM630 Phase 3 power     [W]
  // {0x000C, SDM630_3_ADDR, (uint32_t) &Sdm630_Multi.power_meter3_phase1},   // SDM630 Phase 1 power     [W]
  // {0x000E, SDM630_3_ADDR, (uint32_t) &Sdm630_Multi.power_meter3_phase2},   // SDM630 Phase 2 power     [W]
  // {0x0010, SDM630_3_ADDR, (uint32_t) &Sdm630_Multi.power_meter3_phase3},   // SDM630 Phase 3 power     [W]
};

/*********************************************************************************************/

float Sdm630MultiGetData(uint8_t index) {
  if(index == 1)      return Sdm630_Multi.meter[0].power_phase1;
  else if(index == 2) return Sdm630_Multi.meter[0].power_phase2;
  else if(index == 3) return Sdm630_Multi.meter[0].power_phase3;
  else if(index == 4) return Sdm630_Multi.meter[1].power_phase1;
  else if(index == 5) return Sdm630_Multi.meter[1].power_phase2;
  else if(index == 6) return Sdm630_Multi.meter[1].power_phase3;
  else if(index == 7) return Sdm630_Multi.meter[2].power_phase1;
  else if(index == 8) return Sdm630_Multi.meter[2].power_phase2;
  else if(index == 9) return Sdm630_Multi.meter[2].power_phase3;
  else if(index == 10) return Sdm630_Multi.total_grid_power;
  else if(index == 11) return Sdm630_Multi.total_pv_power;
  else if(index == 12) return Sdm630_Multi.total_load_power;
  else if(index == 13) return Sdm630_Multi.total_battery_power;
  else return 0.0;
}

/*********************************************************************************************/

void Sdm630MultiEvery100ms(void)
{
  if (Sdm630_Multi.init == 1) {
    bool data_ready = Sdm630MultiModbus->ReceiveReady();
    //Serial.print("recv: ");
    //Serial.println(Sdm72Sdm230Modbus->ReceiveCount(), DEC);

    if (data_ready) {
      uint8_t buffer[20];  // At least 5 + (2 * 2) = 9

      uint32_t error = Sdm630MultiModbus->ReceiveBuffer(buffer, 6);
     // uint32_t rcvlen = Sdm630MultiModbus->ReceiveCount();
      AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, Sdm630MultiModbus->ReceiveCount());

      if (error) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("SDM630Multi: error %d"), error);
      } else {

        float value;
        ((uint8_t*)&value)[3] = buffer[3];   // Get float values
        ((uint8_t*)&value)[2] = buffer[4];
        ((uint8_t*)&value)[1] = buffer[5];
        ((uint8_t*)&value)[0] = buffer[6];
        Sdm630_Multi.meter[Sdm630_Multi.read_state].power_phase1 = value;

        ((uint8_t*)&value)[3] = buffer[7];   // Get float values
        ((uint8_t*)&value)[2] = buffer[8];
        ((uint8_t*)&value)[1] = buffer[9];
        ((uint8_t*)&value)[0] = buffer[10];
        Sdm630_Multi.meter[Sdm630_Multi.read_state].power_phase2 = value;

        ((uint8_t*)&value)[3] = buffer[11];   // Get float values
        ((uint8_t*)&value)[2] = buffer[12];
        ((uint8_t*)&value)[1] = buffer[13];
        ((uint8_t*)&value)[0] = buffer[14];
        Sdm630_Multi.meter[Sdm630_Multi.read_state].power_phase3 = value;

        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM630Multi: val %f"), value);
        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM630Multi: total_active_SDM72 %f"), Sdm630_Multi.total_active_SDM72);
        //AddLog(LOG_LEVEL_DEBUG, PSTR("SDM630Multi: total_active_SDM230 %f"), Sdm630_Multi.total_active_SDM230);

        //++Sdm630_Multi.read_state %= nitems(Sdm630_Multi.meter);
        ++Sdm630_Multi.read_state;      
        if (Sdm630_Multi.read_state == Sdm630_Multi.metercount) Sdm630_Multi.read_state = 0;
        
        // if (0 == Sdm630_Multi.read_state && !isnan(Sdm630_Multi.total_active)) {
        //   Energy->import_active[0] = Sdm630_Multi.total_active;
        //   EnergyUpdateTotal();
        // }
      }
    } // end data ready

    if (0 == Sdm630_Multi.send_retry || data_ready) {
      Sdm630_Multi.send_retry = 5;
      //Sdm630MultiModbus->Send((uint8_t) (sdm630_multi_register[Sdm630_Multi.read_state][1]), 0x04, (uint16_t) (sdm630_multi_register[Sdm630_Multi.read_state][0]), 2);
      Sdm630MultiModbus->Send((uint8_t) (Sdm630_Multi.meter[Sdm630_Multi.read_state].address), 0x04, 0x000C, 6);
      #warning fix it
    } else {
      Sdm630_Multi.send_retry--;
    }

  }

  // always calc total values
  Sdm630_Multi.total_grid_power = Sdm630_Multi.meter[0].power_phase1 + Sdm630_Multi.meter[0].power_phase2 + Sdm630_Multi.meter[0].power_phase3;
  Sdm630_Multi.total_pv_power   = Sdm630_Multi.meter[1].power_phase1 + Sdm630_Multi.meter[1].power_phase2 + Sdm630_Multi.meter[1].power_phase3 + Sdm630_Multi.meter[2].power_phase3;
  Sdm630_Multi.total_load_power = Sdm630_Multi.total_grid_power + Sdm630_Multi.total_pv_power - Sdm630_Multi.meter[2].power_phase1;
  Sdm630_Multi.total_battery_power = Sdm630_Multi.meter[2].power_phase1;
}


void Sdm630MultiSnsInit(void)
{ 
  Sdm630_Multi.meter[0].address = SDM630_1_ADDR;
  Sdm630_Multi.meter[1].address = SDM630_2_ADDR;
  Sdm630_Multi.meter[2].address = SDM630_3_ADDR;
  //Sdm630_Multi.metercount = nitems(Sdm630_Multi.meter);
  Sdm630_Multi.metercount = 3;

  if (PinUsed(GPIO_SDM630_MULTI_RX) && PinUsed(GPIO_SDM630_MULTI_TX) && PinUsed(GPIO_NRG_MBS_TX_ENA)) {
    AddLog(LOG_LEVEL_INFO, PSTR("SDM630Multi: Sns Init"));
    Sdm630MultiModbus = new TasmotaModbus(Pin(GPIO_SDM630_MULTI_RX), Pin(GPIO_SDM630_MULTI_TX), Pin(GPIO_NRG_MBS_TX_ENA));
    uint8_t result = Sdm630MultiModbus->Begin(SDM630_MULTI_SPEED);
    if (result) {
      if (1 == result) {
          //ClaimSerial();
          Sdm630_Multi.init = 1;
          AddLog(LOG_LEVEL_INFO, PSTR("SDM630Multi: Sns Init successful"));
      }
      else AddLog(LOG_LEVEL_INFO, PSTR("SDM630Multi: Sns Init result != 1"));

    } else {
      AddLog(LOG_LEVEL_INFO, PSTR("SDM630Multi: Sns Init Error"));
    }
  }
}

void Sdm630MultiDrvInit(void)
{
  if (PinUsed(GPIO_SDM630_MULTI_RX) && PinUsed(GPIO_SDM630_MULTI_TX) && PinUsed(GPIO_NRG_MBS_TX_ENA)) {
    AddLog(LOG_LEVEL_INFO, PSTR("SDM630Multi: Drv Init"));
  }
}

void Sdm630MultiShow(bool json) {  
  if (Sdm630_Multi.init == 1) {

    if (json) {
      // none
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_P(PSTR("{s}SDM630 Multi - Meters{m}{e}"));

      WSContentSend_PD("{s}Leistung Netz ges.{m}%.0f W{e}", Sdm630_Multi.total_grid_power);
      WSContentSend_PD("{s}Leistung PV ges.{m}%.0f W{e}", Sdm630_Multi.total_pv_power);
      WSContentSend_PD("{s}Leistung Last ges.{m}%.0f W{e}", Sdm630_Multi.total_load_power);
      WSContentSend_PD("{s}Leistung Batterie{m}%.0f W{e}", Sdm630_Multi.total_battery_power);

      WSContentSend_PD("{s}Meter 1 - Phase 1 - Grid{m}%.0f W{e}", Sdm630_Multi.meter[0].power_phase1);
      WSContentSend_PD("{s}Meter 1 - Phase 2 - Grid{m}%.0f W{e}", Sdm630_Multi.meter[0].power_phase2);
      WSContentSend_PD("{s}Meter 1 - Phase 3 - Grid{m}%.0f W{e}", Sdm630_Multi.meter[0].power_phase3);
      WSContentSend_PD("{s}Meter 2 - Phase 1 - PV 1{m}%.0f W{e}", Sdm630_Multi.meter[1].power_phase1);
      WSContentSend_PD("{s}Meter 2 - Phase 2 - PV 1{m}%.0f W{e}", Sdm630_Multi.meter[1].power_phase2);
      WSContentSend_PD("{s}Meter 2 - Phase 3 - PV 1{m}%.0f W{e}", Sdm630_Multi.meter[1].power_phase3);
      WSContentSend_PD("{s}Meter 3 - Phase 1 - Battery{m}%.0f W{e}", Sdm630_Multi.meter[2].power_phase1);
      WSContentSend_PD("{s}Meter 3 - Phase 2{m}%.0f W{e}", Sdm630_Multi.meter[2].power_phase2);
      WSContentSend_PD("{s}Meter 3 - Phase 3 - PV 2{m}%.0f W{e}", Sdm630_Multi.meter[2].power_phase3);

      WSContentSend_P(PSTR("{s} {m} {e}"));      
#endif  // USE_WEBSERVER
    }
  }
}

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xsns112(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_100_MSECOND:
      Sdm630MultiEvery100ms();
      break;
    case FUNC_JSON_APPEND:
      Sdm630MultiShow(1);
      break;
#ifdef USE_WEBSERVER
    case FUNC_WEB_SENSOR:
      Sdm630MultiShow(0);
      break;
#endif  // USE_WEBSERVER
    case FUNC_INIT:
      Sdm630MultiSnsInit();
      break;
    case FUNC_PRE_INIT:
      Sdm630MultiDrvInit();
      break;
  }
  return result;
}

#endif  // USE_SDM630_MULTI
