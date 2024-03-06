/*
  xnrg_26_dtsu666h.ino - Chint DTSU666-H Modbus energy meter support for Tasmota

  Copyright (C) 2021  Pablo Zerón and Theo Arends

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

#ifdef USE_ENERGY_SENSOR
#ifdef USE_DTSU666_H
/*********************************************************************************************\
 * Chint DTSU666-H Modbus energy meter
\*********************************************************************************************/

#define XNRG_26             26

// can be user defined in my_user_config.h
#ifndef DTSU666H_SPEED
  #define DTSU666H_SPEED      4800    // default DTSU66-H Modbus address
#endif
// can be user defined in my_user_config.h
#ifndef DTSU666H_ADDR
  #define DTSU666H_ADDR       11      // default DTSU66-H Modbus address
#endif

#include <TasmotaModbus.h>
TasmotaModbus *Dtsu666HModbus;

// combined register config
// const uint16_t Dtsu666H_start_addresses[] {
//   2110,   // DTSU666H_VOLTAGE phase A     [V]
//           // DTSU666H_VOLTAGE phase B     [V]
//           // DTSU666H_VOLTAGE phase C     [V]
//   2102,   // DTSU666H_CURRENT phase A     [A]
//           // DTSU666H_CURRENT phase B     [A]
//           // DTSU666H_CURRENT phase C     [A]
//   2128,   // DTSU666H_POWER phase A       [kW]
//           // DTSU666H_POWER phase B       [kW]
//           // DTSU666H_POWER phase C       [kW]
//   2134,   // DTSU666H_REACTIVE_POWER      [kVar]
//   2150,   // DTSU666H_POWER_FACTOR
//   2124,   // DTSU666H_FREQUENCY           [Hz]
//   2166,   // DTSU666H_IMPORT_ACTIVE       [kWh]
//   2174,   // DTSU666H_EXPORT_ACTIVE       [kWh]
// };


// 2D-array: address, data length (register count)
const uint16_t Dtsu666H_start_addresses[][4] {
  {8198, 12}, // DTSU666H_VOLTAGE phase A     [V]   - 8198
              // DTSU666H_VOLTAGE phase B     [V]   - 8200
              // DTSU666H_VOLTAGE phase C     [V]   - 8202
              // DTSU666H_CURRENT phase A     [A]   - 8204
              // DTSU666H_CURRENT phase B     [A]   - 8206
              // DTSU666H_CURRENT phase C     [A]   - 8208
  {8210, 24}, // DTSU666H_ACTIVE_POWER        [kW]    2126, 24
              // DTSU666H_ACTIVE_POWER ph A   [kW]
              // DTSU666H_ACTIVE_POWER ph B   [kW]
              // DTSU666H_ACTIVE_POWER ph C   [kW]
              // DTSU666H_REACTIVE_POWER      [kVar]
              // DTSU666H_REACTIVE_POWER ph A [kVar]
              // DTSU666H_REACTIVE_POWER ph B [kVar]
              // DTSU666H_REACTIVE_POWER ph C [kVar]
              // DTSU666H_APPARENT_POWER      [kVA]
              // DTSU666H_APPARENT_POWER ph A [kVA]
              // DTSU666H_APPARENT_POWER ph B [kVA]
              // DTSU666H_APPARENT_POWER ph C [kVA]
  {8234, 8},  // DTSU666H_POWER_FACTOR        [-]   2150, 26
              // DTSU666H_POWER_FACTOR ph A   [-]
              // DTSU666H_POWER_FACTOR ph B   [-]
              // DTSU666H_POWER_FACTOR ph C   [-]
  {16414, 12} // DTSU666H_TOTAL_IMPORT        [kWh]  - 16414
              // ??
              // ??
              // ??
              // ??
              // DTSU666H_TOTAL_EXPORT        [kWh]  - 16424
};

  // // single register config
  // const uint16_t Dtsu666H_start_addresses[] {
  //   2110,   // DTSU666H_VOLTAGE phase A     [V]
  //   2112,   // DTSU666H_VOLTAGE phase B     [V]
  //   2114,   // DTSU666H_VOLTAGE phase C     [V]
  //   2102,   // DTSU666H_CURRENT phase A     [A]
  //   2104,   // DTSU666H_CURRENT phase B     [A]
  //   2106,   // DTSU666H_CURRENT phase C     [A]
  //   2128,   // DTSU666H_POWER phase A       [kW]
  //   2130,   // DTSU666H_POWER phase B       [kW]
  //   2132,   // DTSU666H_POWER phase C       [kW]
  //   2134,   // DTSU666H_REACTIVE_POWER      [kVar]
  //   2150,   // DTSU666H_POWER_FACTOR
  //   2124,   // DTSU666H_FREQUENCY           [Hz]
  //   2166,   // DTSU666H_IMPORT_ACTIVE       [kWh]
  //   2174,   // DTSU666H_EXPORT_ACTIVE       [kWh]
  // };

struct DTSU666H {
  uint8_t read_state = 0;
  uint8_t send_retry = 0;
  uint8_t init = 0;
  unsigned long timestamp_prev = 0;
  unsigned long timestamp_new = 0;
  float energy_import = 0;
  float energy_export = 0;
} Dtsu666H;

/*********************************************************************************************/

void Dtsu666HEvery200ms(void)
{  
  unsigned long timedelta;
  float powersum;
  int32_t deca_microWh;

  if (Dtsu666H.init) {
    bool data_ready = Dtsu666HModbus->ReceiveReady();

    if (data_ready) {
      uint8_t buffer[70];  // 
      uint32_t error;

      error = Dtsu666HModbus->ReceiveBuffer(buffer, (Dtsu666H_start_addresses[Dtsu666H.read_state][1]*2) );
      AddLogBuffer(LOG_LEVEL_DEBUG_MORE, buffer, Dtsu666HModbus->ReceiveCount());

      //Serial.println(F("DTSU666-H: data_ready"));

      if (error) {
        AddLog(LOG_LEVEL_DEBUG, PSTR("DTSU666-H: recv error %d"), error);
        //Serial.println(F("DTSU666-H: recv error"));
      } else {
        //AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("DTSU666-H: recv okay"));
        Energy->data_valid[0] = 0;

        float value;
        ((uint8_t*)&value)[3] = buffer[3];   // Get float values
        ((uint8_t*)&value)[2] = buffer[4];
        ((uint8_t*)&value)[1] = buffer[5];
        ((uint8_t*)&value)[0] = buffer[6];

        switch(Dtsu666H.read_state) {
          // case 0:   Energy->current[0]  = value;          // 1.260 A - phase A

          //           ((uint8_t*)&value)[3] = buffer[7];    // Get float values
          //           ((uint8_t*)&value)[2] = buffer[8];
          //           ((uint8_t*)&value)[1] = buffer[9];
          //           ((uint8_t*)&value)[0] = buffer[10];
          //           Energy->current[1] = value;           //  1.260 A - phase B

          //           ((uint8_t*)&value)[3] = buffer[11];   // Get float values
          //           ((uint8_t*)&value)[2] = buffer[12];
          //           ((uint8_t*)&value)[1] = buffer[13];
          //           ((uint8_t*)&value)[0] = buffer[14];
          //           Energy->current[2] = value;           //  1.260 A - phase C


          //           ((uint8_t*)&value)[3] = buffer[19];   // Get float values
          //           ((uint8_t*)&value)[2] = buffer[20];
          //           ((uint8_t*)&value)[1] = buffer[21];
          //           ((uint8_t*)&value)[0] = buffer[22];
          //           Energy->voltage[0] = value;           // 230.2 V - phase A

          //           ((uint8_t*)&value)[3] = buffer[23];   // Get float values
          //           ((uint8_t*)&value)[2] = buffer[24];
          //           ((uint8_t*)&value)[1] = buffer[25];
          //           ((uint8_t*)&value)[0] = buffer[26];
          //           Energy->voltage[1] = value;           // 230.2 V - phase B

          //           ((uint8_t*)&value)[3] = buffer[27];   // Get float values
          //           ((uint8_t*)&value)[2] = buffer[28];
          //           ((uint8_t*)&value)[1] = buffer[29];
          //           ((uint8_t*)&value)[0] = buffer[30];
          //           Energy->voltage[2] = value;           // 230.2 V - phase C


          //           ((uint8_t*)&value)[3] = buffer[47];   // Get float values
          //           ((uint8_t*)&value)[2] = buffer[48];
          //           ((uint8_t*)&value)[1] = buffer[49];
          //           ((uint8_t*)&value)[0] = buffer[50];
          //           Energy->frequency[0] = value;         // 50.0 Hz

          //           break;
          case 0:   Energy->voltage[0] = value;           // 230.2 V - phase A

                    ((uint8_t*)&value)[3] = buffer[7];   // Get float values
                    ((uint8_t*)&value)[2] = buffer[8];
                    ((uint8_t*)&value)[1] = buffer[9];
                    ((uint8_t*)&value)[0] = buffer[10];
                    Energy->voltage[1] = value;           // 230.2 V - phase B

                    ((uint8_t*)&value)[3] = buffer[11];   // Get float values
                    ((uint8_t*)&value)[2] = buffer[12];
                    ((uint8_t*)&value)[1] = buffer[13];
                    ((uint8_t*)&value)[0] = buffer[14];
                    Energy->voltage[2] = value;           // 230.2 V - phase C
          
                    ((uint8_t*)&value)[3] = buffer[15];   // Get float values
                    ((uint8_t*)&value)[2] = buffer[16];
                    ((uint8_t*)&value)[1] = buffer[17];
                    ((uint8_t*)&value)[0] = buffer[18];                 
                    Energy->current[0]  = value;          // 1.260 A - phase A

                    ((uint8_t*)&value)[3] = buffer[19];    // Get float values
                    ((uint8_t*)&value)[2] = buffer[20];
                    ((uint8_t*)&value)[1] = buffer[21];
                    ((uint8_t*)&value)[0] = buffer[22];
                    Energy->current[1] = value;           //  1.260 A - phase B

                    ((uint8_t*)&value)[3] = buffer[23];   // Get float values
                    ((uint8_t*)&value)[2] = buffer[24];
                    ((uint8_t*)&value)[1] = buffer[25];
                    ((uint8_t*)&value)[0] = buffer[26];
                    Energy->current[2] = value;           //  1.260 A - phase C

                    break;

          case 1:   // active power
                    ((uint8_t*)&value)[3] = buffer[7];            // Get float values
                    ((uint8_t*)&value)[2] = buffer[8];
                    ((uint8_t*)&value)[1] = buffer[9];
                    ((uint8_t*)&value)[0] = buffer[10];
                    Energy->active_power[0] = value * 1;       // -196.3 W - phase A

                    ((uint8_t*)&value)[3] = buffer[11];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[12];
                    ((uint8_t*)&value)[1] = buffer[13];
                    ((uint8_t*)&value)[0] = buffer[14];
                    Energy->active_power[1] = value * 1;       // -196.3 W - phase B

                    ((uint8_t*)&value)[3] = buffer[15];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[16];
                    ((uint8_t*)&value)[1] = buffer[17];
                    ((uint8_t*)&value)[0] = buffer[18];
                    Energy->active_power[2] = value * 1;       // -196.3 W - phase C


                    // reactive power
                    ((uint8_t*)&value)[3] = buffer[23];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[24];
                    ((uint8_t*)&value)[1] = buffer[25];
                    ((uint8_t*)&value)[0] = buffer[26];
                    Energy->reactive_power[0] = value * 1;     // -196.3 VA - phase A

                    ((uint8_t*)&value)[3] = buffer[27];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[28];
                    ((uint8_t*)&value)[1] = buffer[29];
                    ((uint8_t*)&value)[0] = buffer[30];
                    Energy->reactive_power[1] = value * 1;     // -196.3 VA - phase B

                    ((uint8_t*)&value)[3] = buffer[31];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[32];
                    ((uint8_t*)&value)[1] = buffer[33];
                    ((uint8_t*)&value)[0] = buffer[34];
                    Energy->reactive_power[2] = value * 1;     // -196.3 VA - phase C


                    // apparent power
                    ((uint8_t*)&value)[3] = buffer[39];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[40];
                    ((uint8_t*)&value)[1] = buffer[41];
                    ((uint8_t*)&value)[0] = buffer[42];
                    Energy->apparent_power[0] = value * 1;     // -196.3 VA - phase A

                    ((uint8_t*)&value)[3] = buffer[43];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[44];
                    ((uint8_t*)&value)[1] = buffer[45];
                    ((uint8_t*)&value)[0] = buffer[46];
                    Energy->apparent_power[1] = value * 1;     // -196.3 VA - phase B

                    ((uint8_t*)&value)[3] = buffer[47];           // Get float values
                    ((uint8_t*)&value)[2] = buffer[48];
                    ((uint8_t*)&value)[1] = buffer[49];
                    ((uint8_t*)&value)[0] = buffer[50];
                    Energy->apparent_power[2] = value * 1;     // -196.3 VA - phase C


                    timedelta = Dtsu666H.timestamp_new - Dtsu666H.timestamp_prev;
                    powersum = Energy->active_power[0] + Energy->active_power[1] + Energy->active_power[2];
                    //powersum = powersum * (float) timedelta / 3600000000.0;

                    deca_microWh = (int32_t) (powersum * (float) timedelta / 36.0);
                    Energy->kWhtoday_delta[0] += deca_microWh;      
                    Energy->kWhtoday_delta[1] = 0;
                    Energy->kWhtoday_delta[2] = 0;
                    //Energy->export_active[0] = 0;              

                    // if (powersum >= 0) {
                    //   Dtsu666H.energy_import += powersum;
                    // }
                    // else {
                    //   Dtsu666H.energy_export += powersum;
                    // }
                    // Energy->import_active[0] = Dtsu666H.energy_import/3;
                    // Energy->import_active[1] = Dtsu666H.energy_import/3;
                    // Energy->import_active[2] = Dtsu666H.energy_import/3;
                    // Energy->export_active[0] = Dtsu666H.energy_export/3;         
                    // Energy->export_active[1] = Dtsu666H.energy_export/3;         
                    // Energy->export_active[2] = Dtsu666H.energy_export/3;         

                    break;

          case 2:   //Energy->power_factor[0] = value;      // 0.99 - power factor mean

                    ((uint8_t*)&value)[3] = buffer[7];        // Get float values
                    ((uint8_t*)&value)[2] = buffer[8];
                    ((uint8_t*)&value)[1] = buffer[9];
                    ((uint8_t*)&value)[0] = buffer[10];
                    Energy->power_factor[0] = value;          // 0.99 - power factor A

                    ((uint8_t*)&value)[3] = buffer[11];       // Get float values
                    ((uint8_t*)&value)[2] = buffer[12];
                    ((uint8_t*)&value)[1] = buffer[13];
                    ((uint8_t*)&value)[0] = buffer[14];
                    Energy->power_factor[1] = value;          // 0.99 - power factor B

                    ((uint8_t*)&value)[3] = buffer[15];       // Get float values
                    ((uint8_t*)&value)[2] = buffer[16];
                    ((uint8_t*)&value)[1] = buffer[17];
                    ((uint8_t*)&value)[0] = buffer[18];
                    Energy->power_factor[2] = value;          // 0.99 - power factor C

                    // ((uint8_t*)&value)[3] = buffer[35];       // Get float values
                    // ((uint8_t*)&value)[2] = buffer[36];
                    // ((uint8_t*)&value)[1] = buffer[37];
                    // ((uint8_t*)&value)[0] = buffer[38];                    
                    // // Energy->import_active[0] = value/3;         // 478.492 kWh
                    // // Energy->import_active[1] = value/3;         // 478.492 kWh
                    // // Energy->import_active[2] = value/3;         // 478.492 kWh
          
                    // ((uint8_t*)&value)[3] = buffer[51];       // Get float values
                    // ((uint8_t*)&value)[2] = buffer[52];
                    // ((uint8_t*)&value)[1] = buffer[53];
                    // ((uint8_t*)&value)[0] = buffer[54];
                    // // Energy->export_active[0] = value/3;         // 6.216 kWh
                    // // Energy->export_active[1] = value/3;         // 6.216 kWh
                    // // Energy->export_active[2] = value/3;         // 6.216 kWh

                    break;

          case 3:   // import
                    //Energy->import_active[0] = value/3;         // 478.492 kWh
                    //Energy->import_active[1] = value/3;         // 478.492 kWh
                    //Energy->import_active[2] = value/3;         // 478.492 kWh

                    // export
                    ((uint8_t*)&value)[3] = buffer[23];         // Get float values
                    ((uint8_t*)&value)[2] = buffer[24];
                    ((uint8_t*)&value)[1] = buffer[25];
                    ((uint8_t*)&value)[0] = buffer[26];
                    //Energy->export_active[0] = value/3;         // 6.216 kWh
                    //Energy->export_active[1] = value/3;         // 6.216 kWh
                    //Energy->export_active[2] = value/3;         // 6.216 kWh

                    break;
          
        }
      }
    } // end data ready

    if (0 == Dtsu666H.send_retry || data_ready) {
      Dtsu666H.send_retry = 5;
      Dtsu666H.read_state++;      
      if (Dtsu666H.read_state > 3) {
        Dtsu666H.read_state = 0;
        EnergyUpdateTotal();  // test 
      }

      if (Dtsu666H.read_state == 1) {
        Dtsu666H.timestamp_prev = Dtsu666H.timestamp_new;
        Dtsu666H.timestamp_new = millis();
      }

      Dtsu666HModbus->Send(DTSU666H_ADDR, 0x04, Dtsu666H_start_addresses[Dtsu666H.read_state][0], Dtsu666H_start_addresses[Dtsu666H.read_state][1]);  
      //AddLog(LOG_LEVEL_DEBUG, PSTR("DTSU666-H: Send request %d"), Dtsu666H.read_state);
      //Serial.println(F("DTSU666-H: Send request"));
    } else {
      Dtsu666H.send_retry--;
    }

  }
}

void Dtsu666HSnsInit(void)
{
  if (PinUsed(GPIO_DTSU666H_RX) && PinUsed(GPIO_DTSU666H_TX) && PinUsed(GPIO_NRG_MBS_TX_ENA)) {
    //Serial.println(F("DTSU666-H: SnsInit"));

    Dtsu666HModbus = new TasmotaModbus(Pin(GPIO_DTSU666H_RX), Pin(GPIO_DTSU666H_TX), Pin(GPIO_NRG_MBS_TX_ENA));
    uint8_t result = Dtsu666HModbus->Begin(DTSU666H_SPEED);
    if (result) {
      //Serial.print(F("DTSU666-H: init - "));   Serial.println(result, DEC);
      if (1 == result) { 
        //ClaimSerial(); 
        AddLog(LOG_LEVEL_INFO, PSTR("DTSU666-H: init okay %d"), result);
        //Serial.println(F("DTSU666-H: init okay 1"));
        Dtsu666H.init = 1;
        
        // Energy->phase_count = 3;
        // Energy->frequency_common = true;             // Use common frequency
        // Energy->local_energy_active_export = true; 

        // Serial.println(F("DTSU666-H: array"));
        // Serial.println(Dtsu666H_start_addresses[0][0], DEC);
        // Serial.println(Dtsu666H_start_addresses[0][1], DEC);
        // Serial.println(Dtsu666H_start_addresses[1][0], DEC);
        // Serial.println(Dtsu666H_start_addresses[1][1], DEC);

        //Dtsu666HModbus->Send(0x01, 0x04, 0, 8);
      }
      else {
        TasmotaGlobal.energy_driver = ENERGY_NONE;
        //Serial.print(F("DTSU666-H: init error 1"));
      }
    } else {
      TasmotaGlobal.energy_driver = ENERGY_NONE;
      //Serial.print(F("DTSU666-H: init error"));
    }
  }
}

void Dtsu666HDrvInit(void)
{
  if (PinUsed(GPIO_DTSU666H_RX) && PinUsed(GPIO_DTSU666H_TX) && PinUsed(GPIO_NRG_MBS_TX_ENA)) {
    //Serial.println(F("DTSU666-H: DrvInit"));
    TasmotaGlobal.energy_driver = XNRG_26;
    
    Energy->phase_count = 3;
    Energy->frequency_common = true;             // Use common frequency
    Energy->local_energy_active_export = true; 
  }
}

// void Dtsu666HShow(bool json) {
//   if (isnan(Energy->import_active[0])) { return; }

//   if (json) {
//     ResponseAppend_P(PSTR(",\"" D_JSON_IMPORT_ACTIVE "\":%s"), EnergyFmt(&Energy->import_active[0], Settings->flag2.energy_resolution));
//     ResponseAppend_P(PSTR(",\"" D_JSON_EXPORT_ACTIVE "\":%s"), EnergyFmt(&Energy->export_active[0], Settings->flag2.energy_resolution));
// #ifdef USE_WEBSERVER
//   } else {
//     // WSContentSend_PD(HTTP_SNS_IMPORT_REACTIVE, WebEnergyFmt(&Sdm120.import_reactive, Settings->flag2.energy_resolution, 2));
//     // WSContentSend_PD(HTTP_SNS_EXPORT_REACTIVE, WebEnergyFmt(&Sdm120.export_reactive, Settings->flag2.energy_resolution, 2));
//     // WSContentSend_PD(HTTP_SNS_PHASE_ANGLE, WebEnergyFmt(&Sdm120.phase_angle, 2));
// #endif  // USE_WEBSERVER
//   }
// }

/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

bool Xnrg26(uint32_t function)
{
  bool result = false;

  switch (function) {
    case FUNC_EVERY_200_MSECOND:
      Dtsu666HEvery200ms();
      break;
    // case FUNC_JSON_APPEND:
    //   Dtsu666HShow(1);
    //   break;
    case FUNC_INIT:
      Dtsu666HSnsInit();
      break;
    case FUNC_PRE_INIT:
      Dtsu666HDrvInit();
      break;
  }
  return result;
}

#endif  // USE_DTSU666_H
#endif  // USE_ENERGY_SENSOR
