/*
  user_config_override.h - user configuration overrides my_user_config.h for Tasmota

  Copyright (C) 2021  Theo Arends

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

#ifndef _USER_CONFIG_OVERRIDE_H_
#define _USER_CONFIG_OVERRIDE_H_

/*****************************************************************************************************\
 * USAGE:
 *   To modify the stock configuration without changing the my_user_config.h file:
 *   (1) copy this file to "user_config_override.h" (It will be ignored by Git)
 *   (2) define your own settings below
 *
 ******************************************************************************************************
 * ATTENTION:
 *   - Changes to SECTION1 PARAMETER defines will only override flash settings if you change define CFG_HOLDER.
 *   - Expect compiler warnings when no ifdef/undef/endif sequence is used.
 *   - You still need to update my_user_config.h for major define USE_MQTT_TLS.
 *   - All parameters can be persistent changed online using commands via MQTT, WebConsole or Serial.
\*****************************************************************************************************/

/*
Examples :

// -- Master parameter control --------------------
#undef  CFG_HOLDER
#define CFG_HOLDER        4617                   // [Reset 1] Change this value to load SECTION1 configuration parameters to flash

// -- Setup your own Wifi settings  ---------------
#undef  STA_SSID1
#define STA_SSID1         "YourSSID"             // [Ssid1] Wifi SSID

#undef  STA_PASS1
#define STA_PASS1         "YourWifiPassword"     // [Password1] Wifi password

// -- Setup your own MQTT settings  ---------------
#undef  MQTT_HOST
#define MQTT_HOST         "your-mqtt-server.com" // [MqttHost]

#undef  MQTT_PORT
#define MQTT_PORT         1883                   // [MqttPort] MQTT port (10123 on CloudMQTT)

#undef  MQTT_USER
#define MQTT_USER         "YourMqttUser"         // [MqttUser] Optional user

#undef  MQTT_PASS
#define MQTT_PASS         "YourMqttPass"         // [MqttPassword] Optional password

// You might even pass some parameters from the command line ----------------------------
// Ie:  export PLATFORMIO_BUILD_FLAGS='-DUSE_CONFIG_OVERRIDE -DMY_IP="192.168.1.99" -DMY_GW="192.168.1.1" -DMY_DNS="192.168.1.1"'

#ifdef MY_IP
#undef  WIFI_IP_ADDRESS
#define WIFI_IP_ADDRESS     MY_IP                // Set to 0.0.0.0 for using DHCP or enter a static IP address
#endif

#ifdef MY_GW
#undef  WIFI_GATEWAY
#define WIFI_GATEWAY        MY_GW                // if not using DHCP set Gateway IP address
#endif

#ifdef MY_DNS
#undef  WIFI_DNS
#define WIFI_DNS            MY_DNS               // If not using DHCP set DNS IP address (might be equal to WIFI_GATEWAY)
#endif

#ifdef MY_DNS2
#undef  WIFI_DNS2
#define WIFI_DNS2           MY_DNS2              // If not using DHCP set DNS IP address (might be equal to WIFI_GATEWAY)
#endif

// !!! Remember that your changes GOES AT THE BOTTOM OF THIS FILE right before the last #endif !!!
*/



// PV heater test
#define USE_ENERGY_SENSOR
#define USE_DTSU666_H
#define USE_DEYE_METER
#define USE_SPI
#define USE_RULES
#define USE_BURST_CONTROL
#define USE_TIMEPROP
#define USE_DCOM_LT_MB
#define USE_UVRCAN
//#define USE_MCP2515
#define USE_DDSU666_LISTEN              // Add support for Chint DDSU666 Modbus energy monitor (+0k6 code) in listen only mode
#define USE_MLX90640                    // just for test
#define USE_I2C
#define USE_DISPLAY_LCD
#define USE_DISPLAY
#define USE_SDM72_SDM230


#undef USE_INFLUXDB
#undef USE_HIH6
#undef USE_SHT
#undef USE_SHT3X  
#undef USE_DDSU666
#undef USE_DISPLAY_MODES1TO5 
#undef USE_BERRY
#undef USE_APDS9960_GESTURE                  
#undef USE_APDS9960_PROXIMITY                
#undef USE_APDS9960_COLOR                    
#undef USE_PCA9685_V2
#undef USE_ADE7953 
#undef USE_MAX17043
#undef USE_DISPLAY_SSD1306            
#undef USE_DISPLAY_MATRIX   
#undef USE_DISPLAY_SEVENSEG          
#undef USE_AUTOCONF    
#undef USE_MCP23XXX_DRV  
#undef USE_MCP230xx
#undef USE_BL09XX 
#undef USE_RC_SWITCH
#undef USE_RF_SENSOR
#undef USE_SONOFF_IFAN




#endif  // _USER_CONFIG_OVERRIDE_H_
