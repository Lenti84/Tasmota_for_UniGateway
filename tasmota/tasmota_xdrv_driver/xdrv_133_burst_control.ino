/*
  xdrv_133_burst_control.ino - 


*/

#define USE_BURST_CONTROL
#ifdef USE_BURST_CONTROL


#define D_CMND_BURST_CONTROL_PERIODE "PVPeriode"
#define D_CMND_BURST_NET_POWER_LIMIT "PVNetPowerLimit"
#define D_CMND_BURST_HEATER_POWER    "PVHeaterPower"
#define D_CMND_BURST_MANUAL          "PVManual"
#define D_CMND_BURST_NET_POWER_OVERRIDE  "PVNetOver"

#define D_BURST_CONTROL_NAME         "PV Heizer"
#define D_BURST_CONTROL_HEATER       "Heizstab"
#define D_BURST_CONTROL_NETPOWER     "Netzleistung"

#define _TIMERINTERRUPT_LOGLEVEL_     4

#include "ESP32TimerInterrupt.h"

#include <LiquidCrystal_I2C.h>
extern LiquidCrystal_I2C *lcd;

// Hardware
#define BURST_CTRL_PINSTATE_ON        HIGH
#define BURST_CTRL_PINSTATE_OFF       LOW

// Init ESP32 timer 0
ESP32Timer ITimer0(0);


// Power Control
#define TIMER0_INTERVAL_MS                 10     // ms
#define POWER_CTRL_PERIODE_LEN_DEFAULT     160    // ms - must be multiples of TIMER0_INTERVAL_MS
#define POWER_CTRL_HEATER_POWER_DEFAULT    3000   // Leistung Heizstab
#define POWER_CTRL_NET_POWER_LIMIT         -300   // Leistungsgrenze fuer Beginn Heizen
#define POWER_CTRL_MARGIN_INC              50     // power in W - max margin needed to increase pv heater power
#define POWER_CTRL_MARGIN_DEC              30     // power in W - least margin needed to decrease pv heater power
#define POWER_CTRL_DELAY                   2      // Regelzyklus in s

unsigned int ctrl_gate_periode = POWER_CTRL_PERIODE_LEN_DEFAULT;
unsigned int ctrl_heater_power = POWER_CTRL_HEATER_POWER_DEFAULT;       // Leistung Heizstab in W
signed int   ctrl_net_power_limit = POWER_CTRL_NET_POWER_LIMIT;       // Leistungsgrenze fuer Beginn Heizen in W
unsigned int ctrl_load_step = POWER_CTRL_HEATER_POWER_DEFAULT / POWER_CTRL_PERIODE_LEN_DEFAULT / TIMER0_INTERVAL_MS;      // smallest fraction of power we can control
volatile unsigned int ctrl_steps = POWER_CTRL_PERIODE_LEN_DEFAULT / TIMER0_INTERVAL_MS;

signed int   manual_power = -1;            // manual power overriding all control logic, 0...100 %, -1 = off
unsigned int actual_power = 0;            // actual PV heater power, 0...100 %
signed int   actual_active_power = 0;     // actual active power in W, pos = Consumption, neg = Grid Feeding

signed int   net_power_override = 0;      // DEBUG: actual PV heater power override, W

unsigned int pwm_fraction = 0;


const char kBurstControlCommands[] PROGMEM =         "|" D_CMND_BURST_CONTROL_PERIODE "|" D_CMND_BURST_NET_POWER_LIMIT
                                                     "|" D_CMND_BURST_HEATER_POWER "|" D_CMND_BURST_MANUAL
                                                     "|" D_CMND_BURST_NET_POWER_OVERRIDE;
void (* const BurstControlCommand[])(void) PROGMEM = { &BurstControlSetPeriode, &BurstControlSetNetPowerLimit, 
                                                       &BurstControlSetHeaterPower, &BurstControlSetManual,
                                                       &BurstControlNetPowerOverride };



void BurstControlSetPeriode(void) {
  if ((XdrvMailbox.payload >= 0) && (XdrvMailbox.payload <= 400)) {
    Settings->windmeter_radius = (((uint16_t) XdrvMailbox.payload) / TIMER0_INTERVAL_MS) * TIMER0_INTERVAL_MS;          // match to multiples of TIMER0_INTERVAL_MS
  }
  ResponseCmndNumber(Settings->windmeter_radius);
}

void BurstControlSetHeaterPower(void) {
  if ((XdrvMailbox.payload >= 100) && (XdrvMailbox.payload <= 5000)) {
    Settings->pms_wake_interval = (uint16_t) XdrvMailbox.payload;
  }
  ResponseCmndNumber(Settings->pms_wake_interval);
}

void BurstControlSetNetPowerLimit(void) {
  if ((XdrvMailbox.payload >= -5000) && (XdrvMailbox.payload <= 5000)) {
    Settings->windmeter_speed_factor = (int16_t) XdrvMailbox.payload;
  }
  ResponseCmndNumber(Settings->windmeter_speed_factor);
}

void BurstControlSetManual(void) {
  if ((XdrvMailbox.payload >= -1) && (XdrvMailbox.payload <= 100)) {
    manual_power = (signed int) XdrvMailbox.payload;
  }
  ResponseCmndNumber(manual_power);
}

// debug
void BurstControlNetPowerOverride(void) {
  if ((XdrvMailbox.payload >= -5000) && (XdrvMailbox.payload <= 5000)) {
    net_power_override = (signed int) XdrvMailbox.payload;
  }
  ResponseCmndNumber(net_power_override);
}

// With core v2.0.0+, you can't use Serial.print/println in ISR or crash.
// and you can't use float calculation inside ISR
// Only OK in core v1.0.6-
bool IRAM_ATTR TimerHandler0(void * timerNo)
{ 
  static int count = 0;

  if (pwm_fraction==0) {
    digitalWrite(Pin(GPIO_BURST_CONTROL_PWM), BURST_CTRL_PINSTATE_OFF);   // switch off solid state relay
  }
  else if (count<pwm_fraction) {
    digitalWrite(Pin(GPIO_BURST_CONTROL_PWM), BURST_CTRL_PINSTATE_ON);    // switch on solid state relay
  }
  else {
    digitalWrite(Pin(GPIO_BURST_CONTROL_PWM), BURST_CTRL_PINSTATE_OFF);   // switch off solid state relay
  }
  
  count++;
  if (count >= ctrl_steps) count = 0;

  return true;
}


void BurstControlInit(void) {
  if (PinUsed(GPIO_BURST_CONTROL_PWM)) {

    AddLog(LOG_LEVEL_INFO, PSTR("xdrv: Burst Control Init"));
    // Interval in microsecs
    //if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1000, TimerHandler0))
    if (ITimer0.attachInterruptInterval(TIMER0_INTERVAL_MS * 1010, TimerHandler0))
    {
      Serial.print(F("Starting  ITimer0 OK, millis() = ")); Serial.println(millis());
    }
    else
      Serial.println(F("Can't set ITimer0. Select another freq. or timer"));

    ctrl_gate_periode = (uint16_t) Settings->windmeter_radius;
    // if ((ctrl_gate_periode >= 0) && (ctrl_gate_periode <= 400)) {
    //   ctrl_gate_periode = POWER_CTRL_PERIODE_LEN_DEFAULT;
    //   Settings->windmeter_radius = POWER_CTRL_PERIODE_LEN_DEFAULT;
    // }

    ctrl_heater_power = (uint16_t) Settings->pms_wake_interval;
    // if ((ctrl_heater_power >= 100) && (ctrl_heater_power <= 5000)) {
    //   ctrl_heater_power = POWER_CTRL_HEATER_POWER_DEFAULT;
    //   Settings->pms_wake_interval = POWER_CTRL_HEATER_POWER_DEFAULT;
    // }

    ctrl_net_power_limit = (int16_t) Settings->windmeter_speed_factor;
    // if ((ctrl_net_power_limit >= -5000) && (ctrl_net_power_limit <= 5000)) {
    //   ctrl_net_power_limit = POWER_CTRL_NET_POWER_LIMIT;
    //   Settings->windmeter_speed_factor = POWER_CTRL_NET_POWER_LIMIT;
    // }
    
    if(ctrl_gate_periode > TIMER0_INTERVAL_MS) {
      ctrl_steps = ctrl_gate_periode / TIMER0_INTERVAL_MS;
      ctrl_load_step = ctrl_heater_power / ctrl_steps;
    }
    else {
      ctrl_steps = 10;
      ctrl_load_step = ctrl_heater_power / ctrl_steps;
    }

    pinMode(Pin(GPIO_BURST_CONTROL_PWM), OUTPUT);
    digitalWrite(Pin(GPIO_BURST_CONTROL_PWM), LOW);

  }  
}


/*********************************************************************************************\
 * Presentation
\*********************************************************************************************/

#ifdef USE_WEBSERVER

#define WEB_HANDLE_BURST_CONTROL "pvheater"

#define D_CONFIGURE_BURST_CONTROL "PV Heizer konfigurieren"
#define D_BURST_CONTROL_PARAMETERS "PV Heizer Parameter"
#define D_BURST_CONTROL_PERIODE_LEN "Periodendauer [ms]"
#define BURST_CONTROL_PERIODE_LEN_DEFAULT "200"
#define D_BURST_CONTROL_NET_POWER_LIMIT "Einspeisegrenze [W]"
#define BURST_CONTROL_NET_POWER_LIMIT_LEN_DEFAULT "-200"
#define D_BURST_CONTROL_HEATER_POWER "Heizstab Leistung [W]"
#define BURST_CONTROL_HEATER_POWER_DEFAULT "1500"

const char S_CONFIGURE_BURST_CONTROL[] PROGMEM = D_CONFIGURE_BURST_CONTROL;

const char HTTP_BTN_MENU_BURST_CONTROL[] PROGMEM =
  "<p><form action='" WEB_HANDLE_BURST_CONTROL "' method='get'><button>" D_CONFIGURE_BURST_CONTROL "</button></form></p>";

const char HTTP_FORM_BURST_CONTROL1[] PROGMEM =
  "<fieldset><legend><b>&nbsp;" D_BURST_CONTROL_PARAMETERS "&nbsp;</b></legend>"
  "<form method='get' action='" WEB_HANDLE_BURST_CONTROL "'>"
  "<p><b>" D_BURST_CONTROL_PERIODE_LEN "</b> (" BURST_CONTROL_PERIODE_LEN_DEFAULT ")<br><input id='pl' placeholder=\"" BURST_CONTROL_PERIODE_LEN_DEFAULT "\" value=\"%d\"></p>"
  "<p><b>" D_BURST_CONTROL_NET_POWER_LIMIT "</b> (" BURST_CONTROL_NET_POWER_LIMIT_LEN_DEFAULT ")<br><input id='nl' placeholder=\"" BURST_CONTROL_NET_POWER_LIMIT_LEN_DEFAULT "\" value=\"%d\"></p>"
  "<p><b>" D_BURST_CONTROL_HEATER_POWER "</b> (" BURST_CONTROL_HEATER_POWER_DEFAULT ")<br><input id='hp' placeholder=\"" BURST_CONTROL_HEATER_POWER_DEFAULT "\" value=\"%d\"></p>";
  //"<p><b>" D_PORT "</b> (" STR(MQTT_PORT) ")<br><input id='ml' placeholder='" STR(MQTT_PORT) "' value='%d'></p>"


const char HTTP_MSG_SLIDER_BURST[] PROGMEM =  
  "<div name='%s' style='background-image:linear-gradient(to right,%s,%s);'>"
  "<form method='get' action='pvh' id='pvhf'>"
  "<input name='manual%d' type='range' min='%d' max='%d' value='%d' onchange='document.getElementById(\"pvhf\").submit();'>"
  "</form>"
  "</div>";

  //   "<div id='%s' class='r' style='background-image:linear-gradient(to right,%s,%s);'>"
  // "<form method='get' action='mlx'>"
  // "<input id='sl%d' type='range' min='%d' max='%d' value='%d' onchange='lc(\"%c\",%d,value)'>"
  // "</form>"
  // "</div>";



void HandleBurstControlWebGui(void){
  if (!HttpCheckPriviledgedAccess()) { return; }
  //MLX90640HandleWebGuiResponse();
  //MLX90640UpdateGUI();

  char tmp[20]; 
  WebGetArg("sl5", tmp, sizeof(tmp));                  // update line
  if (strlen(tmp)) {
      Serial.println("get sl5");
    }


  Serial.println("get mlx");
}



void HandleBurstControlConfiguration(void)
{
   if (!HttpCheckPriviledgedAccess()) { return; }

  AddLog(LOG_LEVEL_DEBUG, PSTR(D_LOG_HTTP D_CONFIGURE_BURST_CONTROL));

  if (Webserver->hasArg(F("save"))) {
    BurstControlSaveSettings();
    WebRestart(1);
    return;
  }

  char str[TOPSZ];

  WSContentStart_P(PSTR(D_CONFIGURE_BURST_CONTROL));
  WSContentSendStyle();
  WSContentSend_P(HTTP_FORM_BURST_CONTROL1,
    Settings->windmeter_radius,
    Settings->windmeter_speed_factor,
    Settings->pms_wake_interval
    );
  WSContentSend_P(HTTP_FORM_END);
  WSContentSpaceButton(BUTTON_CONFIGURATION);
  WSContentStop();
}


void BurstControlSaveSettings(void) {
  String cmnd = F(D_CMND_BACKLOG "0 ");
  cmnd += AddWebCommand(PSTR(D_CMND_BURST_CONTROL_PERIODE), PSTR("pl"), PSTR("1"));
  cmnd += AddWebCommand(PSTR(D_CMND_BURST_NET_POWER_LIMIT), PSTR("nl"), PSTR("1"));
  cmnd += AddWebCommand(PSTR(D_CMND_BURST_HEATER_POWER), PSTR("hp"), PSTR("1"));

  ExecuteWebCommand((char*)cmnd.c_str());
}


#endif


#ifdef USE_WEBSERVER
const char HTTP_SNS_BURST_DATA[] PROGMEM =
  //"{s}%s " D_VOLTAGE "{m}%s " D_UNIT_VOLT "{e}"
  //"{s}%s " D_CURRENT "{m}%s " D_UNIT_AMPERE "{e}"
  "{s}%s " D_BURST_CONTROL_HEATER "{m}%s " D_UNIT_PERCENT "{e}"
  "{s}%s " D_BURST_CONTROL_NETPOWER "{m}%s " D_UNIT_WATT "{e}";
#endif  // USE_WEBSERVER

void BurstControlShow(bool json)
{
    // char voltage[16];
    // dtostrfd(123, Settings->flag2.voltage_resolution, voltage);
    // char current[16];
    // dtostrfd(456, Settings->flag2.current_resolution, current);
    
    char heaterpercent[16];
    dtostrfd(actual_power, 0, heaterpercent);
    char netpower[16];
    dtostrfd(actual_active_power, Settings->flag2.wattage_resolution, netpower);
    
    char name[16];    
    snprintf_P(name, sizeof(name), PSTR("%s"), D_BURST_CONTROL_NAME);

    if (json) {
      ResponseAppend_P(PSTR(",\"%s\":{\"Id\":%02x,\"" D_JSON_USAGE "\":%s,\"" D_JSON_ACTIVE_POWERUSAGE "\":%s}"),
                       name, 1, heaterpercent, netpower);
#ifdef USE_DOMOTICZ
      if (0 == TasmotaGlobal.tele_period) {
        DomoticzSensor(DZ_COUNT, heaterpercent);
        DomoticzSensor(DZ_POWER_ENERGY, netpower);
      }
#endif  // USE_DOMOTICZ
#ifdef USE_WEBSERVER
    } else {
      WSContentSend_PD(HTTP_SNS_BURST_DATA, name, heaterpercent, name, netpower);

      // WSContentSend_P(HTTP_MSG_SLIDER_BURST,  // Brightness - Black to White
      //     PSTR("c"),               // c - Unique HTML id
      //     PSTR("#000"), PSTR("#fff"),    // Black to White
      //     5,                 // sl4 - Unique range HTML id - Used as source for Saturation begin color
      //     Settings->flag3.slider_dimmer_stay_on, 100,  // Range 0/1 to 100% (SetOption77 - Do not power off if slider moved to far left)
      //     Settings->light_dimmer,
      //     'd', 0);           // d0 - Value id is related to lc("d0", value) and WebGetArg("d0", tmp, sizeof(tmp));


#endif  // USE_WEBSERVER
    }
  
}



void power_ctrl_cycle (int enable) {

  static unsigned int power_integrator = 0;
  static unsigned int zero_power_cnt = 0;
  static unsigned int ctrldelay = 0;

  if (net_power_override == 0) {
    actual_active_power = (signed int) (1 * Energy->active_power[0]);
  }
  else {
    actual_active_power = net_power_override;
    Serial.print("Net Power Override: "); Serial.print(actual_active_power, DEC); Serial.println(" W");
  }

  // manual or controlled mode?
  if (manual_power == -1) {      // controlled mode

    ctrldelay++;
    if(ctrldelay >= POWER_CTRL_DELAY) {
      ctrldelay = 0;

      // fast stop if no sun power is available (over setting net power limit)
      if (actual_active_power >= ctrl_net_power_limit) {        
        zero_power_cnt++;
        if (zero_power_cnt > 2) {    // 2 seconds dead time
          zero_power_cnt = 0;
          power_integrator = 0;
        }
      }
      // fast lane - if accessible power is bigger than half of heater power demand jump right in
      // else if ((actual_active_power < -(ctrl_heater_power/2)) && (power_integrator < (ctrl_gate_periode / 10 / 2) - 1)) {
      //   power_integrator = (ctrl_gate_periode / 10 / 2) - 1;
      //   zero_power_cnt = 0;
      // }    
      // slow increase of power if available
      else if (actual_active_power < -(-ctrl_net_power_limit + ctrl_load_step + POWER_CTRL_MARGIN_INC)) {
        power_integrator++;
        zero_power_cnt = 0;
      }
      // slow decrease of power
      else if (actual_active_power > -(-ctrl_net_power_limit + ctrl_load_step + POWER_CTRL_MARGIN_DEC)) {
        if (power_integrator > 0) power_integrator--;
        zero_power_cnt = 0;
      }
      else {
        //power_integrator = 0;
      }
    }

      // limit
      if(power_integrator >= ctrl_steps) power_integrator = ctrl_steps;

      pwm_fraction = power_integrator;

      // actual power in percent
      actual_power = (100 * power_integrator) / ctrl_steps;

  }
  else {  // manual mode overrides max value and control mode
      pwm_fraction = (manual_power) * ctrl_gate_periode / 100 / TIMER0_INTERVAL_MS;
      actual_power = manual_power;

      Serial.print("Manual Setting: "); Serial.print(manual_power, DEC); Serial.println(" %");
  }

  // if display is available, switch on LCD light when heater is working  
  if (0x01 == Settings->display_model) {
    //Serial.println("Display ready");
    if (actual_power > 0) lcd->backlight();
    else lcd->noBacklight();
  }


  AddLog(LOG_LEVEL_INFO, PSTR("PV Heizer: "
     "perc: %d"),
     actual_power);


  if (1) {
    Response_P(PSTR("{\"%s\":{"), "BURSTCTRL");
    ResponseAppend_P(PSTR("\"%s\":%d"), "power", actual_power);
    ResponseJsonEndEnd();

    XdrvRulesProcess(0);
  }

  
}


/*********************************************************************************************\
 * Interface
\*********************************************************************************************/

#define XDRV_91       91

bool Xdrv91(uint32_t function) {
  bool result = false;

  switch (function) {
    case FUNC_INIT:
      BurstControlInit();
      break;
    case FUNC_EVERY_SECOND:      
      power_ctrl_cycle(1);
      break;
    case FUNC_SET_POWER:
      //TimepropXdrvPower();
      break;
    case FUNC_JSON_APPEND:
        BurstControlShow(1);
        break;
    case FUNC_COMMAND:
        result = DecodeCommand(kBurstControlCommands, BurstControlCommand);
        break;
  #ifdef USE_WEBSERVER
      case FUNC_WEB_SENSOR:
        BurstControlShow(0);
        break;
      case FUNC_WEB_ADD_BUTTON:
        WSContentSend_P(HTTP_BTN_MENU_BURST_CONTROL);
        break;
      case FUNC_WEB_ADD_HANDLER:
        WebServer_on(PSTR("/" WEB_HANDLE_BURST_CONTROL), HandleBurstControlConfiguration);
        //WebServer_on(PSTR("/pvh"), HandleBurstControlWebGui);
        break;
  #endif  // USE_WEBSERVER

  }
  return result;
}

#endif // USE_BURST_CONTROL
