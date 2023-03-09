#include <Arduino.h>
#include <FastLED.h>

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <EEPROM.h>
#include <AutoConnect.h>

#include <Ticker.h>
// #include <TimerInterrupt_Generic.h>

// key change step
#define KEY_CHANGE_STEP     5

//battery val
#define BATTERY_VOL_MAX     4200
#define BATTERY_VOL_MIN     3300

//TIMER
#define TIMER_INTERVAL_MS   1000

//motor
#define MOTOR_MIN_PWM       300
#define MOTOR_MAX_PWM       1000
#define MOTOR_MIN_SPEED     10
#define MOTOR_MAX_SPEED     100

//pin map
//RGB LED
#define RGB_PIN             4
//BUTTON
#define BUTTON_LEFT_PIN     5
#define BUTTON_MIDDLE_PIN   16
#define BUTTON_RIGHT_PIN    14
//MOTOR
#define MOTOR_AIN1_PIN      12
#define MOTOR_AIN2_PIN      13
//BATTERY VALUE
#define BAT_VAL_PIN         A0

//RGB LED
#define NUM_RGB_LEDS        2
CRGB leds[NUM_RGB_LEDS];
uint8_t max_bright = 0;

//WEB SERVER
ESP8266WebServer    server;
AutoConnect         portal(server);
AutoConnectConfig   config;

//BATTERY PERCENT
int battery_percent = 0;

Ticker timer1;

//hand mode rgb
int hand_mode_rgb[19][3] =
{
    { 220, 20, 60 },
    { 199, 21, 133 },
    { 219, 112, 147 },
    { 255, 105, 180 },
    { 255, 192, 203 },
    { 238, 130, 238 },
    { 221, 160, 221 },
    { 216, 191, 216 },
    { 255, 240, 245 },
    { 153, 50, 204 },
    { 147, 112, 219 },
    { 100, 149, 237 },
    { 65, 105, 225 },
    { 123, 104, 238 },
    { 106, 90, 205 },
    { 72, 61, 139 },
    { 0, 0, 205 },
    { 25, 25, 112 },
    { 0, 0, 139 },
};

const char AUX_EEPROM_IO[] PROGMEM = R"(
[{
    "title": "FAN CFG",
    "uri": "/cfgs",
    "menu": true,
    "element": [{
        "name": "fanpower",
        "type": "ACText",
        "posterior": "br",
        "style": "color:green",
        "format": "电量: %s"
      },
      {
        "name": "fanswitch",
        "type": "ACInput",
        "label": "电源 0关 1开",
        "pattern": "[0-1]",
        "global": true
      },
      {
        "name": "fanmode",
        "type": "ACInput",
        "label": "模式 0手动 1自动",
        "pattern": "[0-1]",
        "global": true
      },
      {
        "name": "hdspeed",
        "type": "ACInput",
        "label": "手动速度 10-100",
        "pattern": "^([1-9][0-9]|100|all)$",
        "global": true
      },
      {
        "name": "atmax",
        "type": "ACInput",
        "label": "最大速度 10-100",
        "pattern": "^([1-9][0-9]|100|all)$",
        "global": true
      },
      {
        "name": "atmin",
        "type": "ACInput",
        "label": "最小速度 10-100",
        "pattern": "^([1-9][0-9]|100|all)$",
        "global": true
      },
      {
        "name": "attime",
        "type": "ACInput",
        "label": "循环时间 1-60s",
        "pattern": "^([1-9]|[1-5][0-9]|60|all)$",
        "global": true
      },
      {
        "name": "rgbbright",
        "type": "ACInput",
        "label": "亮度 0-255",
        "pattern": "^(25[0-5]|2[0-4][0-9]|[0-1]?[0-9]?[0-9])$",
        "global": true
      },
      {
        "name": "write",
        "type": "ACSubmit",
        "value": "设置",
        "uri": "/cfg"
      }
    ]
  },
  {
    "title": "FAN CFG",
    "uri": "/cfg",
    "menu": false,
    "element": [{
        "name": "fanpower",
        "type": "ACText",
        "posterior": "br",
        "format": "电量: %s"
      },
      {
        "name": "fanswitch",
        "type": "ACText",
        "format": "电源 0关 1开: %s",
        "posterior": "br",
        "global": true
      },
      {
        "name": "fanmode",
        "type": "ACText",
        "format": "模式 0手动 1自动: %s",
        "posterior": "br",
        "global": true
      },
      {
        "name": "hdspeed",
        "type": "ACText",
        "format": "手动速度: %s",
        "posterior": "br",
        "global": true
      },
      {
        "name": "atmax",
        "type": "ACText",
        "format": "最大速度: %s",
        "posterior": "br",
        "global": true
      },
      {
        "name": "atmin",
        "type": "ACText",
        "format": "最小速度: %s",
        "posterior": "br",
        "global": true
      },
      {
        "name": "attime",
        "type": "ACText",
        "format": "循环时间: %s",
        "posterior": "br",
        "global": true
      },
      {
        "name": "rgbbright",
        "type": "ACText",
        "format": "亮度: %s",
        "posterior": "br",
        "global": true
      }
    ]
  }
]
)";

// AUTO connect write in AutoConnectDfs.h
// #define AUTOCONNECT_APID        "KIR_FAN"
// #define AUTOCONNECT_HOMEURI     "/_ac"
// #define AUTOCONNECT_MENU_TITLE  "KirFan"
// #define AUTOCONNECT_TIMEOUT     10000   //ms

// Defines the custom data should be stored in EEPROM.
typedef struct
{
    int fan_power; //0:off 1:on
    int factory_state;
    int fan_mode;  //0:hand_mode 1:auto_mode
    int auto_mode_max_speed;
    int auto_mode_min_speed;
    int auto_mode_time;
    int hand_mode_speed;
    int rgb_level; //0:off 1-255:brightness
    int battery_level;
    int wifi_func;
} EEPROM_CONFIG_t;

EEPROM_CONFIG_t eepromConfig;

void model_eeprom_init(void)
{
    EEPROM.begin(sizeof(eepromConfig));
    EEPROM.get<EEPROM_CONFIG_t>(0, eepromConfig);

    if(eepromConfig.factory_state != 999)
    {
        eepromConfig.fan_power = 0;
        eepromConfig.fan_mode = 0;
        eepromConfig.auto_mode_max_speed = 80;
        eepromConfig.auto_mode_min_speed = 20;
        eepromConfig.auto_mode_time = 40; //s
        eepromConfig.hand_mode_speed = MOTOR_MAX_SPEED / 2;
        eepromConfig.rgb_level = 128;
        eepromConfig.factory_state = 999;
        eepromConfig.wifi_func = 1; //1:open 0:close

        EEPROM.put<EEPROM_CONFIG_t>(0, eepromConfig);
        EEPROM.commit();
    }
    EEPROM.end();

    //eepromConfig.fan_power = 0;
}

void model_eeprom_show(void)
{
    Serial.printf("fan_power:%d\r\n", eepromConfig.fan_power);
    Serial.printf("fan_mode:%d\r\n", eepromConfig.fan_mode);
    Serial.printf("auto_mode_max_speed:%d\r\n", eepromConfig.auto_mode_max_speed);
    Serial.printf("auto_mode_min_speed:%d\r\n", eepromConfig.auto_mode_min_speed);
    Serial.printf("auto_mode_time:%d\r\n", eepromConfig.auto_mode_time);
    Serial.printf("hand_mode_speed:%d\r\n", eepromConfig.hand_mode_speed);
    Serial.printf("rgb_level:%d\r\n", eepromConfig.rgb_level);
    Serial.printf("factory_state:%d\r\n", eepromConfig.factory_state);
    Serial.printf("wifi_func:%d\r\n", eepromConfig.wifi_func);
}

void model_eeprom_write(void)
{
    EEPROM.begin(sizeof(eepromConfig));
    EEPROM.put<EEPROM_CONFIG_t>(0, eepromConfig);
    EEPROM.commit();
    EEPROM.end();
}

void battery_vol_check(void)
{
    int bat_vol = analogRead(A0) * 100 / 1024 * 57;

    if(bat_vol >= BATTERY_VOL_MAX)
    {
        battery_percent = 100;
    }
    else if(bat_vol <= BATTERY_VOL_MIN)
    {
        battery_percent = 0;
    }
    else
    {
        battery_percent = (bat_vol - BATTERY_VOL_MIN) / ((BATTERY_VOL_MAX - BATTERY_VOL_MIN) / 100); //简单百分比计算电量，图一乐，不准
    }
    Serial.printf("bat:%d\r\n", battery_percent);

    if(battery_percent<3)
    {
        eepromConfig.fan_power = 0;
        motor_pwm_set();
        model_modelight_set();
        model_wifilight_set();
        model_eeprom_write();
        Serial.println("Power off!");
    }
}

void model_modelight_set(void)
{
    if(eepromConfig.fan_power == 0)
    {
        leds[0] = CRGB(0, 0, 0); //不亮
    }
    else
    {
        if(eepromConfig.fan_mode == 1)
        {
            leds[0] = CRGB(34, 139, 34); //PaleGreen
        }
        else
        {
            int r = hand_mode_rgb[(int)((eepromConfig.hand_mode_speed - MOTOR_MIN_SPEED) / KEY_CHANGE_STEP)][0];
            int g = hand_mode_rgb[(int)((eepromConfig.hand_mode_speed - MOTOR_MIN_SPEED) / KEY_CHANGE_STEP)][1];
            int b = hand_mode_rgb[(int)((eepromConfig.hand_mode_speed - MOTOR_MIN_SPEED) / KEY_CHANGE_STEP)][2];

            leds[0] = CRGB(r, g, b);
        }
    }
}

void model_wifilight_set(void)
{
    if(eepromConfig.fan_power == 0)
    {
        leds[1] = CRGB(0, 0, 0); //不亮
    }
    else
    {
        if(eepromConfig.wifi_func == 0)
        {
            leds[1] = CRGB(0, 0, 0); //不亮
        }
        else
        {
            if(WiFi.status() == WL_CONNECTED)
            {
                leds[1] = CRGB( 30, 144, 255 );
            }
            else
            {
                leds[1] = CRGB( 255, 20, 147 );
            }
        }
    }
    FastLED.show(); // 更新LED色彩
}

int auto_mode_calc = 0;
void motor_pwm_set(void)
{
    if(eepromConfig.fan_power == 0)
    {
        analogWrite(MOTOR_AIN1_PIN, 0);
        auto_mode_calc = 0;
    }
    else
    {
        if(eepromConfig.fan_mode == 0)
        {
            int pwm = 0;
            int range = (MOTOR_MAX_PWM - MOTOR_MIN_PWM) / (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED) + 1;
            pwm = MOTOR_MIN_PWM + ((eepromConfig.hand_mode_speed - MOTOR_MIN_SPEED) * range);
            if(pwm > MOTOR_MAX_PWM)
            {
                pwm = MOTOR_MAX_PWM;
            }
            analogWrite(MOTOR_AIN1_PIN, pwm);
            //Serial.printf("pwm:%d\r\n",pwm);
        }
        else
        {
            analogWrite(MOTOR_AIN1_PIN, 0);
            auto_mode_calc = 0;
        }
    }
}

void motor_auto_pwm_set(void)
{
    if(eepromConfig.fan_power == 1 && eepromConfig.fan_mode == 1 && eepromConfig.auto_mode_max_speed > eepromConfig.auto_mode_min_speed)
    {
        int n_pwm = 0;
        int range = (MOTOR_MAX_PWM - MOTOR_MIN_PWM) / (MOTOR_MAX_SPEED - MOTOR_MIN_SPEED) + 1;
        int range_max_pwm = MOTOR_MIN_PWM + ((eepromConfig.auto_mode_max_speed - MOTOR_MIN_SPEED) * range);
        int range_min_pwm = MOTOR_MIN_PWM + ((eepromConfig.auto_mode_min_speed - MOTOR_MIN_SPEED) * range);
        if(range_max_pwm > MOTOR_MAX_PWM)
        {
            range_max_pwm = MOTOR_MAX_PWM;
        }
        if(range_min_pwm > MOTOR_MAX_PWM)
        {
            range_min_pwm = MOTOR_MAX_PWM;
        }

        int range_t = (range_max_pwm - range_min_pwm) / (eepromConfig.auto_mode_time / 2);
        if(auto_mode_calc < (eepromConfig.auto_mode_time / 2))
        {
            n_pwm = range_min_pwm + range_t *auto_mode_calc;
        }
        else
        {
            n_pwm = range_max_pwm - range_t *(auto_mode_calc - (eepromConfig.auto_mode_time / 2));
        }
        //Serial.printf("pwm:%d\r\n",n_pwm);
        analogWrite(MOTOR_AIN1_PIN, n_pwm);
        auto_mode_calc++;
        if(auto_mode_calc >= eepromConfig.auto_mode_time)
        {
            auto_mode_calc = 0;
        }
    }
}

int key_count_l = 0;
int key_long_press_l = 0;
int key_count_m = 0;
int key_long_press_m = 0;
int key_count_r = 0;
int key_long_press_r = 0;
void key_process(void)
{
    if(digitalRead(BUTTON_LEFT_PIN) == 1)
    {
        if(key_count_l >= 50 && key_count_l <= 150)
        {
            if(eepromConfig.fan_mode == 0 && eepromConfig.fan_power == 1) //hand mode
            {
                eepromConfig.hand_mode_speed = eepromConfig.hand_mode_speed + KEY_CHANGE_STEP;
                if(eepromConfig.hand_mode_speed > MOTOR_MAX_SPEED)
                {
                    eepromConfig.hand_mode_speed = MOTOR_MAX_SPEED;
                }
                motor_pwm_set();
                model_modelight_set();
                model_eeprom_write();
            }

            //fast
            Serial.printf("key_contlf:%d %d\r\n", key_count_l, eepromConfig.hand_mode_speed);
        }
        key_count_l = 0;
        key_long_press_l = 0;
    }
    else
    {
        if(key_long_press_l != 1)
        {
            key_count_l++;
        }
        // delay(10);
        //重置
        if(key_count_l >= 1500)
        {
            Serial.printf("key_contls:%d\r\n", key_count_l);
            key_count_l = 0;
            key_long_press_l = 1;
            //slow
            if(eepromConfig.fan_power == 1)
            {
                if(eepromConfig.wifi_func == 1)
                {
                    eepromConfig.wifi_func = 0;
                }
                else
                {
                    eepromConfig.wifi_func = 1;
                }
                model_eeprom_write();

                Serial.println("Restart!");
                ESP.reset();
            }
        }
    }

    if(digitalRead(BUTTON_MIDDLE_PIN) == 1)
    {
        if(key_count_m >= 50 && key_count_m <= 150)
        {
            if(eepromConfig.fan_power == 1)
            {
                eepromConfig.fan_power = 0;
            }
            else
            {
                eepromConfig.fan_power = 1;
            }
            motor_pwm_set();
            model_modelight_set();
            model_wifilight_set();
            model_eeprom_write();
            //fast
            Serial.printf("key_contmf:%d %d\r\n", key_count_m, eepromConfig.fan_power);
        }
        key_count_m = 0;
        key_long_press_m = 0;
    }
    else
    {
        if(key_long_press_m != 1)
        {
            key_count_m++;
        }
        // delay(10);
        //重置
        if(key_count_m >= 1000)
        {
            key_count_m = 0;
            key_long_press_m = 1;
            //slow
            if(eepromConfig.fan_power == 1)
            {
                if(eepromConfig.fan_mode == 1)
                {
                    eepromConfig.fan_mode = 0;
                }
                else
                {
                    eepromConfig.fan_mode = 1;
                }
            }
            motor_pwm_set();
            model_modelight_set();
            model_eeprom_write();
            //fast
            Serial.printf("key_contml:%d %d\r\n", key_count_m, eepromConfig.fan_mode);
        }
    }

    if(digitalRead(BUTTON_RIGHT_PIN) == 1)
    {
        if(key_count_r >= 50 && key_count_r <= 150)
        {
            if(eepromConfig.fan_mode == 0 && eepromConfig.fan_power == 1) //hand mode
            {
                eepromConfig.hand_mode_speed = eepromConfig.hand_mode_speed - KEY_CHANGE_STEP;
                if(eepromConfig.hand_mode_speed < MOTOR_MIN_SPEED)
                {
                    eepromConfig.hand_mode_speed = MOTOR_MIN_SPEED;
                }
                motor_pwm_set();
                model_modelight_set();
                model_eeprom_write();
            }

            //fast
            Serial.printf("key_contrf:%d %d\r\n", key_count_r, eepromConfig.hand_mode_speed);
        }
        key_count_r = 0;
        key_long_press_r = 0;
    }
    else
    {
        if(key_long_press_r != 1)
        {
            key_count_r++;
        }
        // delay(10);
        //重置
        if(key_count_r >= 1500)
        {
            Serial.printf("key_contrs:%d\r\n", key_count_r);
            key_count_r = 0;
            key_long_press_r = 1;
            //slow
            // void AutoConnect::disconnect(void) //write in autoconnect.c
            // {
            //   _rfDisconnect = true;
            // }

            portal.disconnect();
            Serial.println("Reset wifi!");
        }
    }
}

// fanpower
// fanmode
// atmax
// atmin
// attime
// rgbbright
// Read from EEPROM
String onEEPROM(AutoConnectAux &page, PageArgument &args)
{
    //  EEPROM_CONFIG_t eepromConfig;
    char c_fanpower[4];
    char c_fanswitch[2];
    char c_fanmode[2];
    char c_hdspeed[4];
    char c_atmax[4];
    char c_atmin[4];
    char c_attime[4];
    char c_rgbbright[4];

    String fanpower_c;
    String fanswitch_c;
    String fanmode_c;
    String hdspeed_c;
    String atmax_c;
    String atmin_c;
    String attime_c;
    String rgbbright_c;

    sprintf(c_fanpower, "%d", battery_percent);
    sprintf(c_fanswitch, "%d", eepromConfig.fan_power);
    sprintf(c_fanmode, "%d", eepromConfig.fan_mode);
    sprintf(c_hdspeed, "%d", eepromConfig.hand_mode_speed);
    sprintf(c_atmax, "%d", eepromConfig.auto_mode_max_speed);
    sprintf(c_atmin, "%d", eepromConfig.auto_mode_min_speed);
    sprintf(c_attime, "%d", eepromConfig.auto_mode_time);
    sprintf(c_rgbbright, "%d", eepromConfig.rgb_level);

    fanpower_c = c_fanpower;
    fanswitch_c = c_fanswitch;
    fanmode_c = c_fanmode;
    hdspeed_c = c_hdspeed;
    atmax_c = c_atmax;
    atmin_c = c_atmin;
    attime_c = c_attime;
    rgbbright_c = c_rgbbright;

    page["fanpower"].value = fanpower_c;
    page["fanswitch"].value = fanswitch_c;
    page["fanmode"].value = fanmode_c;
    page["hdspeed"].value = hdspeed_c;
    page["atmax"].value = atmax_c;
    page["atmin"].value = atmin_c;
    page["attime"].value = attime_c;
    page["rgbbright"].value = rgbbright_c;
    return String();
}

// Write to EEPROM
String onEEPROMWrite(AutoConnectAux& page, PageArgument& args) 
{
    eepromConfig.fan_power = page["fanswitch"].value.toInt();
    eepromConfig.fan_mode = page["fanmode"].value.toInt();
    eepromConfig.hand_mode_speed = page["hdspeed"].value.toInt();
    eepromConfig.auto_mode_max_speed = page["atmax"].value.toInt();
    eepromConfig.auto_mode_min_speed = page["atmin"].value.toInt();
    eepromConfig.auto_mode_time = page["attime"].value.toInt();
    eepromConfig.rgb_level = page["rgbbright"].value.toInt();

    // The actual area size of the EEPROM region to be given to
    // EEPROM.begin is the sum of the size of the own custom data and
    // the size of the currently stored AutoConnect credentials.
    // eg.
    //   EEPROM.begin(portal.getEEPROMUsedSize())
    model_eeprom_write();

    FastLED.setBrightness(eepromConfig.rgb_level);
    motor_pwm_set();
    model_modelight_set();
    model_wifilight_set();
            
    return String();
}

int bat_timer_calc = 0;
void TimerHandler(void)
{
    //bat
    bat_timer_calc++;
    if(bat_timer_calc >= 60) //1min
    {
        bat_timer_calc = 0;
        battery_vol_check();
    }

    //wifi rgb
    model_wifilight_set();

    //motor auto
    motor_auto_pwm_set();
}

void model_timer_init(void)
{
    timer1.attach(1, TimerHandler);
}

void model_hw_init(void)
{
    //uart init
    Serial.begin(115200);

    //IO init
    pinMode(BUTTON_LEFT_PIN, INPUT_PULLUP);
    pinMode(BUTTON_MIDDLE_PIN, INPUT_PULLUP);
    pinMode(BUTTON_RIGHT_PIN, INPUT_PULLUP);

    //rgb init
    FastLED.addLeds<NEOPIXEL, RGB_PIN>(leds, NUM_RGB_LEDS);
    FastLED.setBrightness(eepromConfig.rgb_level);
    //FastLED.setBrightness(0);

    analogWriteFreq(20000); //20k
    analogWriteRange(MOTOR_MAX_PWM);
    pinMode(MOTOR_AIN1_PIN, OUTPUT);
    analogWrite(MOTOR_AIN1_PIN, 0);
    pinMode(MOTOR_AIN2_PIN, INPUT);
    //digitalWrite(MOTOR_AIN2_PIN, LOW);
    
    battery_vol_check();
}

void setup()
{
    // put your setup code here, to run once:
    model_eeprom_init();
    model_hw_init();
    model_timer_init();

    model_modelight_set();
    model_wifilight_set();
    motor_pwm_set();

    if(eepromConfig.wifi_func == 1)
    {
        portal.load(FPSTR(AUX_EEPROM_IO));
        portal.on("/cfgs", onEEPROM);
        portal.on("/cfg", onEEPROMWrite);

        config.ota = AC_OTA_BUILTIN;
        config.boundaryOffset = sizeof(EEPROM_CONFIG_t);
        portal.config(config);
        portal.begin();
    }

    Serial.print("FAN Start\r\n");
    model_eeprom_show();
}

void loop()
{
    // put your main code here, to run repeatedly:
    if(eepromConfig.wifi_func == 1)
    {
        //web server handle
        portal.handleClient();
    }

    key_process();

    FastLED.show();                // 更新LED色彩
}
