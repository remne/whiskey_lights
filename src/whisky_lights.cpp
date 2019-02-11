/*
 * ----------------------------------------------------------------------------
 * "THE BEER-WARE LICENSE" (Revision 42):
 * <remne@mail.datamupp.com> wrote this file.  As long as you retain this notice you
 * can do whatever you want with this stuff. If we meet some day, and you think
 * this stuff is worth it, you can buy me a beer in return.
 * ----------------------------------------------------------------------------
 */

/*

Example configuration for Home Assistant

HA >= 0.84:
light:
  - platform: mqtt
    schema: json
    name: whisky_lights
    state_topic: "whisky_lights/state"
    command_topic: "whisky_lights/set"
    brightness: true
    rgb: true
    white_value: true
    color_temp: true
    qos: 0
    optimistic: false
    effect: true
    effect_list:
      - Solid
      - Xmas
      - Welcome
      - Falling
      - Fire
      - MeteorRain
      - SfIntro
      - DiscoFloor
      - DiscoFloorDistinct
      - Rainbow


HA < 0.84:
light:
  - platform: mqtt_json
    name: whisky_lights
    state_topic: "whisky_lights/state"
    command_topic: "whisky_lights/set"
    brightness: true
    rgb: true
    white_value: true
    color_temp: true
    qos: 0
    optimistic: false
    effect: true
    effect_list:
      - Solid
      - Xmas
      - Welcome
      - Falling
      - Fire
      - MeteorRain
      - SfIntro
      - DiscoFloor
      - DiscoFloorDistinct
      - Rainbow

*/

#include <PubSubClient.h>
#include <NeoPixelAnimator.h>
#include <NeoPixelBrightnessBus.h>
#include <NeoPixelBus.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

/****** Defines *******/
#define NUM_OF_LEDS_PER_SEGMENT     114
#define NUM_OF_SEGMENTS             8
#define NUM_OF_LEDS                 (NUM_OF_LEDS_PER_SEGMENT * NUM_OF_SEGMENTS)

#define ON_BOARD_LED_BLINK_DELAY    2000                      /*!< Delay in ms between led toggle */

#define WIFI_AP_SSID                ""            /*!< Wifi accesspoint SSID */
#define WIFI_AP_PASSWORD            ""               /*!< Wifi accesspoint password */
#define WIFI_HOSTNAME               "whisky_lights"           /*!< Hostname of this controller */
#define WIFI_NUM_RETRIES_REBOOT     5
#define WIFI_LOG                    "WIFI: "

#define MQTT_SERVER_IP              "192.168.0.1"             /*!< IP to an MQTT server */
#define MQTT_SERVER_PORT            1883                      /*!< Port number to an MQTT server */
#define MQTT_CLIENT_ID              "whisky_lights"           /*!< MQTT client id */
#define MQTT_SET_TOPIC              "whisky_lights/set"       /*!< MQTT set topic */
#define MQTT_STATE_TOPIC            "whisky_lights/state"     /*!< MQTT state topic */
#define MQTT_LOG                    "MQTT: "
#define MQTT_NUM_RETRIES_REBOOT     5

#define NL                          "\r\n"

#define OTA_LOG                     "LOG: "

#define LED_LOG                     "LED: "

#define LED_STATE_ON                1
#define LED_STATE_OFF               0
#define LED_INVALID_EFFECT          0xFF

#define SIZE_OF(x)                  (sizeof(x) / sizeof(x[0]))
#define MAX(a, b)                   ((a > b) ? (a) : (b))
#define MIN(a, b)                   ((a < b) ? (a) : (b))
#define ABS(x)                      (((x) < 0 ) ? -(x) : (x))

/****** Typedefs *******/

typedef struct MqttContext_st
{
  uint8_t retries;
} MqttContext;

typedef struct LedContext_st
{
  uint8_t state;
  uint8_t whiteValue;
  float colorTemp;
  uint16_t colorTempRaw;
  uint8_t brightness;
  uint8_t colorR;
  uint8_t colorG;
  uint8_t colorB;
  uint8_t effect;
  uint8_t nextEffect;
  uint8_t transition;

  uint8_t effect_state;
} LedContext;

typedef struct Effect_st
{
  const char* effect;
  void (*InitFunc)(void);
  void (*LoopFunc)(void);
} Effect;

typedef struct
{
  uint8_t a;
  uint8_t b;
} Pair;

/****** Static functions *******/

static void initBoard(void);
static void initWifi(void);
static void initOta(void);

static void initMqttClient(void);
static void mqttHandle(void);
static void mqttCallback(char* topic, byte* payload, unsigned int length);
static bool mqttParsePayload(const char *jsonData);
static void mqttSendResponse(void);

static void initSk6812(void);
static void sk6812Clear(void);

static void initLed(void);
static void ledHandle(void);

/* LED effect static functions */
static void effectSolidLoop(void);
static void effectWelcomeInit(void);
static void effectWelcomeLoop(void);

static void welcomeSpark(void);
static void welcomeFadeToWhite(void);

static void effectXmasInit(void);
static void effectXmasLoop(void);

static void effectFallingInit(void);
static void effectFallingLoop(void);

static void effectFireInit(void);
static void effectFireLoop(void);

static void effectMeteorRainInit(void);
static void effectMeteorRainLoop(void);
void meteorRain(uint8_t red, uint8_t green, uint8_t blue, uint8_t white, boolean randomDecay);

static void effectSfIntroInit(void);
static void effectSfIntroLoop(void);

static void effectDiscoFloorNoDistinctInit(void);
static void effectDiscoFloorDistinctInit(void);
static void effectDiscoFloorInit(void);
static void effectDiscoFloorLoop(void);

static void effectRainbowInit(void);
static void effectRainbowLoop(void);

/* Helper functions */
RgbwColor getColor(uint8_t segment, uint16_t pixel);
static void setColor(uint8_t segment, uint16_t pixel, RgbwColor color);
RgbwColor getColorInv(uint8_t segment, uint16_t pixel);
static void setColorInv(uint8_t segment, uint16_t pixel, RgbwColor color);

static void fadeToColor(RgbwColor& current, const RgbwColor target, const uint8_t value);
static void fadeTo(const uint16_t pixel, const RgbwColor targetColor, const uint8_t value);
static void fadeToWhite(RgbwColor& color, const uint8_t value, const uint8_t max);
static void fadeToBlack(RgbwColor& color, const uint8_t value);
static RgbwColor rgbColorPicker(bool distinct);

static void handleWifiMqtt();

/****** Globals *******/

Effect effects[] = 
{
  { "Solid",              NULL,                             &effectSolidLoop },
  { "Welcome",            &effectWelcomeInit,               &effectWelcomeLoop },
  { "Xmas",               &effectXmasInit,                  &effectXmasLoop},
  { "Falling",            &effectFallingInit,               &effectFallingLoop},
  { "Fire",               &effectFireInit,                  &effectFireLoop},
  { "MeteorRain",         &effectMeteorRainInit,            &effectMeteorRainLoop},
  { "SfIntro",            &effectSfIntroInit,               &effectSfIntroLoop},
  { "DiscoFloor",         &effectDiscoFloorNoDistinctInit,  &effectDiscoFloorLoop},
  { "DiscoFloorDistinct", &effectDiscoFloorDistinctInit,    &effectDiscoFloorLoop},
  { "Rainbow",            &effectRainbowInit,               &effectRainbowLoop},
  
};


#define NUM_OF_BOXES 6
static const Pair discoFloorBoxes[NUM_OF_SEGMENTS][NUM_OF_BOXES] {
  {
    {1, 18},
    {21, 37},
    {41, 56},
    {59, 75},
    {79, 94},
    {98, 114}
  },
  {
    {1, 18},
    {21, 37},
    {41, 56},
    {59, 75},
    {79, 94},
    {98, 114}
  },

  {
    {1, 18},
    {21, 37},
    {41, 56},
    {59, 77},
    {80, 93},
    {97, 114}
  },
  {
    {1, 18},
    {21, 37},
    {41, 56},
    {59, 77},
    {80, 93},
    {97, 114}
  },

  {
    {1, 18},
    {21, 37},
    {41, 56},
    {59, 77},
    {80, 93},
    {97, 114}
  },
  {
    {1, 18},
    {21, 37},
    {41, 56},
    {59, 77},
    {80, 93},
    {97, 114}
  },

  {
    {1, 20},
    {23, 35},
    {39, 56},
    {59, 72},
    {75, 91},
    {95, 114}
  },
  {
    {1, 20},
    {23, 35},
    {39, 56},
    {59, 72},
    {75, 91},
    {95, 114}
  }  
};


/* Pixelpin is ignored on ESP8266 with NeoEsp8266Dma800KbpsMethod. Connect to D4 on Wemos mini */
NeoPixelBus<NeoGrbwFeature, NeoEsp8266Dma800KbpsMethod> strip(NUM_OF_LEDS, 0); 

/* Pixelpin is ignored on ESP8266 with NeoEsp8266AsyncUart1800KbpsMethod. Connect to RX on Wemos mini */
//NeoPixelBus<NeoGrbwFeature, NeoEsp8266AsyncUart1800KbpsMethod> strip2(NUM_OF_LEDS, 2); 

LedContext ledContext;

long last = 0;
long prev = 0;
long now = 0;
long timeDelta;
uint8_t ledToggle = 0;
uint8_t frameCnt = 0;


WiFiClient wifiClient;

PubSubClient mqttClient(wifiClient);
MqttContext mqttContext;

/****** Static Functions *******/

static void initBoard(void)
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  Serial.printf(NL NL "Booting %s" NL, WIFI_HOSTNAME);

}

static void initWifi(void)
{
  WiFi.mode(WIFI_STA);
  Serial.printf(WIFI_LOG "Connecting to AP %s ..." NL, WIFI_AP_SSID);
  WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASSWORD);
  
  uint8_t numRetries = 0;
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    WiFi.begin(WIFI_AP_SSID, WIFI_AP_PASSWORD);
    Serial.printf(WIFI_LOG "Retrying connection (attempt %u)..." NL, numRetries);
    numRetries++;
    delay(1000);
    
    if (numRetries > WIFI_NUM_RETRIES_REBOOT)
    {
      Serial.printf(WIFI_LOG "Rebooting" NL);
      ESP.restart();
    }
  }

  Serial.printf(WIFI_LOG "Connected!" NL);
  Serial.printf(WIFI_LOG "IP address: ");
  Serial.println(WiFi.localIP());
}

static void initOta(void)
{
  ArduinoOTA.setHostname(WIFI_HOSTNAME);
  ArduinoOTA.onStart([]() 
  {
    sk6812Clear();
  });

  ArduinoOTA.onEnd([]() 
  {
    sk6812Clear();
    Serial.printf(NL OTA_LOG "Done!" NL);
    for (uint8_t i = 0; i < 100; i++)
    {
      strip.SetPixelColor(i, RgbwColor(0, 150, 0));
    }
    strip.Show();
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) 
  {
    uint8_t p = (progress / (total / 100));
    Serial.printf(OTA_LOG "Progress: %u" NL, p);

    for (uint8_t i = 0; i < p; i++)
    {
      strip.SetPixelColor(i, RgbwColor(0, 50, 100 - p));
    }
    strip.SetPixelColor(p, RgbwColor(0, 255, 0));
    strip.Show();
  });
  
  ArduinoOTA.onError([](ota_error_t error) 
  {
    Serial.printf(OTA_LOG "Error code %u " NL, error);
    switch(error)
    {
      case OTA_AUTH_ERROR:
      break;

      case OTA_BEGIN_ERROR:
      break;

      case OTA_CONNECT_ERROR:
      break;

      case OTA_RECEIVE_ERROR:
      break;

      case OTA_END_ERROR:
      break;
    }
    for (uint8_t i = 0; i < error; i++)
    {
      strip.SetPixelColor(i, RgbwColor(255, 0, 0));
    }
    strip.Show();
    delay(5000);

  });

  ArduinoOTA.begin();
}

static void initMqttClient(void)
{
  mqttClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  mqttClient.setCallback(mqttCallback);
  memset(&mqttContext, 0, sizeof(MqttContext));
  
}

static void mqttHandle(void)
{
  if (!mqttClient.connected())
  {
    Serial.printf(MQTT_LOG "Attempting connection...");
    if (mqttClient.connect(MQTT_CLIENT_ID))
    {
      Serial.printf("connected!" NL);

      mqttClient.subscribe(MQTT_SET_TOPIC);
      mqttContext.retries = 0;
    }
    else
    {
      Serial.printf("failed, rc=%u" NL, mqttClient.state());
      mqttContext.retries++;

      if (mqttContext.retries > MQTT_NUM_RETRIES_REBOOT)
      {
        Serial.printf(MQTT_LOG "Restarting due to failed MQTT server connection retries.");  
        ESP.restart();
      }
    }
  }

  mqttClient.loop();
}

/**
 * 
 * 
{
  "brightness": 255,
  "color_temp": 155,
  "color": {
    "r": 255,
    "g": 180,
    "b": 200,
    "x": 0.406,
    "y": 0.301,
    "h": 344.0,
    "s": 29.412
  },
  "effect": "colorloop",
  "state": "ON",
  "transition": 2,
  "white_value": 150
}

 */
static void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  Serial.println();
  Serial.printf(MQTT_LOG "Received topic: %s" NL, topic);
  for (uint16_t i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  char buffer[length + 1];
  memcpy(buffer, payload, length);
  buffer[length] = '\0';

  if (mqttParsePayload(buffer))
  {
    mqttSendResponse();
  }
}

static bool mqttParsePayload(const char *jsonData)
{
  StaticJsonBuffer<150> jsonBuffer;
  JsonObject& jsonObj = jsonBuffer.parseObject(jsonData);
  if (!jsonObj.success())
  {
    Serial.println(MQTT_LOG "Error parsing json data" NL);
    return false;
  }

  if (jsonObj.containsKey("state"))
  {
    if (strcmp(jsonObj["state"], "ON") == 0)
    {
      if (ledContext.state == LED_STATE_OFF)
      {
        initLed();
        ledContext.colorR = 255;
        ledContext.colorG = 255;
        ledContext.colorB = 255;
        ledContext.whiteValue = 0;
        ledContext.colorTemp = 0.0f;
        ledContext.colorTempRaw = 153;
        ledContext.brightness = 1; // Brightness range is 1-100 in HA for some reason.
      }
      ledContext.state = LED_STATE_ON;
    }
    else
    {
      initLed();
      ledContext.state = LED_STATE_OFF;
    }
  }

  if (jsonObj.containsKey("brightness"))
  {
    ledContext.brightness = jsonObj["brightness"];
  }

  if (jsonObj.containsKey("color"))
  {
    auto color = jsonObj["color"];
    ledContext.colorR = color["r"];
    ledContext.colorG = color["g"];
    ledContext.colorB = color["b"];
  }

  if (jsonObj.containsKey("white_value"))
  {
    ledContext.whiteValue = jsonObj["white_value"];
  }

  if (jsonObj.containsKey("color_temp"))
  {
    ledContext.colorTempRaw = jsonObj["color_temp"]; // range between 153-500
    uint16_t temp = ledContext.colorTempRaw - 153; // offset value from 153-500 -> 0-346
    ledContext.colorTemp = (float)(temp / 346.0f); // convert to 0.0-1.0f
  }

  if (jsonObj.containsKey("effect"))
  {
    if (jsonObj.containsKey("transition"))
    {
      ledContext.transition = jsonObj["transition"];
    }
    
    bool found = false;
    const uint8_t numEntries = SIZE_OF(effects);
    for (uint8_t i = 0; i < numEntries; i++)
    {
      if (strcmp(effects[i].effect, jsonObj["effect"]) == 0)
      {
        Serial.printf(MQTT_LOG "Setting effect: %u" NL, i);
        ledContext.nextEffect = i;
        found = true;
        break;
      }
    }

    if (!found)
    {
      // todo: error mode
      Serial.println(MQTT_LOG "Error. Unknown effect name received." NL);
      ledContext.effect = 0;
      ledContext.nextEffect = 0;
    }
  }

  return true;
}

static void mqttSendResponse(void)
{
  StaticJsonBuffer<250> jsonBuffer;
  JsonObject& jsonObj = jsonBuffer.createObject();
  jsonObj["state"] = ledContext.state;
  jsonObj["brightness"] = ledContext.brightness;
  jsonObj["state"] = ((ledContext.state == LED_STATE_ON) ? "ON" : "OFF");
  jsonObj["white_value"] = ledContext.whiteValue;
  jsonObj["color_temp"] = ledContext.colorTempRaw;
  if (ledContext.nextEffect != LED_INVALID_EFFECT)
  {
    jsonObj["effect"] = effects[ledContext.nextEffect].effect;
  }
  else
  {
    jsonObj["effect"] = effects[ledContext.effect].effect;
  }
  
  JsonObject& color = jsonObj.createNestedObject("color");
  color["r"] = ledContext.colorR;
  color["g"] = ledContext.colorG;
  color["b"] = ledContext.colorB;

  uint8_t size = jsonObj.measureLength() + 1;
  char buffer[size];
  jsonObj.printTo(buffer, sizeof(buffer));
  buffer[size] = '\0';
  mqttClient.publish(MQTT_STATE_TOPIC, buffer, true);

  Serial.printf(MQTT_LOG "Response topic: %s" NL, MQTT_STATE_TOPIC);
  for (int i = 0; i < size; i++)
  {
    Serial.print((char)buffer[i]);
  }
  Serial.printf(NL NL NL);
}

static void initSk6812(void)
{
  strip.Begin();
  strip.Show();
  sk6812Clear();
}

static void sk6812Clear()
{
  strip.ClearTo(RgbwColor(0, 0, 0, 0));
  strip.Show();
}

static void initLed()
{
    memset(&ledContext, 0, sizeof(ledContext));
}

static void ledHandle()
{
  static uint8_t dirty = false;

  if (ledContext.nextEffect != LED_INVALID_EFFECT)
  {
    ledContext.effect = ledContext.nextEffect;
    ledContext.nextEffect = LED_INVALID_EFFECT;
    void (*InitFunc)(void) = effects[ledContext.effect].InitFunc;
    if (InitFunc != NULL)
    {
      (*InitFunc)();
    }
      
  }

  if (!dirty)
  {
    dirty = true;
    void (*LoopFunc)(void) = effects[ledContext.effect].LoopFunc;
    (*LoopFunc)();
  }

  if (strip.CanShow())
  {
    strip.Show();
    frameCnt++;
    dirty = false;
  }
}

static void effectSolidLoop(void)
{
  double intensity = (double)((double)ledContext.brightness / (double)255);
  uint8_t r = ledContext.colorR;
  uint8_t g = ledContext.colorG;
  uint8_t b = ledContext.colorB;
  r = (int)((double)r * intensity);
  g = (int)((double)g * intensity);
  b = (int)((double)b * intensity);

  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    strip.SetPixelColor(i, RgbwColor(r,
                          g,
                          b,
                          ledContext.whiteValue));
  }
}

double wSparkBrightness;
double wWhiteValue;
uint8_t welcomeState;
long wTimeStart;
static void effectWelcomeInit(void)
{
  sk6812Clear();
  wSparkBrightness = 1;
  wWhiteValue = 1;

  wTimeStart = millis();
}

/**
 * (ms)
 *  0     sparks starts fade up until reaching 99% brightness from black.
 *  5000  sparks starts fade to color white until 100% RGB white.
 *  6000  Fade up goes to fade glow with Warm white led.
 *  30000 Fade out to black.
 * 
 */
static void effectWelcomeLoop(void)
{
  long delta = millis() - wTimeStart;
  if (delta < 20000)
  {
    // sparks fade up from black
    wSparkBrightness = MIN(254, wSparkBrightness + wSparkBrightness*0.02);
    welcomeSpark();
    //Serial.printf("sparkbrightness: %u\n", wSparkBrightness);
  }
  else if (delta < 40000)
  {
    // spark with fade to white
    welcomeSpark();
    welcomeFadeToWhite();
  }
  else if (delta < 80000)
  {
    // fade glow

  }
  else if (delta < 90000)
  {
    for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
    {
      RgbwColor color = strip.GetPixelColor(i);
      fadeToBlack(color, 2);
      strip.SetPixelColor(i, color);
    }
  }
  else
  {
    // do nothing
  }
  
}


long wLast = 0;
static void welcomeSpark(void)
{
  double intensity = (double)((double)wSparkBrightness / (double)255);
  RgbwColor blueWhiteDim(
                        0,
                        0,
                        (uint8_t)((double)100 * intensity),
                        (uint8_t)((double)100 * intensity)
                        );
  RgbwColor blueWhite(
                        0,
                        0,
                        (uint8_t)((double)255 * intensity),
                        (uint8_t)((double)255 * intensity)
                        );

  //long r = random(0, 200);
  //if (r == 0) 
  long wNow = millis();
  if (wNow - wLast > random(50, 250))
  {
    wLast = wNow;

    uint8_t randomNumOfStars = random(1, 5);
    for (uint16_t i = 0; i < randomNumOfStars; i++)
    {
      // draw a new big star random times
      long pixel = random(NUM_OF_LEDS-2);
      strip.SetPixelColor(pixel-1, blueWhiteDim);
      strip.SetPixelColor(pixel, blueWhite);
      strip.SetPixelColor(pixel+1, blueWhiteDim);
    }
  }
  // start fading stars and sparks
  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    RgbwColor color = strip.GetPixelColor(i);

    fadeToBlack(color, 100);
    strip.SetPixelColor(i, color);
  }
  
  uint8_t randomNumOfSparks = random(1, 10);
  for (uint16_t i = 0; i < randomNumOfSparks; i++)
  {
    long t = random(NUM_OF_LEDS-1);
    strip.SetPixelColor(t, RgbwColor(
      0,
      0,
      (uint8_t)(random(1, wSparkBrightness)),
      0));
  }
}

static void welcomeFadeToWhite(void)
{
    static uint16_t prevTime = 0;
  if ((now - prevTime) > 50)
  {
    wWhiteValue = MIN(250, wWhiteValue + (wWhiteValue * 0.02));
    prevTime = now;
  }

  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    RgbwColor color = strip.GetPixelColor(i);
    color.R = MAX((uint8_t)wWhiteValue, color.R);
    color.G = MAX((uint8_t)wWhiteValue, color.G);
    color.B = MAX((uint8_t)wWhiteValue, color.B);
    color.W = MAX((uint8_t)wWhiteValue, color.W);
    strip.SetPixelColor(i, color);  
  }
}

static void effectXmasInit(void)
{
  sk6812Clear();
}

static void effectXmasLoop(void)
{
  uint8_t doit = random(0, 200);
  if (doit == 0)
  {
    //Serial.printf("FLash!\n");
    uint8_t pixel = random(NUM_OF_LEDS-1);
  
    if (pixel > 0)
    {
      strip.SetPixelColor(pixel-1, RgbwColor(0, 0, 100, 30));
    }
    
    strip.SetPixelColor(pixel, RgbwColor(0, 0, 255, 255));
  
    if (pixel < (NUM_OF_LEDS-1))
    {
      strip.SetPixelColor(pixel+1, RgbwColor(0, 0, 100, 30));
    }
  }
  
  double intensity = (double)((double)ledContext.brightness / (double)255);
  uint8_t r = (int)((double)255 * intensity);  
  
  // calculate decay of brightness 
  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    RgbwColor color = strip.GetPixelColor(i);

    if (color.W >= 10)
    {
      color.W -= 10;
    }
    else if (color.W > 0)
    {
      color.W--;
    }

    if (color.B >= 5)
    {
      color.B -= 5;
    }
    else if (color.B > 0)
    {
      color.B--;
    }
    
    if (color.R < r)
    {
      color.R++;
    }

    //Serial.printf("%u  W:%u \t R:%u \t B:%u \n", i, color.W, color.R, color.B);
    strip.SetPixelColor(i, color);
  }
}


/*** Falling globals ***/
uint8_t fallingPos = 0;
RgbwColor fallingColors[NUM_OF_SEGMENTS];

static void effectFallingInit(void)
{
  sk6812Clear();
  fallingPos = 0;
}

static void effectFallingLoop(void)
{
  for (uint16_t segment = 0; segment < NUM_OF_SEGMENTS; segment++)
  {
    if (fallingPos > 0)
    {
      setColor(segment, fallingPos - 1, RgbwColor(0, 0, 0, 0));
    }
    else if (fallingPos == 0)
    {
      setColor(segment, NUM_OF_LEDS_PER_SEGMENT - 1, RgbwColor(0, 0, 0, 0));
      fallingColors[segment] = rgbColorPicker(true);
    }
    
    setColor(segment, fallingPos, fallingColors[segment]);

  }
  fallingPos = (fallingPos + 1) % NUM_OF_LEDS_PER_SEGMENT;
}

static void effectFireInit(void)
{
  sk6812Clear();
}

static void effectFireLoop(void)
{
  uint8_t cooling = 55;
  uint8_t sparking = (uint8_t)((float)120 * ledContext.colorTemp);

  static uint8_t heat[NUM_OF_SEGMENTS][NUM_OF_LEDS_PER_SEGMENT];
  uint16_t cooldown;
  
  for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment++)
  {

    // Cool down every cell a little
    for (uint16_t i = 0; i < NUM_OF_LEDS_PER_SEGMENT; i++) 
    {
      cooldown = random(0, ((cooling * 10) / NUM_OF_LEDS_PER_SEGMENT) + 2);
      
      if (cooldown > heat[segment][i]) 
      {
        heat[segment][i] = 0;
      } 
      else 
      {
        heat[segment][i] = heat[segment][i] - cooldown;
      }
    }
    
    // Heat from each cell drifts 'up' and diffuses a little
    for (int k = NUM_OF_LEDS_PER_SEGMENT - 1; k >= 2; k--) 
    {
      heat[segment][k] = (heat[segment][k - 1] + heat[segment][k - 2] + heat[segment][k - 2]) / 3;
    }

    // Randomly ignite new 'sparks' near the bottom
    if (random(255) < sparking ) 
    {
      int y = random(7);
      heat[segment][y] = heat[segment][y] + random(160,255);
    }

    // Convert heat to LED colors
    for (uint16_t pixel = 0; pixel < NUM_OF_LEDS_PER_SEGMENT; pixel++) 
    {
      byte temperature = heat[segment][pixel];
      // Scale 'heat' down from 0-255 to 0-191
      byte t192 = round((temperature/255.0)*191);
    
      // calculate ramp up from
      uint8_t heatramp = t192 & 0x3F; // 0..63
      heatramp <<= 2; // scale up to 0..252
    
      // figure out which third of the spectrum we're in:
      if (t192 > 0x80) // hottest
      {
        setColorInv(segment, pixel, RgbwColor(255, 255, heatramp, 0));
      }
      else if (t192 > 0x40) // middle
      {
        setColorInv(segment, pixel, RgbwColor(255, heatramp, 0, 0));
      }
      else // coolest
      {
        setColorInv(segment, pixel, RgbwColor(heatramp, 0, 0, 0));
      }
    }
  }

  // set background light
  double intensity = (double)((double)(ledContext.brightness-1) / (double)255);
  uint8_t r = ledContext.colorR;
  uint8_t g = ledContext.colorG;
  uint8_t b = ledContext.colorB;
  r = (int)((double)r * intensity);
  g = (int)((double)g * intensity);
  b = (int)((double)b * intensity);

  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    RgbwColor color = strip.GetPixelColor(i);
    color.R = MAX(color.R, r);
    color.G = MAX(color.G, g);
    color.B = MAX(color.B, b);
    color.W = ledContext.whiteValue;
    strip.SetPixelColor(i, color);
  }
}

/*** MeteorRain globals ***/
typedef struct
{
  float position;
  uint8_t size;
  uint8_t trailDecayRate;
  bool randomDecay;
  float speedFactor;
} Meteor;

Meteor meteors[NUM_OF_SEGMENTS];

static void effectMeteorRainInit(void)
{
  sk6812Clear();
  // start all meteorites at random initial positions
  for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment++)
  {
    meteors[segment].position = random(0, NUM_OF_LEDS_PER_SEGMENT);
    meteors[segment].size = random(3, 10);
    meteors[segment].trailDecayRate = random(8, 64);
    meteors[segment].speedFactor = random(1, 3);
  }
}

static void effectMeteorRainLoop(void)
{
  double intensity = (double)((double)ledContext.brightness / (double)255);
  const uint8_t r = (int)((double)ledContext.colorR * intensity);
  const uint8_t g = (int)((double)ledContext.colorG * intensity);
  const uint8_t b = (int)((double)ledContext.colorB * intensity);
  meteorRain(r, g, b, ledContext.whiteValue, true);
}

void meteorRain(uint8_t red, uint8_t green, uint8_t blue, uint8_t white, boolean randomDecay)
{
  uint16_t i;

  for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment++)
  {
    if (meteors[segment].position > NUM_OF_LEDS_PER_SEGMENT)
    {
      meteors[segment].position = 0.0f;
      meteors[segment].size = random(5, 10); // 10
      meteors[segment].trailDecayRate = random(50, 80); // 64
      meteors[segment].speedFactor = (float)random(5, 20) / 10.0; // 1
    }
    i = (uint8_t)meteors[segment].position;

    // brightness decay
    for (uint16_t j = 0; j < NUM_OF_LEDS_PER_SEGMENT; j++) 
    {
      if ((!randomDecay) || (random(10)>5)) 
      {
        auto color = getColor(segment, j);
        fadeToBlack(color, meteors[segment].trailDecayRate);
        setColor(segment, j, color);
      }
    }
    
    // draw meteor
    for(uint16_t j = 0; j < meteors[segment].size; j++) 
    {
      if ( ((i - j) < NUM_OF_LEDS_PER_SEGMENT) && ((i - j) >= 0) )
      {
        setColor(segment, (i - j), RgbwColor(red, green, blue, white));
      } 
    }

    meteors[segment].position += 1*meteors[segment].speedFactor;
  }
}

/*** SfIntro globals ***/
uint8_t sfState = 0;
uint8_t sfHeight = 0;
float sfX = 0;
static const float sfperiodOffsets[NUM_OF_SEGMENTS] = 
{ 
    PI,         // 0
    PI*5/6,     // 1
    PI*4/6,     // 2
    PI*3/6,     // 3
    PI*3/6,     // 4
    PI*4/6,     // 5
    PI*5/6,     // 6
    PI          // 7
};

static void effectSfIntroInit(void)
{
  sk6812Clear();
  sfState = 0;
  sfHeight = 0;
  sfX = 0;
}

static void effectSfIntroLoop(void)
{
  if (sfState == 0)
  {

    for (uint8_t i = 0; i < sfHeight; i++)
    {
      setColorInv(0, i, RgbwColor(0, 10, 150, 0));
    }
    setColorInv(0, sfHeight, RgbwColor(0, 150, 255, 100));

    sfHeight++;
    if (sfHeight >= (NUM_OF_LEDS_PER_SEGMENT - 1))
    {
      sfState = 1;
    }
  }
  else if (sfState == 1)
  {
    for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment++)
    {
      uint8_t amplitude = 100;
      if (segment == 3 || segment == 4)
      {
        amplitude = 255;
      }

      float brightnessG = ABS((sin(sfperiodOffsets[segment] + sfX)*(amplitude/4)));
      float brightnessB = ABS((sin(sfperiodOffsets[segment] + sfX)*amplitude));
      auto color = RgbwColor(0, (uint8_t)brightnessG, (uint8_t)brightnessB, 0);
      for (uint16_t i = 0; i < NUM_OF_LEDS_PER_SEGMENT; i++)
      {
        setColor(segment, i, color);
      }
    }

    sfX += 0.05;
  }

  else if (sfState == 2)
  {
    // todo
  }
}

/*** Disco globals ***/
RgbwColor discoTargetColor[NUM_OF_SEGMENTS][NUM_OF_BOXES];
long discoLastChangeColor = 0;
long discoLastFadeUpdate = 0;
bool discoDistinct = false;

static void effectDiscoFloorNoDistinctInit(void)
{
  discoDistinct = false;
  effectDiscoFloorInit();
}

static void effectDiscoFloorDistinctInit(void)
{
  discoDistinct = true;
  effectDiscoFloorInit();
}

static void effectDiscoFloorInit(void)
{
  sk6812Clear();
  for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment += 2)
  {
    for (uint8_t box = 0; box < NUM_OF_BOXES; box++)
    {
      Pair pair = discoFloorBoxes[segment][box];
      auto color = rgbColorPicker(discoDistinct);
      discoTargetColor[segment][box] = color;

      for (uint8_t i = pair.a; i <= (pair.b); i++)
      {
        setColor(segment, i, color);
        setColor(segment + 1, i, color);
      }
    }
  }
}

/**
 * brightness (1-255*1000ms) - controls how often one of the boxes (randomly) shall change color.
 * white_value (1-255*100ms) - control how often each box which which has been selected with a new color shall be 
 * updated (faded to new color). Each update change the color one step (total 255 steps in some color changes).
 */
static void effectDiscoFloorLoop(void)
{
  if ((now - discoLastChangeColor) > (ledContext.brightness * 1000))
  {
    discoLastChangeColor = now;
    uint8_t box = random(0, NUM_OF_BOXES);
    uint8_t segment = random(0, NUM_OF_SEGMENTS);
    auto color = rgbColorPicker(discoDistinct);
    discoTargetColor[segment][box] = color;
  }

  bool doFadeUpdate = false;
  if ((now - discoLastFadeUpdate) > (100 + (ledContext.whiteValue * 100)))
  {
    doFadeUpdate = true;
    discoLastFadeUpdate = now;
  }
  for (uint8_t segment = 0; segment < NUM_OF_SEGMENTS; segment += 2)
  {
    for (uint8_t box = 0; box < NUM_OF_BOXES; box++)
    {
      Pair pair = discoFloorBoxes[segment][box];
      auto targetColor = discoTargetColor[segment][box];
      for (uint8_t i = pair.a; i <= (pair.b); i++)
      {
        auto color = getColor(segment, i);

        if (doFadeUpdate)
        {
          fadeToColor(color, targetColor, 1);
        }

        setColor(segment, i, color);
        setColor(segment + 1, i, color);
      }
    }
  }
}

/*** Rainbow globals ***/
#define UP      1
#define DOWN    0
uint8_t RainbowPos = 2;
uint8_t RainbowValue = 0;
uint8_t RainbowDir = UP;
uint8_t RainbowColors[3] = { 255, 0, 0 };

static void effectRainbowInit(void)
{
  RainbowColors[0] = 255;
  RainbowColors[1] = 0;
  RainbowColors[2] = 0;
  RainbowPos = 2;
  RainbowValue = 0;
  RainbowDir = UP;
  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    effectRainbowLoop();
  }
}

static void effectRainbowLoop(void)
{
  double intensity = (double)((double)ledContext.brightness / (double)255);
  RainbowColors[RainbowPos] = (RainbowDir == UP) ? (++RainbowValue) : (--RainbowValue);
  if ((RainbowDir == UP) && (RainbowValue == 255))
  {
    RainbowDir = DOWN;
    RainbowPos = (RainbowPos + 1) % 3;
  }
  else if ((RainbowDir == DOWN) && (RainbowValue == 0))
  {
    RainbowDir = UP;
    RainbowPos = (RainbowPos + 1) % 3;
  }
  strip.RotateRight(1);
  strip.SetPixelColor(0, RgbwColor(
    RainbowColors[0] * intensity, 
    RainbowColors[1] * intensity, 
    RainbowColors[2] * intensity, 
    ledContext.whiteValue));
}

/*** Helper functions ***/

RgbwColor getColor(uint8_t segment, uint16_t pixel)
{
  const uint16_t targetPixel = segment * NUM_OF_LEDS_PER_SEGMENT + pixel;
  return strip.GetPixelColor(targetPixel);
}

static void setColor(uint8_t segment, uint16_t pixel, RgbwColor color)
{
  const uint16_t targetPixel = segment * NUM_OF_LEDS_PER_SEGMENT + pixel;
  strip.SetPixelColor(targetPixel, color);
}

RgbwColor getColorInv(uint8_t segment, uint16_t pixel)
{
  const uint16_t targetPixel = (segment * NUM_OF_LEDS_PER_SEGMENT) + NUM_OF_LEDS_PER_SEGMENT - 1 - pixel;
  return strip.GetPixelColor(targetPixel);
}

static void setColorInv(uint8_t segment, uint16_t pixel, RgbwColor color)
{
  const uint16_t targetPixel = (segment * NUM_OF_LEDS_PER_SEGMENT) + NUM_OF_LEDS_PER_SEGMENT - 1 - pixel;
  strip.SetPixelColor(targetPixel, color);
}

static void fadeToColor(RgbwColor& current, const RgbwColor target, const uint8_t value)
{
  if (current.R != target.R)
  {
    current.R += (current.R > target.R) ? -(value) : (value);
  }
  if (current.G != target.G)
  {
    current.G += (current.G > target.G) ? -(value) : (value);
  }
  if (current.B != target.B)
  {
    current.B += (current.B > target.B) ? -(value) : (value);
  }
}

static void fadeTo(const uint16_t pixel, const RgbwColor targetColor, const uint8_t value)
{
  RgbwColor color = strip.GetPixelColor(pixel);
  color.R = (color.R + targetColor.R) / 2;
  color.G = (color.G + targetColor.G) / 2;
  color.B = (color.B + targetColor.B) / 2;
}

static void fadeToWhite(RgbwColor& color, const uint8_t value, const uint8_t max)
{
    double fadeFactor = (double)value / 256.0;
    color.R = MIN(max, color.R + (MAX(1, color.R * fadeFactor)));
    color.G = MIN(max, color.G + (MAX(1, color.G * fadeFactor)));
    color.B = MIN(max, color.B + (MAX(1, color.B * fadeFactor)));
    color.W = MIN(max, color.W + (MAX(1, color.W * fadeFactor)));
}

static void fadeToBlack(RgbwColor& color, const uint8_t value)
{
    double fadeFactor = (double)value / 256.0;
    color.R = MAX(0, color.R - (color.R * fadeFactor));
    color.G = MAX(0, color.G - (color.G * fadeFactor));
    color.B = MAX(0, color.B - (color.B * fadeFactor));
    color.W = MAX(0, color.W - (color.W * fadeFactor));
}

static RgbwColor rgbColorPicker(bool distinct)
{
  if (distinct)
  {
    uint8_t pos[3] = { 0xFF, 0xFF, 0xFF };
    for (uint8_t i = 0; i < 3; i++)
    {
      bool found = false;
      while (!found)
      {
        uint8_t r = rand() % 3;
        if (pos[0] != r && pos[1] != r && pos[2] != r)
        {
          pos[i] = r;
          found = true;
        }
      }
    }
    uint8_t color[3] = {0};
    color[pos[0]] = random(0, 255);
    color[pos[1]] = 0;
    color[pos[2]] = 255;
    return RgbwColor(color[0], color[1], color[2], 0);
  }
  else
  {
    return RgbwColor(random(255), random(255), random(255), 0);
  }
}

static void handleWifiMqtt()
{
  ArduinoOTA.handle();
  mqttHandle();
}

/****** Arduino framework callbacks *******/

void setup()
{
  initBoard();
  initSk6812();
  initLed();
  initWifi();
  initOta();
  initMqttClient();
}

void loop()
{
  now = millis();
  handleWifiMqtt();
  
  if ((now - prev) > 1000)
  {
    Serial.printf("fCnt: %u  effect: %u (%s)\n", frameCnt, ledContext.effect, effects[ledContext.effect].effect);
    prev = now;
    frameCnt = 0;
  }

  if (now - last >= ON_BOARD_LED_BLINK_DELAY)
  {
    last = now;
    digitalWrite(LED_BUILTIN, ledToggle);
    ledToggle ^= 1;
    //Serial.printf("%lu %u" NL, now, ledToggle);
  }

  ledHandle();
}
