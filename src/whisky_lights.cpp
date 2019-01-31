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
    qos: 0
    optimistic: false
    effect: true
    effect_list:
      - Solid
      - Xmas
      - Welcome
      - Falling


HA < 0.84:
light:
  - platform: mqtt_json
    name: whisky_lights
    state_topic: "whisky_lights/state"
    command_topic: "whisky_lights/set"
    brightness: true
    rgb: true
    white_value: true
    qos: 0
    optimistic: false
    effect: true
    effect_list:
      - Solid
      - Xmas
      - Welcome
      - Falling


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
//#define STRIP2  1

#ifdef STRIP2
#define NUM_OF_LEDS                 115*4                     /*!< Must be 4 or more */
#else
#define NUM_OF_LEDS_PER_SEGMENT     114
#define NUM_OF_SEGMENTS             7
#define NUM_OF_LEDS                 (NUM_OF_LEDS_PER_SEGMENT * NUM_OF_SEGMENTS)
#endif

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
/****** Typedefs *******/

typedef struct MqttContext_st
{
  uint8_t retries;
} MqttContext;

typedef struct LedContext_st
{
  uint8_t state;
  uint8_t whiteValue;
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

static void FadeTo(const uint16_t pixel, const RgbwColor targetColor, const uint8_t value);
static void fadeToWhite(RgbwColor& color, const uint8_t value, const uint8_t max);
static void fadeToBlack(RgbwColor& color, const uint8_t value);

static void handleWifiMqtt();

/****** Globals *******/

Effect effects[] = 
{
  { "Solid", NULL, &effectSolidLoop },
  { "Welcome", &effectWelcomeInit, &effectWelcomeLoop },
  { "Xmas", &effectXmasInit, &effectXmasLoop},
  { "Falling", &effectFallingInit, &effectFallingLoop},
};

/* Pixelpin is ignored on ESP8266 with NeoEsp8266Dma800KbpsMethod. Connect to D4 on Wemos mini */
NeoPixelBus<NeoGrbwFeature, NeoEsp8266Dma800KbpsMethod> strip(NUM_OF_LEDS, 0); 

/* Pixelpin is ignored on ESP8266 with NeoEsp8266AsyncUart1800KbpsMethod. Connect to RX on Wemos mini */
#ifdef STRIP2
NeoPixelBus<NeoGrbwFeature, NeoEsp8266AsyncUart1800KbpsMethod> strip2(NUM_OF_LEDS, 2); 
#endif

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

#ifdef STRIP2
  strip2.Begin();
  strip2.Show();
#endif

  sk6812Clear();
}

static void sk6812Clear()
{
  strip.ClearTo(RgbwColor(0, 0, 0, 0));
  strip.Show();

#ifdef STRIP2
  strip2.ClearTo(RgbwColor(0, 0, 0, 0));
  strip2.Show();
#endif
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
  //delay(100);

  strip.ClearTo(RgbwColor(r,
                          g,
                          b,
                          ledContext.whiteValue));

/*
  for (uint16_t i = 0; i < NUM_OF_LEDS; i++)
  {
    strip.SetPixelColor(i, RgbwColor(r,
                          g,
                          b,
                          ledContext.whiteValue));
  }
  long t = random(NUM_OF_LEDS-1);
  strip.SetPixelColor(t, RgbwColor(0,
                        255,
                        0,
                        0));
*/
#ifdef STRIP2
  strip2.ClearTo(RgbwColor(r,
                           g,
                           b,
                           ledContext.whiteValue));
#endif                           
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

static void effectFallingInit(void)
{
  sk6812Clear();
}

static void effectFallingLoop(void)
{
  RgbwColor color = RgbwColor(random(0, 255), random(0, 255), random(0,255), 0);
  for (uint16_t i = 0; i < NUM_OF_LEDS_PER_SEGMENT; i++)
  {
      for (uint16_t segment = 0; segment < NUM_OF_SEGMENTS; segment++)
      {
          uint16_t pixel = i + segment*NUM_OF_LEDS_PER_SEGMENT;
          strip.SetPixelColor(pixel, color);
          
          if (i > 0)
          {
            strip.SetPixelColor(pixel - 1, RgbwColor(0, 0, 0, 0));
          }
      }
      strip.Show();
    }
}


static void FadeTo(const uint16_t pixel, const RgbwColor targetColor, const uint8_t value)
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
