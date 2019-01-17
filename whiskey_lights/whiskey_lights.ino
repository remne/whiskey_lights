#include <NeoPixelBus.h>

#define NUM_OF_LEDS 600 // must be 4 or more

NeoPixelBus<NeoGrbwFeature, NeoEsp8266Dma800KbpsMethod> strip(NUM_OF_LEDS, 0); // pixelpin is ignored on ESP8266
NeoPixelBus<NeoGrbwFeature, NeoEsp8266AsyncUart1800KbpsMethod> strip2(NUM_OF_LEDS, 2); // pixelpin is ignored on ESP8266


long last = 0;
long prev = 0;


void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(115200);
  Serial.println("Booting");
  
  // this resets all the neopixels to an off state
  strip.Begin();
  strip2.Begin();
  strip.Show();
  strip2.Show();
}

uint8_t r = 0;
uint8_t g = 10;
uint8_t b = 0;


void loop()
{
  long now = millis();

  r = (r+1) % 20;
  g = (g+1) % 20;
  for (uint i = 0; i < NUM_OF_LEDS; i++)
  {
    strip.SetPixelColor(i, RgbwColor(0, g, 0));
    strip2.SetPixelColor(i, RgbwColor(r, 0, 0));
  }

  strip.Show();
  strip2.Show();

  {
    Serial.printf("loop: %lu \n", now - prev);
    prev = now;
  }
    
  if (now - last >= 2000)
  {
    last = now;

    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("%lu\n", now);
  }
  else if (now - last > 1000)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

}
