#include <NeoPixelBus.h>

#define NUM_OF_LEDS 600 // must be 4 or more

NeoPixelBus<NeoGrbwFeature, NeoEsp8266Dma800KbpsMethod> strip(NUM_OF_LEDS, 0); // pixelpin is ignored on ESP8266
NeoPixelBus<NeoGrbwFeature, NeoEsp8266AsyncUart1800KbpsMethod> strip2(NUM_OF_LEDS, 2); // pixelpin is ignored on ESP8266


long lastMsg = 0;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output

  Serial.begin(115200);
  Serial.println("Booting");
  
  // this resets all the neopixels to an off state
  strip.Begin();
  strip2.Begin();
  strip.Show();
  strip2.Show();
}

uint8_t r = 0;
uint8_t g = 25;
uint8_t b = 0;

long prevTime = 0;

void loop()
{
  long now = millis();

  r = (r+1) % 50;
  g = (g+1) % 50;
  for (uint i = 0; i < NUM_OF_LEDS; i++)
  {
    strip.SetPixelColor(i, RgbwColor(0, g, 0));
    strip2.SetPixelColor(i, RgbwColor(r, 0, 0));
  }

  strip.Show();
  strip2.Show();

  {
    Serial.printf("loop: %lu \n", now - prevTime);
    prevTime = now;
  }
    
  if (now - lastMsg >= 2000)
  {
    lastMsg = now;

    digitalWrite(LED_BUILTIN, HIGH);
    Serial.printf("%lu\n", now);
  }
  else if (now - lastMsg > 1000)
  {
    digitalWrite(LED_BUILTIN, LOW);
  }

}
