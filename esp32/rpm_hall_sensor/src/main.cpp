#include <Arduino.h>
#include <FastLED.h>

#define ANALOG_PIN 4
CRGB leds[1];

void setupPixels()
{
    FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, 1);
    leds[0] = CRGB::White;
    FastLED.setBrightness(5);
}


void setup()
{
    Serial.begin(115200);
    delay(50);
    setupPixels();
    pinMode(ANALOG_PIN, INPUT_PULLUP);
}

void loop()
{
    int raw = digitalRead(ANALOG_PIN);
    Serial.println(raw);

    FastLED.setBrightness(raw ? 4 : 0);
    FastLED.show();
    delay(100);
}
