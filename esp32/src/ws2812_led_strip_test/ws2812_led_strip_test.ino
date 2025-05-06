#include <Adafruit_NeoPixel.h>

#define PIN_WS2812B  4
#define NUM_PIXELS 48

Adafruit_NeoPixel ws2812b(NUM_PIXELS, PIN_WS2812B, NEO_GRB + NEO_KHZ800);
uint16_t hsv = 0;
uint8_t pixel = 0;

void setup() {
  ws2812b.begin();
  ws2812b.setBrightness(10);
  ws2812b.clear();
}

void loop() {
  ws2812b.setPixelColor(pixel, ws2812b.ColorHSV(hsv, 255, 255));
  ws2812b.show();
  pixel<NUM_PIXELS?pixel++:pixel=0;
  hsv+=1000;
  delay(50);
}