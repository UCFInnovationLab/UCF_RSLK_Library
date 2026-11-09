#ifndef WS2812_MSP432_H
#define WS2812_MSP432_H

#ifndef __MSP432P401R__
#error "This library only supports the MSP432P401R (Energia)"
#endif

#include <Arduino.h>
#include <msp432.h>

// SPI encodes one WS2812 data bit as one SPI byte.
// Energia MSP432 sets SMCLK = 12 MHz (MCLK/4). UCB0BRW = 2 -> 6 MHz SPI -> 167 ns per SPI bit.
// 0xF8 (11111000): HIGH 5x167=833 ns, LOW 3x167=500 ns -> WS2812 "1" bit  (spec: T1H 800±150 ns)
// 0xC0 (11000000): HIGH 2x167=333 ns, LOW 6x167=1000 ns -> WS2812 "0" bit (spec: T0H 400±150 ns)
#define WS2812_HIGH_CODE  (0xF8)
#define WS2812_LOW_CODE   (0xC0)

class WS2812_MSP432 {
public:
  WS2812_MSP432(uint16_t numLEDs);
  ~WS2812_MSP432();

  void     begin();
  void     show();
  void     setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b);
  void     fill(uint8_t r, uint8_t g, uint8_t b);
  void     clear();
  uint16_t numPixels() const { return _numLEDs; }

private:
  uint16_t _numLEDs;
  uint8_t *_pixels;   // stored in GRB order (WS2812 wire format)

  void     spiSendByte(uint8_t b);
};

#endif // WS2812_MSP432_H
