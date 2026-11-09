#include "WS2812_MSP432.h"

// -----------------------------------------------------------------------------
// Constructor / Destructor
// -----------------------------------------------------------------------------

WS2812_MSP432::WS2812_MSP432(uint16_t numLEDs) : _numLEDs(numLEDs), _pixels(NULL) {
  _pixels = (uint8_t *)malloc(numLEDs * 3); // 3 bytes per LED (GRB)
  if (_pixels) {
    memset(_pixels, 0, numLEDs * 3);
  }
}

WS2812_MSP432::~WS2812_MSP432() {
  free(_pixels);
}

// -----------------------------------------------------------------------------
// begin() - Configure EUSCI_B0 SPI on P1.6 (SIMO) for WS2812 output
//
// UCB0SIMO = P1.6 on MSP432P401R
// Energia sets SMCLK = 12 MHz (MCLK/4). UCB0BRW = 2 -> 6 MHz SPI -> 167 ns per bit -> 1.33 us per byte
//
// MSP432 port select uses two registers (SEL0 + SEL1) to pick from 4 functions:
//   SEL0=1, SEL1=0 -> primary peripheral function (UCB0SIMO on P1.6)
// -----------------------------------------------------------------------------

void WS2812_MSP432::begin() {
  // Drive P1.6 LOW as GPIO first to guarantee a clean reset pulse on the
  // data line before handing the pin to EUSCI. After UCSWRST is cleared the
  // SIMO idle state is undefined until the first byte is loaded, so we cannot
  // rely on the peripheral to hold the line LOW for the WS2812 reset window.
  P1DIR  |=  BIT6;    // output
  P1OUT  &= ~BIT6;    // drive LOW
  P1SEL0 &= ~BIT6;    // GPIO function (not EUSCI)
  P1SEL1 &= ~BIT6;
  delayMicroseconds(50);   // guaranteed LOW for >50 us — WS2812 reset

  // Now configure EUSCI_B0 SPI and assign P1.6 to SIMO
  UCB0CTLW0 |= UCSWRST;                                   // hold in reset while configuring
  P1SEL0    |=  BIT6;                                      // assign pin to EUSCI_B0 SIMO
  P1SEL1    &= ~BIT6;
  UCB0CTLW0  = UCCKPH | UCMSB | UCMST | UCSYNC            // SPI: clock phase, MSB first, master, synchronous
              | UCSSEL__SMCLK | UCSWRST;                   // clock = SMCLK, keep in reset
  UCB0BRW    = 2;                                          // SMCLK (12 MHz) / 2 = 6 MHz
  UCB0CTLW0 &= ~UCSWRST;                                   // release reset, SPI is live
}

// -----------------------------------------------------------------------------
// spiSendByte() - Transmit one byte via EUSCI_B0 SPI
// -----------------------------------------------------------------------------

void WS2812_MSP432::spiSendByte(uint8_t b) {
  while (!(UCB0IFG & UCTXIFG));  // wait for TX buffer empty
  UCB0TXBUF = b;
}

// -----------------------------------------------------------------------------
// show() - Transmit all pixel data to the strip
//
// Each WS2812 data bit is encoded as one SPI byte (HIGH_CODE or LOW_CODE).
// Interrupts are disabled for the duration to prevent timing glitches.
//
// P1.6 is driven LOW via GPIO before and after EUSCI transmission because
// EUSCI_B idles SIMO HIGH when not shifting — which would corrupt WS2812
// timing if the line floats high between frames or during the reset pulse.
// -----------------------------------------------------------------------------

void WS2812_MSP432::show() {
  if (!_pixels) return;

  // // Guarantee line is LOW before handing to EUSCI (EUSCI_B idles SIMO HIGH)
  // P1SEL0 &= ~BIT6;          // GPIO function
  // P1SEL1 &= ~BIT6;
  // P1DIR  |=  BIT6;
  // P1OUT  &= ~BIT6;          // drive LOW

  noInterrupts();

  P1SEL0 |= BIT6;           // hand pin to EUSCI_B0 SIMO immediately before first byte

  uint8_t *p = _pixels;
  spiSendByte(0x00);
  spiSendByte(0x00);
  for (uint16_t i = 0; i < _numLEDs * 3; i++) {
    uint8_t byte = *p++;
    uint8_t mask = 0x80;
    while (mask) {
      spiSendByte(byte & mask ? WS2812_HIGH_CODE : WS2812_LOW_CODE);
      mask >>= 1;
    }
  }
  spiSendByte(0x00);
  spiSendByte(0x00);

  // Wait for shift register to fully finish (UCBUSY clears when last bit is clocked out)
  // UCTXIFG only means the TX buffer is empty, not that the shift register is done
  while (UCB0STATW & UCBUSY);

  // Take pin back to GPIO and drive LOW — EUSCI_B would idle SIMO HIGH otherwise,
  // which would prevent the WS2812 reset pulse from being a valid LOW
  P1SEL0 &= ~BIT6;
  P1SEL1 &= ~BIT6;
  P1DIR  |=  BIT6;
  P1OUT  &= ~BIT6;          // guaranteed LOW for reset pulse

  interrupts();

  delayMicroseconds(50);    // >50 us LOW = WS2812 reset
}

// -----------------------------------------------------------------------------
// setPixelColor() - Store a pixel color in GRB order (WS2812 wire format)
// -----------------------------------------------------------------------------

void WS2812_MSP432::setPixelColor(uint16_t n, uint8_t r, uint8_t g, uint8_t b) {
  if (!_pixels || n >= _numLEDs) return;
  uint8_t *p = &_pixels[n * 3];
  p[0] = g;
  p[1] = r;
  p[2] = b;
}

// -----------------------------------------------------------------------------
// fill() - Set all pixels to the same color and push to strip
// -----------------------------------------------------------------------------

void WS2812_MSP432::fill(uint8_t r, uint8_t g, uint8_t b) {
  for (uint16_t i = 0; i < _numLEDs; i++) {
    setPixelColor(i, r, g, b);
  }
  show();
}

// -----------------------------------------------------------------------------
// clear() - Turn off all pixels
// -----------------------------------------------------------------------------

void WS2812_MSP432::clear() {
  if (_pixels) {
    memset(_pixels, 0, _numLEDs * 3);
  }
  show();
}
