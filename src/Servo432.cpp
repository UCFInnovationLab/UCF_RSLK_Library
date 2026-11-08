/*
 * Servo432.cpp
 *
 * See Servo432.h for full description.
 *
 * Clock maths (SMCLK = 12 MHz, confirmed from Energia Board_init.c):
 *   servo_clock  = SMCLK / 8       = 1,500,000 Hz
 *   timer_period = servo_clock / 50 = 30,000 ticks  (20 ms, 50 Hz)
 *   ticks_per_us = 1,500,000 / 1e6 = 1.5
 *
 * Pulse width examples (Timer_A OUTMOD_7 = Reset/Set):
 *   500 µs → 750 ticks    (minimum, full reverse/closed)
 *   1500 µs → 2250 ticks  (centre / neutral)
 *   2500 µs → 3750 ticks  (maximum, full forward/open)
 */

#if defined(__MSP432P401R__) || defined(__MSP432__)

#include "Servo432.h"
#include <driverlib.h>

// ---------------------------------------------------------------------------
// Clock / timer constants
// ---------------------------------------------------------------------------
#define SMCLK_HZ         12000000UL
#define TIMER_DIV        TIMER_A_CLOCKSOURCE_DIVIDER_8
#define SERVO_CLK_HZ     (SMCLK_HZ / 8UL)          // 1,500,000 Hz
#define TIMER_PERIOD     ((uint16_t)(SERVO_CLK_HZ / 50UL))  // 30,000 ticks

#define DEFAULT_MIN_US   564
#define DEFAULT_MAX_US   2436
#define MIN_PULSE_WIDTH  544   // Arduino Servo convention: below this = degrees

// ---------------------------------------------------------------------------
// Pin-to-timer mapping
// Derived from Energia pins.c and MSP432P401R datasheet peripheral functions.
// ---------------------------------------------------------------------------
struct PinEntry {
    uint8_t  energiaPin;
    uint32_t timerBase;      // TIMER_A0_BASE or TIMER_A2_BASE
    uint16_t ccrRegister;    // TIMER_A_CAPTURECOMPARE_REGISTER_x
    uint8_t  ccrIndex;       // 1, 3, or 4  (index into TIMER_A->CCR[])
    uint8_t  gpioPort;       // GPIO_PORT_Px
    uint8_t  gpioPinMask;    // GPIO_PIN4, GPIO_PIN6, GPIO_PIN7
};

static const PinEntry PIN_TABLE[] = {
    { 38, TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 1, GPIO_PORT_P2, GPIO_PIN4 },
    { 39, TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 3, GPIO_PORT_P2, GPIO_PIN6 },
    { 40, TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 4, GPIO_PORT_P2, GPIO_PIN7 },
    { 37, TIMER_A2_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 1, GPIO_PORT_P5, GPIO_PIN6 },
};
static const uint8_t PIN_TABLE_LEN = sizeof(PIN_TABLE) / sizeof(PIN_TABLE[0]);

// ---------------------------------------------------------------------------
// Static state
// ---------------------------------------------------------------------------
bool Servo432::_timerA0Started = false;
bool Servo432::_timerA2Started = false;

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

/*
 * Initialise one Timer_A for 50 Hz hardware PWM using SMCLK/8.
 * Safe to call multiple times; subsequent calls for the same base are no-ops.
 * CCR0 sets the 20 ms period.  Individual CCRs start at centre pulse (1500 µs)
 * but will be overwritten by writeMicroseconds() on attach().
 */
void Servo432::_initTimer(uint32_t base)
{
    Timer_A_PWMConfig param = {0};
    param.clockSource       = TIMER_A_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_DIV;
    param.timerPeriod       = TIMER_PERIOD;
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;
    param.dutyCycle         = _usToPulse(1500);  // neutral — overwritten on attach

    // CCR1 initialises CCR0 (period) for the whole timer
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    Timer_A_outputPWM(base, &param);

    // Pre-configure the other CCRs used by this timer so the output mode is
    // set correctly even before a Servo432 is attached to those pins.
    if (base == TIMER_A0_BASE) {
        param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;
        Timer_A_outputPWM(base, &param);
        param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;
        Timer_A_outputPWM(base, &param);
    }
}

/*
 * Convert pulse width in microseconds to Timer_A ticks.
 * Uses 32-bit integer arithmetic (no float).
 *
 * ticks = us * SERVO_CLK_HZ / 1,000,000
 *       = us * 1,500,000    / 1,000,000
 *       = us * 3 / 2
 *
 * Max value: 2500 µs * 1,500,000 / 1,000,000 = 3750 — fits in uint16_t.
 */
uint16_t Servo432::_usToPulse(int us)
{
    return (uint16_t)((uint32_t)us * SERVO_CLK_HZ / 1000000UL);
}

/*
 * Write ticks directly to the hardware CCR register.
 * This is a single 16-bit register write — no ISR, no driver overhead,
 * no floating-point.  The hardware picks up the new value on the next cycle.
 */
void Servo432::_writePulse(uint16_t ticks) const
{
    Timer_A_setCompareValue(_timerBase,
                            PIN_TABLE[_channel].ccrRegister,
                            ticks);
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

Servo432::Servo432()
    : _channel(SERVO432_INVALID)
    , _timerBase(0)
    , _minUs(DEFAULT_MIN_US)
    , _maxUs(DEFAULT_MAX_US)
    , _currentUs(DEFAULT_MIN_US + (DEFAULT_MAX_US - DEFAULT_MIN_US) / 2)
{}

uint8_t Servo432::attach(int pin)
{
    return attach(pin, DEFAULT_MIN_US, DEFAULT_MAX_US);
}

uint8_t Servo432::attach(int pin, int minUs, int maxUs)
{
    // Find the pin in the table
    int8_t tableIdx = -1;
    for (uint8_t i = 0; i < PIN_TABLE_LEN; i++) {
        if (PIN_TABLE[i].energiaPin == (uint8_t)pin) {
            tableIdx = (int8_t)i;
            break;
        }
    }
    if (tableIdx < 0) return SERVO432_INVALID;

    const PinEntry& e = PIN_TABLE[tableIdx];

    _minUs    = minUs;
    _maxUs    = maxUs;
    _channel  = (uint8_t)tableIdx;   // index into PIN_TABLE, not CCR number
    _timerBase = e.timerBase;
    _currentUs = minUs + (maxUs - minUs) / 2;

    // Start the timer if this is the first servo on it
    if (e.timerBase == TIMER_A0_BASE && !_timerA0Started) {
        _initTimer(TIMER_A0_BASE);
        _timerA0Started = true;
    }
    if (e.timerBase == TIMER_A2_BASE && !_timerA2Started) {
        _initTimer(TIMER_A2_BASE);
        _timerA2Started = true;
    }

    // Switch the GPIO pin to its Timer_A peripheral function (primary = SEL0=1, SEL1=0)
    GPIO_setAsPeripheralModuleFunctionOutputPin(
        e.gpioPort, e.gpioPinMask, GPIO_PRIMARY_MODULE_FUNCTION);

    // Set the initial pulse width
    _writePulse(_usToPulse(_currentUs));

    return e.ccrIndex;
}

void Servo432::detach()
{
    if (!attached()) return;

    const PinEntry& e = PIN_TABLE[_channel];

    // Set CCR to 0 — output stays low after current cycle
    Timer_A_setCompareValue(e.timerBase, e.ccrRegister, 0);

    // Return pin to GPIO (high-Z input)
    GPIO_setAsInputPin(e.gpioPort, e.gpioPinMask);

    _channel = SERVO432_INVALID;
}

void Servo432::writeMicroseconds(int us)
{
    if (!attached()) return;
    if (us < _minUs) us = _minUs;
    if (us > _maxUs) us = _maxUs;
    _currentUs = us;
    _writePulse(_usToPulse(us));
}

void Servo432::write(int value)
{
    if (!attached()) return;
    if (value <= 180) {
        // Treat as degrees
        if (value < 0)   value = 0;
        if (value > 180) value = 180;
        int us = _minUs + ((int32_t)value * (_maxUs - _minUs)) / 180;
        writeMicroseconds(us);
    } else {
        // Treat as microseconds
        writeMicroseconds(value);
    }
}

int Servo432::read() const
{
    if (!attached()) return SERVO432_INVALID;
    return (int)(((int32_t)(_currentUs - _minUs) * 180) / (_maxUs - _minUs));
}

int Servo432::readMicroseconds() const
{
    if (!attached()) return SERVO432_INVALID;
    return _currentUs;
}

bool Servo432::attached() const
{
    return _channel != SERVO432_INVALID;
}

void Servo432::setPeriodHertz(int /*hz*/)
{
    // Period is fixed at 50 Hz — no-op for API compatibility
}

#endif // __MSP432P401R__
