/*
 * Servo432.h
 *
 * Jitter-free RC servo driver for MSP432P401R using Timer_A hardware PWM.
 * Replaces the Energia Servo library for MSP432 targets.
 *
 * The Energia Servo library drives servos via software in a Timer32 ISR,
 * causing jitter from floating-point tick calculation and driver overhead.
 * This class uses Timer_A in hardware PWM mode: once the compare register
 * is written, the hardware toggles the pin with zero CPU involvement.
 *
 * Supported pins and their hardware assignments:
 *
 *   Energia pin 38 → P2.4 → Timer_A0, CCR1
 *   Energia pin 39 → P2.6 → Timer_A0, CCR3
 *   Energia pin 40 → P2.7 → Timer_A0, CCR4
 *   Energia pin 37 → P5.6 → Timer_A2, CCR1
 *
 * Timer_A0 and Timer_A2 are both configured at 50 Hz (20 ms period) using
 * SMCLK/8 = 1.5 MHz. Do not use analogWrite() on these pins while any
 * Servo432 instance is attached.
 *
 * The interface is intentionally identical to the Arduino/Energia Servo class
 * so it can be used as a drop-in replacement under a #ifdef __MSP432P401R__.
 */

#pragma once

#if defined(__MSP432P401R__) || defined(__MSP432__)

#include <stdint.h>

// Returned by attach() on failure, and by read()/readMicroseconds() if unattached
#define SERVO432_INVALID  0xFF

class Servo432 {
public:
    Servo432();

    // Attach to a Timer_A hardware PWM pin.
    // Returns the CCR channel (1, 3, or 4) on success, SERVO432_INVALID on failure.
    uint8_t attach(int pin);
    uint8_t attach(int pin, int minUs, int maxUs);

    void detach();

    // Write servo position.  Values 0–180 are treated as degrees; values above
    // MIN_PULSE_WIDTH are treated as microseconds (same convention as Energia Servo).
    void write(int value);
    void writeMicroseconds(int us);

    int  read() const;            // current position in degrees (0–180)
    int  readMicroseconds() const;// current pulse width in microseconds
    bool attached() const;

    // No-op: period is fixed at 50 Hz. Provided for API compatibility with
    // ESP32Servo which requires this call before attach().
    void setPeriodHertz(int hz);

private:
    uint8_t  _channel;    // CCR index (1, 3, or 4), or SERVO432_INVALID
    uint32_t _timerBase;  // TIMER_A0_BASE or TIMER_A2_BASE
    int      _minUs;
    int      _maxUs;
    int      _currentUs;

    // One-time timer initialisation, called by attach()
    static void _initTimer(uint32_t base);

    // Convert microseconds to Timer_A ticks (integer, no float)
    static uint16_t _usToPulse(int us);

    // Write ticks directly to the hardware CCR register
    void _writePulse(uint16_t ticks) const;

    static bool _timerA0Started;
    static bool _timerA2Started;
};

#endif // __MSP432P401R__
