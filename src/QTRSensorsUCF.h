/// \file QTRSensorsUCF.h

#pragma once

#include <stdint.h>

/// Default timeout for RC sensors (in microseconds).
const uint16_t QTRRCDefaultTimeout = 2500;

/// \brief Represents a QTR sensor array.
///
/// An instance of this class represents a QTR sensor array, consisting of one
/// or more sensors of the same type. This could be either a single QTR sensor
/// board or multiple boards controlled as a group.
///
/// See \ref md_usage for an overview of how this library can be used and some
/// example code.
class QTRSensorsUCF
{
  public:

    QTRSensorsUCF() = default;

    ~QTRSensorsUCF();

    /// \brief Sets the sensor pins.
    ///
    /// \param[in] pins A pointer to an array containing the Arduino pins that
    /// the sensors are connected to.
    ///
    /// \param sensorCount The number of sensors, which should match the length
    /// of the pins array.
    ///
    /// Example usage:
    /// ~~~{.cpp}
    /// // Set pins for four RC sensors connected to pins 6, 7, A0, and A1.
    /// // (Most analog pins can also be used as digital pins.)
    /// qtr.setTypeRC();
    /// qtr.setSensorPins((const uint8_t[]){6, 7, A0, A1}, 4);
    /// ~~~
    /// ~~~{.cpp}
    /// // Set pins for four analog sensors connected to pins A2, A3, A4, and A5.
    /// qtr.setTypeAnalog();
    /// qtr.setSensorPins((const uint8_t[]){A2, A3, A4, A5}, 4);
    /// ~~~
    ///
    /// If \link CalibrationData calibration data \endlink has already been
    /// stored, calling this method will force the storage for the calibration
    /// values to be reallocated and reinitialized the next time calibrate() is
    /// called (it sets `calibrationOn.initialized` and
    /// `calibrationOff.initialized` to false).
    void setSensorPin(const uint8_t pin);

    /// \brief Sets the timeout for RC sensors.
    ///
    /// \param timeout The length of time, in microseconds, beyond which you
    /// consider the sensor reading completely black.
    ///
    /// If the pulse length for a pin exceeds \p timeout, pulse timing will
    /// stop and the reading for that pin will be considered full black. It is
    /// recommended that you set \p timeout to be between 1000 and 3000
    /// &micro;s, depending on factors like the height of your sensors and
    /// ambient lighting. This allows you to shorten the duration of a
    /// sensor-reading cycle while maintaining useful measurements of
    /// reflectance. The default timeout is 2500 &micro;s.
    ///
    /// The maximum allowed timeout is 32767.
    /// (This prevents any possibility of an overflow when using
    /// QTRReadMode::OnAndOff or QTRReadMode::OddEvenAndOff).
    ///
    /// The timeout setting only applies to RC sensors.
    void setTimeout(uint16_t timeout);

    /// \brief Returns the timeout for RC sensors.
    ///
    /// \return The RC sensor timeout in microseconds.
    ///
    /// See also setTimeout().
    uint16_t getTimeout() { return _timeout; }

    /// \brief Reads the sensors for calibration.
    ///
    /// \param mode The emitter behavior during calibration, as a member of the
    /// ::QTRReadMode enum. The default is QTRReadMode::On. Manual emitter
    /// control with QTRReadMode::Manual is not supported.
    ///
    /// This method reads the sensors 10 times and uses the results for
    /// calibration. The sensor values are not returned; instead, the maximum
    /// and minimum values found over time are stored in #calibrationOn and/or
    /// #calibrationOff for use by the readCalibrated() method.
    ///
    /// If the storage for the calibration values has not been initialized,
    /// this function will (re)allocate the arrays and initialize the maximum
    /// and minimum values to 0 and the maximum possible sensor reading,
    /// respectively, so that the very first calibration sensor reading will
    /// update both of them.
    ///
    /// Note that the `minimum` and `maximum` pointers in the CalibrationData
    /// structs will point to arrays of length \p sensorCount, as specified in
    /// setSensorPins(), and they will only be allocated when calibrate() is
    /// called. If you only calibrate with the emitters on, the calibration
    /// arrays that hold the off values will not be allocated (and vice versa).
    ///
    /// See \ref md_usage for more information and example code.
    void calibrate();

    /// \brief Resets all calibration that has been done.
    void resetCalibration();

    /// \brief Reads the raw sensor values into an array.
    ///
    /// \param[out] sensorValues A pointer to an array in which to store the
    /// raw sensor readings. There **MUST** be space in the array for as many
    /// values as there were sensors specified in setSensorPins().
    ///
    /// \param mode The emitter behavior during the read, as a member of the
    /// ::QTRReadMode enum. The default is QTRReadMode::On.
    ///
    /// Example usage:
    /// ~~~{.cpp}
    /// uint16_t sensorValues[8];
    /// qtr.read(sensorValues);
    /// ~~~
    ///
    /// The values returned are a measure of the reflectance in abstract units,
    /// with higher values corresponding to lower reflectance (e.g. a black
    /// surface or a void).
    ///
    /// Analog sensors will return a raw value between 0 and 1023 (like
    /// Arduino's `analogRead()` function).
    ///
    /// RC sensors will return a raw value in microseconds between 0 and the
    /// timeout setting configured with setTimeout() (the default timeout is
    /// 2500 &micro;s).
    ///
    /// See \ref md_usage for more information and example code.
    uint16_t read();

    /// \brief Reads the sensors and provides calibrated values between 0 and
    /// 1000.
    ///
    /// \param[out] sensorValues A pointer to an array in which to store the
    /// calibrated sensor readings.  There **MUST** be space in the array for
    /// as many values as there were sensors specified in setSensorPins().
    ///
    /// \param mode The emitter behavior during the read, as a member of the
    /// ::QTRReadMode enum. The default is QTRReadMode::On. Manual emitter
    /// control with QTRReadMode::Manual is not supported.
    ///
    /// 0 corresponds to the minimum value stored in #calibrationOn or
    /// #calibrationOff, depending on \p mode, and 1000 corresponds to the
    /// maximum value. Calibration values are typically obtained by calling
    /// calibrate(), and they are stored separately for each sensor, so that
    /// differences in the sensors are accounted for automatically.
    ///
    /// See \ref md_usage for more information and example code.
    uint16_t readCalibrated();

    /// \brief Stores sensor calibration data.
    ///
    /// See calibrate() and readCalibrated() for details.
    struct CalibrationData
    {
      /// Whether array pointers have been allocated and initialized.
      bool initialized = false;
      /// Lowest readings seen during calibration.
      uint16_t minimum = 0;
      /// Highest readings seen during calibration.
      uint16_t maximum = 0;
    };

    /// \name Calibration data
    ///
    /// See calibrate() and readCalibrated() for details.
    ///
    /// These variables are made public so that you can use them for your own
    /// calculations and do things like saving the values to EEPROM, performing
    /// sanity checking, etc.
    /// \{



    /// \}

  private:

    /// Data from calibrating with emitters on.
    CalibrationData _calibrationData;

    uint8_t _sensorPin = 0;

    uint16_t _timeout = QTRRCDefaultTimeout; // only used for RC sensors
    uint16_t _maxValue = QTRRCDefaultTimeout; // the maximum value returned by readPrivate()

    uint16_t _lastPosition = 0;
};
