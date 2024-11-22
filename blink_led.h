/*****************************************************************************/
/**
 * @file       blink_led.h
 * @brief      arduino library
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE file.
 * @date       2024-11-01
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include "Arduino.h"

/*****************************************************************************/
namespace andrgrue::arduino::std {

/*****************************************************************************/

/**
 *
 */
class BlinkLed {
  // construction
public:
  BlinkLed(const int ledPin = LED_BUILTIN, const long interval = 1000)
      : interval_(interval)
      , ledPin_(ledPin) {}
  virtual ~BlinkLed() = default;

  // operations
public:
  void setup() { pinMode(ledPin_, OUTPUT); }

  void loop() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis_ >= interval_) {
      previousMillis_ = currentMillis;
      toggle();
    }
  }

protected:
  void toggle() {
    ledState_ = (ledState_ == LOW) ? HIGH : LOW;
    digitalWrite(ledPin_, ledState_);
  }

private:
  // data
public:
protected:
private:
  const long interval_ {1000};  ///< interval at which to blink (milliseconds)
  const int  ledPin_ {LED_BUILTIN};   ///< the number of the LED pin
  int        ledState_ {LOW};         ///< ledState used to set the LED
  unsigned long previousMillis_ {0};  // will store last time LED was updated
};

/*****************************************************************************/
}  // namespace andrgrue::arduino::std
