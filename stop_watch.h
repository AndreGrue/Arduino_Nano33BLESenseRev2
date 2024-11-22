/*****************************************************************************/
/**
 * @file       stop_watch.h
 * @brief      base library
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE file.
 * @date       2024-11-01
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include <cstdint>

/*****************************************************************************/
namespace andrgrue::base {

/*****************************************************************************/

/**
 * @brief A simple stop watch
 */
template<typename TimeT = uint64_t>
class StopWatch {
  // data types
public:
  /// basic time data type
  using TimeType = TimeT;
  /**
   */
  enum class TimeUnit : uint64_t {
    SEC  = 1,
    MSEC = 1000,
    USEC = 1000000,
    NSEC = 1000000000
  };
  /// time function pointer type
  using TimeFunctionType = TimeType (*)(void);

  // construction
public:
  /**
   * @brief constructor
   * @param fct time function pointer
   * @param timeUnit time unit
   */
  StopWatch(const TimeFunctionType fct, const TimeUnit timeUnit)
      : timeFunction_(fct)
      , timeUnit_(timeUnit) {}
  virtual ~StopWatch() = default;

  // operations
public:
  TimeUnit timeUnit() const { return timeUnit_; }
  TimeType startTime() const { return startTime_; }
  TimeType stopTime() const { return stopTime_; }
  TimeType elapsedTime() const { return elapsedTime_; }
  void     start() { startTime_ = timeFunction_(); }
  void     stop() {
    stopTime_ = timeFunction_();
    if (startTime_ < stopTime_) {
      elapsedTime_ = stopTime_ - startTime_;
    }
  }
  TimeType next_round() {
    stop();
    start();
    return elapsedTime_;
  }
  void restart() {
    reset();
    start();
  }
  void reset() {
    startTime_   = 0;
    stopTime_    = 0;
    elapsedTime_ = 0;
  }

protected:
private:
  // data
public:
protected:
private:
  const TimeUnit   timeUnit_ {TimeUnit::USEC};
  TimeFunctionType timeFunction_ {nullptr};
  TimeType         startTime_ {0};
  TimeType         stopTime_ {0};
  TimeType         elapsedTime_ {0};
};

/*****************************************************************************/
}  // namespace andrgrue::base
