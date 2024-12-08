/*****************************************************************************/
/**
 * @file       low_pass_filter.h
 * @brief      base library
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE file.
 * @date       2024-11-01
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include <cstdint>
//#include <type_traits>

/*****************************************************************************/
namespace andrgrue::base {

/*****************************************************************************/

/**
 * @brief A simple low pass filter
 */
template<typename DataT = float
/* , typename = typename std::enable_if_t<std::is_floating_point<DataT>::value> */
>
class LowPassFilter {
  // data types
public:
  /// basic data type
  using DataType = DataT;

  // construction
public:
  /**
   * @brief constructor
   * @param alpha
   */
  LowPassFilter(const DataType& alpha = 0.5)
      : alpha_(alpha) {}
  virtual ~LowPassFilter() = default;

  // operations
public:
  /// get current estimate
  DataType estimate() const { return x_km; }
  /**
   * @brief update low pass filter, computes current estimate
   * @param x_k current measurement
   */
  void update(const DataType& x_k) {
    x_km = alpha_ * x_km + (1.0 - alpha_) * x_k;
  }

protected:
private:
  // data
public:
protected:
private:
  const DataType alpha_ {0.1};
  DataType       x_km {0.0};
};

/*****************************************************************************/
}  // namespace andrgrue::base
