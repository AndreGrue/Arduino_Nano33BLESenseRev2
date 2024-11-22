/*****************************************************************************/
/**
 * @file       atmospheric_pressure.h
 * @brief      library
 * @author     André Grüttner
 * @copyright  Please see the accompanying LICENSE.txt file.
 * @date       2024-11-13
 */
/*****************************************************************************/
#pragma once

/*****************************************************************************/
#include <cmath>
#include <cstdint>

/*****************************************************************************/
namespace andrgrue::base {

/*****************************************************************************/

/**
 * @brief atmospheric pressure
 */
template<typename DataT = float>
class AtmosphericPressure {
  // data types
public:
  /// basic data type
  using DataType = DataT;

  /// pressure reference using Q-code
  enum class PressureReference : uint32_t {
    NONE,
    QNE,  ///< Nautic elevation
    QNH,  ///< Nautic height
    QFE,  ///< (Air)field Elevation
  };

  /// pressure units
  enum class PressureUnit : uint32_t {
    PASCAL,       ///< [Pa]
    HECTOPASCAL,  ///< [hPa]
    KILOPASCAL,   ///< [kPa]
    MILLIBAR,     ///< [mbar]
    PSI,          ///< [psi]
    ATMOSPHERES,  ///< [atm]
    MM_HG,        ///< [mmHg]
    IN_HG,        ///< [inHg]
  };

  // construction
public:
  /**
   * @brief constructor
   */
  AtmosphericPressure()          = default;
  virtual ~AtmosphericPressure() = default;

  // operations
public:
  /*
   * @brief set none reference pressure
   * @param p reference pressure [Pa]
   * @param h reference altitude [m]
   * @param t reference temperature [K]
   */
  void setNoneReference(const DataType& p,
                        const DataType& h,
                        const DataType& t = -1.0) {
    Reference = PressureReference::NONE;
    P_ref     = p;
    h_ref     = h;
    T_ref     = t;
  }

  /*
   * @brief set QNE reference pressure
   */
  void setQNEReference() { Reference = PressureReference::QNE; }

  /*
   * @brief set QNH reference pressure
   * @param qnh reference pressure [Pa]
   */
  void setQNHReference(const DataType& qnh) {
    Reference = PressureReference::QNH;
    P_ref_qnh = qnh;
  }

  /*
   * @brief set QFE reference pressure
   * @param qfe reference pressure [Pa]
   * @param t_ref QFE reference temperature [K]
   */
  void setQFEReference(const DataType& qfe, const DataType& t_ref = 288.15) {
    Reference = PressureReference::QFE;
    P_ref_qfe = qfe;
    T_ref_qfe = t_ref;
  }

  /*
   * @brief compute altitude from barometric pressure
   * @note pressure reference needs to be set first
   * @param p current pressure [Pa]
   * @param t current temperature [K], needed for QNH reference
   */
  DataType altitude(const DataType& p, const DataType& t = -1.0) const {
    return altitude(Reference, p, t);
  }

  /**
   * @brief compute altitude using barometric formula
   * @param p current pressure [Pa]
   * @param p_ref reference pressure [Pa]
   * @param h_ref reference altitude [m]
   * @param t_ref reference temperature [K]
   * @return altitude in [m]
   */
  static DataType barometric_formula(const DataType& p,
                                     const DataType& p_ref,
                                     const DataType& h_ref,
                                     const DataType& t_ref) {
    DataType h;
    h = h_ref
        + (t_ref / L_r) * (std::pow((p / p_ref), (-R * L_r / (g * M))) - 1.0);
    return h;
  }

  /*
   * @brief convert pressure value from [Pa] to another unit
   * @param unit pressure unit to convert to
   * @param p pressure value in [Pa]
   */
  static DataType conversion(const PressureUnit& unit, const DataType& p) {
    switch (unit) {
    case PressureUnit::PASCAL:
      return p;
    case PressureUnit::HECTOPASCAL:
      return p / 100.0;
    case PressureUnit::KILOPASCAL:
      return p / 1000.0;
    case PressureUnit::MILLIBAR:
      return p / 100.0;
    case PressureUnit::PSI:
      return p * 0.000145038;
    case PressureUnit::ATMOSPHERES:
      return p / 101325.0;
    case PressureUnit::MM_HG:
      return p * 0.007500620;
    case PressureUnit::IN_HG:
      return p * 0.00029530;
    }
  }

protected:
  ///
  DataType altitude(const PressureReference ref,
                    const DataType&         p,
                    const DataType&         t = -1.0) const {
    switch (ref) {
    case PressureReference::NONE:
      if (T_ref == -1.0) {
        return barometric_formula(p, P_ref, h_ref, t);
      }
      return barometric_formula(p, P_ref, h_ref, T_ref);
    case PressureReference::QNE:
      return barometric_formula(p, P_ref_qne, h_ref_zero, T_ref_qne);
    case PressureReference::QFE:
      return barometric_formula(p, P_ref_qfe, h_ref_zero, T_ref_qfe);
    case PressureReference::QNH:
      return barometric_formula(p, P_ref_qnh, h_ref_zero, t);
    }
  }

private:
  // data
public:
protected:
  /// pressure reference system
  PressureReference Reference {PressureReference::NONE};

  /// current pressure estimate [Pa]
  DataType P {0.0};

  /// reference pressure [Pa]
  DataType P_ref {0.0};
  /// reference altitude [m]
  DataType h_ref {0.0};
  /// reference temperature [K]
  DataType T_ref {0.0};

  /// QNH reference pressure [Pa]
  DataType P_ref_qnh {0.0};

  /// QFE reference pressure [Pa]
  DataType P_ref_qfe {0.0};
  /// QFE reference temperature [K]
  DataType T_ref_qfe {0.0};

private:
  /// QNH reference altitude [m]
  static constexpr DataType h_ref_zero {0.0};

  /// QNE reference pressure [Pa]
  static constexpr DataType P_ref_qne {101325.0};
  /// QNE reference temperature [K]
  static constexpr DataType T_ref_qne {288.15};

  /// mean gravitational constant at sea level [m/s^2]
  static constexpr DataType g {9.80665};
  /// universal gas constant [J/(mol*K)]
  static constexpr DataType R {8.3144598};
  /// molar mass of dry air [kg/mol]
  static constexpr DataType M {0.0289644};
  /// temperature gradient (negative lapse rate) [K/m]
  static constexpr DataType L_r {-0.0065};
};

/*****************************************************************************/
}  // namespace andrgrue::base
