/**
 *
 */
#include <Arduino.h>
#include <Wire.h>

#include "stop_watch.h"
andrgrue::base::StopWatch<uint32_t> stpwtch(&micros, andrgrue::base::StopWatch<uint32_t>::TimeUnit::USEC);
#include "low_pass_filter.h"
andrgrue::base::LowPassFilter<float> lpf(0.5f);
#include "blink_led.h"
andrgrue::arduino::std::BlinkLed led(LED_BUILTIN, 250);
#include "st_lps22hb.h"
andrgrue::sensor::st_lps22hb Pressure(Wire1,
                                      andrgrue::sensor::LPS22HB_DEVICE_ADDRESS_NANO33BLE,
                                      p12);
#include "atmospheric_pressure.h"
andrgrue::base::AtmosphericPressure<float> ap;
#include "bosch_bmi270.h"
andrgrue::sensor::bosch_bmi270 Imu(Wire1,
                                   BMI2_I2C_PRIM_ADDR,
                                   p11);
#include "bosch_bmm150.h"
andrgrue::sensor::bosch_bmm150 Mag(Wire1);

andrgrue::sensor::st_lps22hb::Data pressure_data;
andrgrue::sensor::st_lps22hb::Data temperature_data;
andrgrue::sensor::bosch_bmi270::Data acc_data;
andrgrue::sensor::bosch_bmi270::Data gyro_data;
andrgrue::sensor::bosch_bmm150::Data mag_data;

int acc_count = 0;
int gyro_count = 0;
int mag_count = 0;
int press_count = 0;
volatile bool interruptFlag = false;
volatile bool imuInterruptFlag = false;
int cycle = 0;

void imu_interrupt_handler() {
  imuInterruptFlag = true;
}

void pressure_interrupt_handler() {
  interruptFlag = true;
}

/**
 *
 */
void setup() {
  Serial.begin(500000);
  Serial.flush();
  while (!Serial);
  Serial.println("Version: 0.0.37");
  delay(1000);

  Wire1.begin();
  Wire1.setClock(400000);
  Wire1.flush();
  delay(1000);

  if (Pressure.initialize(andrgrue::sensor::st_lps22hb::Rate::RATE_50HZ,
                          andrgrue::sensor::st_lps22hb::LowPassFilter::LPF_20,
                          pressure_interrupt_handler)) {
    Serial.println("LPS22HB Pressure Sensor found.");
    // ap.setQNEReference();
    ap.setQFEReference(102400.0);
  }
  else {
    Serial.println("LPS22HB Pressure Sensor not found.");
    while (true) {  }   // loop forever
  }

  Wire1.flush();
  delay(1000);

  imuInterruptFlag = false;
  if(!Imu.initialize(BMI2_ACC_ODR_100HZ, BMI2_ACC_RANGE_8G,
                     BMI2_GYR_ODR_100HZ, BMI2_GYR_RANGE_2000,
                     imu_interrupt_handler)) {
    Serial.println("IMU init failed!");
    while (1);
  }

  Wire1.flush();
  delay(1000);

  if(!Mag.initialize(BMM150_DATA_RATE_25HZ)) {
    Serial.println("MAG init failed!");
    while (1);
  }

  Serial.print("sample rate: acc=");
  Serial.println(Imu.accelerationSampleRate());
  Serial.print("sample rate: gyro=");
  Serial.println(Imu.angularrateSampleRate());
  Serial.print("sample rate: mag=");
  Serial.println(Mag.magneticfieldSampleRate());

  led.setup();
 }
 
/**
 *
 */
void loop() {
  andrgrue::base::StopWatch<>::TimeType t = stpwtch.next_round();
  lpf.update(t);
  led.loop();
  // delay(1);


  if(imuInterruptFlag){
    if(Imu.accelerationAvailable()){
      Imu.acceleration(acc_data);
      acc_count++;
    }
    if(Imu.angularrateAvailable()){
      Imu.angularrate(gyro_data);
      gyro_count++;
    }
    imuInterruptFlag = false;

    if(Mag.magneticfieldAvailable()){
      Mag.magneticfield(mag_data);
      mag_count++;
    }
  } else {
    // Serial.println("Interrupt failed!");
    // if(!Imu.initialize(imu_interrupt_handler)) {
    //   Serial.println("IMU init failed!");
    // }
    // return;
  }

  // if (Pressure.dataAvailable()) {
  if (interruptFlag) {
    // pressure = Pressure.pressure();
    // temperature = Pressure.temperature();
    Pressure.pressure(pressure_data);
    Pressure.temperature(temperature_data);
    press_count++;
    interruptFlag = false;
    // Serial.println("Pressure read");
  }


  cycle++;
  if(cycle % 20000 == 0) {

    Serial.print("loop_freq[Hz]:");
    Serial.print(static_cast<std::underlying_type_t<andrgrue::base::StopWatch<uint32_t>::TimeUnit>>(stpwtch.timeUnit()) / lpf.estimate());
    // Serial.print(t);
    Serial.print(",");

    Serial.print("acc_count:");
    Serial.print(acc_count);
    Serial.print(",");
    // Serial.print("acc_x:");
    // Serial.print(acc_x);
    // Serial.print(",");
    // Serial.print("acc_y:");
    // Serial.print(acc_y);
    // Serial.print(",");
    // Serial.print("acc_z:");
    // Serial.print(acc_z);
    // Serial.print(",");
    //
    // Serial.print("gyro_count:");
    // Serial.print(gyro_count);
    // Serial.print(",");
    // Serial.print("gyro_x:");
    // Serial.print(gyro_x);
    // Serial.print(",");
    // Serial.print("gyro_y:");
    // Serial.print(gyro_y);
    // Serial.print(",");
    // Serial.print("gyro_z:");
    // Serial.print(gyro_z);
    // Serial.print(",");
    //
    Serial.print("mag_count:");
    Serial.print(mag_count);
    Serial.print(",");
    Serial.print("mag_x:");
    Serial.print(mag_data.x);
    Serial.print(",");
    Serial.print("mag_y:");
    Serial.print(mag_data.y);
    Serial.print(",");
    Serial.print("mag_z:");
    Serial.print(mag_data.z);
    Serial.print(",");
 
    // Serial.print("press_count:");
    // Serial.print(press_count);
    // Serial.print(",");
    Serial.print("altitude:");
    Serial.println(ap.altitude(pressure_data.value));
    // Serial.print(", ");
    // Serial.print("timestamp:");
    // Serial.print(static_cast<uint32_t>(pressure_data.timestamp/1000));
    // Serial.print(",");
    // Serial.print("pressure:");
    // // Serial.print(andrgrue::base::AtmosphericPressure<>::conversion(andrgrue::base::AtmosphericPressure<>::PressureUnit::HECTOPASCAL, pressure_data.value));
    // Serial.print( pressure_data.value);
    // Serial.print(",");
    // Serial.print("p_var:");
    // Serial.print(pressure_data.variance);
    // Serial.print(",");
    // Serial.print("temperature:");
    // Serial.println(temperature_data.value);
  
  }
}
