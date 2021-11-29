/*!
 * @file DFRobot_SHT3x.h
 * @brief Define the infrastructure of the DFRobot_SHT3x class
 * @n This is a library of digital temperature and humidity sensors used to
 * drive the SHT3x series SHT30, SHT31 and SHT35 to read ambient temperature and
 * relative humidity.
 * @copyright   Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @licence     The MIT License (MIT)
 * @author [fengli](li.feng@dfrobot.com)
 * @version  V1.0
 * @date  2019-08-19
 * @get from https://www.dfrobot.com
 * @url https://github.com/DFRobot/DFRobot_SHT3x
 */

#pragma once

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

//#define ENABLE_DBG

#ifdef ENABLE_DBG
#define DBG(...)                                                               \
  {                                                                            \
    Serial.print("[");                                                         \
    Serial.print(__FUNCTION__);                                                \
    Serial.print("(): ");                                                      \
    Serial.print(__LINE__);                                                    \
    Serial.print(" ] ");                                                       \
    Serial.println(__VA_ARGS__);                                               \
  }
#else
#define DBG(...)
#endif
#define SHT3X_CMD_READ_SERIAL_NUMBER (0x3780) // Read the chip serial number
#define SHT3X_CMD_GETDATA_H_CLOCKENBLED                                        \
  (0x2C06) // Measurement:high repeatability
#define SHT3X_CMD_GETDATA_M_CLOCKENBLED                                        \
  (0x2C0D) // Measurement: medium repeatability
#define SHT3X_CMD_GETDATA_L_CLOCKENBLED                                        \
  (0x2C10) // Measurement: low repeatability

#define SHT3X_CMD_SETMODE_H_FREQUENCY_HALF_HZ                                  \
  (0x2032) // Measurement: periodic 0.5 mps, high repeatability
#define SHT3X_CMD_SETMODE_M_FREQUENCY_HALF_HZ                                  \
  (0x2024) // Measurement: periodic 0.5 mps, medium
#define SHT3X_CMD_SETMODE_L_FREQUENCY_HALF_HZ                                  \
  (0x202F) // Measurement: periodic 0.5 mps, low repeatability
#define SHT3X_CMD_SETMODE_H_FREQUENCY_1_HZ                                     \
  (0x2130) // Measurement: periodic 1 mps, high repeatability
#define SHT3X_CMD_SETMODE_M_FREQUENCY_1_HZ                                     \
  (0x2126) // Measurement: periodic 1 mps, medium repeatability
#define SHT3X_CMD_SETMODE_L_FREQUENCY_1_HZ                                     \
  (0x212D) // Measurement: periodic 1 mps, low repeatability
#define SHT3X_CMD_SETMODE_H_FREQUENCY_2_HZ                                     \
  (0x2236) // Measurement: periodic 2 mps, high repeatability
#define SHT3X_CMD_SETMODE_M_FREQUENCY_2_HZ                                     \
  (0x2220) // Measurement: periodic 2 mps, medium repeatability
#define SHT3X_CMD_SETMODE_L_FREQUENCY_2_HZ                                     \
  (0x222B) // Measurement: periodic 2 mps, low repeatability
#define SHT3X_CMD_SETMODE_H_FREQUENCY_4_HZ                                     \
  (0x2334) // Measurement: periodic 4 mps, high repeatability
#define SHT3X_CMD_SETMODE_M_FREQUENCY_4_HZ                                     \
  (0x2322) // Measurement: periodic 4 mps, medium repeatability
#define SHT3X_CMD_SETMODE_L_FREQUENCY_4_HZ                                     \
  (0x2329) // Measurement: periodic 4 mps, low repeatability
#define SHT3X_CMD_SETMODE_H_FREQUENCY_10_HZ                                    \
  (0x2737) // Measurement: periodic 10 mps, high repeatability
#define SHT3X_CMD_SETMODE_M_FREQUENCY_10_HZ                                    \
  (0x2721) // Measurement: periodic 10 mps, medium
#define SHT3X_CMD_SETMODE_L_FREQUENCY_10_HZ                                    \
  (0x272A) // Measurement: periodic 10 mps, low repeatability
#define SHT3X_CMD_GETDATA (0xE000) // Readout measurements for periodic mode

#define SHT3X_CMD_STOP_PERIODIC_ACQUISITION_MODE (0x3093)
#define SHT3X_CMD_SOFT_RESET (0x30A2)       // Soft reset
#define SHT3X_CMD_HEATER_ENABLE (0x306D)    // Enabled heater
#define SHT3X_CMD_HEATER_DISABLE (0x3066)   // Disable heater
#define SHT3X_CMD_READ_STATUS_REG (0xF32D)  // Read status register
#define SHT3X_CMD_CLEAR_STATUS_REG (0x3041) // Clear status register

#define SHT3X_CMD_READ_HIGH_ALERT_LIMIT_SET                                    \
  (0xE11F) // Read alert limits, high set
#define SHT3X_CMD_READ_HIGH_ALERT_LIMIT_CLEAR                                  \
  (0xE114) // Read alert limits, high clear
#define SHT3X_CMD_READ_LOW_ALERT_LIMIT_CLEAR                                   \
  (0xE109) // Read alert limits, low clear
#define SHT3X_CMD_READ_LOW_ALERT_LIMIT_SET                                     \
  (0xE102) // Read alert limits, low set
#define SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_SET                                   \
  (0x611D) // Write alert limits, high set
#define SHT3X_CMD_WRITE_HIGH_ALERT_LIMIT_CLEAR                                 \
  (0x6116) // Write alert limits, high clear
#define SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_CLEAR                                  \
  (0x610B) // Write alert limits, low clear
#define SHT3X_CMD_WRITE_LOW_ALERT_LIMIT_SET                                    \
  (0x6100) // Write alert limits, low set
class DFRobot_SHT3x {
public:
#define ERR_OK 0          // No error
#define ERR_DATA_BUS -1   // Data bus error
#define ERR_IC_VERSION -2 // Chip version does not match
#define ERROR_BAD_CRC 999
  /*!
   The status register contains information on the operational status of the
   heater, the alert mode and on the execution status of the last command and
   the last write sequence.

   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
            b15       |       b14 |      b13   |      b12 |        b11   | b10
   |  b5~b9    |    b4              |    b2~b3 |    b1       |       b0 |
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
    alertPendingStatus| reserved3 | heaterStaus|reserved2 |humidityAlert |
   temperatureAlert | reserved1 | systemResetDeteced |reserved0
   |commendStatus|writeDataChecksumStatus|
   ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   writeDataChecksumStatus:   '0' : checksum of last write transfer was correct
                              '1' : checksum of last write transfer failed
   commendStatus              '0' : last command executed successfully
                              '1' : last command not processed.
   systemResetDeteced         '0' : no reset detected since last ‘clear status
   register’ command '1' : reset detected (hard reset, soft reset command or
   supply fail) temperatureAlert           '0' : no alert '1' : alert
   humidityAlert              '0' : no alert
                              '1' : alert
   heaterStaus                '0' : heater OFF
                              '1' : heater ON
   alertPendingStatus         '0' : no pending alerts
                              '1' : at least one pending alert
  */
  typedef struct {
    uint8_t writeDataChecksumStatus : 1;
    uint8_t commandStatus : 1;
    uint8_t reserved0 : 2;
    uint8_t systemResetDeteced : 1;
    uint8_t reserved1 : 5;
    uint8_t temperatureAlert : 1;
    uint8_t humidityAlert : 1;
    uint8_t reserved2 : 1;
    uint8_t heaterStaus : 1;
    uint8_t reserved3 : 1;
    uint8_t alertPendingStatus : 1;
  } __attribute__((packed)) sStatusRegister_t;

  /**
    Two measurement modes for the chip
  */
  typedef enum {
    ePeriodic, /**<Cycle measurement mode*/
    eOneShot,  /**<Single measurement mode*/
  } eMode_t;

  /*!
   *We can choose the repeatability of the chip to measure temperature and
   *humidity data (which means the difference between the data measured by the
   *chip under two identical measurementconditions). There are 3 repeatabilities
   *to choose: low, medium and high. The higher repeatability, the more accurate
   *data.
   */
  typedef enum {
    eRepeatability_High =
        0, /**<In high repeatability mode, the humidity repeatability is
              0.10%RH, the temperature repeatability is 0.06°C*/
    eRepeatability_Medium =
        1, /**<In medium repeatability mode, the humidity repeatability is
              0.15%RH, the temperature repeatability is 0.12°C*/
    eRepeatability_Low =
        2, /**<In low repeatability mode, the humidity repeatability is0.25%RH,
              the temperature repeatability is 0.24°C*/
  } eRepeatability_t;

  /*!
    Under the periodic data acquisition mode, we can select the frequency at
    which the chip measures temperature and humidity data. Optional frequencies
    are 0.5Hz, 1Hz, 2Hz, 4Hz, 10Hz.
   */
  typedef enum {
    eMeasureFreq_Hz5 = 0,
    eMeasureFreq_1Hz = 1,
    eMeasureFreq_2Hz = 2,
    eMeasureFreq_4Hz = 3,
    eMeasureFreq_10Hz = 4,
  } eMeasureFrequency_t;

  /**
    Structures used to store temperature and relative humidity
  */
  typedef struct {
    float TemperatureC;
    float Humidity;
    float TemperatureF;
    int ERR;
  } sRHAndTemp_t;

  /**
    Structures used to store the limits of temperature and relative humidity
    read
  */
  typedef struct {
    float highSet;   /**<Define the temperature (C)/humidity (%RH) range upper
                        threshold, ALERT generates a high-level alarm once the
                        data greater than the value defined*/
    float highClear; /**<Clear the alarm once the temperature (C)/humidity (%RH)
                        less than the value defined>*/
    float lowSet;    /**<Define the temperature (C)/humidity (%RH) range low
                        threshold,ALERT generates a high-level alarm once the data
                        lower than the value defined>*/
    float lowClear;  /**<Clear the alarm once the temperature (C)/humidity (%RH)
                        more than the value defined>*/
  } sLimitData_t;

public:
  DFRobot_SHT3x(TwoWire *pWire = &Wire, uint8_t address = 0x45,
                uint8_t RST = 4);

  void init();
  float readTemperature();
  float readHumidity();
  bool measure_is_ok(float measure);

  uint32_t readSerialNumber();
  int begin();
  bool softReset();
  bool pinReset();
  sRHAndTemp_t readTemperatureAndHumidity(eRepeatability_t repeatability);
  float getTemperatureC();
  float getTemperatureF();
  float getHumidityRH();
  bool startPeriodicMode(eMeasureFrequency_t measureFreq,
                         eRepeatability_t repeatability = eRepeatability_High);
  sRHAndTemp_t readTemperatureAndHumidity();
  bool stopPeriodicMode();
  bool heaterEnable();
  bool heaterDisable();
  void clearStatusRegister();
  bool readAlertState();
  uint8_t environmentState();
  uint8_t setTemperatureLimitC(float highset, float highclear, float lowset,
                               float lowclear);
  uint8_t setTemperatureLimitF(float highset, float highclear, float lowset,
                               float lowclear);
  uint8_t setHumidityLimitRH(float highset, float highclear, float lowset,
                             float lowclear);
  bool measureTemperatureLimitC();
  float getTemperatureHighSetC();
  float getTemperatureHighClearC();
  float getTemperatureLowClearC();
  float getTemperatureLowSetC();
  bool measureTemperatureLimitF();
  float getTemperatureHighSetF();
  float getTemperatureHighClearF();
  float getTemperatureLowClearF();
  float getTemperatureLowSetF();
  bool measureHumidityLimitRH();
  float getHumidityHighSetRH();
  float getHumidityHighClearRH();
  float getHumidityLowClearRH();
  float getHumidityLowSetRH();

private:
  void writeCommand(uint16_t cmd, size_t size);
  sStatusRegister_t readStatusRegister();
  void writeLimitData(uint16_t cmd, uint16_t limitData);
  uint8_t readLimitData(uint16_t cmd, uint16_t *pBuf);
  uint8_t readData(void *pBuf, size_t size);
  uint8_t checkCrc(uint8_t data[]);
  float convertTemperature(uint8_t rawTemperature[]);
  float convertHumidity(uint8_t rawHumidity[]);
  uint16_t convertRawTemperature(float value);
  uint16_t convertRawHumidity(float value);
  float convertTempLimitData(uint16_t limit[]);
  float convertHumidityLimitData(uint16_t limit[]);
  void write(const void *pBuf, size_t size);

private:
  sLimitData_t limitData;
  sRHAndTemp_t tempRH;
  TwoWire *_pWire;
  eMode_t measurementMode;
  uint8_t _address;
  uint8_t _RST;
  float tempHighSet;
  float tempLowSet;
};