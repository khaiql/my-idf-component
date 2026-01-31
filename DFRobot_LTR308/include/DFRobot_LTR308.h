/**
 * @file DFRobot_LTR308.h
 * @brief Define the basic structure of class DFRobot_LTR308
 * @details Define the LTR308 functions, ported for ESP-IDF
 * @copyright Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license The MIT License (MIT)
 * @author [thdyyl](yuanlong.yu@dfrobot.com) (Ported by Antigravity)
 * @version V1.0.0
 * @date 2024-07-25
 * @url https://github.com/DFRobot/DFRobot_LTR308
 */
#ifndef __DFROBOT_LTR308_H
#define __DFROBOT_LTR308_H

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LTR308_ADDR 0x53 ///< Default address of the LTR308 sensor
#define LTR308_PART_V_ID                                                       \
  0xB1 ///< The part number and revision identification of the sensor.

// LTR308 register addresses
#define LTR308_REG_CONTR 0x00
#define LTR308_REG_MEAS_RATE 0x04
#define LTR308_REG_ALS_GAIN 0x05
#define LTR308_REG_PART_ID 0x06
#define LTR308_REG_STATUS 0x07
#define LTR308_REG_DATA_0 0x0D
#define LTR308_REG_DATA_1 0x0E
#define LTR308_REG_DATA_2 0x0F
#define LTR308_REG_INTERRUPT 0x19
#define LTR308_REG_INTR_PERS 0x1A
#define LTR308_REG_THRES_UP_0 0x21
#define LTR308_REG_THRES_UP_1 0x22
#define LTR308_REG_THRES_UP_2 0x23
#define LTR308_REG_THRES_LOW_0 0x24
#define LTR308_REG_THRES_LOW_1 0x25
#define LTR308_REG_THRES_LOW_2 0x26

class DFRobot_LTR308 {
public:
  /**
   * @enum eGain_t
   * @brief Enumerated gain
   */
  typedef enum {
    eGain_1X = 0x00,
    eGain_3X,
    eGain_6X,
    eGain_9X,
    eGain_18X
  } eGain_t;

  /**
   * @enum eIntrPersist_t
   * @brief Enumeration interrupts persistence
   */
  typedef enum {
    eInterruptTrigger_1 = 0x00,
    eInterruptTrigger_2,
    eInterruptTrigger_3,
    eInterruptTrigger_4,
    eInterruptTrigger_5,
    eInterruptTrigger_6,
    eInterruptTrigger_7,
    eInterruptTrigger_8,
    eInterruptTrigger_9,
    eInterruptTrigger_10,
    eInterruptTrigger_11,
    eInterruptTrigger_12,
    eInterruptTrigger_13,
    eInterruptTrigger_14,
    eInterruptTrigger_15,
    eInterruptTrigger_16
  } eIntrPersist_t;

  /**
   * @enum eResolution_t
   * @brief Enumeration conversion time
   */
  typedef enum {
    eConversion_400ms_20b = 0x00,
    eConversion_200ms_19b,
    eConversion_100ms_18b,
    eConversion_50ms_17b,
    eConversion_25ms_16b
  } eResolution_t;

  /**
   * @enum eMeasurementRate_t
   * @brief Enumeration measurement rate
   */
  typedef enum {
    eRate_25ms = 0x00,
    eRate_50ms,
    eRate_100ms,
    eRate_500ms,
    eRate_1000ms = 5,
    eRate_2000ms,
    eRate_2000ms_2
  } eMeasurementRate_t;

  /**
   * @struct sMeasRate_t
   * @brief ALS measurement rate and resolution in Active Mode
   */
  typedef struct {
    eResolution_t resolution;
    eMeasurementRate_t measurementRate;
  } sMeasRate_t;

  /**
   * @struct sMainStatus_t
   * @brief  The sensor status: Power-On status, Interrupt status, Data status
   */
  typedef struct {
    bool ponStatus;
    bool intrStatus;
    bool dataStatus;
  } sMainStatus_t;

  /**
   * @struct sThres_t
   * @brief  ALS interrupt threshold
   */
  typedef struct {
    uint32_t upperLimit;
    uint32_t lowerLimit;
  } sThres_t;

private:
  i2c_master_dev_handle_t _i2cDevice;
  eGain_t _gain;
  eResolution_t _resolution;
  eMeasurementRate_t _measurementRate;

public:
  /**
   * @brief Constructor
   * @param handle I2C device handle
   */
  DFRobot_LTR308(i2c_master_dev_handle_t handle);

  /**
   * @fn begin
   * @brief Initialize the function.
   * @return bool ,Indicates the initialization state.
   */
  bool begin(void);

  /**
   * @fn setPowerUp
   * @brief Set the Power Up LTR308.
   */
  void setPowerUp(void);

  /**
   * @fn setPowerDown
   * @brief Set the Power Down LTR308
   */
  void setPowerDown(void);

  /**
   * @fn getPower
   * @brief Get the Power value
   * @return uint8_t: content of LTR308_CONTR control register
   */
  uint8_t getPower(void);

  /**
   * @fn setGain
   * @brief Set the Gain of LTR308
   * @param gain eGain_t
   */
  void setGain(eGain_t gain);

  /**
   * @fn getGain
   * @brief Get the Gain of LTR308
   * @return uint8_t: the control register values
   */
  uint8_t getGain(void);

  /**
   * @fn setMeasurementRate
   * @brief Set the Measurement Rate and resolution
   * @param measRate: (sMeasRate_t)  Measurement Rate and resolution
   */
  void setMeasurementRate(sMeasRate_t measRate);

  /**
   * @fn setMeasurementRate
   * @brief Set the Measurement Rate and resolution
   * @param resolution: (eResolution_t)
   * @param measurementRate: (eMeasurementRate_t)
   */
  void setMeasurementRate(eResolution_t resolution,
                          eMeasurementRate_t measurementRate);

  /**
   * @fn getMeasurementRate
   * @brief Get the Measurement Rate and resolution
   * @return sMeasRate_t: Measurement Rate and resolution
   */
  sMeasRate_t getMeasurementRate(void);

  /**
   * @fn getPartID
   * @brief Gets the part number ID and revision ID of the chip
   * @return uint8_t: Default value is 0xB1
   */
  uint8_t getPartID(void);

  /**
   * @fn getStatus
   * @brief Get the Status of LTR308
   * @return sMainStatus_t: The status information of LTR308.
   */
  sMainStatus_t getStatus(void);

  /**
   * @fn getData
   * @brief Get the ALS channel data
   * @return uint32_t: ALS data, Default value is 0
   */
  uint32_t getData(void);

  /**
   * @fn setInterruptControl
   * @brief Set the Interrupt Control of LTR308
   * @param mode: interrupt mode
   */
  void setInterruptControl(bool mode);

  /**
   * @fn getInterruptControl
   * @brief Get the Interrupt Control of LTR308
   * @return bool: interrupt mode
   */
  bool getInterruptControl(void);

  /**
   * @fn setIntrPersist
   * @brief Set the Intr Persist of LTR308
   * @param persist: controls the N number of times the measurement data is
   * outside the range
   */
  void setIntrPersist(eIntrPersist_t persist);

  /**
   * @fn getIntrPersist
   * @brief Get the Intr Persist of LTR308
   * @return uint8_t
   */
  uint8_t getIntrPersist(void);

  /**
   * @fn setThreshold
   * @brief Sets the upper limit and lower limit of the threshold
   * @param thres: upperLimit and lowerLimit
   */
  void setThreshold(sThres_t thres);

  /**
   * @fn setThreshold
   * @brief Sets the upper limit and lower limit of the threshold
   * @param upperLimit
   * @param lowerLimit
   */
  void setThreshold(uint32_t upperLimit, uint32_t lowerLimit);

  /**
   * @fn getThreshold
   * @brief Gets the upper limit and lower limit of the threshold
   * @return sThres_t: upperLimit and lowerLimit
   */
  sThres_t getThreshold();

  /**
   * @fn getLux
   * @brief Convert raw data to lux
   * @param gain: The gain of this sensor
   * @param resolution: The resolution of this sensor
   * @param alsData: The ALS Data of this sensor
   * @return double: The converted lux value
   */
  double getLux(eGain_t gain, eResolution_t resolution, uint32_t alsData);

  /**
   * @fn getLux
   * @brief Convert raw data to lux
   * @param alsData: The ALS Data of this sensor
   * @return double: The converted lux value
   */
  double getLux(uint32_t alsData);
  ~DFRobot_LTR308();

private:
  /**
   * @fn readReg
   * @brief Read register function
   * @param reg: Register address 8bits
   * @param pBuf: Storage and buffer for data to be read
   * @param size: Length of data to be read
   * @return bool: Indicates returning read register status
   */
  bool readReg(uint8_t reg, void *pBuf, size_t size);

  /**
   * @fn writeReg
   * @brief Write register function
   * @param reg: Register address 8bits
   * @param pBuf: Storage and buffer for data to be written
   * @param size: Length of data to be written
   * @return bool: Indicates returning write register status
   */
  bool writeReg(uint8_t reg, void *pBuf, size_t size);
};

#endif
