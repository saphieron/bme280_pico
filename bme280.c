#include "bme280.h"
#include "bme280_internal.h"
#include "i2c_handler.h"

#include <stdbool.h>

// Register Addresses of the BME280
#define REG_RESET_ADDR 0xE0
#define REG_HUMIDITY_CTRL_ADDR 0xF2
#define REG_STATUS_ADDR 0xF3
#define REG_MEASURE_CONTROL_ADDR 0xF4
#define REG_CONFIG_ADDR 0xF5
#define REG_DEVICE_ID_ADDR 0xD0


// Magic values used by/in the BME280
#define REG_RESET_VALUE 0xB6
#define STANDBY_TIME_POS 5
#define IIR_FILTER_COEFFICIENT_POS 2

// Local helpers to avoid magic numbers
#define BITMASK_LOWEST_THREE 0x07
#define BITMASK_LOWEST_TWO 0x03

#define TEMP_OVERSAMPLING_POS 5
#define PRESSURE_OVERSAMPLING_POS 2

// ###############################################
// Helper Function definitions
// ###############################################

// ###############################################
//
// ###############################################

int32_t bme280_init(uint8_t address, bmeDevice_t* device) {
    if(device == 0){
        return BME280_NULL_POINTER_ERROR;
    }
    if(address == 0x00 || address== 0x01 ||address== 0x02||address== 0x03){
        return BME280_RESERVED_ADDR_ERROR;
    }
    device->address = address;
    int32_t errorCode = bme280_internal_readTrimmingValues(device);
    if(errorCode != BME280_NO_ERROR){
        return errorCode;
    }
    return BME280_NO_ERROR;
}

int32_t bme280_getId(bmeDevice_t* device) {
//    uint8_t idRegister = REG_DEVICE_ID_ADDR;
    uint8_t response = 0;
    int32_t commResult = bme280_internal_readFromRegister(device, REG_DEVICE_ID_ADDR, &response, 1);
    if (commResult != BME280_NO_ERROR) {
        return commResult;
    } else {
        return ((int32_t) response);
    }
}

int32_t bme280_resetDevice(bmeDevice_t* device) {
    uint8_t buffer[2] = {REG_RESET_ADDR, REG_RESET_VALUE};
    return bme280_internal_writeToRegister(device, buffer, 2);
}

void
bme280_prepareMeasureCtrlReg(bmeDevice_t* device, uint8_t tempOversampling, uint8_t pressureOversampling,
                                 uint8_t sensorMode) {
    pressureOversampling &= BITMASK_LOWEST_THREE;
    sensorMode &= BITMASK_LOWEST_TWO;
    device->registerMeasureCtrl =
            tempOversampling << TEMP_OVERSAMPLING_POS | pressureOversampling << PRESSURE_OVERSAMPLING_POS | sensorMode;
}

int32_t bme280_commitMeasureCtrlReg(bmeDevice_t* device) {
    uint8_t buffer[2] = {REG_MEASURE_CONTROL_ADDR, device->registerMeasureCtrl};
    return bme280_internal_writeToRegister(device, buffer, 2);
}

void bme280_prepareConfigReg(bmeDevice_t* device, uint8_t standbyTime, uint8_t iirFilterCoefficient) {
    iirFilterCoefficient &= BITMASK_LOWEST_THREE;
    device->registerConfig =
            standbyTime << STANDBY_TIME_POS | iirFilterCoefficient << IIR_FILTER_COEFFICIENT_POS;
}

int32_t bme280_commitConfigReg(bmeDevice_t* device) {
    uint8_t buffer[] = {REG_CONFIG_ADDR, device->registerConfig};
    return bme280_internal_writeToRegister(device, buffer, 2);
}

void bme280_prepareCtrlHumidityReg(bmeDevice_t* device, uint8_t humidityOversampling) {
    device->registerCtrlHumidity = humidityOversampling & BITMASK_LOWEST_THREE;
}

int32_t bme280_commitCtrlHumidity(bmeDevice_t* device) {
    uint8_t buffer[] = {REG_HUMIDITY_CTRL_ADDR, device->registerCtrlHumidity};
    return bme280_internal_writeToRegister(device, buffer, 2);
}

int32_t bme280_status(bmeDevice_t* device, uint8_t* buffer) {
    return bme280_internal_readFromRegister(device, REG_STATUS_ADDR, buffer, 1);
}

int32_t bme280_getMeasurements(bmeDevice_t* device, bmeMeasurements_t* result) {
    bmeRawMeasurements_t rawValues = {0, 0, 0};
    int32_t errorCode = bme280_internal_getRawMeasurement(device, &rawValues);
    if (errorCode != BME280_NO_ERROR) {
        return errorCode;
    }
    bmeMeasurements_t compensatedValues = bme280_internal_compensateMeasurements(device, &rawValues);
    result->pressure = compensatedValues.pressure;
    result->temperature = compensatedValues.temperature;
    result->humidity = compensatedValues.humidity;
    return errorCode;
}

//int32_t saphBme280_getPressure(saphBmeDevice_t* device, uint32_t* resultBuffer) {
//    uint32_t readAmount = 3;
//    int32_t commResult = saphBme280_internal_readFromRegister(device, REG_PRESSURE_START_ADDR, (uint8_t*) resultBuffer, readAmount);
//    if (commResult != BME280_NO_ERROR) {
//        return commResult;
//    }
//    *resultBuffer = getMeasurement20BitFromBuffer((uint8_t*) resultBuffer);
//    return BME280_NO_ERROR;
//}


// ###############################################
// Helper Functions
// ###############################################




