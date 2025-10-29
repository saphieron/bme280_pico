#ifndef BME280_INTERNAL_H
#define BME280_INTERNAL_H

#include <stdbool.h>
#include "bme280.h"

typedef struct bmeRawMeasurements_t {
    int32_t pressure;
    int32_t temperature;
    int32_t humidity;
} bmeRawMeasurements_t;

int32_t bme280_internal_readTrimmingValues(bmeDevice_t* device);
int32_t bme280_internal_getErrorCode(int32_t commResult, bool wasWriting);
int32_t bme280_internal_writeToRegister(bmeDevice_t* device, uint8_t* buffer, uint32_t bufferSize);
int32_t bme280_internal_readFromRegister(bmeDevice_t* device, uint8_t regAddress, uint8_t* readingBuffer, uint32_t readAmount);
int32_t bme280_internal_getRawMeasurement(bmeDevice_t* device, bmeRawMeasurements_t* result);
bmeMeasurements_t bme280_internal_compensateMeasurements(bmeDevice_t* device, bmeRawMeasurements_t* rawMeasurements);

#endif // BME280_INTERNAL_H
