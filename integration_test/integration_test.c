
//////////////////////////////////////////////////////////////////////////////
//
// Created by saphieron on 4/2/21.
//
//////////////////////////////////////////////////////////////////////////////

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include <i2c_test_wrapper.h>
#include <bme280.h>

#ifndef I2C_BAUDRATE
#define I2C_BAUDRATE 100000UL
#endif

#define BME_DEFAULT_ADDRESS 0x76


static void printError(int32_t errorCode, const char* onWhat);

static int32_t configIntoKnownSensorState(bmeDevice_t* device);

static void integration_runAllTests(bmeDevice_t* device);
static void integration_initDevice(bmeDevice_t* device);
static void integration_resetDevice(bmeDevice_t* device);
static void integration_getId(bmeDevice_t* device);
static void integration_getStatus(bmeDevice_t* device);
static void integration_setCtrlHumidity(bmeDevice_t* device);
static void integration_setMeasureControlReg(bmeDevice_t* device);
static void integration_setConfigReg(bmeDevice_t* device);
static void integration_getMeasurements(bmeDevice_t* device);

int main() {
    stdio_init_all();
    i2c_handler_selectHwInstance(0);
    int actualBaudSet = i2c_handler_initialise(I2C_BAUDRATE);
    printf("i2c_handler_initialise actualBaudSet %d\n", actualBaudSet);
    if (actualBaudSet != I2C_BAUDRATE) {
        while (1) {
            printf("Unable to perform tests due to failed(?) i2c baud rate setting\n");
            sleep_ms(100);
        }
    } else {
        printf("i2c_handler initialised to baud rate %lu\n", I2C_BAUDRATE);
    }

    bmeDevice_t myBmeDevice;
    while (1) {
        integration_runAllTests(&myBmeDevice);
        sleep_ms(5000);
    }
   return 0;
}

// ############################
// # Helpers
// ############################
static void printError(int32_t errorCode, const char* onWhat) {
    printf("failed\n");
    printf("Step that failed: %s\n", onWhat);
    printf("Error code: %ld\n\n", errorCode);
}

static int32_t configIntoKnownSensorState(bmeDevice_t* device) {
    bme280_prepareMeasureCtrlReg(device, OVERSAMPLING_x1, OVERSAMPLING_x1, BME280_SENSOR_MODE_SLEEP);
    int32_t errorCode = bme280_commitMeasureCtrlReg(device);
    if (errorCode != BME280_NO_ERROR) {
        printError(errorCode, "committing MeasureControlReg settings");
        return errorCode;
    }

    bme280_prepareCtrlHumidityReg(device, OVERSAMPLING_x1);
    errorCode = bme280_commitCtrlHumidity(device);
    if (errorCode != BME280_NO_ERROR) {
        printError(errorCode, "committing ctrlHumidity settings");
        return errorCode;
    }

    bme280_prepareConfigReg(device, BME280_STANDBY_TIME_MS_0_5, BME280_IIR_FILTER_COEFFICIENT_2);
    errorCode = bme280_commitConfigReg(device);
    if (errorCode != BME280_NO_ERROR) {
        printError(errorCode, "committing ConfigurationReg settings");
        return errorCode;
    }
    bme280_prepareMeasureCtrlReg(device, OVERSAMPLING_x1, OVERSAMPLING_x1, BME280_SENSOR_MODE_NORMAL);
    errorCode = bme280_commitMeasureCtrlReg(device);
    if (errorCode != BME280_NO_ERROR) {
        printError(errorCode, "committing MeasureControlReg settings");
        return errorCode;
    }
    sleep_ms(10);
    return BME280_NO_ERROR;
}


// ############################
// # test cases
// ############################

static void integration_runAllTests(bmeDevice_t* device) {
    printf("########## Bme280 Integration tests ##########\nRunning all tests:\n");
    printf("i2c_handler initialised to baud rate %lu\n", I2C_BAUDRATE);
    integration_initDevice(device);
    integration_resetDevice(device);
    integration_getId(device);
    integration_setCtrlHumidity(device);
    integration_setConfigReg(device);
    integration_setMeasureControlReg(device);
    integration_getStatus(device);
    integration_getMeasurements(device);
    printf("################### test done ###################\n\n");
}

static void integration_initDevice(bmeDevice_t* device) {
    printf("integration_initDevice: ");
    int32_t errorCode = bme280_init(BME_DEFAULT_ADDRESS, device);
    if (errorCode == BME280_NO_ERROR) {
        printf("successful\n\nDevice address is %u\n\"Trimming values are: ", device->address);
        bmeTrimmingValues_t trimmingVals = device->trimmingValues;
        printf("%u, ", trimmingVals.dig_T1);
        printf("%d, ", trimmingVals.dig_T2);
        printf("%d | ", trimmingVals.dig_T3);

        printf("%u, ", trimmingVals.dig_P1);
        printf("%d, ", trimmingVals.dig_P2);
        printf("%d, ", trimmingVals.dig_P3);
        printf("%d, ", trimmingVals.dig_P4);
        printf("%d, ", trimmingVals.dig_P5);
        printf("%d, ", trimmingVals.dig_P6);
        printf("%d, ", trimmingVals.dig_P7);
        printf("%d, ", trimmingVals.dig_P8);
        printf("%d | ", trimmingVals.dig_P9);

        printf("%u, ", trimmingVals.dig_H1);
        printf("%d, ", trimmingVals.dig_H2);
        printf("%u, ", trimmingVals.dig_H3);
        printf("%d, ", trimmingVals.dig_H4);
        printf("%d, ", trimmingVals.dig_H5);
        printf("%d\n\n", trimmingVals.dig_H6);
    } else {
        printf("failed\n");
        printf("Error code: %ld\n\n", errorCode);
    }
}

static void integration_resetDevice(bmeDevice_t* device) {
    printf("integration_resetDevice: ");
    int32_t errorCode = bme280_resetDevice(device);
    if (errorCode == BME280_NO_ERROR) {
        printf("successful\n\n");
    } else {
        printf("failed\n");
        printf("Error code: %ld\n\n", errorCode);
    }
}

static void integration_getId(bmeDevice_t* device) {
    int32_t id = 0;
    printf("integration_getId: ");
    id = bme280_getId(device);
    if (id < 0) {
        printf("failed\n");
        printf("error code: %ld\n\n", id);
    } else {
        printf("successful\n");
        printf("Device id seems to be: 0x%02X\n\n", (uint8_t) id);
    }
}

static void integration_getStatus(bmeDevice_t* device) {
    printf("integration_getStatus: ");
    uint8_t status = 0xFF;
    int32_t errorCode = bme280_status(device, &status);
    if (errorCode == BME280_NO_ERROR) {
        printf("successful\n");
        printf("status is: measuring %u, im_update %u\n\n", status & (1 << 3), status & (1 << 0));
    } else {
        printf("failed\n");
        printf("Error code: %ld\n\n", errorCode);
    }
}

static void integration_setCtrlHumidity(bmeDevice_t* device) {
    printf("integration_setMeasureControlReg: ");
    bme280_prepareCtrlHumidityReg(device, OVERSAMPLING_x4);
    int32_t errorCode = bme280_commitCtrlHumidity(device);
    if (errorCode != BME280_NO_ERROR) {
        printError(errorCode, "committing ctrl humidity value");
    } else {
        printf("success\n\n");
    }
    sleep_ms(100);
    uint8_t bufferSent[] = {0xF2};
    i2c_handler_write(device->address, bufferSent, 1);
    uint8_t bufferReceived[12];
    memset(bufferReceived, 0xAB, 12);
    i2c_handler_read(device->address, bufferReceived, 1);
    printf("reading all: ");
    for (int i = 0; i < 1; ++i) {
        printf("%u,", bufferReceived[i]);
    }
    printf("\n");
}

static void integration_setMeasureControlReg(bmeDevice_t* device) {
    printf("integration_setMeasureControlReg: ");
    bme280_prepareMeasureCtrlReg(device, OVERSAMPLING_x16, OVERSAMPLING_x16, BME280_SENSOR_MODE_NORMAL);
    int32_t errorCode = bme280_commitMeasureCtrlReg(device);

    if (errorCode == BME280_NO_ERROR) {
        printf("successful\n\n");
    } else {
        printf("failed\n");
        printf("Error code: %ld\n\n", errorCode);
    }
}

static void integration_setConfigReg(bmeDevice_t* device) {
    printf("integration_setConfigReg: ");
    bme280_prepareConfigReg(device, BME280_STANDBY_TIME_MS_0_5, BME280_IIR_FILTER_COEFFICIENT_16);
    int32_t errorCode = bme280_commitConfigReg(device);
    if (errorCode == BME280_NO_ERROR) {
        printf("successful\n\n");
    } else {
        printf("failed\n");
        printf("Error code: %ld\n\n", errorCode);
    }
}

static void integration_getMeasurements(bmeDevice_t* device) {
    configIntoKnownSensorState(device);
    bmeMeasurements_t results;
    for (int i = 0; i < 20; ++i) {
        int32_t errorCode = bme280_getMeasurements(device, &results);
        if (errorCode == BME280_NO_ERROR) {

            float temperature = results.temperature / 100.0f;
            printf("Temperature: %3.3f °C\n", temperature);
            float pressure = results.pressure / 25600.0f;
            printf("Pressure: %f hPa\n", pressure);
            float humidity = results.humidity / 1024.0f;
            printf("Humidity: %f %%\n\n", humidity);
        } else {
            printf("failed\n");
            printf("Error code: %ld\n\n", errorCode);
            return;
        }
        sleep_ms(1000);
    }
    printf("successful\n\n");
}



//static void integration_useAllFunctions(bmeDevice_t* device) {
//    printf("integration_useAllFunctions: ");
//    bme280_resetDevice(device);
//    sleep_ms(10);
//    configIntoKnownSensorState(device);
//    sleep_ms(10);
//    gpio_put(LED_YELLOW_1, 1);
//    for (int j = 0; j < 40; ++j) {
//        uint8_t bufferSent[] = {0xF2};
//        i2c_handler_write(device->address, bufferSent, 1);
//        uint8_t bufferReceived[12];
//        memset(bufferReceived, 0xAB, 12);
//        i2c_handler_read(device->address, bufferReceived, 12);
//        printf("raw read: ");
//        for (int i = 0; i < 12; ++i) {
//            printf("%u,", bufferReceived[i]);
//        }
//
//        printf("\n");
//        uint32_t pressure = (bufferReceived[4] << 11) + (bufferReceived[5] << 3) + (0x07 & bufferReceived[6]);
//        uint32_t temp = (bufferReceived[7] << 11) + (bufferReceived[8] << 3) + (0x07 & bufferReceived[9]);
//        uint16_t humidity = (bufferReceived[10] << 8) + (bufferReceived[11]);
//        printf("pressure %lu, temp %lu, humidity %u\n", pressure, temp, humidity);
//
//        sleep_ms(100);
//    }
//    gpio_put(LED_YELLOW_2, 1);
//}
