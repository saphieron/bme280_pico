# BME 280 library
A small library to control and read out data from the Bosch BME280 sensor family, accessed via I2C.

The library is written against and tested with the Raspberry Pi Pico's C/C++ SDK.
However, the library uses an I2C wrapper (header) to be as independent as possible of the SDK's concrete implementation.

## Usage
You need to incorporate the `bme280_*` files into your project and provide an implementation to the functions defined in `i2c_handler.h`.

Consider creating a git submodule of this project.