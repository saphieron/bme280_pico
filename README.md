# BME 280 library
A small library to controll and read out data from the Bosch BME280 sensor family, accessed via I2C.

This implementation is written with the Raspberry Pi Pico in mind, using the Pico SDK library.
However, the code aims to provide an abstract interface that is to be implemented by the library user to be as independent of the SDK as possible.
