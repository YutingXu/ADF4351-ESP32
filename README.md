# ADF4351-ESP32
ESP32 library for the ADF4351 PLL Frequency Synthesizer chip, the original library for Arduino can be found [here](https://github.com/dfannin/adf4351).

## Introduction

This library supports the [ADF4351 Chip](https://goo.gl/tkMjw6) from Analog Devices on ESP32s. The chip is a wideband (35 MHz to 4.4 GHz ) Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO), covering a very wide range frequency range
under digital control.

The chip generates the frequency using a programmable Fractional-N and Integer-N Phase-Locked Loop (PLL) and Voltage Controlled Oscillator (VCO) with an external loop filter and frequency reference. The chip is controlled by 
a SPI interface, which is controlled by a microcontroller such as the Arduino.

The library provides an SPI control interface for the ADF4351, and also provides functions to calculate and set the
frequency, which greatly simplifies the integration of this chip into a design.

## Notes
The original code was written in C++, hence function declarations had to be changed. For a working example, please see main_example.c

## References

+ [ADF4351 Product Page](https://goo.gl/tkMjw6) Analog Devices
+ [ADF4351 Arduino Library](https://github.com/dfannin/adf4351) By David Fannin
