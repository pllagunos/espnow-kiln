/*
  ShiftRegisterSPI.h - Library for simplified control of 74HC595 shift registers with SPI.
  adapted from Timo Denk.
*/

#pragma once

#include <Arduino.h>
#include <SPI.h>

template<uint8_t Size>
class ShiftRegisterSPI
{
public:
    ShiftRegisterSPI(const uint8_t latchPin);

    void setAll(const uint8_t * digitalValues);
#ifdef __AVR__
    void setAll_P(const uint8_t * digitalValuesProgmem); // Experimental, PROGMEM data
#endif
    uint8_t * getAll();
    void set(const uint8_t pin, const uint8_t value);
    void setNoUpdate(const uint8_t pin, uint8_t value);
    void updateRegisters();
    void setAllLow();
    void setAllHigh();
    uint8_t get(const uint8_t pin);

private:
    uint8_t _latchPin;
    uint8_t  _digitalValues[Size];
};

#include "ShiftRegisterSPI.hpp"
