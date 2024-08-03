// Author: David Medin
// Date : 07/22/2024
// Source code for a super bare-bones C binding for the arduino.
// Please note! : This is named very similarily to Arduino.h / Arduino.cpp.
// These are different files! (Note the capitalization).
extern "C"{
    #include "arduino.h"
}
#include <Arduino.h>
#include <stdlib.h>
#include <FastLED.h>

extern "C" {

    void serial_begin(unsigned long baudrate) {
        Serial.begin(baudrate);
    }

    void serial_end(){
        Serial.end();
    }

    size_t println(const char c[]) {
        return Serial.println(c);
    }

    int serial_available() {
        return Serial.available();
    }
    void wait(unsigned long ms){
        delay(ms);
    }

    void wait_us(unsigned long us) {
        delayMicroseconds(us);
    }

    void pin_mode(size_t pin, size_t mode){
        pinMode(pin,mode);
    }
    void digital_write(size_t pin, size_t val){
        digitalWrite(pin,val);
    }

    // void write_one() {
    //     R_IOPORT_PinWrite(NULL, BSP_IO_PORT_04_PIN_10, BSP_IO_LEVEL_HIGH);
    // }

    // void write_zero() {
    //     R_IOPORT_PinWrite(NULL, BSP_IO_PORT_04_PIN_10, BSP_IO_LEVEL_LOW);
    // }

    uint32_t micro() {
        return micros();
    }
    CRGB leds[200];
    void setup_fastled(){
        FastLED.addLeds<NEOPIXEL, 200>(leds, 200);
    }

    // Useful for finding the addresses to write to to bring a pin high or low.
    // uint32_t stuff() {
    //     return (uint32_t)(&(R_PORT4->PORR));
    // }
}
