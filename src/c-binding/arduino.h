#pragma once
#include <stdlib.h>
#include <stdint.h>

struct RGB {
    uint8_t raw[3];
};

void serial_begin(unsigned long baudrate);
void serial_end();
size_t println(const char c[]);
int serial_available();
void wait(unsigned long ms);
void wait_us(unsigned long us);
void pin_mode(size_t pin, size_t mode);
void digital_write(size_t pin, size_t val);

void write_one();
void write_zero();
uint32_t micro();
