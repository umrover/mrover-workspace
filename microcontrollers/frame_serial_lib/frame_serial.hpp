#pragma once

#include <mbed.h>

void write_frame(Serial &s, const uint8_t *buf, size_t len);
