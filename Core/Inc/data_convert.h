#ifndef DATA_CONVERT_H
#define DATA_CONVERT_H

#include <stdint.h>

void float_to_u8(float *req, uint8_t *des, uint32_t float_len);
void u8_to_int(uint8_t *req, int32_t *des, uint32_t uint8_len);

#endif