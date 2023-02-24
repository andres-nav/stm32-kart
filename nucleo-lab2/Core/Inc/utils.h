#ifndef __UTILS_H
#define __UTILS_H

#include <stdint.h>

#include "define.h"

void espera(char seconds);

char getBitFromUInt32_t(volatile uint32_t *reg, unsigned char bit);

#endif /* __UTILS_H */
