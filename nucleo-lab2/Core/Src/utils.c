#include "utils.h"

char getBitFromUInt32_t(volatile uint32_t* reg, unsigned char bit) {
  uint32_t aux = (*reg & ( 1 << bit ));

  return(aux > 0 ? 1 : 0);
}

