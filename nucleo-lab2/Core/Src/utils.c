#include "utils.h"

/*
 * Waits a specific number of seconds approx.
 */
void espera(char seconds) {
  for (int i=0; i<(seconds*2000000); i++);
}


char getBitFromUInt32_t(volatile uint32_t* reg, unsigned char bit) {
  uint32_t aux = (*reg & ( 1 << bit ));

  return(aux > 0 ? 1 : 0);
}

