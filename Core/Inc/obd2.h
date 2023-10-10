#ifndef __OBD2_H
#define __OBD2_H
#include "main.h"

void formOBD2Packet(uint32_t, uint8_t, uint8_t, uint8_t, uint8_t[]);
void requestVehicleSpeed();

#endif
