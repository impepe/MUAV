#ifndef __NRF24_H__
#define __NRF24_H__

#include <wiringPi.h>
#include <RF24/RF24.h>
#include "constants.h"

void rf24Init();

void sendrf24(struct UAV *UAV);

int readrf24(struct rf24Data *data);

#endif
