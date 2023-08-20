#include "../include/nrf2.h"

RF24 radio(24,26);

const uint64_t address0 = 0x1;
const uint64_t address1 = 0x2;

void rf24Init()
{
  	radio.begin();
  	radio.setPALevel(RF24_PA_LOW);
//  	radio.openWritingPipe(address0);
  	radio.openReadingPipe(1,address1);

	printf("rf24 inited\n");
}

void sendrf24(struct UAV *UAV)
{
	float angle[2];
	angle[0] = UAV->fbk.roll;
	angle[1] = UAV->fbk.pitch;
  	radio.stopListening();
	radio.write(&angle,sizeof(angle));
}

int readrf24(struct rf24Data *data)
{
	static int safeLock = 0;
	safeLock += 1;

  	radio.startListening();
 	if(radio.available())
	{
		safeLock = 0;
      	radio.read(data, sizeof(*data));
	}

	if(safeLock >= 30)	return -1;
	else				return 0;
}

