#define MISC_GLOBAL

#include "tiza_misc.h"


#define SERIAL_ID_BASE	((uint8 *)0x08003FF0)

//�豸ΨһID
void GetDeviceSerial(uint8 *DevSerial, uint8 Len)
{
	uint8 *device_sn_addr;
	uint8 i;

	if(DevSerial == NULL)
	{
		return;
	}
	
	if(Len != 12)
	{
		DevSerial[0] = '\0';
		return;
	}
	
	device_sn_addr = SERIAL_ID_BASE;
	
	for(i=0; i < 12; i++)
	{
		DevSerial[i] = *device_sn_addr;
		device_sn_addr++;
	}
}
