#include "obd2.h"

extern CAN_TxHeaderTypeDef txHeader;
extern CAN_RxHeaderTypeDef rxHeader;


extern uint8_t txData[8];
extern uint8_t rxData[8];


void requestVehicleSpeed()
{
	uint32_t Id = 0x7df; /* Request ID here 11 bits is used */
	uint8_t len = 0x02; /* Length of data */
	uint8_t mode = 0x01;  /* Mode: in this case 0x01 refers to real
	 	 	 	 	 	 	 	   time data i.e. current vehicle speed	 */
	uint8_t pid = 0x0d;   /* Vehicle Speed */
	uint8_t data[5] = {0x55, 0x55, 0x55, 0x55, 0x55}; /* Data Bytes */

	//uint32_t *obd2Req = Id + len + mode + pid + data[0] + data[1] + data[2] + data[3] + data[4];

	configCANTxStandard(Id, len);
	txData[0] = mode;
	txData[1] = pid;
	txData[2] = data[0];
	txData[3] = data[1];
	txData[4] = data[2];
	txData[5] = data[3];
	txData[6] = data[4];

	canTransmit(txData);

}

#if 0
uint8_t formOBD2Packet(char *);
uint8_t send_obd2_request(char *);

uint8_t formOBD2Packet(char *pid)
{


}
#endif
