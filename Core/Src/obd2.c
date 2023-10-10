#include "obd2.h"

extern CAN_TxHeaderTypeDef txHeader;
extern CAN_RxHeaderTypeDef rxHeader;


extern uint8_t txData[8];
extern uint8_t rxData[8];
extern int dataCheck;   /* Flag for Checking if CAN Response is received. */


/*
 * @brief: A Function to request Vehicle Speed and get the response.
 * @param: None.
 * @return: None.
 * */
void requestVehicleSpeed()
{

	int i = 0; /* Loop Var. */
	uint32_t Id = 0x7df; /* Request ID here 11 bits is used */
	uint8_t len = 0x02; /* Length of data */
	uint8_t mode = 0x01;  /* Mode: in this case 0x01 refers to real
	 	 	 	 	 	 	 	   time data i.e. current vehicle speed	 */
	uint8_t pid = 0x0d;   /* Vehicle Speed */
	uint8_t data[5] = {0x55, 0x55, 0x55, 0x55, 0x55}; /* Data Bytes */

	formOBD2Packet(Id, len, mode, pid, data);

	HAL_Delay(500);  /* Provide Some appropriate delay for getting the response */
	if (dataCheck == 1) {
		for (i = 0; i < 8; i++)
			printf("received CAN RAW Response = %x",rxData[i]);

	}
	dataCheck = 0; /* Clear Receive Interrupt Flag. */
#if 0
	configCANTxStandard(Id, len);
	txData[0] = mode;
	txData[1] = pid;
	txData[2] = data[0];
	txData[3] = data[1];
	txData[4] = data[2];
	txData[5] = data[3];
	txData[6] = data[4];

	canTransmit(txData);
#endif
}
/*
 *@brief: An API to form OBD2 Packet and Transmit it and handle the response.
 *@param1: OBD2 PID.
 *@param2: OBD2 Packet Length.
 *@param3: OBD2 Mode used.
 *@param4: OBD2 Payload Array.
 *@return: None.
 * */
void formOBD2Packet(uint32_t Id, uint8_t len, uint8_t mode, uint8_t pid, uint8_t data[])
{
	int i = 2, j = 0; /* Loop Vars. */
	uint16_t highId = 0, lowId = 0; /* Temp Vars. for use with Can Filter API */

	lowId = 0x00;
	highId = Id;
	CAN_Filter(lowId, highId, false); /* Standard 11 bit CAN Filtering is used in this case, for 11 bit OBD2 PID */
	configCANTxStandard(Id, len);
	txData[0] = mode;
	txData[1] = pid;

	for(; i < 7; i++) {
		txData[i] = data[j];
		j++;
	}
	canTransmit(txData);


}
#if 0
uint8_t formOBD2Packet(uint32_t, uint8_t, uint8_t, uint8_t, uint8_t[]);
uint8_t send_obd2_request(char *);

uint8_t formOBD2Packet(char *pid)
{


}
#endif
