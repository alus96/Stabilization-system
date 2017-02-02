#include "epos_can.h"
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef* can1stand;

CanTxMsgTypeDef canTxBuffer;
CanRxMsgTypeDef canRxBuffer;
CAN_FilterConfTypeDef canFilterConfig;

can_status nodeStatus[3];

#define DEBUG_VER

uint8_t* nodeIDs;
uint8_t nEpos;
uint16_t currentRegister = 0x2030;
uint16_t velocityRegister = 0x206B;
uint16_t velocityActualRegister = 0x2028;

uint16_t randRegister = 0x60F6;
uint16_t randRegisterSubIndex = 0x01;

void setNodes(uint8_t nodes[], uint8_t n) {
    nEpos = n;
    nodeIDs = nodes;
}

void clearCanRxBuffer() {
    canRxBuffer.StdId = 0x00;
    canRxBuffer.RTR = CAN_RTR_DATA;
    canRxBuffer.IDE = CAN_ID_STD;
    canRxBuffer.DLC = 0;
    int i;
    for (i = 0; i < 8; ++i)
        canRxBuffer.Data[i] = 0;
}

void printErrorStatus(HAL_StatusTypeDef status) {
    if (status == 0)
        printf("OK\n");
    else if (status == 1)
        printf("error\n");
    else if (status == 2)
        printf("busy\n");
    else
        printf("time out error");
}

int canRead(uint8_t id, uint16_t reg, uint8_t subIndex) {
    canTxBuffer.StdId = 0x600 + nodeIDs[id];
    canTxBuffer.DLC = 8;
    canTxBuffer.Data[0] = 0x40;
    canTxBuffer.Data[1] = reg & 0xFF;
    canTxBuffer.Data[2] = reg >> 8;
    canTxBuffer.Data[3] = subIndex;
    canTxBuffer.Data[4] = 0x00;
    canTxBuffer.Data[5] = 0x00;
    canTxBuffer.Data[6] = 0x00;
    canTxBuffer.Data[7] = 0x00;
    //clearCanRxBuffer();

    HAL_StatusTypeDef status;
    can1stand->pTxMsg = &canTxBuffer;
    status = HAL_CAN_Transmit(can1stand,10);
    //printf("transmition status: ");
    //printErrorStatus(status);

    status = HAL_CAN_Receive_IT(can1stand, CAN_FIFO0);
    //printf("reception status: ");
    //printErrorStatus(status);
    int result = (can1stand->pRxMsg->Data[7] << 24)
            + (can1stand->pRxMsg->Data[6] << 16)
            + (can1stand->pRxMsg->Data[5] << 8) + can1stand->pRxMsg->Data[4];
    //printf("register value %d\n",result);
#ifdef DEBUG_VER
    if (reg == 0x2028) {
        printf("0x2028: %d\n\r", result);
    }
#endif
    return result;
}

// void canSDOWrite(uint16_t reg, uint16_t timeout)
// {
// 	canTxBuffer.Data[0] = 0x22;
// 	canTxBuffer.Data[1] = reg & 0xFF;
// 	canTxBuffer.Data[2] = reg >> 8;

// 	HAL_StatusTypeDef status;
// 	can1stand.pTxMsg = &canTxBuffer;
// 	status = HAL_CAN_Transmit_IT(&can1stand, timeout);
// 	printf("transmition status: ");
// 	printErrorStatus(status);

// 	HAL_CAN_Receive_IT(&can1stand,CAN_FIFO0,timeout);

// }

void setSDOCurrent(uint8_t i, int current) {

    canWrite(i, currentRegister, 0x00, current);
}

void EposCanInit(CAN_HandleTypeDef* canPort) {
    can1stand = canPort;

    canTxBuffer.StdId = 0x600;
    canTxBuffer.ExtId = 0;
    canTxBuffer.RTR = CAN_RTR_DATA;
    canTxBuffer.IDE = CAN_ID_STD;
    canTxBuffer.DLC = 8;
    canTxBuffer.Data[0] = 0x00;
    canTxBuffer.Data[1] = 0x00;
    canTxBuffer.Data[2] = 0x00;
    canTxBuffer.Data[3] = 0x00;
    canTxBuffer.Data[4] = 0x00;
    canTxBuffer.Data[5] = 0x00;
    canTxBuffer.Data[6] = 0;
    canTxBuffer.Data[7] = 0;

    canFilterConfig.FilterNumber = 0;
    canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    canFilterConfig.FilterIdHigh = 0x0000;
    canFilterConfig.FilterIdLow = 0x0000;
    canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
    canFilterConfig.FilterMaskIdLow = 0x0000;
    canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
    canFilterConfig.FilterActivation = ENABLE;
    canFilterConfig.BankNumber = 1;
    HAL_CAN_ConfigFilter(can1stand, &canFilterConfig);

    can1stand->pTxMsg = &canTxBuffer;
    can1stand->pRxMsg = &canRxBuffer;
}

void canWrite(uint8_t id, uint16_t reg, uint8_t subIndex, int value) {
    canTxBuffer.StdId = 0x600 + nodeIDs[id];
    canTxBuffer.DLC = 8;
    canTxBuffer.RTR = CAN_RTR_DATA;

    canTxBuffer.Data[0] = 0x22;
    canTxBuffer.Data[1] = reg & 0xFF;
    canTxBuffer.Data[2] = reg >> 8;
    canTxBuffer.Data[3] = subIndex;
    canTxBuffer.Data[4] = value & 0xFF;
    canTxBuffer.Data[5] = (value >> 8) & 0xFF;
    canTxBuffer.Data[6] = (value >> 16) & 0xFF;
    canTxBuffer.Data[7] = (value >> 24) & 0xFF;

    HAL_CAN_Transmit(can1stand,10);
    HAL_CAN_Receive_IT(can1stand, CAN_FIFO0);
}

void setCurrent(uint16_t cob_id, int current) {

    canTxBuffer.StdId = cob_id;
    canTxBuffer.RTR = CAN_RTR_DATA;
    canTxBuffer.DLC = 4;

    if (current >= 0) {
        canTxBuffer.Data[0] = current & 0xFF;
        canTxBuffer.Data[1] = current >> 8;
        canTxBuffer.Data[2] = 0x00;
        canTxBuffer.Data[3] = 0x00;
    } else {
        int c = 0xFFFF + current + 1;
        canTxBuffer.Data[0] = c & 0xFF;
        canTxBuffer.Data[1] = c >> 8;
        canTxBuffer.Data[2] = 0xFF;
        canTxBuffer.Data[3] = 0xFF;
    }
    HAL_CAN_Transmit(can1stand,10);
}

// void setRegister(uint8_t id,int value)
// {
// 	canTxBuffer.StdId = 0x600 + id;
// 	canTxBuffer.Data[4] = value & 0xFF;
// 	canTxBuffer.Data[5] = (value >> 8) & 0xFF;
// 	canTxBuffer.Data[6] = (value >> 16) & 0xFF;
//     canTxBuffer.Data[7] = (value >> 24) & 0xFF;
// 	canWrite(randRegister, randRegisterSubIndex,10);

// }

void enterPreOperational() {
    canTxBuffer.StdId = 0x00;
    canTxBuffer.RTR = 0;
    canTxBuffer.DLC = 2;
    canTxBuffer.Data[0] = 0x80;
    canTxBuffer.Data[1] = 0x00;
    HAL_CAN_Transmit(can1stand,10);
}

void enterOperational() {
    canTxBuffer.StdId = 0x00;
    canTxBuffer.RTR = 0;
    canTxBuffer.DLC = 2;
    canTxBuffer.Data[0] = 0x01;
    canTxBuffer.Data[1] = 0x00;
    HAL_CAN_Transmit(can1stand,10);
}

void eposReset() {
    canTxBuffer.StdId = 0x00;
    canTxBuffer.RTR = 0;
    canTxBuffer.DLC = 2;
    canTxBuffer.Data[0] = 0x81;
    canTxBuffer.Data[1] = 0x00;
    HAL_CAN_Transmit(can1stand,10);
}

void canPDOConfiguration(uint8_t i, uint16_t cob_id) {
    //enterPreOperational();

    printf("\nPDO configuratione for node %d\n", nodeIDs[i]);

    canWrite(i, 0x1400, 0x01, cob_id);

    canWrite(i, 0x1400, 0x02, 0xFF);
    canWrite(i, 0x1600, 0x00, 0x00);
    canWrite(i, 0x1600, 0x01, 0x20300010);
    canWrite(i, 0x1600, 0x00, 0x01);

    //enterOperational();
}

void canRxPDOConfiguration(uint8_t i, uint16_t cob_id) {
    //enterPreOperational();

    printf("\nPDO configuratione for node %d\n", nodeIDs[i]);

    canWrite(i, 0x1400, 0x01, cob_id);

    canWrite(i, 0x1400, 0x02, 0xFF);
    canWrite(i, 0x1600, 0x00, 0x00);
    canWrite(i, 0x1600, 0x01, 0x20300010);
    canWrite(i, 0x1600, 0x00, 0x01);

    //enterOperational();
}

void canTxPDOConfiguration(uint8_t i, uint16_t cob_id) {
    //enterPreOperational();

    printf("\nPDO configuratione for node %d\n", nodeIDs[i]);

    canWrite(i, 0x1801, 0x01, cob_id);

    canWrite(i, 0x1801, 0x02, 0xFF);
    canWrite(i, 0x1801, 0x03, 0x01);
    canWrite(i, 0x1A01, 0x00, 0x00);
    canWrite(i, 0x1A01, 0x01, 0x20280020);
    canWrite(i, 0x1A01, 0x00, 0x01);

    //enterOperational();
}

void enableEpos(uint8_t id) {

    canWrite(id, 0x6040, 0x00, 0x0006);
    HAL_Delay(10);
    canWrite(id, 0x6040, 0x00, 0x000F);
    HAL_Delay(10);
}

int32_t getVelocity(uint16_t cob_id) {
	uint8_t nodeIdx;

		for(nodeIdx = 0; nodeIdx < sizeof(nodeIDs) && cob_id != nodeIDs[nodeIdx]; nodeIdx++);

		if( nodeIdx < sizeof(nodeIDs)){
			return nodeStatus[nodeIdx].velocity;
		}

	return -1;
}

void enableQuickStop(uint8_t id){
    canWrite(id, 0x6040, 0x00, 0x0002);

}

void disableQuickStop(uint8_t id){
    canWrite(id, 0x6040, 0x00, 0x000F);
}


void SetCurrent3Maxon(int16_t current) {
    setCurrent(0x181, current);
    setCurrent(0x182, -current);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan){

	int32_t idx = 0;

	uint8_t nodeIdx;

	uint8_t nodeID = 0x1;
//
//	for(nodeIdx = 0; nodeIdx < sizeof(nodeIDs) && nodeID != nodeIDs[nodeIdx]; nodeIdx++);
	nodeIdx = 0x00;

	if(canRxBuffer.StdId  == 0x182 && canRxBuffer.DLC == 4){
		nodeStatus[nodeIdx].id = nodeID;
		nodeStatus[nodeIdx].velocity = 	((uint32_t) canRxBuffer.Data[3]) << 24 |
										((uint32_t) canRxBuffer.Data[2]) << 16 |
										((uint32_t) canRxBuffer.Data[1]) << 8 |
										((uint32_t) canRxBuffer.Data[0]);
		nodeStatus[nodeIdx].current = 0x00;
		nodeStatus[nodeIdx].duty = 0x00;

	}

	if(HAL_CAN_Receive_IT(can1stand, CAN_FIFO0) != HAL_OK){
			/* Reception Error */
			//Error_Handler();
	}

}


