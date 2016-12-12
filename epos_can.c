#include "epos_can.h"
#include "stm32f4xx_hal.h"

CAN_HandleTypeDef can1stand;

CanTxMsgTypeDef canTxBuffer;
CanRxMsgTypeDef canRxBuffer;
CAN_FilterConfTypeDef canFilterConfig;

//uint8_t nodeIDs[3] = {0x01,0x03,0x07};
//uint16_t currentRegister = 0x2030;
//uint16_t velocityRegister = 0x206B;
//uint16_t randRegister = 0x60F6;

 uint8_t nodeIDs[3] = {0x03,0x0F,0x07};
// uint8_t nodeIDs[3] = {0x01,0x0F,0x07};
uint16_t currentRegister = 0x2030;
uint16_t velocityRegister = 0x206B;
uint16_t randRegister = 0x60F6;
uint16_t randRegisterSubIndex = 0x01;



// void EposCanInit(CAN_HandleTypeDef canPort)
// {
// 	can1stand = canPort;

// 	clearCanRxBuffer();

// 	canTxBuffer.StdId= 0x600;
// 	canTxBuffer.ExtId=0;
// 	canTxBuffer.RTR=CAN_RTR_DATA;
// 	canTxBuffer.IDE=CAN_ID_STD;
// 	canTxBuffer.DLC=8;
// 	canTxBuffer.Data[0] = 0x00;
// 	canTxBuffer.Data[1] = 0x00;
// 	canTxBuffer.Data[2] = 0x00;
// 	canTxBuffer.Data[3] = 0x00;
// 	canTxBuffer.Data[4] = 0x00;
// 	canTxBuffer.Data[5] = 0x00;
// 	canTxBuffer.Data[6] = 0;
// 	canTxBuffer.Data[7] = 0;

// 	canFilterConfig.FilterNumber = 0;
// 	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
// 	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
// 	canFilterConfig.FilterIdHigh = 0x0000;
// 	canFilterConfig.FilterIdLow = 0x0000;
// 	canFilterConfig.FilterMaskIdHigh = 0x0000 << 5;
// 	canFilterConfig.FilterMaskIdLow = 0x0000;
// 	canFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
// 	canFilterConfig.FilterActivation = ENABLE;
// 	canFilterConfig.BankNumber = 1;
// 	HAL_CAN_ConfigFilter(&can1stand, &canFilterConfig);

// 	can1stand.pTxMsg = &canTxBuffer;
// 	can1stand.pRxMsg = &canRxBuffer;
// }

// void clearCanRxBuffer()
// {
// 	canRxBuffer.StdId = 0x00;
// 	canRxBuffer.RTR=CAN_RTR_DATA;
// 	canRxBuffer.IDE=CAN_ID_STD;
// 	canRxBuffer.DLC=0;
// 	int i;
// 	for(i = 0; i < 8; ++i) canRxBuffer.Data[i] = 0;
// }

// void printErrorStatus(HAL_StatusTypeDef status)
// {
// 	if(status == 0) printf("OK\n");
// 	else if(status == 1) printf("error\n");
//  	else if(status == 2) printf("busy\n");
//  	else printf("time out error");
// }

// int canRead(uint16_t reg, uint16_t timeout)
// {
// 	canTxBuffer.Data[0] = 0x40;
// 	canTxBuffer.Data[1] = reg & 0xFF;
// 	canTxBuffer.Data[2] = reg >> 8;
// 	canTxBuffer.Data[3] = 0x00;
// 	canTxBuffer.Data[4] = 0x00;
// 	canTxBuffer.Data[5] = 0x00;
// 	clearCanRxBuffer();

// 	HAL_StatusTypeDef status;
// 	can1stand.pTxMsg = &canTxBuffer;
// 	status = HAL_CAN_Transmit(&can1stand, timeout);
// 	printf("transmition status: ");
// 	printErrorStatus(status);

// 	status = HAL_CAN_Receive(&can1stand,CAN_FIFO0,timeout);
// 	printf("reception status: ");
// 	printErrorStatus(status);
// 	int result = (can1stand.pRxMsg->Data[7] << 24) + (can1stand.pRxMsg->Data[6] << 16)+
// 							 (can1stand.pRxMsg->Data[5] << 8) + can1stand.pRxMsg->Data[4];
// 	printf("register value %d\n",result);
// 	return result;
// }


// void canWrite(uint16_t reg, uint16_t timeout)
// {
// 	canTxBuffer.Data[0] = 0x22;
// 	canTxBuffer.Data[1] = reg & 0xFF;
// 	canTxBuffer.Data[2] = reg >> 8;

// 	HAL_StatusTypeDef status;
// 	can1stand.pTxMsg = &canTxBuffer;
// 	status = HAL_CAN_Transmit(&can1stand, timeout);
// 	printf("transmition status: ");
// 	printErrorStatus(status);

// 	HAL_CAN_Receive(&can1stand,CAN_FIFO0,timeout);

// }

// void setCurrent(uint8_t id,int current)
// {
// 	canTxBuffer.StdId = 0x600 + id;
// 	canTxBuffer.Data[3] = 0x00;
// 	if(current >= 0){
// 		canTxBuffer.Data[4] = current & 0xFF;
// 		canTxBuffer.Data[5] = current >> 8;
// 		canTxBuffer.Data[6] = 0x00;
// 		canTxBuffer.Data[7] = 0x00;
// 	}
// 	else{
// 		int c = 0xFFFF + current + 1;
// 		canTxBuffer.Data[4] = c & 0xFF ;
// 		canTxBuffer.Data[5] = c >> 8;
// 		canTxBuffer.Data[6] = 0xFF;
// 		canTxBuffer.Data[7] = 0xFF;
// 	}

// 	canWrite(currentRegister,10);
// }







void EposCanInit(CAN_HandleTypeDef canPort)
{	
	can1stand = canPort;

	canTxBuffer.StdId= 0x600;
	canTxBuffer.ExtId=0;
	canTxBuffer.RTR=CAN_RTR_DATA;
	canTxBuffer.IDE=CAN_ID_STD;
	canTxBuffer.DLC=8;
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
	HAL_CAN_ConfigFilter(&can1stand, &canFilterConfig);										
	
	can1stand.pTxMsg = &canTxBuffer;
	can1stand.pRxMsg = &canRxBuffer;
}

void canWrite(uint16_t reg, uint8_t subIndex, uint16_t timeout){
	canTxBuffer.DLC = 8;
	canTxBuffer.RTR = CAN_RTR_DATA;
	canTxBuffer.Data[0] = 0x22;
	canTxBuffer.Data[1] = reg & 0xFF;
	canTxBuffer.Data[2] = reg >> 8;
	canTxBuffer.Data[3] = subIndex;
	
	can1stand.pTxMsg = &canTxBuffer;
	
  HAL_CAN_Transmit(&can1stand, timeout);		
	HAL_CAN_Receive(&can1stand,CAN_FIFO0,timeout);
	
  printf("abort code: %#04x %#04x\n",can1stand.pRxMsg->Data[4] + (can1stand.pRxMsg->Data[5] << 8),
					 (can1stand.pRxMsg->Data[6] ) + (can1stand.pRxMsg->Data[7] << 8));
	
}

void setCurrent(uint16_t cob_id,int current)
{
	
	canTxBuffer.StdId = cob_id;
	canTxBuffer.RTR = CAN_RTR_DATA;
	canTxBuffer.DLC = 4;
	
	if(current >= 0){
		canTxBuffer.Data[0] = current & 0xFF;
		canTxBuffer.Data[1] = current >> 8;
		canTxBuffer.Data[2] = 0x00;
		canTxBuffer.Data[3] = 0x00;
	}
	else{
		int c = 0xFFFF + current + 1;
		canTxBuffer.Data[0] = c & 0xFF ;
		canTxBuffer.Data[1] = c >> 8;
		canTxBuffer.Data[2] = 0xFF;
		canTxBuffer.Data[3] = 0xFF;
	}
	can1stand.pTxMsg = &canTxBuffer;
	 HAL_CAN_Transmit(&can1stand,10);
}


void setRegister(uint8_t id,int value)
{
	canTxBuffer.StdId = 0x600 + id;	
	canTxBuffer.Data[4] = value & 0xFF;
	canTxBuffer.Data[5] = (value >> 8) & 0xFF;
	canTxBuffer.Data[6] = (value >> 16) & 0xFF; 
  canTxBuffer.Data[7] = (value >> 24) & 0xFF;
	canWrite(randRegister, randRegisterSubIndex,10);
	
}

void enterPreOperational()
{
	canTxBuffer.StdId = 0x00;
	canTxBuffer.RTR = 0;
	canTxBuffer.DLC = 2;
	canTxBuffer.Data[0] = 0x80;
	canTxBuffer.Data[1] = 0x00;
	can1stand.pTxMsg = &canTxBuffer;
	HAL_CAN_Transmit(&can1stand,10);
}

void enterOperational()
{
	canTxBuffer.StdId = 0x00;
	canTxBuffer.RTR = 0;
	canTxBuffer.DLC = 2;
	canTxBuffer.Data[0] = 0x01;
	canTxBuffer.Data[1] = 0x00;
	can1stand.pTxMsg = &canTxBuffer;
	HAL_CAN_Transmit(&can1stand,10);
}


void canPDOConfiguration()
{
	enterPreOperational();
	for(int i = 0; i < 2; i++){
		printf("\nPDO configuratione for node %d\n",nodeIDs[i]);
		randRegister = 0x1400;
	  randRegisterSubIndex = 0x01;
		
		randRegisterSubIndex = 0x01;
		setRegister(nodeIDs[i],0x181);
			
		randRegisterSubIndex = 0x02;
		setRegister(nodeIDs[i],0xFF);
		
		randRegister = 0x1600;
		randRegisterSubIndex = 0x00;
		setRegister(nodeIDs[i],0x00);
		
		randRegisterSubIndex = 0x01;
		setRegister(nodeIDs[i],0x20300010);
			
		randRegisterSubIndex = 0x00;
		setRegister(nodeIDs[i],0x01);
	}
	
	printf("\nPDO configuratione for node %d\n",nodeIDs[2]);
	randRegister = 0x1400;
	randRegisterSubIndex = 0x01;
	
	randRegisterSubIndex = 0x01;
	setRegister(nodeIDs[2],0x182);
			
	randRegisterSubIndex = 0x02;
	setRegister(nodeIDs[2],0xFF);
		
	randRegister = 0x1600;
	randRegisterSubIndex = 0x00;
	setRegister(nodeIDs[2],0x00);
		
	randRegisterSubIndex = 0x01;
	setRegister(nodeIDs[2],0x20300010);
			
	randRegisterSubIndex = 0x00;
	setRegister(nodeIDs[2],0x01);

	
	enterOperational();
}

void enableEpos(uint8_t id){
	randRegister = 0x6040;
	randRegisterSubIndex = 0x00;
	setRegister(id,0x0006);
	HAL_Delay(10);
	setRegister(id,0x000F);
	HAL_Delay(10);	
}

void SetCurrent3Maxon(int16_t current)
{
	setCurrent(0x181,current);
	setCurrent(0x182,-current);
}