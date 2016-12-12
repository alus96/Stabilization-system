#ifndef EPOS_CAN_H
#define EPOS_CAN_H

#include "stm32f4xx_hal.h"

// void EposCanInit(CAN_HandleTypeDef canPort);

// void clearCanRxBuffer();

// void printErrorStatus(HAL_StatusTypeDef status);

// int canRead(uint16_t reg, uint16_t timeout);
// void canWrite(uint16_t reg, uint16_t timeout);

// void setCurrent(uint8_t id,int current);


void EposCanInit(CAN_HandleTypeDef canPort);
void canWrite(uint16_t reg, uint8_t subIndex, uint16_t timeout);
void setCurrent(uint16_t cob_id,int current);
void setRegister(uint8_t id,int value);
void enterPreOperational();
void enterOperational();
void canPDOConfiguration();
void enableEpos(uint8_t id);

void SetCurrent3Maxon(int16_t current);

#endif
