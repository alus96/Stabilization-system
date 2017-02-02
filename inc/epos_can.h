#ifndef EPOS_CAN_H
#define EPOS_CAN_H

#include "stm32f4xx_hal.h"

// void EposCanInit(CAN_HandleTypeDef canPort);

// void clearCanRxBuffer();

// void printErrorStatus(HAL_StatusTypeDef status);

// int canRead(uint16_t reg, uint16_t timeout);
// void canWrite(uint16_t reg, uint16_t timeout);

// void setCurrent(uint8_t id,int current);

#ifdef AW1STAND_RASPBERRY
#ifdef STAND_2D_RPI
//Flywheel speeds
int16_t flywheel_one_desired_speed;
int16_t flywheel_two_desired_speed;
#else
// Flywheel speed
int16_t flywheel_desired_speed;
#endif
#endif

typedef struct {
	int id;
	int32_t velocity;
	float current;
	float duty;
} can_status;


void setNodes(uint8_t nodes[], uint8_t n);
void EposCanInit(CAN_HandleTypeDef* canPort);
void enableEpos(uint8_t id);
void enterPreOperational();
void enterOperational();
void canPDOConfiguration(uint8_t i, uint16_t cob_id);
void canRxPDOConfiguration(uint8_t i, uint16_t cob_id);
void canTxPDOConfiguration(uint8_t i, uint16_t cob_id);

void clearCanRxBuffer();
void eposReset();
void printErrorStatus(HAL_StatusTypeDef status);

int canRead(uint8_t id, uint16_t reg, uint8_t subIndex);
void canWrite(uint8_t id, uint16_t reg, uint8_t subIndex, int value);

void setSDOCurrent(uint8_t i, int current);
void setCurrent(uint16_t cob_id, int current);
void SetCurrent3Maxon(int16_t current);
void enableQuickStop(uint8_t id);
void disableQuickStop(uint8_t id);

int32_t getVelocity(uint16_t cob_id);

#endif
