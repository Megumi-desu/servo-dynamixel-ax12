/*
 * dynamixel_ax12.h
 *
 *  Created on: May 8, 2025
 *      Author: rahmat bocchi lover
 */

#ifndef INC_DYNAMIXEL_AX12_H_
#define INC_DYNAMIXEL_AX12_H_

#include "main.h"
#include "stm32f4xx_hal.h"

/*************** Header **************/

#define AX_HEADER                   0xFF

#define AX_MODEL_NUMBER_L           0x00
#define AX_MODEL_NUMBER_H           0x01
#define AX_VERSION                  0x02
#define AX_ID                       0x03
#define AX_BAUD_RATE                0x04
#define AX_RETURN_DELAY_TIME        0x05
#define AX_CW_ANGLE_LIMIT_L         0x06
#define AX_CW_ANGLE_LIMIT_H         0x07
#define AX_CCW_ANGLE_LIMIT_L        0x08
#define AX_CCW_ANGLE_LIMIT_H        0x09
#define AX_SYSTEM_DATA2             0x0A
#define AX_LIMIT_TEMPERATURE        0x0B
#define AX_DOWN_LIMIT_VOLTAGE       0x0C
#define AX_UP_LIMIT_VOLTAGE         0x0D
#define AX_MAX_TORQUE_L             0x0E
#define AX_MAX_TORQUE_H             0x0F
#define AX_RETURN_LEVEL             0x10
#define AX_ALARM_LED                0x11
#define AX_ALARM_SHUTDOWN           0x12
#define AX_OPERATING_MODE           0x13
#define AX_DOWN_CALIBRATION_L       0x14
#define AX_DOWN_CALIBRATION_H       0x15
#define AX_UP_CALIBRATION_L         0x16
#define AX_UP_CALIBRATION_H         0x17

/************** RAM AREA **************/
#define AX_TORQUE_ENABLE            0x18
#define AX_LED                      0x19
#define AX_CW_COMPLIANCE_MARGIN     0x1A
#define AX_CCW_COMPLIANCE_MARGIN    0x1B
#define AX_CW_COMPLIANCE_SLOPE      0x1C
#define AX_CCW_COMPLIANCE_SLOPE     0x1D
#define AX_GOAL_POSITION_L          0x1E
#define AX_GOAL_POSITION_H          0x1F
#define AX_GOAL_SPEED_L             0x20
#define AX_GOAL_SPEED_H             0x21
#define AX_TORQUE_LIMIT_L           0x22
#define AX_TORQUE_LIMIT_H           0x23
#define AX_PRESENT_POSITION_L       0x24
#define AX_PRESENT_POSITION_H       0x25
#define AX_PRESENT_SPEED_L          0x26
#define AX_PRESENT_SPEED_H          0x27
#define AX_PRESENT_LOAD_L           0x28
#define AX_PRESENT_LOAD_H           0x29
#define AX_PRESENT_VOLTAGE          0x2A
#define AX_PRESENT_TEMPERATURE      0x2B
#define AX_REGISTERED_INSTRUCTION   0X2C
#define AX_PAUSE_TIME               0x2D
#define AX_MOVING                   0x2E
#define AX_LOCK                     0x2F
#define AX_PUNCH_L                  0x30
#define AX_PUNCH_H                  0x31

/******** Status Return Levels ********/
#define AX_RETURN_NONE              0x00
#define AX_RETURN_READ              0x01
#define AX_RETURN_ALL               0x02

/*********** Instruction Set ***********/
#define AX_PING                     0x01
#define AX_READ_DATA                0x02
#define AX_WRITE_DATA               0x03
#define AX_REG_WRITE                0x04
#define AX_ACTION                   0x05
#define AX_RESET                    0x06
#define AX_SYNC_WRITE               0x83

/************** Specials **************/

#define OFF                         0
#define ON                          1
#define LEFT						0
#define RIGHT                       1

#define AX_BYTE_READ                0x01
#define AX_BYTE_READ_POS            0x02
#define AX_RESET_LENGTH				0x02
#define AX_ACTION_LENGTH			0x02
#define AX_ID_LENGTH                0x04
#define AX_LR_LENGTH                0x04
#define AX_SRL_LENGTH               0x04
#define AX_RDT_LENGTH               0x04
#define AX_LEDALARM_LENGTH          0x04
#define AX_SALARM_LENGTH            0x04
#define AX_TL_LENGTH                0x04
#define AX_VL_LENGTH                0x06
#define AX_CM_LENGTH                0x06
#define AX_CS_LENGTH                0x06
#define AX_CCW_CW_LENGTH            0x08
#define AX_BD_LENGTH                0x04
#define AX_TEM_LENGTH               0x04
#define AX_MOVING_LENGTH            0x04
#define AX_RWS_LENGTH               0x04
#define AX_VOLT_LENGTH              0x04
#define AX_LED_LENGTH               0x04
#define AX_TORQUE_LENGTH            0x04
#define AX_POS_LENGTH               0x04
#define AX_GOAL_LENGTH              0x05
#define AX_MT_LENGTH                0x05
#define AX_PUNCH_LENGTH             0x05
#define AX_SPEED_LENGTH             0x05
#define AX_GOAL_SP_LENGTH           0x07
#define AX_ACTION_CHECKSUM		    0xFA
#define BROADCAST_ID                0xFE


#define USART_TIMEOUT         		50 // in ms



extern uint8_t Checksum;
extern uint16_t posArray[18]	= {0};
extern uint16_t speedArray[18]  = {0};


/* Function Prototypes */
void Dynamixel_Init(void);
HAL_StatusTypeDef Dynamixel_MoveSpeed(uint8_t ID, uint16_t Position, uint16_t Speed);
HAL_StatusTypeDef Dynamixel_SetSlope(uint8_t ID, uint8_t CW_Slope, uint8_t CCW_Slope);
HAL_StatusTypeDef Dynamixel_SetPunch(uint8_t ID, uint16_t Punch);
HAL_StatusTypeDef Dynamixel_SetAngle(uint8_t ID, uint16_t Angle, uint16_t Speed);
HAL_StatusTypeDef Dynamixel_SetMaxTorque(uint8_t ID, uint16_t MaxTorque);
HAL_StatusTypeDef Dynamixel_LockEEPROM(uint8_t ID);
HAL_StatusTypeDef Dynamixel_ReadPosition(uint8_t ID, uint16_t *Position);
HAL_StatusTypeDef Dynamixel_Ping(uint8_t ID);
HAL_StatusTypeDef Dynamixel_Reset(uint8_t ID);
HAL_StatusTypeDef Start_Pose_SYNC(void);

#endif /* INC_DYNAMIXEL_AX12_H_ */
