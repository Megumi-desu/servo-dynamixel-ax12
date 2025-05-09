/*
 * dynamixel_ax12.c
 *
 *  Created on: May 8, 2025
 *      Author: rahmat bocchi lover
 */

#include "dynamixel_ax12.h"

/* External global variables */
extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim5;

/* Define fixed hardware connections */
#define DYNAMIXEL_UART         huart2
#define DYNAMIXEL_DIR_GPIO     GPIOA
#define DYNAMIXEL_DIR_PIN      GPIO_PIN_0
#define DYNAMIXEL_BAUDRATE     1000000
#define TX_DELAY_TIME          20

/* Direction pin control macros */
#define SET_TX_MODE()		   HAL_GPIO_WritePin(DYNAMIXEL_DIR_GPIO, DYNAMIXEL_DIR_PIN, GPIO_PIN_SET)
#define SET_RX_MODE()          HAL_GPIO_WritePin(DYNAMIXEL_DIR_GPIO, DYNAMIXEL_DIR_PIN, GPIO_PIN_RESET)

static void UART_WaitUntilTxComplete(void)
{
    while ((__HAL_UART_GET_FLAG(&DYNAMIXEL_UART, UART_FLAG_TC) == RESET) ||
           (__HAL_UART_GET_FLAG(&DYNAMIXEL_UART, UART_FLAG_TXE) == RESET))
    {
        /* Wait for transmission to complete */
    }
    delay_us(5);
}

static HAL_StatusTypeDef Dynamixel_SendByte(uint8_t data)
{
    HAL_StatusTypeDef status;
    status = HAL_UART_Transmit(&DYNAMIXEL_UART, &data, 1, USART_TIMEOUT);
    return status;
}

void Dynamixel_Init(void)
{
    SET_RX_MODE();
    /* Note: UART is already configured in CubeMX and MX_USART2_UART_Init() */
    __HAL_UART_FLUSH_DRREGISTER(&DYNAMIXEL_UART);
    HAL_Delay(100);
}

HAL_StatusTypeDef Dynamixel_MoveSpeed(uint8_t ID, uint16_t Position, uint16_t Speed)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Position_H, Position_L, Speed_H, Speed_L;
    uint8_t Checksum;

    Position_H = (uint8_t)(Position >> 8);
    Position_L = (uint8_t)(Position & 0xFF);
    Speed_H = (uint8_t)(Speed >> 8);
    Speed_L = (uint8_t)(Speed & 0xFF);

    Checksum = (~(ID + AX_GOAL_SP_LENGTH + AX_WRITE_DATA + AX_GOAL_POSITION_L +
                Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(AX_GOAL_SP_LENGTH);
    status |= Dynamixel_SendByte(AX_WRITE_DATA);
    status |= Dynamixel_SendByte(AX_GOAL_POSITION_L);
    status |= Dynamixel_SendByte(Position_L);
    status |= Dynamixel_SendByte(Position_H);
    status |= Dynamixel_SendByte(Speed_L);
    status |= Dynamixel_SendByte(Speed_H);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(TX_DELAY_TIME);
    SET_RX_MODE();

    return status;
}

HAL_StatusTypeDef Dynamixel_SetSlope(uint8_t ID, uint8_t CW_Slope, uint8_t CCW_Slope)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Checksum;

    Checksum = (~(ID + AX_CS_LENGTH + AX_WRITE_DATA + AX_CW_COMPLIANCE_SLOPE +
                CW_Slope + AX_CCW_COMPLIANCE_SLOPE + CCW_Slope)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(AX_CS_LENGTH);
    status |= Dynamixel_SendByte(AX_WRITE_DATA);
    status |= Dynamixel_SendByte(AX_CW_COMPLIANCE_SLOPE);
    status |= Dynamixel_SendByte(CW_Slope);
    status |= Dynamixel_SendByte(AX_CCW_COMPLIANCE_SLOPE);
    status |= Dynamixel_SendByte(CCW_Slope);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(10);
    SET_RX_MODE();

    return status;
}

HAL_StatusTypeDef Dynamixel_SetPunch(uint8_t ID, uint16_t Punch)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Punch_H, Punch_L;
    uint8_t Checksum;

    Punch_H = (uint8_t)(Punch >> 8);
    Punch_L = (uint8_t)(Punch & 0xFF);

    Checksum = (~(ID + AX_PUNCH_LENGTH + AX_WRITE_DATA + AX_PUNCH_L +
                Punch_L + Punch_H)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(AX_PUNCH_LENGTH);
    status |= Dynamixel_SendByte(AX_WRITE_DATA);
    status |= Dynamixel_SendByte(AX_PUNCH_L);
    status |= Dynamixel_SendByte(Punch_L);
    status |= Dynamixel_SendByte(Punch_H);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(10);
    SET_RX_MODE();

    return status;
}

HAL_StatusTypeDef Dynamixel_SetAngle(uint8_t ID, uint16_t Angle, uint16_t Speed)
{
    uint16_t Position;

    if (Angle > 300) Angle = 300;

    Position = (Angle * 1024) / 300;
    if (Position > 0) {
        Position = Position - 1;
    }

    return Dynamixel_MoveSpeed(ID, Position, Speed);
}

HAL_StatusTypeDef Dynamixel_SetMaxTorque(uint8_t ID, uint16_t MaxTorque)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t MaxTorque_H, MaxTorque_L;
    uint8_t Checksum;

    if (MaxTorque == 0) MaxTorque = 1023;

    MaxTorque_H = (uint8_t)(MaxTorque >> 8);
    MaxTorque_L = (uint8_t)(MaxTorque & 0xFF);

    Checksum = (~(ID + AX_MT_LENGTH + AX_WRITE_DATA + AX_MAX_TORQUE_L +
                MaxTorque_L + MaxTorque_H)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(AX_MT_LENGTH);
    status |= Dynamixel_SendByte(AX_WRITE_DATA);
    status |= Dynamixel_SendByte(AX_MAX_TORQUE_L);
    status |= Dynamixel_SendByte(MaxTorque_L);
    status |= Dynamixel_SendByte(MaxTorque_H);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(250);
    SET_RX_MODE();

    return status;
}

HAL_StatusTypeDef Dynamixel_LockEEPROM(uint8_t ID)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Checksum;

    Checksum = (~(ID + AX_LR_LENGTH + AX_WRITE_DATA + AX_LOCK + 0x01)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(AX_LR_LENGTH);
    status |= Dynamixel_SendByte(AX_WRITE_DATA);
    status |= Dynamixel_SendByte(AX_LOCK);
    status |= Dynamixel_SendByte(0x01);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(250);
    SET_RX_MODE();

    return status;
}

HAL_StatusTypeDef Dynamixel_ReadPosition(uint8_t ID, uint16_t *Position)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Checksum;
    uint8_t RxBuffer[8];

    Checksum = (~(ID + 4 + AX_READ_DATA + AX_PRESENT_POSITION_L + 2)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(4);  // Length
    status |= Dynamixel_SendByte(AX_READ_DATA);
    status |= Dynamixel_SendByte(AX_PRESENT_POSITION_L);
    status |= Dynamixel_SendByte(2);  // Number of bytes to read
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    SET_RX_MODE();
    delay_us(100);

    status = HAL_UART_Receive(&DYNAMIXEL_UART, RxBuffer, 8, 100);

    if (status == HAL_OK) {
        if (RxBuffer[0] == 0xFF && RxBuffer[1] == 0xFF && RxBuffer[2] == ID) {
            *Position = (uint16_t)RxBuffer[5] | ((uint16_t)RxBuffer[6] << 8);
        } else {
            status = HAL_ERROR;  // Invalid response
        }
    }

    return status;
}

HAL_StatusTypeDef Dynamixel_Ping(uint8_t ID)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Checksum;
    uint8_t RxBuffer[6];  // Expected response size

    Checksum = (~(ID + 2 + AX_PING)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(2);  // Length
    status |= Dynamixel_SendByte(AX_PING);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    SET_RX_MODE();
    delay_us(100);

    status = HAL_UART_Receive(&DYNAMIXEL_UART, RxBuffer, 6, 100);

    return status;  // HAL_OK if servo responded, HAL_ERROR/HAL_TIMEOUT if not
}

HAL_StatusTypeDef Dynamixel_Reset(uint8_t ID)
{
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t Checksum;

    Checksum = (~(ID + 2 + AX_RESET)) & 0xFF;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(ID);
    status |= Dynamixel_SendByte(2);  // Length
    status |= Dynamixel_SendByte(AX_RESET);
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(500);
    SET_RX_MODE();

    return status;
}

HAL_StatusTypeDef Start_Pose_SYNC(void)
{
    HAL_StatusTypeDef status = HAL_OK;
    int counts_send = 0;
    int Checksum_buf = 0;
    uint8_t Checksum;
    uint8_t Length = 94; // 18 servos
    // Length = 3 (instruction + starting address + data) + 1 (checksum) + (5 * num of servo)
    uint8_t Position_H, Position_L, Speed_H, Speed_L;

    SET_TX_MODE();
    delay_us(10);

    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(AX_HEADER);
    status |= Dynamixel_SendByte(BROADCAST_ID);  // Send to all servos
    status |= Dynamixel_SendByte(Length);
    status |= Dynamixel_SendByte(AX_SYNC_WRITE);
    status |= Dynamixel_SendByte(AX_GOAL_POSITION_L);  // Start address
    status |= Dynamixel_SendByte(4);  // Data length per servo (position L, H, speed L, H)

    Checksum_buf = BROADCAST_ID + Length + AX_SYNC_WRITE + AX_GOAL_POSITION_L + 4;

    while (counts_send <= 17) {
    	Position_H = posArray[counts_send] >> 8;
    	Position_L = posArray[counts_send] & 0xFF;
    	Speed_H	   = speedArray[counts_send] >> 8;
    	Speed_L	   = speedArray[counts_send] & 0xFF;

    	Dynamixel_SendByte((uint8_t)counts_send + 1);
    	Dynamixel_SendByte(Position_L);
    	Dynamixel_SendByte(Position_H);
    	Dynamixel_SendByte(Speed_L);
    	Dynamixel_SendByte(Speed_H);

    	Checksum_buf += ((counts_send + 1) + Position_L + Position_H + Speed_L + Speed_H);

    	if (counts_send >= 17) break;
    	counts_send++;
    }

    // Checksum = (~(BROADCAST_ID + Length + AX_SYNC_WRITE + AX_GOAL_POSITION_L + Checksum_buf + 4)) & 0xFF;
    Checksum = (~Checksum_buf) & 0xFF;
    status |= Dynamixel_SendByte(Checksum);

    UART_WaitUntilTxComplete();
    delay_us(10);

    SET_RX_MODE();

    return status;
}
