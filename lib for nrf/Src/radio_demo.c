#include <memory.h>
#include "support.h"
#include "nrf24.h"
#include "stdint.h"
#include "stdio.h"

#define RX_SINGLE 0
#define TX_SINGLE 1

extern uint32_t value[5];
extern UART_HandleTypeDef huart2;

void Toggle_LED()
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}
void UART_SendStr(char *string)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)string, (uint16_t)strlen(string), 200);
}
typedef struct
{
    uint8_t throttle; // left hand
    uint8_t yaw;      // left hand
    uint8_t pitch;    // right hand
    uint8_t roll;     // right hand
    uint8_t button;
    uint8_t button_1;
} NRF_Packet;

typedef enum
{
    HEIGHT = (uint8_t)0x00,
    SERVO_LEFT,
    SERVO_RIGHT
} Controler;

NRF_Packet payload_packet;
uint8_t payload_length;

uint8_t convert_to_8bits(uint32_t val, uint32_t min, uint32_t middle, uint32_t max)
{
    if (val > max)
        val = max;
    if (val < min)
        val = min;
    if (val < middle)
        return (val - min) * (127 - 0) / (middle - min) + 0;
    else
        return (val - middle) * (255 - 128) / (max - middle) + 128;
}

// Kinda foolproof :)
#if ((RX_SINGLE + TX_SINGLE) != 1)
#error "Define only one DEMO_xx, use the '1' value"
#endif

#if ((TX_SINGLE))

// Helpers for transmit mode demo

// Timeout counter (depends on the CPU speed)
// Used for not stuck waiting for IRQ
#define nRF24_WAIT_TIMEOUT (uint32_t)0x000FFFFF

// Result of packet transmission
typedef enum
{
    nRF24_TX_ERROR = (uint8_t)0x00,
    nRF24_TX_SUCCESS,
    nRF24_TX_TIMEOUT,
    nRF24_TX_MAXRT
} nRF24_TXResult;

// Length of received payload

nRF24_TXResult nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length)
{
    volatile uint32_t wait = nRF24_WAIT_TIMEOUT;
    uint8_t status;
    nRF24_CE_L();
    nRF24_WritePayload(pBuf, length);
    nRF24_CE_H();

    do
    {
        status = nRF24_GetStatus();
        if (status & (nRF24_FLAG_TX_DS | nRF24_FLAG_MAX_RT))
        {
            break;
        }
    } while (wait--);
    nRF24_CE_L();

    if (!wait)
        return nRF24_TX_TIMEOUT;

    nRF24_ClearIRQFlags();

    if (status & nRF24_FLAG_MAX_RT)
        return nRF24_TX_MAXRT;

    if (status & nRF24_FLAG_TX_DS)
        return nRF24_TX_SUCCESS;

    nRF24_FlushTX();

    return nRF24_TX_ERROR;
}
void reset_controller(void)
{
    payload_packet.throttle = 0;
    payload_packet.yaw = 127;
    payload_packet.pitch = 127;
    payload_packet.roll = 127;
    payload_packet.button = 0;
    payload_packet.button_1 = 0;
}

#endif // DEMO_TX_

int runRadio(void)
{
    nRF24_CE_L();
    if (!nRF24_Check())
    {
        while (1)
        {
            Toggle_LED();
            Delay_ms(50);
        }
    }
    nRF24_Init();

/***************************************************************************/
#if (RX_SINGLE)

    nRF24_SetRFChannel(40);
    nRF24_SetDataRate(nRF24_DR_2Mbps);
    nRF24_SetCRCScheme(nRF24_CRC_2byte);
    nRF24_SetAddrWidth(3);
    static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
    nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR);
    nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 10);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetOperationalMode(nRF24_MODE_RX);
    nRF24_ClearIRQFlags();
    nRF24_SetPowerMode(nRF24_PWR_UP);
    nRF24_CE_H();
    uint8_t data_char[30];
    payload_length = sizeof(payload_packet);

    // The main loop
    while (1)
    {
        if (nRF24_GetStatus_RXFIFO() != nRF24_STATUS_RXFIFO_EMPTY)
        {
            nRF24_RXResult pipe = nRF24_ReadPayload((uint8_t *)&payload_packet, &payload_length);
            // Clear all pending IRQ flags
            nRF24_ClearIRQFlags();
            sprintf((char *)data_char, "value: %u  %u  %u  %u\n", (size_t)payload_packet.throttle, (size_t)payload_packet.roll, (size_t)payload_packet.pitch, (size_t)payload_packet.yaw);
            UART_SendStr((char *)data_char);
        }
    }

#endif // RX_SINGLE

    /***************************************************************************/

#if (TX_SINGLE)

    nRF24_SetRFChannel(40);
    nRF24_SetDataRate(nRF24_DR_2Mbps);
    nRF24_SetCRCScheme(nRF24_CRC_2byte);
    nRF24_SetAddrWidth(3);
    static const uint8_t nRF24_ADDR[] = {'E', 'S', 'B'};
    nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR);
    nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR);
    nRF24_SetTXPower(nRF24_TXPWR_0dBm);
    nRF24_SetAutoRetr(nRF24_ARD_2500us, 10);
    nRF24_EnableAA(nRF24_PIPE0);
    nRF24_SetOperationalMode(nRF24_MODE_TX);
    nRF24_ClearIRQFlags();
    nRF24_SetPowerMode(nRF24_PWR_UP);

    payload_length = sizeof(payload_packet);
    uint8_t data_char[30];
    reset_controller();
    while (1)
    {

        payload_packet.throttle = convert_to_8bits(value[0], 450, 1585, 3620);
        payload_packet.roll = convert_to_8bits(value[1], 450, 1585, 3620);
        payload_packet.pitch = convert_to_8bits(value[2], 450, 1585, 3620);
        payload_packet.yaw = convert_to_8bits(value[3], 450, 1585, 3620);

        sprintf((char *)data_char, "value: %u  %u  %u  %u\n", (size_t)payload_packet.throttle, (size_t)payload_packet.roll, (size_t)payload_packet.pitch, (size_t)payload_packet.yaw);
        UART_SendStr((char *)data_char);
        nRF24_TXResult result = nRF24_TransmitPacket((uint8_t *)&payload_packet, payload_length);
        switch (result)
        {
        case nRF24_TX_SUCCESS:
            // todo: đã truyền thành công
            break;
        case nRF24_TX_MAXRT:
            nRF24_ResetPLOS();
        case nRF24_TX_TIMEOUT:
        default:
            // todo: Bị lỗi khi truyền đi
            break;
        }
        Delay_ms(10);
    }

#endif // TX_SINGLE
}
