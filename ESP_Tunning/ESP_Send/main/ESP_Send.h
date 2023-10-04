/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "stdint.h"

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H

#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF ESP_IF_WIFI_STA
#define ESPNOW_QUEUE_SIZE 6

#define EX_UART_NUM UART_NUM_0
#define EX_UART_NUM UART_NUM_0
#define PATTERN_CHR_NUM    (1)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE (1024)
#define RD_BUF_SIZE (BUF_SIZE)
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

typedef struct {
  /* data for angle*/
  float a_Kp;
  float a_Ki;
  float a_Kd;

/* data for gys*/
  float g_Kp;
  float g_Ki;
  float g_Kd;
/*offset for 4 canhs*/
  float offset_1;
  float offset_2;
  float offset_3;
  float offset_4;

  uint16_t crc;  // check sum CRC
}Data_tune_t;

#endif