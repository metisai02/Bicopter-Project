/* ESPNOW Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#ifndef ESPNOW_EXAMPLE_H
#define ESPNOW_EXAMPLE_H
#include "stdint.h"
/* ESPNOW can work in both station and softap mode. It is configured in menuconfig. */
#define ESPNOW_WIFI_MODE WIFI_MODE_STA
#define ESPNOW_WIFI_IF   ESP_IF_WIFI_STA
#define TXD_PIN (GPIO_NUM_4)
#define RXD_PIN (GPIO_NUM_5)

#define ESPNOW_QUEUE_SIZE           6
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