/*
 * GUI.c
 *
 *  Created on: Apr 29, 2023
 *      Author: DELL
 */
#include "GUI.h"
#include "string.h"
void floatToBytes(float val,uint8_t * bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  // Overite bytes of union with float variable
  u.float_variable = val;
  // Assign bytes to input array
  memcpy(bytes_array, u.temp_array, 4);
}
float bytesToFloat(uint8_t * bytes_array){
  // Create union of shared memory space
  union {
    float float_variable;
    uint8_t temp_array[4];
  } u;
  for(uint8_t i = 0; i<4; i++){
	  u.temp_array[i] = bytes_array[i];
  }
  return   u.float_variable;
}
