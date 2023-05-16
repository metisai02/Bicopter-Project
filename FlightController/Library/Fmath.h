/******************** (C) COPYRIGHT 2015 DUT ********************************
 * ����    �����Ĳ�
 * �ļ���  ��Fmath.c
 * ����    ���Զ�����ѧ��ͷ�ļ�
 * ����    ��2015/11/30 12:43:38
 * ��ϵ��ʽ��1461318172��qq��
**********************************************************************************/



#ifndef __FMATH_H
#define __FMATH_H
#include "math.h"
#include "main.h"

#define PitchRollEXP  50   //0.5
#define PitchRollRate 100  //1.0

#define ThrMid   0
#define Thr_EXP  40


float Math_fConstrain(float value, float min, float max);
int16_t Math_Constrain(int16_t value, int16_t min, int16_t max);
int16_t Math_abs(int16_t value);
int16_t Math_min(int16_t value1, int16_t value2);
int16_t Math_max(int16_t value1, int16_t value2);
void Math_init_EXP(void);
int16_t Math_ThrEXP(int16_t RCThr);
int16_t Math_AngelEXP(int16_t in);
uint16_t Math_u16_Constrain(uint16_t value, uint16_t min, uint16_t max);
#endif

//------------------End of File----------------------------
