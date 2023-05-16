#include "IMU.h"
#include "common.h"
#define DATA_SIZE 200
extern I2C_HandleTypeDef hi2c1;

extern SPI_HandleTypeDef hspi2;
volatile float exInt, eyInt, ezInt;  
volatile float integralFBx, integralFBy, integralFBz;
//volatile float q0, q1, q2, q3,
volatile float qa0, qa1, qa2, qa3;
float q_ahrs[4];
volatile float integralFBhand, handdiff;
volatile double halfT ,elapsedT;
volatile uint32_t lastUpdate, now;
volatile float acc_vector = 0;  //  M/S^2
volatile float IMU_Pitch, IMU_Roll, IMU_Yaw;

/*********************************************************************
Initialise MPU6050 and HMC5883, configure gyro and acc readings.
Hardware dependent
*******************************************************************************/
float offset_gx, offset_gy, offset_gz;
void imu_hardware_setup(void)
{

	MPU6050_Init(&hi2c1);
	HMC5883L_initialize();

	Initialize_Q();
}

/**************************ʵ�ֺ���********************************************

*******************************************************************************/
#define new_weight 0.4f
#define old_weight 0.6f
/*
	Get values from sensors
	MPU6050 returns 6 values.
	0, 1, 2 are accelemeter data
	3 4 5 are gyro data.
*/
static int16_t IMU_unscaled[6];

void IMU_getValues(float *values)
{
	int16_t accgyroval[6];
	MPU6050_t mpu;
	static volatile float lastacc[3] = {0, 0, 0};
	uint8_t i;
	
	MPU6050_Read_All(&hi2c1, &mpu);
	accgyroval[0] = mpu.Accel_X_RAW;
	accgyroval[1] = mpu.Accel_Y_RAW;
	accgyroval[2] = mpu.Accel_Z_RAW;

	accgyroval[3] = mpu.Gyro_X_RAW;
	accgyroval[4] = mpu.Gyro_Y_RAW;
	accgyroval[5] = mpu.Gyro_Y_RAW;

	// added to watch the 'raw' values
	for (i = 0;i<6; i++)
	{
		IMU_unscaled[i] = accgyroval[i];
	}
	
	for(i = 0; i < 6; i++)
	{
		if(i < 3)
		{
			// Complementary Filter for new accelemeter data from MPU6050
			values[i] = (float) accgyroval[i] * new_weight + lastacc[i] * old_weight ;
			lastacc[i] = values[i];
		}
		else
		{
			// raw to real deg/sec -> *2000/32767 = 1/16.4
			values[i] = ((float) accgyroval[i]) / 16.4f; 
		}
	}

	/* 
		pass the address of values[6], after this function finishes, values[6],[7],[8] are filled with mag data
	*/
	HMC58X3_mgetValues(&values[6]);
}


//float acc_x,acc_y,acc_z;
float IMU_data[9];

extern float roll_offset, pitch_offset;
/*
* @brief:
	Calculate angles and angular rates from raw sensors' data and timestamp
* @input:
	*RPY pointer to first element of roll, pitch, yaw array to be filled with calculated values.

*/
static float alpha_rate_rpy = 0.63f;
void IMU_getAttitude(float *RPY,float *rate_RPY)
{
	float gx, gy, gz;
	/* This function to get the scaled values from GY-86 should be call here once. 
	used by all algorithms
	*/
	IMU_getValues(IMU_data);
	/*
	time elapsed should be calculated right after IMU data is read
	*/
	now = HAL_GetTick();
	// in case the 32 bits timer is overflow
	if(now < lastUpdate) 
	{
		elapsedT =  (float)(now + (0xffffffff - lastUpdate));
	}
	else
	{
		elapsedT =  (now - lastUpdate);
	}
	//convert us to second
	elapsedT = elapsedT * 0.000001f;
	halfT = elapsedT / 2.0f;
	lastUpdate = now;

    // acc_x= IMU_data[0];
    // acc_y= IMU_data[1];
    // acc_z= IMU_data[2];
	/*
		Compensate for gyroscope offset, at startup, when copters at static state. 
		We calculate the offsets by average the first N values read from gyroscope sensor
	*/
    gx = IMU_unscaled[3] - offset_gx;
    gy = IMU_unscaled[4] - offset_gy;
    gz = IMU_unscaled[5] - offset_gz;
	// complimentary filter for angular rate
    rate_RPY[0] = rate_RPY[0] * alpha_rate_rpy + gx * (1.0f - alpha_rate_rpy)/16.4f;
    rate_RPY[1] = rate_RPY[1] * alpha_rate_rpy + gy * (1.0f - alpha_rate_rpy)/16.4f;
    rate_RPY[2] = rate_RPY[2] * alpha_rate_rpy + gz * (1.0f - alpha_rate_rpy)/16.4f;
	/*
		Calculate angles
	*/
	IMU_getQ(q_ahrs, IMU_data); 
	float temp_rpy[3];
	IMU_getRollPitchYaw(temp_rpy, q_ahrs);
	/*
		compensate for IMU orientation and offset
		application dependent
	*/
	RPY[0] = temp_rpy[0] - roll_offset;
	RPY[1] = (-temp_rpy[1]) - pitch_offset;
	RPY[2] = -temp_rpy[2];
}

/*
	Initialize orientation quaternion, using angles calculated from accelerometer 
*/
void Initialize_Q(void)
{
	float acc[9];
	int i;
	volatile float temp, roll, pitch, yaw, yh, xh;
	volatile float  acc_X, acc_Y, acc_Z, acc_MX, acc_MY, acc_MZ; //
	
	// initialize quaternion
	q_ahrs[0] = 1.0f;  //
	q_ahrs[1] = 0.0f;
	q_ahrs[2] = 0.0f;
	q_ahrs[3] = 0.0f;
	exInt = 0.0;
	eyInt = 0.0;
	ezInt = 0.0;
	integralFBx = 0.0;
	integralFBy = 0.0;
	integralFBz	= 0.0;
	/*
    	Get DATA_SIZE raw samples and calculate the average.
		Calculate the initial quaternion values using traditional method. 
	*/
	for(i = 0; i < DATA_SIZE; i++) {
		IMU_getValues(acc);
		acc_X += acc[0];
		acc_Y += acc[1];
		acc_Z += acc[2];
		acc_MX += acc[6];
		acc_MY += acc[7];
		acc_MZ += acc[8];
	}
	acc_X /= DATA_SIZE;
	acc_Y /= DATA_SIZE;
	acc_Z /= DATA_SIZE;
	acc_MX /= DATA_SIZE;
	acc_MY /= DATA_SIZE;
	acc_MZ /= DATA_SIZE;

	temp = acc_X * invSqrt((acc_Y * acc_Y + acc_Z * acc_Z));
	pitch = atan(temp) * 57.3;

	temp = acc_Y * invSqrt((acc_X * acc_X + acc_Z * acc_Z));
	roll = atan(temp) * 57.3;

	yh = acc_MY * cos(roll) + acc_MZ * sin(roll);
	xh = acc_MX * cos(pitch) + acc_MY * sin(roll) * sin(pitch) - acc_MZ * cos(roll) * sin(pitch);
	yaw = atan2(yh, xh);
	// Initial quaternion values
	q_ahrs[0] = cos(roll / 2) * cos(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q_ahrs[1] = sin(roll / 2) * cos(pitch / 2) * cos(yaw / 2) - cos(roll / 2) * sin(pitch / 2) * sin(yaw / 2);
	q_ahrs[2] = cos(roll / 2) * sin(pitch / 2) * cos(yaw / 2) + sin(roll / 2) * cos(pitch / 2) * sin(yaw / 2);
  	q_ahrs[3] = cos(roll / 2) * cos(pitch / 2) * sin(yaw / 2) - sin(roll / 2) * sin(pitch / 2) * cos(yaw / 2);
}


/****************************************************************
Requirements:
previous q0...q3
Based on MahonyAHRS

*******************************************************************************/

#define Kp 3.0f
#define Ki 0.03f
/*
	Get calculated gyroscope from Mahony filter, test rate controller with these values.
*/
static float gyro_mahony[3];
void IMU_AHRSupdate(float* q_update, volatile float gx, volatile float gy, volatile float gz, volatile float ax, volatile float ay, volatile float az, volatile float mx, volatile float my, volatile float mz)
{
	volatile  float norm;
	volatile float hx, hy, hz, bx, bz;
	volatile float vx, vy, vz, wx, wy, wz;
	volatile float ex, ey, ez;
	volatile float temp0, temp1, temp2, temp3;
	volatile float temp;

	float q0q0 = q_update[0] * q_update[0];
	float q0q1 = q_update[0] * q_update[1];
	float q0q2 = q_update[0] * q_update[2];
	float q0q3 = q_update[0] * q_update[3];
	float q1q1 = q_update[1] * q_update[1];
	float q1q2 = q_update[1] * q_update[2];
	float q1q3 = q_update[1] * q_update[3];
	float q2q2 = q_update[2] * q_update[2];
	float q2q3 = q_update[2] * q_update[3];
	float q3q3 = q_update[3] * q_update[3];

	temp = sqrt(ax * ax + ay * ay + az * az);
	temp = (temp / 16384.0f) * 9.8f;   //M/S^2
	acc_vector = acc_vector +   //��ͨ�˲�����ֹƵ��20hz
		     (halfT * 2.0f / (7.9577e-3f + halfT * 2.0f)) * (temp - acc_vector);
	
	// Normalise accelerometer measurement
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	//�ü��ٶȼ���roll��pitch
	//	temp = ax * invSqrt((ay * ay + az * az));
	//	ACC_Pitch = atan(temp)* 57.3;
	//
	//	temp = ay * invSqrt((ax * ax + az * az));
	//	ACC_Roll = atan(temp)* 57.3;
	// Normalise magnetometer measurement
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	
	// Reference direction of Earth's magnetic field
	// compute reference direction of flux

	hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
	hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
	hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
	/*

	*/
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	// estimated direction of gravity and flux (v and w)

	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	/*

	*/
	wx = 2 * bx * (0.5f - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
	wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
	wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5f - q1q1 - q2q2);

	// error is sum of cross product between reference direction of fields and direction measured by sensors

	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	/*

	*/
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;

		// adjusted gyroscope measurements

		gx = gx + (Kp * ex + exInt);
		gy = gy + (Kp * ey + eyInt);
		gz = gz + (Kp * ez + ezInt);
    	gyro_mahony[0] = gx*RAD_TO_DEG;
		gyro_mahony[1] = gy*RAD_TO_DEG;
		gyro_mahony[2] = gz*RAD_TO_DEG;
	}

	// integrate quaternion rate and normalise
	temp0 = q_update[0] + (-q_update[1] * gx - q_update[2] * gy - q_update[3] * gz) * halfT;
	temp1 = q_update[1] + (q_update[0] * gx + q_update[2] * gz - q_update[3] * gy) * halfT;
	temp2 = q_update[2] + (q_update[0] * gy - q_update[1] * gz + q_update[3] * gx) * halfT;
	temp3 = q_update[3] + (q_update[0] * gz + q_update[1] * gy - q_update[2] * gx) * halfT;

	// normalise quaternion
	norm = invSqrt(temp0 * temp0 + temp1 * temp1 + temp2 * temp2 + temp3 * temp3);
	q_update[0] = temp0 * norm;
	q_update[1] = temp1 * norm;
	q_update[2] = temp2 * norm;
	q_update[3] = temp3 * norm;
}
/*
Calculate qa0...qa3
*/

#define twoKpDef  (1.0f ) // 2 * proportional gain
#define twoKiDef  (0.2f) // 2 * integral gain


/****************************************************************************
Get Q using AHRS update
*******************************************************************************/
void IMU_getQ(float *q,volatile float IMU_values[9])
{
	// deg to radian
	IMU_AHRSupdate(q, IMU_values[3] * M_PI / 180, IMU_values[4] * M_PI / 180, IMU_values[5] * M_PI / 180,
		       IMU_values[0], IMU_values[1], IMU_values[2], IMU_values[6], IMU_values[7], IMU_values[8]);
}
void IMU_getRollPitchYaw(float *angles, float *qa)
{
	/*
	 	Quaternion To Euler function 
	*/
	IMU_Roll = angles[0] = (atan2(2.0f * (qa[0] * qa[1] + qa[2] * qa[3]),
				      1 - 2.0f * (qa[1] * qa[1] + qa[2] * qa[2]))) * 180 / M_PI;
	//angles[2]-=0.8f;
	// we let safe_asin() handle the singularities near 90/-90 in pitch
	IMU_Pitch = angles[1] = -safe_asin(2.0f * (qa[0] * qa[2] - qa[3] * qa[1])) * 180 / M_PI;
	IMU_Yaw = angles[2] = -atan2(2 * qa[1] * qa[2] + 2 * qa[0] * qa[3], -2*qa[2]*qa[2] - 2*qa[3]*qa[3] + 1) * 180 / M_PI; // yaw
}

float average_filter(float buffer[], uint8_t buffer_size, float sample, uint8_t *count) {
	buffer[buffer_size]-= buffer[*count];
	buffer[*count] = sample;
	buffer[buffer_size]+= buffer[*count];
	*count = (*count+1)%buffer_size;
	return buffer[buffer_size]/(float)buffer_size;
}

/*
sample: new measured sample
return: filtered value
*/
float LPF(float sample, float pre_value, float cut_off, float dt)
{
    float RC, alpha, y;
    RC = 1.0f/(cut_off*2*3.1416f);
    alpha = dt/(RC+dt);
    y = pre_value + alpha * ( sample - pre_value );
    return y;
}

float safe_asin(float v)
{
	if (isnan(v))
	{
		return 0.0f;
	}
	if (v >= 1.0f)
	{
		return M_PI / 2;
	}
	if (v <= -1.0f)
	{
		return -M_PI / 2;
	}
	return asin(v);
}

// Fast inverse square-root
float invSqrt(float x)
{
	volatile float halfx = 0.5f * x;
	volatile float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

