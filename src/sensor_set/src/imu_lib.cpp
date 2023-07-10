#include "sensor_set/imu_lib.hpp"

uint64_t micro_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}
ICM20948::ICM20948(){

    time_interval=0.005f;
    ex_i = 0;
    ey_i = 0;
    ez_i = 0;
    Kp=3.0;
    Ki=0.5;




  
}
void ICM20948::I2Cinit(int i2c_devnum){
    fd = i2cInit(i2c_devnum);
    fd_address=&fd;
    state_check bRet = false;
    gstGyroOffset ={0,0,0}; 
    bRet = ICM20948::Check();
  
    if( true == bRet)
    {
        std::cout<< "Motion sersor is ICM-20948\n"<<std::endl;
        ICM20948::Init();
    }
    else
    {
     
        std::cout<<    "Motion sersor NULL\n"<<std::endl;
    }
    q0 = 1.0f;  
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}
state_check ICM20948::Check(void)
{
    state_check bRet = false;
    if(REG_VAL_WIA == I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_WIA,fd_address))
    {I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address);
        bRet = true;
    }
    return bRet;
}

void ICM20948::Init(void)
{
    
  /* user bank 0 register */

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET,fd_address);
  usleep(5000);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE,fd_address);  

  /* user bank 2 register */
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2,fd_address);
  I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x04,fd_address);
  I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1,   
                  REG_VAL_BIT_GYRO_DLPCFG_6 | REG_VAL_BIT_GYRO_FS_250DPS | REG_VAL_BIT_GYRO_DLPF,fd_address);
  I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2,  0x04,fd_address);
  I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG,
                  REG_VAL_BIT_ACCEL_DLPCFG_6 | REG_VAL_BIT_ACCEL_FS_2g | REG_VAL_BIT_ACCEL_DLPF,fd_address);

  /* user bank 0 register */
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); 

  usleep(5000);
  /* offset */
  ICM20948::GyroOffset();

  return;
}
void ICM20948::GyroRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z)
{
    uint8_t u8Buf[6];
    int16_t s16Buf[3] = {0}; 
    uint8_t i;
    //int32_t s32OutBuf[3] = {0};
    static int16_t ss16c = 0;
    ss16c++;

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_L,fd_address); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_H,fd_address);
    s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_L,fd_address); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_YOUT_H,fd_address);
    s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_L,fd_address); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_GYRO_ZOUT_H,fd_address);
    s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];
    

    ps16X = s16Buf[0] - gstGyroOffset.s16X;
    ps16Y = s16Buf[1] - gstGyroOffset.s16Y;
    ps16Z = s16Buf[2] - gstGyroOffset.s16Z;

    return;
}
void ICM20948::AccelRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z)
{
    uint8_t u8Buf[2];
    int16_t s16Buf[3] = {0}; 
    uint8_t i;
    //int32_t s32OutBuf[3] = {0};

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_L,fd_address); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H,fd_address);

    s16Buf[0]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_L,fd_address); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_YOUT_H,fd_address);
    s16Buf[1]=  (u8Buf[1]<<8)|u8Buf[0];

    u8Buf[0]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_L,fd_address); 
    u8Buf[1]=I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_ZOUT_H,fd_address);
    s16Buf[2]=  (u8Buf[1]<<8)|u8Buf[0];


    ps16X = s16Buf[0];
    ps16Y = s16Buf[1];
    ps16Z = s16Buf[2];

    return;
}

void ICM20948::GyroOffset(void)
{
  uint8_t i;
  int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
  int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;

  for(i = 0; i < 128; i ++)
  {
    ICM20948::GyroRead(s16Gx, s16Gy, s16Gz);
    s32TempGx += s16Gx;
    s32TempGy += s16Gy;
    s32TempGz += s16Gz;
    usleep(5000);
  }
  gstGyroOffset.s16X = s32TempGx >> 7;
  gstGyroOffset.s16Y = s32TempGy >> 7;
  gstGyroOffset.s16Z = s32TempGz >> 7;
  
  return;
}


void ICM20948::imuDataGet(IMU_QUATERNION_DATA &pstQuaternion, 
                IMU_ST_SENSOR_DATA &pstGyroRawData,
                IMU_ST_SENSOR_DATA &pstAccelRawData)
                {
        
        float MotionVal[6];
        int16_t s16Gyro[3], s16Accel[3];
        ICM20948::AccelRead(s16Accel[0], s16Accel[1], s16Accel[2]);

        ICM20948::GyroRead(s16Gyro[0], s16Gyro[1], s16Gyro[2]);


        MotionVal[0]=s16Gyro[0]/131;
        MotionVal[1]=s16Gyro[1]/131;
        MotionVal[2]=s16Gyro[2]/131;
        MotionVal[3]=s16Accel[0];
        MotionVal[4]=s16Accel[1];
        MotionVal[5]=s16Accel[2];

        mahonyAHRSupdate((float)MotionVal[0] * 0.01745, (float)MotionVal[1] * 0.01745, (float)MotionVal[2] * 0.01745,
                        (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5]);
        pstQuaternion.q0=q0;
        pstQuaternion.q1=q1;
        pstQuaternion.q2=q2;
        pstQuaternion.q3=q3;

        /*
        pstAngles.fPitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.295; // pitch
        pstAngles.fRoll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.295; // roll
        pstAngles.fYaw = atan2(-2 * q1 * q2 - 2 * q0 * q3, 2 * q2 * q2 + 2 * q3 * q3 - 1) * 57.295; 
        */
        pstGyroRawData.s16X = s16Gyro[0];
        pstGyroRawData.s16Y = s16Gyro[1];
        pstGyroRawData.s16Z = s16Gyro[2];

        pstAccelRawData.s16X = s16Accel[0];
        pstAccelRawData.s16Y = s16Accel[1];
        pstAccelRawData.s16Z  = s16Accel[2];
        }


void ICM20948::mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az){
    float Norm;
	float vx, vy, vz;
	float ex, ey, ez;
	float qa, qb, qc;
        
	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		Norm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= Norm;
		ay *= Norm;
		az *= Norm;        

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		vx = 2*(q1 * q3 - q0 * q2);
		vy = 2*(q0 * q1 + q2 * q3);
		vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
	
		// Error is sum of cross product between estimated and measured direction of gravity
		ex = (ay * vz - az * vy);
		ey = (az * vx - ax * vz);
		ez = (ax * vy - ay * vx);
    ex_i += ex*time_interval;
    ey_i += ey*time_interval;
    ez_i += ez*time_interval;

    gx += Kp * ex + Ki * ex_i;
    gy += Kp * ey + Ki * ey_i;
    gz += Kp * ez + Ki * ez_i;

	}
	

	qa = q0;
	qb = q1;
	qc = q2;
 
	q0 += 0.5f*(-qb * gx - qc * gy - q3 * gz)*time_interval;
	q1 += 0.5f*(qa * gx + qc * gz - q3 * gy)*time_interval;
	q2 += 0.5f*(qa * gy - qb * gz + q3 * gx)*time_interval;
	q3 += 0.5f*(qa * gz + qb * gy - qc * gx)*time_interval; 
	
	// Normalise quaternion
	Norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= Norm;
	q1 *= Norm;
	q2 *= Norm;
	q3 *= Norm;

}

void ICM20948::Close(void)
{
    i2cClose(fd_address);
    std::cout<<    "imu sensor close"<<std::endl;
}