#include "sensor_set/imu_lib.hpp"

uint64_t micro_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return tv.tv_sec * (uint64_t)1000000 + tv.tv_usec;
}
ICM20948::ICM20948(){


    ex_i = 0;
    ey_i = 0;
    ez_i = 0;
    Kp=5.0;
    Ki=0;


}
int32_t ICM20948::CalAvgValue( int16_t *AvgBuffer)
{ 
  uint8_t i;
  

    
  int32_t OutVal = 0;
  for(i = 0; i < 7; i ++) 
    {
      OutVal += *(AvgBuffer+i);
    }
  OutVal /= 7;
  return OutVal;
}





void ICM20948::I2Cinit(int i2c_devnum,uint8_t Gscale,uint8_t Ascale){
    fd = i2cInit(i2c_devnum);
    fd_address=&fd;
    state_check bRet = false;
    gstGyroOffset ={0,0,0};
    stAccelmatrix={1,0,0,0,1,0,0,0,1};
     
    bRet = ICM20948::Check();
  
    if( true == bRet)
    {
        std::cout<< "Motion sersor is ICM-20948\n"<<std::endl;
        ICM20948::Init(Gscale,Ascale);
        switch (Gscale)
        {
        // Possible gyro scales (and their register bit settings) are:
          case REG_VAL_BIT_GYRO_FS_250DPS:
              GYRO_SCALE_FACTOR=GFS_250DPS_SCALE_FACTOR;
                break;
          case REG_VAL_BIT_GYRO_FS_500DPS:
              GYRO_SCALE_FACTOR=GFS_500DPS_SCALE_FACTOR;
                break;
          case REG_VAL_BIT_GYRO_FS_1000DPS:
              GYRO_SCALE_FACTOR=GFS_1000DPS_SCALE_FACTOR;
              break;
          case REG_VAL_BIT_GYRO_FS_2000DPS:
              GYRO_SCALE_FACTOR=GFS_2000DPS_SCALE_FACTOR;
              break;
        }
        switch (Ascale)
        {
        // Possible gyro scales (and their register bit settings) are:
          case REG_VAL_BIT_ACCEL_FS_2g:
              ACCEL_SCALE_FACTOR=AFS_2G_SCALE_FACTOR;
                break;
          case REG_VAL_BIT_ACCEL_FS_4g:
              ACCEL_SCALE_FACTOR=AFS_4G_SCALE_FACTOR;
                break;
          case REG_VAL_BIT_ACCEL_FS_8g:
              ACCEL_SCALE_FACTOR=AFS_8G_SCALE_FACTOR;
              break;
          case REG_VAL_BIT_ACCEL_FS_16g:
              ACCEL_SCALE_FACTOR=AFS_16G_SCALE_FACTOR;
              break;
        }
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

void ICM20948::Init(uint8_t Gscale,uint8_t Ascale)
{
    
    /* user bank 0 register */

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_ALL_RGE_RESET,fd_address);
    usleep(3000);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_PWR_MIGMT_1,  REG_VAL_RUN_MODE,fd_address);  

    /* user bank 2 register */
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2,fd_address);

    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_SMPLRT_DIV, 0x00,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_GYRO_CONFIG_1,   
                    REG_VAL_BIT_GYRO_DLPCFG_6 | Gscale | REG_VAL_BIT_GYRO_DLPF,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_1,  0x00,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_SMPLRT_DIV_2,  0x00,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948, REG_ADD_ACCEL_CONFIG,
                    REG_VAL_BIT_ACCEL_DLPCFG_5 | Ascale | REG_VAL_BIT_ACCEL_DLPF,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_ADD_ODR_ALIGN_EN,0x01,fd_address);

    /* user bank 0 register */
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); 


    /* offset */

    // ICM20948::GyroOffset(); //this is offset code

    // MagCheck();
    // WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,REG_ADD_MAG_CNTL3,0x01);
    // WriteSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_WRITE,
    //                             REG_ADD_MAG_CNTL2, REG_VAL_MAG_MODE_100HZ);  

    // //magnetic_setting                           
    // I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3,fd_address); //swtich bank3
    // I2C_WriteOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_MST_CTRL,0x07,fd_address);
    // I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,fd_address);
    // I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  REG_ADD_MAG_DATA,fd_address);

    // //I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  REG_ADD_MAG_ST2,fd_address);
    // I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|0x07,fd_address);
    // I2C_WriteOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_MST_ODR_CONFIG,0x04,fd_address);
    // I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); //swtich bank0
    // I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, 0x20,fd_address);
    usleep(1000);
    
  return;
}
void ICM20948::GyroOffsetConfig(void){
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_2,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_XG_OFFS_USRH,0x00,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_XG_OFFS_USRL,gstGyroOffset.s16X,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_YG_OFFS_USRH,0x00,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_YG_OFFS_USRL, gstGyroOffset.s16Y,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_ZG_OFFS_USRH,0x00,fd_address);
    I2C_WriteOneByte( I2C_ADD_ICM20948,REG_ZG_OFFS_USRL,gstGyroOffset.s16Z,fd_address);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); 
}
bool ICM20948::MagCheck(void)
{
    bool bRet = false;
    uint8_t u8Ret[2];
    
    ReadSecondary( I2C_ADD_ICM20948_AK09916|I2C_ADD_ICM20948_AK09916_READ,
                                REG_ADD_MAG_WIA1, 2,u8Ret);
    if( (u8Ret[0] == REG_VAL_MAG_WIA1) && ( u8Ret[1] == REG_VAL_MAG_WIA2) )
    {
        std::cout<<"magnetic on"<<std::endl;
        bRet = true;
    }
    
    return bRet;
}


void ICM20948::ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data)
{
    uint8_t i;
    uint8_t u8Temp;

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3,fd_address); //swtich bank3
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_ADDR, u8I2CAddr,fd_address);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_REG,  u8RegAddr,fd_address);
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL, REG_VAL_BIT_SLV0_EN|u8Len,fd_address);

    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); //swtich bank0
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL,fd_address);
    u8Temp |= REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp,fd_address);
    usleep(2500);
    u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp,fd_address);
    
    for(i=0; i<u8Len; i++)
    {
        *(pu8data+i) = I2C_ReadOneByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00+i,fd_address);
        
    }
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3,fd_address); //swtich bank3
    
    u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL,fd_address);
    u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp,fd_address);
    
    I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); //swtich bank0

}





void ICM20948::WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data)
{
  uint8_t u8Temp;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL,  REG_VAL_REG_BANK_3,fd_address); //swtich bank3
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_ADDR, u8I2CAddr,fd_address);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_REG,  u8RegAddr,fd_address);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_DO,   u8data,fd_address);
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV1_CTRL, REG_VAL_BIT_SLV0_EN|1,fd_address);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); //swtich bank0

  u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_USER_CTRL,fd_address);
  u8Temp |= REG_VAL_BIT_I2C_MST_EN;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp,fd_address);
  usleep(2500);
  u8Temp &= ~REG_VAL_BIT_I2C_MST_EN;
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_USER_CTRL, u8Temp,fd_address);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_3,fd_address); //swtich bank3

  u8Temp = I2C_ReadOneByte(I2C_ADD_ICM20948,REG_ADD_I2C_SLV0_CTRL,fd_address);
  u8Temp &= ~((REG_VAL_BIT_I2C_MST_EN)&(REG_VAL_BIT_MASK_LEN));
  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_I2C_SLV0_CTRL,  u8Temp,fd_address);

  I2C_WriteOneByte(I2C_ADD_ICM20948, REG_ADD_REG_BANK_SEL, REG_VAL_REG_BANK_0,fd_address); //swtich bank0
    
    return;
}









void ICM20948::GyroRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z)
{
    uint8_t u8Buf[burst_part]={0};
    int16_t s16Buf[3] = {0}; 
    uint8_t i;


    I2C_ReadBurstByte(I2C_ADD_ICM20948,REG_ADD_GYRO_XOUT_H,u8Buf,burst_part,fd_address);

    s16Buf[0]=  (u8Buf[0]<<8)|u8Buf[1];

    s16Buf[1]=  (u8Buf[2]<<8)|u8Buf[3];


    s16Buf[2]=  (u8Buf[4]<<8)|u8Buf[5];
    

    ps16X = s16Buf[0] - gstGyroOffset.s16X;
    ps16Y = s16Buf[1] - gstGyroOffset.s16Y;
    ps16Z = s16Buf[2] - gstGyroOffset.s16Z;

    return;
}
void ICM20948::AccelRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z)
{
    uint8_t u8Buf[burst_part]={0};
    int16_t s16Buf[3] = {0}; 
    uint8_t i;



    I2C_ReadBurstByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H,u8Buf,burst_part,fd_address);

    s16Buf[0]=  (u8Buf[0]<<8)|u8Buf[1];

    s16Buf[1]=  (u8Buf[2]<<8)|u8Buf[3];


    s16Buf[2]=  (u8Buf[4]<<8)|u8Buf[5];


    ps16X = s16Buf[0];
    ps16Y = s16Buf[1];
    ps16Z = s16Buf[2];

    return;
}

void ICM20948::MagRead(int16_t& s16X, int16_t& s16Y, int16_t& s16Z)
{

    uint8_t st_buf;
    uint8_t u8Data[MAG_DATA_LEN];
    int16_t s16Buf[3] = {0};
    int32_t s32Buf[3];
    uint8_t i;


    I2C_ReadBurstByte(I2C_ADD_ICM20948, REG_ADD_EXT_SENS_DATA_00,u8Data,MAG_DATA_LEN,fd_address);
    if(((u8Data[6]<<4)!=0x80)){
        
    s16Buf[0] = (u8Data[1]<<8) | u8Data[0];
    s16Buf[1] = (u8Data[3]<<8) | u8Data[2];
    s16Buf[2] = (u8Data[5]<<8) | u8Data[4];
    }
    else{
        std::cout<<"fail"<<std::endl;
    }


    

    s16Buf[0] =  (s16Buf[0]-stMagoffsets.s16X)*stMagscales.s16X;
    s16Buf[1]= -(s16Buf[1]-stMagoffsets.s16Y)*stMagscales.s16Y;
    s16Buf[2] = -(s16Buf[2]-stMagoffsets.s16Z)*stMagscales.s16Z;

    s16X=s16Buf[0];
    s16Y=s16Buf[1];
    s16Z=s16Buf[2];

    return;
}


void ICM20948::GyroOffset(void)
{
  int i;
  int16_t s16Gx = 0, s16Gy = 0, s16Gz = 0;
  int32_t s32TempGx = 0, s32TempGy = 0, s32TempGz = 0;

  for(i = 0; i < 2048; i ++)
  {
    ICM20948::GyroRead(s16Gx, s16Gy, s16Gz);
    s32TempGx += s16Gx;
    s32TempGy += s16Gy;
    s32TempGz += s16Gz;
    usleep(1000);
  }
  gstGyroOffset.s16X = s32TempGx>>11;
  gstGyroOffset.s16Y = s32TempGy>>11;
  gstGyroOffset.s16Z = s32TempGz>>11;
  printf("x:%x\t%x\t%x\n",gstGyroOffset.s16X,gstGyroOffset.s16Y,gstGyroOffset.s16Z);
  printf("x:%d\t%d\t%d\n",gstGyroOffset.s16X,gstGyroOffset.s16Y,gstGyroOffset.s16Z);
  return;
}


void ICM20948::imuDataGet(bool mag_ready)
                {
        int8_t i;
        float MotionVal[9];
        int16_t s16Gyro[3], s16Accel[3], s16Mag[3], s16Temp;
        int32_t s32Gyro[3], s32Accel[3], s32Mag[3];
        
        float pstAngles[3];
         
    
        if (mag_ready){
        u_int8_t u8Buf[burst_all]={0};
        I2C_ReadBurstByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H,u8Buf,burst_all,fd_address);

        s16Accel[0] = (u8Buf[0]<<8)|u8Buf[1];

        s16Accel[1] = (u8Buf[2]<<8)|u8Buf[3];


        s16Accel[2] = (u8Buf[4]<<8)|u8Buf[5];

        
        s16Gyro[0] = (u8Buf[6]<<8)|u8Buf[7];

        s16Gyro[1]= (u8Buf[8]<<8)|u8Buf[9];

        s16Gyro[2] = (u8Buf[10]<<8)|u8Buf[11];
        s16Temp = (u8Buf[12]<<8)|u8Buf[13];
        if(((u8Buf[20]<<4)!=0x80)){
        s16Mag[0] = (u8Buf[15]<<8) | u8Buf[14];
        s16Mag[1] = (u8Buf[17]<<8) | u8Buf[16];
        s16Mag[2] = (u8Buf[19]<<8) | u8Buf[18];
        }
        else{
            std::cout<<"fail"<<std::endl;
        }
        s16Mag[0] =  (s16Mag[0]-stMagoffsets.s16X)*stMagscales.s16X;
        s16Mag[1]= -(s16Mag[1]-stMagoffsets.s16Y)*stMagscales.s16Y;
        s16Mag[2] = -(s16Mag[2]-stMagoffsets.s16Z)*stMagscales.s16Z;
        stMagRawData.s16X=s16Mag[0];
        stMagRawData.s16Y=s16Mag[1];
        stMagRawData.s16Z=s16Mag[2];
        }
        else{
        u_int8_t u8Buf[burst_imu]={0};
        I2C_ReadBurstByte(I2C_ADD_ICM20948,REG_ADD_ACCEL_XOUT_H,u8Buf,burst_imu,fd_address);

        s16Accel[0] = (u8Buf[0]<<8)|u8Buf[1];

        s16Accel[1] = (u8Buf[2]<<8)|u8Buf[3];


        s16Accel[2] = (u8Buf[4]<<8)|u8Buf[5];

        
        s16Gyro[0] = (u8Buf[6]<<8)|u8Buf[7];

        s16Gyro[1]= (u8Buf[8]<<8)|u8Buf[9];

        s16Gyro[2] = (u8Buf[10]<<8)|u8Buf[11];
        }
        s16Accel[0] = (s16Accel[0]-stAcceloffsets.s16X);
        s16Accel[1] = (s16Accel[1]-stAcceloffsets.s16Y);
        s16Accel[2] = (s16Accel[2]-stAcceloffsets.s16Z);
        s32Accel[0] = stAccelmatrix.s1*s16Accel[0]+stAccelmatrix.s2*s16Accel[1]+stAccelmatrix.s3*s16Accel[2];
        s32Accel[1] = stAccelmatrix.s4*s16Accel[0]+stAccelmatrix.s5*s16Accel[1]+stAccelmatrix.s6*s16Accel[2];
        s32Accel[2] = stAccelmatrix.s7*s16Accel[0]+stAccelmatrix.s8*s16Accel[1]+stAccelmatrix.s9*s16Accel[2];
        s16Gyro[0] = s16Gyro[0];
        s16Gyro[1] = s16Gyro[1];
        s16Gyro[2] = s16Gyro[2];

       


        MotionVal[0]=s16Gyro[0]/GYRO_SCALE_FACTOR;
        MotionVal[1]=s16Gyro[1]/GYRO_SCALE_FACTOR;
        MotionVal[2]=s16Gyro[2]/GYRO_SCALE_FACTOR;
        MotionVal[3]=s32Accel[0]/ACCEL_SCALE_FACTOR;
        MotionVal[4]=s32Accel[1]/ACCEL_SCALE_FACTOR;
        MotionVal[5]=s32Accel[2]/ACCEL_SCALE_FACTOR;
        MotionVal[6]=stMagRawData.s16X;
        MotionVal[7]=stMagRawData.s16Y;
        MotionVal[8]=stMagRawData.s16Z;

        imuAHRSupdate((float)MotionVal[0] * 0.017453, (float)MotionVal[1] * 0.017453, (float)MotionVal[2] * 0.017453,
                  (float)MotionVal[3], (float)MotionVal[4], (float)MotionVal[5],
                  (float)MotionVal[6], (float)MotionVal[7], (float)MotionVal[8]);

        stQuaternion.q0=q0;
        stQuaternion.q1=q1;
        stQuaternion.q2=q2;
        stQuaternion.q3=q3;  

        // Eigen::Quaterniond q_(q0,q1,q2,q3);

      

        // euler_ang_=q_.toRotationMatrix().eulerAngles(1,0,2);
        // stAngles.fPitch = euler_ang_[0]* 180/M_PI; // pitch
        // stAngles.fRoll = euler_ang_[1] * 180/M_PI; // roll
        // stAngles.fYaw = euler_ang_[2] * 180/M_PI; 

        stGyroRawData.s16X = s16Gyro[0];
        stGyroRawData.s16Y = s16Gyro[1];
        stGyroRawData.s16Z = s16Gyro[2];

        stAccelRawData.s16X = s32Accel[0];
        stAccelRawData.s16Y = s32Accel[1];
        stAccelRawData.s16Z  = s32Accel[2];
        // long end = micro_time();
        // cout<<end-start<<endl;

}






  void ICM20948::imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
  {
    float norm;
    float hx, hy, hz, bx, bz;
    float vx, vy, vz, wx, wy, wz;
    float exInt = 0.0, eyInt = 0.0, ezInt = 0.0;
    float ex, ey, ez;

    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;


    norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    norm = invSqrt(mx * mx + my * my + mz * mz);
    mx = mx * norm;
    my = my * norm;
    mz = mz * norm;

    // compute reference direction of flux
    hx = 2 * mx * (0.5f - q2q2 - q3q3) + 2 * my * (q1q2 - q0q3) + 2 * mz * (q1q3 + q0q2);
    hy = 2 * mx * (q1q2 + q0q3) + 2 * my * (0.5f - q1q1 - q3q3) + 2 * mz * (q2q3 - q0q1);
    hz = 2 * mx * (q1q3 - q0q2) + 2 * my * (q2q3 + q0q1) + 2 * mz * (0.5f - q1q1 - q2q2);
    bx = sqrt((hx * hx) + (hy * hy));
    bz = hz;

    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    wx = 2 * bx * (0.5 - q2q2 - q3q3) + 2 * bz * (q1q3 - q0q2);
    wy = 2 * bx * (q1q2 - q0q3) + 2 * bz * (q0q1 + q2q3);
    wz = 2 * bx * (q0q2 + q1q3) + 2 * bz * (0.5 - q1q1 - q2q2);

    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) + (my * wz - mz * wy);
    ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
    ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

    if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
    {
      exInt = exInt + ex * Ki * halfT;
      eyInt = eyInt + ey * Ki * halfT;
      ezInt = ezInt + ez * Ki * halfT;

      gx = gx + Kp * ex + exInt;
      gy = gy + Kp * ey + eyInt;
      gz = gz + Kp * ez + ezInt;
    }

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;

    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

  }


void ICM20948::Close(void)
{
    i2cClose(fd_address);
    std::cout<<    "imu sensor close"<<std::endl;
}

