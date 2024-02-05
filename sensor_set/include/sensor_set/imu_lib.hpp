#include "IMU_i2c.h"
#include <iostream>
#include <thread>

// Eigen
#include <Eigen/Dense>

uint64_t micro_time(void);
// #define ACCEL_SCALE_FACTOR 8192.0f;
#define G_CONSTANT 9.806;
#define RAD_CONSTANT 0.01745329;
using namespace std;
class ICM20948
{
    public:
        ICM20948();
        void imuDataGet(bool mag_ready);
        void I2Cinit(int i2c_devnum,uint8_t Gscale, uint8_t Ascale);
        void GyroOffsetConfig(void);
        void MagRead(int16_t& s16X, int16_t& s16Y, int16_t& s16Z);
        void Close(void);
        
        int32_t CalAvgValue( int16_t *AvgBuffer);
        float halfT;
        float ex_i;
        float ey_i;
        float ez_i;
        float Kp;
        float Ki;
        float q0,q1,q2,q3;
        float GYRO_SCALE_FACTOR;
        float ACCEL_SCALE_FACTOR;
        int *fd_address;
        IMU_QUATERNION_DATA stQuaternion;
        IMU_ST_SENSOR_DATA stGyroRawData;
        IMU_ST_SENSOR_DATA stAccelRawData;
        IMU_ST_SENSOR_DATA stMagRawData;
        IMU_ST_ANGLES_DATA stAngles;
        IMU_ST_SENSOR_DATA gstGyroOffset;
        IMU_ST_SCALE_DATA stMagscales;
        IMU_ST_SENSOR_DATA stMagoffsets;
        IMU_ST_MATRIX_DATA stAccelmatrix;
        IMU_ST_SENSOR_DATA stAcceloffsets;
        ST_AVG_DATA accel_stAvgBuf[3];
        ST_AVG_DATA gyro_stAvgBuf[3];
        // ST_AVG_DATA mag_stAvgBuf[3];
        
    private:
        
        state_check Check(void);
        void Init(uint8_t Gscale,uint8_t Ascale);
        void GyroRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z);
        void AccelRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z);
        void GyroOffset(void);



        void imuAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
        void ReadSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8Len, uint8_t *pu8data);
        void WriteSecondary(uint8_t u8I2CAddr, uint8_t u8RegAddr, uint8_t u8data);

        bool MagCheck(void) ;
        
        uint8_t burst_part = 6;
        uint8_t burst_imu = 12;
        uint8_t burst_all = 21;
        int fd;

        Eigen::Vector3d euler_ang_;





};