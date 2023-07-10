#include "IMU_i2c.h"
#include <iostream>
uint64_t micro_time(void);
#define GYRO_SCALE_FACTOR 131.0f;
#define ACCEL_SCALE_FACTOR 16384.0f;
class ICM20948
{
    public:
        ICM20948();
        void imuDataGet(IMU_QUATERNION_DATA &pstQuaternion, 
                IMU_ST_SENSOR_DATA &pstGyroRawData,
                IMU_ST_SENSOR_DATA &pstAccelRawData);
        void I2Cinit(int i2c_devnum);

        void Close(void);
        float time_interval;
        float ex_i;
        float ey_i;
        float ez_i;
        float Kp;
        float Ki;
        float q0,q1,q2,q3;
    private:
        
        state_check Check(void);
        void Init(void);
        void GyroRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z);
        void AccelRead(int16_t& ps16X, int16_t& ps16Y, int16_t& ps16Z);
        void GyroOffset(void);

        void mahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);



        IMU_ST_SENSOR_DATA gstGyroOffset;
        int fd;
        int *fd_address;

};