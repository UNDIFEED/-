#include "zf_common_headfile.h"
#include "fliter.h"

int16 imu_gyro_z = 0; //z轴角速度
//上电消除零漂
int8 zero_drift_min = 0; //零漂最小值
int8 zero_drift_max = 0; //零漂最大值
int8 zero_drift_average = 0; //零漂平均值
void imu_zero_drift(void)
{
    uint8 i;
    int temp = 0;
    system_delay_ms(500); //防止手抖,你发车时开机了就先别动他！！！！

    imu963ra_get_gyro();
    zero_drift_max = zero_drift_min = imu963ra_gyro_z;
    for (i = 0; i < 200; i++)
    {
        imu963ra_get_gyro();
        temp += imu963ra_gyro_z;
        if (imu963ra_gyro_z >= zero_drift_max) zero_drift_max = imu963ra_gyro_z;
        if (imu963ra_gyro_z <= zero_drift_min) zero_drift_min = imu963ra_gyro_z;
    }
    zero_drift_average = (int8)((temp / 200) + 0.5);
}

//获取陀螺仪数据
float angle = 0.0; //车角度
void imu_get_gyro(void)
{
    imu963ra_get_gyro();
    imu_gyro_z = Average_Filter(imu963ra_gyro_z);

    //在零漂范围内直接强制给0
    if (zero_drift_max > 0 && zero_drift_min > 0)
    {
        if (imu_gyro_z <= zero_drift_max && imu_gyro_z > 0) imu_gyro_z = 0;
        else imu_gyro_z = imu_gyro_z - zero_drift_average;
    }
    else if (zero_drift_max < 0 && zero_drift_min < 0)
    {
        if (imu_gyro_z >= zero_drift_min && imu_gyro_z < 0) imu_gyro_z = 0;
        else imu_gyro_z = imu_gyro_z - zero_drift_average;
    }
    else if (zero_drift_max > 0 && zero_drift_min < 0)
    {
        if (imu_gyro_z >= zero_drift_min && imu_gyro_z <= zero_drift_max) imu_gyro_z = 0;
        else imu_gyro_z = imu_gyro_z - zero_drift_average;
    }
    //不在零漂范围正常给值，这里减去了零漂的平均值
    // angle += (float)imu_gyro_z / 400226; //400226 一圈360
}