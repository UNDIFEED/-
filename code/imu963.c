#include "zf_common_headfile.h"
#include "fliter.h"

//上电消除零漂
int8 zero_drift_min = 0; //零漂最小值
int8 zero_drift_max = 0; //零漂最大值
int8 zero_drift_average = 0; //零漂平均值
void imu_zero_drift(void)
{
    uint8 i;
    float temp = 0;
    system_delay_ms(500); //防止手抖,你发车时开机了就先别动他！！！！

    zero_drift_max = -50;
    zero_drift_min = 50;
    for (i = 0; i < 50; i++)
    {
        imu963ra_get_gyro();
        temp += imu963ra_gyro_z;
        if (imu963ra_gyro_z > zero_drift_max) zero_drift_max = imu963ra_gyro_z;
        if (imu963ra_gyro_z < zero_drift_min) zero_drift_min = imu963ra_gyro_z;
        system_delay_ms(5);
    }
    zero_drift_average = (int8)((temp / 200) + 0.5);
}

//获取陀螺仪数据
int16 imu_gyro_z = 0; //z轴角速度
float angle = 0.0; //车角度,向左增大，向右减小
void imu_get_gyro(void)
{
    imu963ra_get_gyro();
    // imu_gyro_z = Average_Filter(imu963ra_gyro_z);
    imu_gyro_z = imu963ra_gyro_z;

    //在零漂范围内直接强制给0
    //不在零漂范围正常给值，这里减去了零漂的平均值
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
    // angle += (float)imu_gyro_z * 4.186E-4;
}

/*
函数名称：void integeal_angle(float dt)
功能说明：对角度积分
参数说明：
dt  ：中断周期,单位为ms
函数返回：无
修改时间：2022年9月25日
备    注：
example：void integeal_angle(6);
 */
uint8 AngleGet_Flag = 0; //开始获取角度的标志位
void integeal_angle(float dt)
{
    angle += (float)imu_gyro_z * 7.025E-5 * dt;
}