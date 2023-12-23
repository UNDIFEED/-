#include "zf_common_headfile.h"
#include "motor.h"
#include "encoder.h"

//编码器初始化
void Encoder_Init(void)
{
    encoder_dir_init(QTIMER1_ENCODER1, QTIMER1_ENCODER1_CH1_C0, QTIMER1_ENCODER1_CH2_C1); // 左后
    encoder_dir_init(QTIMER1_ENCODER2, QTIMER1_ENCODER2_CH1_C2, QTIMER1_ENCODER2_CH2_C24); // 左前
    encoder_dir_init(QTIMER2_ENCODER1, QTIMER2_ENCODER1_CH1_C3, QTIMER2_ENCODER1_CH2_C25); // 右前
    encoder_dir_init(QTIMER3_ENCODER2, QTIMER3_ENCODER2_CH1_B18, QTIMER3_ENCODER2_CH2_B19); // 右后
}

void Encoder_Get(void)
{
    motor_zuoqian.encoder = -encoder_get_count(QTIMER1_ENCODER2);
    motor_youqian.encoder = encoder_get_count(QTIMER2_ENCODER1);
    motor_zuohou.encoder = -encoder_get_count(QTIMER1_ENCODER1);
    motor_youhou.encoder = encoder_get_count(QTIMER3_ENCODER2);

    //10ms内获得的脉冲数 * 每个脉冲对应距离 / 10ms = 当前实际速度
    //当前实际速度 * 转换系数 = 速度对应cm/s
    motor_zuoqian.speed = -(float)encoder_get_count(QTIMER1_ENCODER2) * IMPULSE_DIS * 100 * M_TRANSFER_CM;
    motor_youqian.speed = (float)encoder_get_count(QTIMER2_ENCODER1) * IMPULSE_DIS * 100 * M_TRANSFER_CM;
    motor_zuohou.speed = -(float)encoder_get_count(QTIMER1_ENCODER1) * IMPULSE_DIS * 100 * M_TRANSFER_CM;
    motor_youhou.speed = (float)encoder_get_count(QTIMER3_ENCODER2) * IMPULSE_DIS * 100 * M_TRANSFER_CM;

    encoder_clear_count(QTIMER1_ENCODER2);
    encoder_clear_count(QTIMER2_ENCODER1);
    encoder_clear_count(QTIMER1_ENCODER1);
    encoder_clear_count(QTIMER3_ENCODER2);
}
