#ifndef _LADRC_H
#define _LADRC_H
/**
   *@Brief  ����ΪLADRCϵͳ����
   *@WangShun  2022-07-03  ע��
   */
typedef struct LADRC
{
    float v1,v2;         //�������ֵ
    float r;             //�ٶ�����
    float h;             //���ֲ���
    float z1,z2,z3;      //�۲������
    float w0,wc,b0,u;    //�۲������� ���������� ϵͳ���� ���������
}LADRC_NUM;

/*
	wu = 2*3.1415/Pu;
    ku = 4*h/3.1415*a;

	wc = 2.3997wu - 0.4731;
	w0 = 0.7332wu + 3.5070;
	b0 = 3.6105wu + 4.8823;
*/
typedef struct Auto_Tuning 
{
	float Pu; //�̵�ʵ���������
	float a;  //�̵�ʵ�������ֵ
	float h;  //ָ�������ֵ
	float Wu; //ϵͳ�ٽ�Ƶ��
	float Kp; //ϵͳ�ٽ��ֵ
}AuTu;

/**
   *@Brief  ����Ϊ��Ҫ�����Ĳ���
   *@WangShun  2022-07-03  ע��
   */
extern LADRC_NUM MOTOR_LF;
extern LADRC_NUM MOTOR_RF;
extern LADRC_NUM MOTOR_LB;
extern LADRC_NUM MOTOR_RB;
/**
   *@Brief  ����ΪLADRC��غ���
   *@WangShun  2022-07-03  ע��
   */
void LADRC_Init(LADRC_NUM *LADRC_TYPE1);
void LADRC_Init_All(void);
void LADRC_REST(LADRC_NUM *LADRC_TYPE1);
void LADRC_TD(LADRC_NUM *LADRC_TYPE1,float Expect);
void LADRC_ESO(LADRC_NUM *LADRC_TYPE1,float FeedBack);
void LADRC_LF(LADRC_NUM *LADRC_TYPE1);
void LADRC_Loop(LADRC_NUM *LADRC_TYPE1,float Expect,float RealTimeOut);
#endif
