#include "common.h"
#include "include.h"
#include "com.h"

//���ڶ���ȫ�ֱ�������Ϊ��.h�ļ��в��ܶ������


uint8 imgbuff[CAMERA_SIZE];                             
uint8 img[CAMERA_H][CAMERA_W]; 




int16 left_line[CAMERA_H];        //��߽�
int16 right_line[CAMERA_H];       //�ұ߽�
int16 midline[CAMERA_H];       //����
int16 error_duoji[CAMERA_H];   //���ƫ��

int16 midpoint = 40;               //��ʼ�е��е�
int16 last_right = 81;             //��һ�ε��ҵ�
int16 last_left = -2;             //��һ�ε����
int8 start_line = 50;              //ͼ����ʼ��
int8 end_line = 29;               //ͼ���������
int16 error_duoji_final;          //���ն��ƫ��
int16 error_error_duoji;          //ƫ���ƫ��
int16 error_duoji_final_last;      //��һ�������ƫ��
uint16 steer;                     //�������
float kp = 1.5;                    //���ƫ��pֵ
float kd = 0.6;                   //���ƫ��dֵ
uint8 left_lost;                  //��߽綪ʧ����
uint8 right_lost;                  //�ұ߽綪ʧ����
uint8 dip_switch_1;               //���뿪��1
uint8 dip_switch_2;               //���뿪��2
uint8 dip_switch_3;               //���뿪��3
uint8 dip_switch_4;               //���뿪��4
uint8 car_go = 1;                 //�ó��ӿ�ʼ����
float stop_second = 10.0;          //������֮��ͣ��
uint16 left_speed = 360;            //����ռ�ձ�
uint16 right_speed = 360;           //����ռ�ձ�
uint8 car_run = 0;                //��ͣ������
uint8 flag = 1;
uint8 speed_switch = 1;           //ռ�ձȿ��ر���
uint16 n = 1000;                           //ͣ��ʱ��
uint16 val = 0;                   //����������
int16 error = 0;
int16 d_error = 0; 
int16 dd_error = 0;
int16 pre_error = 0;
int16 pre_pre_error = 0;
uint16 speed_ideal = 350;
int16 pwm = 300;
int16 pwm_last = 300;
float PID_P = 4.0;
float PID_I = 2.0;
float PID_D = 0.00;
int16 var[6];

uint8 gradient[80];//����׵��ݶ�����
uint8 max_number;  //�������ݶ����ӵ���
uint8 small_number;  //�������ݶȼ��ٵ���

uint8 car_start_left=0;
uint8 car_start_right=0;

uint16 img_number=0;  //�ɼ���ͼ�����
uint16 sudu_biaozhi=0;  //���ٱ�־

uint16 ideal[32] = {  450,450,450,450,450,450,
                      450,450,420,420,360,360,
                      360,360,350,350,350,350,
                      350,350,350,350,340,340,
                      340,340,340,340,340,330,
                      330,330};
