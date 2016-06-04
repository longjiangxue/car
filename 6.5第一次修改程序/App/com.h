#ifndef __COM_H__
#define __COM_H__

#include "common.h"
#include "include.h"
#include "math.h"

//���ں�������������extern���궨��


void PORTA_IRQHandler();
void PORTE_IRQHandler(void);
void DMA0_IRQHandler();
void camera_int_init();
void int_char_uart(int num,char str[]);
void float_char_uart(char *str, double number, uint8_t g, uint8_t l);
void motor_init(void);      //�����ʼ��
void motor_control(int16 left,int16 right, uint8 flag);   //����ٶȿ���
void servo_init(void);      //�����ʼ��
void servo_control(uint16 steer);       //�������
void center_line(void);     //������ȡ
void servo_error(void);     //���ƫ�����
void oled_init(void);       //OLED�ĳ�ʼ������
void led_all_init(void);    //��ʼ��4��LED��
void car_stop_init(float second);  //ͣ����ʼ��
void PIT0_IRQHandler(void);      //PIT0ִ�к���
void PIT1_IRQHandler(void);      //PIT1ִ�к���
void key_init_all(void);              //������ʼ������������PORTE�ж�
void beep_init(void);         //��������ʼ��
void beer_on(void);           //��������
void beer_off(void);          //��������
void dip_switch_init(void);   //���뿪�س�ʼ��
void line_lost(uint8 line);         //���ߴ���
void led_all_on(void);        //LEDȫ��
void led_all_off(void);       //LEDȫ��
void image_processing(void);  //ͼ����
void uart4_handler(void);     //�����жϴ�����
void spp_init(void);        //�����жϳ�ʼ��
void bt_send(void);         //�������ͺ���
void speed_control_init(void);     //�������ٶȿ���



#define MOTOR_FTM   FTM0
#define MOTOR1_PWM  FTM_CH3
#define MOTOR2_PWM  FTM_CH4
#define MOTOR3_PWM  FTM_CH5
#define MOTOR4_PWM  FTM_CH6
#define MOTOR_HZ    20*1000

#define S3010_FTM   FTM1
#define S3010_CH    FTM_CH0
#define S3010_HZ    (100)
#define DJ_ZZ       (102)

#define DIP_SWITCH_ON   (0)     //���뿪��ON
#define DIP_SWITCH_OFF  (1)     //���뿪��OFF
  

extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W]; 
extern int16 left_line[CAMERA_H];        //��߽�
extern int16 right_line[CAMERA_H];       //�ұ߽�
extern int16 midline[CAMERA_H];      //����
extern int16 error_duoji[CAMERA_H];  //���ƫ��

extern int16 midpoint;               //��ʼ�е��е�
extern int16 last_right;             //��һ�ε��ҵ�
extern int16 last_left;             //��һ�ε����
extern int8 start_line;             //ͼ����ʼ��
extern int8 end_line;               //ͼ���������
extern int16 error_duoji_final;     //���ն��ƫ��
extern int16 error_error_duoji;      //ƫ���ƫ��
extern int16 error_duoji_final_last;  //��һ�������ƫ��
extern uint16 steer;                //�������
extern float kp;                    //���ƫ��pֵ
extern float kd;                   //���ƫ��dֵ
extern uint8 left_lost;                  //��߽綪ʧ����
extern uint8 right_lost;                  //�ұ߽綪ʧ����
extern uint8 dip_switch_1;               //���뿪��1
extern uint8 dip_switch_2;               //���뿪��2
extern uint8 dip_switch_3;               //���뿪��3
extern uint8 dip_switch_4;               //���뿪��4
extern uint8 car_go;                     //�ó��ӿ�ʼ����
extern float stop_second;              //������֮��ͣ��
extern uint16 left_speed;            //����ռ�ձ�
extern uint16 right_speed;           //����ռ�ձ�
extern uint8 car_run;                //��ͣ������
extern uint8 flag;
extern uint8 speed_switch;          //ռ�ձȿ��ر���
extern uint16 n;                           //ͣ��ʱ��
extern uint16 val;                   //����������
extern int16 error;
extern int16 d_error; 
extern int16 dd_error;
extern int16 pre_error;
extern int16 pre_pre_error;
extern uint16 speed_ideal;
extern int16 pwm;
extern int16 pwm_last;
extern float PID_P;
extern float PID_I;
extern float PID_D;
extern int16 var[6];

extern uint8 gradient[80];//����׵��ݶ�����

extern uint8 max_number;  //�������ݶ����ӵ���
extern uint8 small_number;  //�������ݶȼ��ٵ���

extern uint16 img_number;  //�ɼ���ͼ�����
extern uint8 car_start_left;
extern uint8 car_start_right;

extern uint16 sudu_biaozhi;    //���ٱ�־
extern uint16 ideal[32];

#endif  