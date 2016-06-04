#ifndef __COM_H__
#define __COM_H__

#include "common.h"
#include "include.h"
#include "math.h"

//用于函数声明，变量extern，宏定义


void PORTA_IRQHandler();
void PORTE_IRQHandler(void);
void DMA0_IRQHandler();
void camera_int_init();
void int_char_uart(int num,char str[]);
void float_char_uart(char *str, double number, uint8_t g, uint8_t l);
void motor_init(void);      //电机初始化
void motor_control(int16 left,int16 right, uint8 flag);   //电机速度控制
void servo_init(void);      //舵机初始化
void servo_control(uint16 steer);       //舵机控制
void center_line(void);     //中线提取
void servo_error(void);     //舵机偏差计算
void oled_init(void);       //OLED的初始化函数
void led_all_init(void);    //初始化4个LED灯
void car_stop_init(float second);  //停车初始化
void PIT0_IRQHandler(void);      //PIT0执行函数
void PIT1_IRQHandler(void);      //PIT1执行函数
void key_init_all(void);              //按键初始化函数，开启PORTE中断
void beep_init(void);         //蜂鸣器初始化
void beer_on(void);           //蜂鸣器开
void beer_off(void);          //蜂鸣器关
void dip_switch_init(void);   //拨码开关初始化
void line_lost(uint8 line);         //丢线处理
void led_all_on(void);        //LED全开
void led_all_off(void);       //LED全关
void image_processing(void);  //图像处理
void uart4_handler(void);     //蓝牙中断处理函数
void spp_init(void);        //蓝牙中断初始化
void bt_send(void);         //蓝牙发送函数
void speed_control_init(void);     //编码器速度控制



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

#define DIP_SWITCH_ON   (0)     //拨码开关ON
#define DIP_SWITCH_OFF  (1)     //拨码开关OFF
  

extern uint8 imgbuff[CAMERA_SIZE];
extern uint8 img[CAMERA_H][CAMERA_W]; 
extern int16 left_line[CAMERA_H];        //左边界
extern int16 right_line[CAMERA_H];       //右边界
extern int16 midline[CAMERA_H];      //中线
extern int16 error_duoji[CAMERA_H];  //舵机偏差

extern int16 midpoint;               //起始行的中点
extern int16 last_right;             //上一次的右点
extern int16 last_left;             //上一次的左点
extern int8 start_line;             //图像处理开始行
extern int8 end_line;               //图像处理结束行
extern int16 error_duoji_final;     //最终舵机偏差
extern int16 error_error_duoji;      //偏差的偏差
extern int16 error_duoji_final_last;  //上一场舵机的偏差
extern uint16 steer;                //舵机参数
extern float kp;                    //舵机偏差p值
extern float kd;                   //舵机偏差d值
extern uint8 left_lost;                  //左边界丢失个数
extern uint8 right_lost;                  //右边界丢失个数
extern uint8 dip_switch_1;               //拨码开关1
extern uint8 dip_switch_2;               //拨码开关2
extern uint8 dip_switch_3;               //拨码开关3
extern uint8 dip_switch_4;               //拨码开关4
extern uint8 car_go;                     //让车子开始运行
extern float stop_second;              //多少秒之后停车
extern uint16 left_speed;            //左轮占空比
extern uint16 right_speed;           //右轮占空比
extern uint8 car_run;                //不停车参数
extern uint8 flag;
extern uint8 speed_switch;          //占空比开关变量
extern uint16 n;                           //停车时间
extern uint16 val;                   //编码器脉冲
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

extern uint8 gradient[80];//定义白点梯度数组

extern uint8 max_number;  //从左到右梯度增加的数
extern uint8 small_number;  //从左到右梯度减少的数

extern uint16 img_number;  //采集的图像幅数
extern uint8 car_start_left;
extern uint8 car_start_right;

extern uint16 sudu_biaozhi;    //控速标志
extern uint16 ideal[32];

#endif  