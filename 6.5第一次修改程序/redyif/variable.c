#include "common.h"
#include "include.h"
#include "com.h"

//用于定义全局变量，因为在.h文件中不能定义变量


uint8 imgbuff[CAMERA_SIZE];                             
uint8 img[CAMERA_H][CAMERA_W]; 




int16 left_line[CAMERA_H];        //左边界
int16 right_line[CAMERA_H];       //右边界
int16 midline[CAMERA_H];       //中线
int16 error_duoji[CAMERA_H];   //舵机偏差

int16 midpoint = 40;               //起始行的中点
int16 last_right = 81;             //上一次的右点
int16 last_left = -2;             //上一次的左点
int8 start_line = 50;              //图像处理开始行
int8 end_line = 29;               //图像处理结束行
int16 error_duoji_final;          //最终舵机偏差
int16 error_error_duoji;          //偏差的偏差
int16 error_duoji_final_last;      //上一场舵机的偏差
uint16 steer;                     //舵机参数
float kp = 1.5;                    //舵机偏差p值
float kd = 0.6;                   //舵机偏差d值
uint8 left_lost;                  //左边界丢失个数
uint8 right_lost;                  //右边界丢失个数
uint8 dip_switch_1;               //拨码开关1
uint8 dip_switch_2;               //拨码开关2
uint8 dip_switch_3;               //拨码开关3
uint8 dip_switch_4;               //拨码开关4
uint8 car_go = 1;                 //让车子开始运行
float stop_second = 10.0;          //多少秒之后停车
uint16 left_speed = 360;            //左轮占空比
uint16 right_speed = 360;           //右轮占空比
uint8 car_run = 0;                //不停车参数
uint8 flag = 1;
uint8 speed_switch = 1;           //占空比开关变量
uint16 n = 1000;                           //停车时间
uint16 val = 0;                   //编码器脉冲
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

uint8 gradient[80];//定义白点梯度数组
uint8 max_number;  //从左到右梯度增加的数
uint8 small_number;  //从左到右梯度减少的数

uint8 car_start_left=0;
uint8 car_start_right=0;

uint16 img_number=0;  //采集的图像幅数
uint16 sudu_biaozhi=0;  //控速标志

uint16 ideal[32] = {  450,450,450,450,450,450,
                      450,450,420,420,360,360,
                      360,360,350,350,350,350,
                      350,350,350,350,340,340,
                      340,340,340,340,340,330,
                      330,330};
