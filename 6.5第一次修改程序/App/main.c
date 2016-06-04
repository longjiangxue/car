
#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"




void main(void)
{ 
  spp_init();               //蓝牙中断初始化
  oled_init();              //OLED初始化
  led_all_init();           //LED初始化
  camera_int_init();        //摄像头及PORTA中断初始化
  motor_init();             //电机初始化
  servo_init();             //舵机初始化
  dip_switch_init();        //拨码开关初始化
  key_init_all();           //按键初始化
  beep_init();              //蜂鸣器初始化
  while(car_go);
  led_all_off();                      //LED全关
  car_stop_init(stop_second);                         //PIT0停车
  motor_control(200,200,speed_switch);              //参数是left和right
  midline[start_line+1] = 40;
  speed_control_init();
  while(1)
  {   
    camera_get_img();                                         //获取图像，未解压                           
    img_extract(img, imgbuff,CAMERA_SIZE);                    //解压图像
    if(img_number<700)
    {
      img_number++;
    }
  
    
    image_processing();                                       //图像处理
    servo_error();                                            //舵机偏差计算
    servo_control(steer);                                     //调整舵机，是否加延时
    if(flag == 254)
    {
      vcan_sendimg(img, CAMERA_W * CAMERA_H);                 //发送到上位机
    }
  
  }
  
}
