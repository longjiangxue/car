#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"

void PORTE_IRQHandler(void)
{
  DELAY_MS(10);
  if(PORTE_ISFR & (1 << 0))           //PTE0 �����ж� ���°���1��ִ�еĺ���
  {
    PORTE_ISFR  = (1 << 0);
    PID_P = PID_P+0.1;
    /*
    left_speed = left_speed + 10;
    right_speed = right_speed + 10;
    if(0 == car_go)
    {
      motor_control(left_speed,right_speed,speed_switch);
    }
    
    LCD_P6x8Str(36,0,"  ");       //����ԭ�ȵ���
    LCD_P6x8Str(52,0,"  ");       //����ԭ�ȵ���
    LCD_P6x8number(36,0,left_speed);
    LCD_P6x8number(52,0,right_speed);
    bt_send();
    */
  }
  
  if(PORTE_ISFR & (1 << 1))           //PTE1 �����ж� ���°���2��ִ�еĺ���
  {
    PORTE_ISFR  = (1 << 1); 
    PID_P = PID_P-0.1;
    /*
    left_speed = left_speed - 10;
    right_speed = right_speed - 10;
    LCD_P6x8Str(36,0,"  ");       //����ԭ�ȵ���
    LCD_P6x8Str(52,0,"  ");       //����ԭ�ȵ���
    if(0 == car_go)
    {
      motor_control(left_speed,right_speed,speed_switch);
    }
    LCD_P6x8number(36,0,left_speed);
    LCD_P6x8number(52,0,right_speed);
    bt_send();
    */
  }
  if(PORTE_ISFR & (1 << 2))           //PTE2 �����ж� ���°���3��ִ�еĺ���
  {
    PORTE_ISFR  = (1 << 2);
    /*
    stop_second = stop_second + 0.5;
    n = (uint16)(stop_second * 1000); 
    LCD_P6x8number(24,6,n);       //��ʾͣ��ʱ��
    bt_send();
    */
  }
  if(PORTE_ISFR & (1 << 3))           //PTE3 �����ж� ���°���4��ִ�еĺ���
  {
    PORTE_ISFR  = (1 << 3);
    /*
    stop_second = stop_second - 0.5;
    n = (uint16)(stop_second * 1000);
    LCD_P6x8Str(24,6,"     ");  //����
    LCD_P6x8number(24,6,n);       //��ʾͣ��ʱ��
    bt_send();
    */
  }
  if(PORTE_ISFR & (1 << 4))           //PTE4 �����ж� ���°���5��ִ�еĺ���
  {
    PORTE_ISFR  = (1 << 4);
    flag = ~flag;
  }
  if(PORTE_ISFR & (1 << 5))           //PTE5 �����ж� ���°���6��ִ�еĺ���
  {
    PORTE_ISFR  = (1 << 5);
    car_go = 0;
  }
  while(!GPIO_GET(PTE0) && !GPIO_GET(PTE1) && !GPIO_GET(PTE2) && !GPIO_GET(PTE3) && !GPIO_GET(PTE4) && !GPIO_GET(PTE5));
  
}