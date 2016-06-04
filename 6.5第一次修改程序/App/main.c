
#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"




void main(void)
{ 
  spp_init();               //�����жϳ�ʼ��
  oled_init();              //OLED��ʼ��
  led_all_init();           //LED��ʼ��
  camera_int_init();        //����ͷ��PORTA�жϳ�ʼ��
  motor_init();             //�����ʼ��
  servo_init();             //�����ʼ��
  dip_switch_init();        //���뿪�س�ʼ��
  key_init_all();           //������ʼ��
  beep_init();              //��������ʼ��
  while(car_go);
  led_all_off();                      //LEDȫ��
  car_stop_init(stop_second);                         //PIT0ͣ��
  motor_control(200,200,speed_switch);              //������left��right
  midline[start_line+1] = 40;
  speed_control_init();
  while(1)
  {   
    camera_get_img();                                         //��ȡͼ��δ��ѹ                           
    img_extract(img, imgbuff,CAMERA_SIZE);                    //��ѹͼ��
    if(img_number<700)
    {
      img_number++;
    }
  
    
    image_processing();                                       //ͼ����
    servo_error();                                            //���ƫ�����
    servo_control(steer);                                     //����������Ƿ����ʱ
    if(flag == 254)
    {
      vcan_sendimg(img, CAMERA_W * CAMERA_H);                 //���͵���λ��
    }
  
  }
  
}
