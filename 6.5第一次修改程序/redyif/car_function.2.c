#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"



/*********************************************************************************
* �� �� ��          : image_processing
* ��������          : ͼ����
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void image_processing(void)
{
  center_line();            //������ȡ
}

/*********************************************************************************
* �� �� ��          : center_line
* ��������          : ͼ��������ȡ
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void center_line(void)
{
  uint8 line, column;
  uint8 ji_wan=0; //���弱��
  

 //ͣ����־�߼��/////////////////////////// 
   car_start_left=0;
   car_start_right=0;
  for(line=59;line>40;line--)
  {
    if(img_number >= 650 && 0 == img[line][26] && 255 == img[line-5][26] )
    {
         car_start_left=1;
    }
  }
  for(line=59;line>40;line--)
  {
    if(img_number >= 650 && 0 == img[line][54] && 255 == img[line-5][54] )
    {
         car_start_right=1;
    }
  }  
//������///////////////////////////   
    
  
 
  
  max_number = 0;
  small_number = 0;
  
  gradient[0] = 0;
    for(line=59;line>5;line--)
    {
      if(255 == img[line][0])
      {
        gradient[0]++;
      }
       else
      {
        break;
      }
    }//ͳ�Ƶ�0�а׵���
   
  for(column=1;column<=79;column++)//ͳ���ݶ���
  {
   gradient[column] = 0;
   for(line=59;line>5;line--)
    {
      if(255 == img[line][column])
      {
        gradient[column]++;
      }
       else
      {
        break;
      }
    }//ͳ�Ƶ�ǰ�׵���
   
   if(gradient[column] - gradient[column-1] >= 0)
   {
      max_number++;
   }
   if(gradient[column] - gradient[column-1] <= 0)
   {
      small_number++;
   }
  }
  if(max_number - small_number > 10)//��ת��
  {
    ji_wan=0;
    for(column=79;column>=65;column--)
    {
      
      if(gradient[column] - gradient[column-1]>=0)
      {
        ji_wan++; 
      }
    }
      if(ji_wan > 14 && gradient[40] < 30 )
      {
        led(LED2, LED_ON);
        midline[start_line+1] = 74;
        end_line = 59-gradient[78]+3;               //ͼ��������� 
      }
      else
      {
        led(LED2, LED_OFF);
        midline[start_line+1] = 40;
        end_line = 29;               //ͼ��������� 
      }
    
  }

  if(small_number - max_number > 10)//��ת��
  {
     ji_wan=0;
     for(column=1;column<=15;column++)
    {
     
      if(gradient[column] - gradient[column-1]<=0)
      {
        ji_wan++; 
      }
    }  
      if(ji_wan > 14 && gradient[40] < 30)
      {
        led(LED3, LED_ON);
        midline[start_line+1] = 5;
        end_line = 59-gradient[1]+3;               //ͼ��������� 
      }
     else
     {
       led(LED3, LED_OFF);
       midline[start_line+1] = 40;
       end_line = 29;               //ͼ��������� 
     }
  }















  

  
  
   for(line=start_line; line>end_line; line--)
   {
     if(midline[line+1] >= 78)
     {
       midline[line+1] = 70;
     }
     
     if(midline[line+1] <= 1)
     {
       midline[line+1] = 10;
     }
      for(column=midline[line+1]; column<77; column++)       //������
      {
        if(255 == img[line][column] - img[line][column+1])
       {
          right_line[line] = column;
          last_right = column;
          break;
       }
        else
       {
         right_line[line] = 1000;        
       }
     }//������ѭ��
    
     for(column=midline[line+1];column>2;column--)       //������
     {
        if(255 == img[line][column] - img[line][column-1])
        {
          left_line[line] = column;
          last_left = column;
          break; 
        }
       else
       {
          left_line[line] = -1000;        
       }     
      }//������ѭ��
    
     if((-1000 == left_line[line]) || (1000 == right_line[line]))
      {
        line_lost(line);
      }
      else
      {
        
      }
      midline[line] = (right_line[line]+left_line[line]+1)/2;
      img[line][midline[line]] = 1;  
   }//��forѭ��
    last_left = -2;
    last_right = 81;
}
   










/*********************************************************************************
* �� �� ��          : line_lost
* ��������          : ������ֶ�������
* ��    ��          : line
* ��    ��          : ��
*********************************************************************************/
void line_lost(uint8 line)
{ 
  //��ʱ�Ѿ��жϵ�����ĳ���߶���
  if((-1000 == left_line[line]) && (1000 != right_line[line]))     //��߶����ұ߲�����
  {

    left_line[line] = last_left + (right_line[line] - last_right);
    last_left = left_line[line];
    left_lost ++;
  }
  
  if((-1000 != left_line[line]) && (1000 == right_line[line]))     //��߲������ұ߶���
  {
   
    right_line[line] = last_right + (left_line[line] - last_left);
    last_right = right_line[line];
    right_lost ++;
    midline[line] =  (int16)(left_line[line] + 40+(59-line)*0.5);
  }
  
  if((-1000 == left_line[line]) && (1000 == right_line[line]))     //���Ҷ�����
  {
    left_line[line] = last_left;
    right_line[line] = last_right;
    left_lost ++;
    right_lost ++;
    midline[line] = 40;
  }
}




/*********************************************************************************
* �� �� ��          : servo_error
* ��������          : ���ƫ�����
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void servo_error(void)
{
  uint8 i;
  int16 error_duoji_add = 0;
  for(i=start_line-14;i>end_line;i--)
  {
    error_duoji[i] = 40 - midline[i];       //ÿ��ƫ�����
    error_duoji_add = error_duoji[i] + error_duoji_add;   //���ƫ���ܺ�
  }

  error_duoji_final = (error_duoji_add ) / (start_line-14 - end_line);  //����ƽ��ƫ�� 
  error_error_duoji= error_duoji_final - error_duoji_final_last;  //����ƫ���ƫ��
  error_duoji_final_last = error_duoji_final;                     //������һ�����ƫ��

  
  steer= (uint16)(DJ_ZZ + kp * error_duoji_final + kd * error_error_duoji);
  if(steer >= 134)
  {
    steer = 134;
  }
  if(steer <= 73)
  {
    steer = 73;
  }
  speed_ideal = ideal[ABS(steer - 103)];
  //speed_ideal = 300;
  
  if(car_start_left == 1 && car_start_right == 1 && 
     (gradient[32]>45||gradient[35]>45||gradient[38]>45||gradient[42]>45||gradient[45]>45||gradient[48]>45))
  {
    motor_control(0,0,speed_switch);
    speed_switch = 0;    
  }  
  
  
  /*
   if(ABS(steer - 103) > 10 )
  {
    speed_ideal = 280;
    led(LED0, LED_OFF);
  }  
   else 
  {
    led(LED0, LED_OFF);
    speed_ideal = 400;
  }   
  */
 /* 
  if(car_start_left == 1 && car_start_right == 1 && 
     (gradient[32]>45||gradient[35]>45||gradient[38]>45||gradient[42]>45||gradient[45]>45||gradient[48]>45))
  {
    motor_control(0,0,speed_switch);
    speed_switch = 0;    
  }

  else if(ABS(steer - 103) < 10 )
  {
    sudu_biaozhi++;
    if(sudu_biaozhi <= 10)
    {
      motor_control(left_speed+90,right_speed+90,speed_switch);
    }
    else if(sudu_biaozhi >= 36)
    {
      motor_control(left_speed-60,right_speed-60,speed_switch);
    }
    led(LED0, LED_OFF);
  }

  else if(ABS(steer - 103) > 16 )
  {
      motor_control(left_speed-70,right_speed-70,speed_switch);
      sudu_biaozhi=10;
  }
 
  else 
  {
    led(LED0, LED_OFF);
    motor_control(left_speed,right_speed,speed_switch);
    sudu_biaozhi=10;
  }  
*/
  
 //var[0] = error_duoji_final;
  // vcan_sendware(var, sizeof(var));




}

