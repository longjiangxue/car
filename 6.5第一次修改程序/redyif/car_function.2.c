#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"



/*********************************************************************************
* 函 数 名          : image_processing
* 函数功能          : 图像处理
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void image_processing(void)
{
  center_line();            //中线提取
}

/*********************************************************************************
* 函 数 名          : center_line
* 函数功能          : 图像中线提取
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void center_line(void)
{
  uint8 line, column;
  uint8 ji_wan=0; //定义急弯
  

 //停车标志线检测/////////////////////////// 
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
//检测结束///////////////////////////   
    
  
 
  
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
    }//统计第0列白点数
   
  for(column=1;column<=79;column++)//统计梯度数
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
    }//统计当前白点数
   
   if(gradient[column] - gradient[column-1] >= 0)
   {
      max_number++;
   }
   if(gradient[column] - gradient[column-1] <= 0)
   {
      small_number++;
   }
  }
  if(max_number - small_number > 10)//右转弯
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
        end_line = 59-gradient[78]+3;               //图像处理结束行 
      }
      else
      {
        led(LED2, LED_OFF);
        midline[start_line+1] = 40;
        end_line = 29;               //图像处理结束行 
      }
    
  }

  if(small_number - max_number > 10)//左转弯
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
        end_line = 59-gradient[1]+3;               //图像处理结束行 
      }
     else
     {
       led(LED3, LED_OFF);
       midline[start_line+1] = 40;
       end_line = 29;               //图像处理结束行 
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
      for(column=midline[line+1]; column<77; column++)       //找右线
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
     }//找右线循环
    
     for(column=midline[line+1];column>2;column--)       //找左线
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
      }//找左线循环
    
     if((-1000 == left_line[line]) || (1000 == right_line[line]))
      {
        line_lost(line);
      }
      else
      {
        
      }
      midline[line] = (right_line[line]+left_line[line]+1)/2;
      img[line][midline[line]] = 1;  
   }//大for循环
    last_left = -2;
    last_right = 81;
}
   










/*********************************************************************************
* 函 数 名          : line_lost
* 函数功能          : 处理各种丢线问题
* 输    入          : line
* 输    出          : 无
*********************************************************************************/
void line_lost(uint8 line)
{ 
  //此时已经判断到了有某条边丢线
  if((-1000 == left_line[line]) && (1000 != right_line[line]))     //左边丢线右边不丢线
  {

    left_line[line] = last_left + (right_line[line] - last_right);
    last_left = left_line[line];
    left_lost ++;
  }
  
  if((-1000 != left_line[line]) && (1000 == right_line[line]))     //左边不丢线右边丢线
  {
   
    right_line[line] = last_right + (left_line[line] - last_left);
    last_right = right_line[line];
    right_lost ++;
    midline[line] =  (int16)(left_line[line] + 40+(59-line)*0.5);
  }
  
  if((-1000 == left_line[line]) && (1000 == right_line[line]))     //左右都丢线
  {
    left_line[line] = last_left;
    right_line[line] = last_right;
    left_lost ++;
    right_lost ++;
    midline[line] = 40;
  }
}




/*********************************************************************************
* 函 数 名          : servo_error
* 函数功能          : 舵机偏差计算
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void servo_error(void)
{
  uint8 i;
  int16 error_duoji_add = 0;
  for(i=start_line-14;i>end_line;i--)
  {
    error_duoji[i] = 40 - midline[i];       //每行偏差计算
    error_duoji_add = error_duoji[i] + error_duoji_add;   //舵机偏差总和
  }

  error_duoji_final = (error_duoji_add ) / (start_line-14 - end_line);  //计算平均偏差 
  error_error_duoji= error_duoji_final - error_duoji_final_last;  //计算偏差的偏差
  error_duoji_final_last = error_duoji_final;                     //保存上一场舵机偏差

  
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

