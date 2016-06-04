


//porta中断在摄像头中已经配置 
//在上位机中看到的图像最上面是0，最下面是59
//FTM0是电机
//FTM1是舵机，精度改为1000,在chip->MK60_ftm.c->MK60_FTM.h里面改
//舵机  105中 135左  75右
//关于OLED，首先需要初始化函数，常用的函数有LCD_P6x8Str(0,0,"speed:")和;LCD_P6x8number(36,1,a);
//LED亮是led(LED0, LED_ON); 关是led(LED0, LED_OFF);  
//拨码开关使用历程 if(dip_switch_1 == DIP_SWITCH_ON)
//dip_switch_1这家伙会受按键中断显示num的函数有影响，待查

/*
void center_line(void)
{
  uint8 line,column;
  uint8 center,left,right;
  uint8 right_lost;
  uint8 left_lost;
  for(line=59; line>20; line--)
  {
    if(img[line][39] - img[line-1][39] == 255)
    {
      center = line;
      break;
    }
    else
    {
      center = 100;
    }
  }
  
  for(line=59; line>20; line--)
  {
    if(img[line][36] - img[line-1][36] == 255)
    {
      left = line;
      break;     
    }
    else
    {
      left = 100;
    }   
  }
  
  for(line=59; line>20; line--)
  {
    if(img[line][42] - img[line-1][42] == 255)
    {
      right = line;
      break;
    }
    else
    {
      right = 100;
    }   
  }

  if((center != 100) && (left != 100) && (right != 100))
  {
    if((left - center >0) && (left - right >0))               //右边丢线
    {
      right_lost = 1;
    }
    if((left - center <0) && (left - right <0))               //左边丢线
    {
      left_lost = 1;
    }
  }
  else
  {
    right_lost = 0;
    left_lost = 0;
  }
  
  if(right_lost == 1)
  {
    midline[start_line+1] = 65;
  }
  
  if(left_lost == 1)
  {
    midline[start_line+1] = 15;
  }
  
  if(left_lost == 0 && right_lost ==0)
  {
    midline[start_line+1] = 40;
  }
  
  for(line=start_line;line>end_line;line--)
  {
    for(column=midline[line+1];column<77;column++)
    {
      if(img[line][column] - img[line][column+1] == 255)
      {
        right_line[line] = column;
        last_right = column;
        break;
      }
      else
      {
        right_line[line] = 1000;
      }
    }
    
    for(column=midline[line+1];column>2;column--) 
    {
      if(img[line][column] - img[line][column-1] == 255)
      {
        left_line[line] = column;
        last_left = column;
        break;           
      }
      else
      {
       left_line[line] = -1000;
      } 
    }

    if((-1000 == left_line[line]) || (1000 == right_line[line]))
    {
      line_lost(line);
    }
    midline[line] = (right_line[line]+left_line[line]+1)/2;
    img[line][midline[line]] = 1;      
  }
}




void center_line(void)
{
  uint8 i,j;
  for(i=start_line;i>end_line;i--)
  {
    for(j=40;j<77;j++)          //寻找右边界
    {
      if(img[i][j] - img[i][j+1] == 255)
      {
        right_line[i] = j;
        last_right = j;
        break;
      }
      else
      {
        right_line[i] = 1000;       //右边界没找到
      }
    }
    
    for(j=40;j>2;j--)         //寻找左边界
    {
      if(img[i][j] - img[i][j-1] == 255)
      {
        left_line[i] = j;
        last_left = j;   
        break;
      }
      else
      {
        left_line[i] = -1000;       //左边界没找到
      }
    }  
    
    if(right_line[i]==1000)
    {
      right_line[i]=90;
    }
    if(left_line[i]==-1000)
    {
      left_line[i]=-9;
    }
    midline[i] = (right_line[i]+left_line[i]+1)/2;
    img[i][midline[i]] = 1;       //打黑中线
  }
}
*/