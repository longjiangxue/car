


//porta�ж�������ͷ���Ѿ����� 
//����λ���п�����ͼ����������0����������59
//FTM0�ǵ��
//FTM1�Ƕ�������ȸ�Ϊ1000,��chip->MK60_ftm.c->MK60_FTM.h�����
//���  105�� 135��  75��
//����OLED��������Ҫ��ʼ�����������õĺ�����LCD_P6x8Str(0,0,"speed:")��;LCD_P6x8number(36,1,a);
//LED����led(LED0, LED_ON); ����led(LED0, LED_OFF);  
//���뿪��ʹ������ if(dip_switch_1 == DIP_SWITCH_ON)
//dip_switch_1��һ���ܰ����ж���ʾnum�ĺ�����Ӱ�죬����

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
    if((left - center >0) && (left - right >0))               //�ұ߶���
    {
      right_lost = 1;
    }
    if((left - center <0) && (left - right <0))               //��߶���
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
    for(j=40;j<77;j++)          //Ѱ���ұ߽�
    {
      if(img[i][j] - img[i][j+1] == 255)
      {
        right_line[i] = j;
        last_right = j;
        break;
      }
      else
      {
        right_line[i] = 1000;       //�ұ߽�û�ҵ�
      }
    }
    
    for(j=40;j>2;j--)         //Ѱ����߽�
    {
      if(img[i][j] - img[i][j-1] == 255)
      {
        left_line[i] = j;
        last_left = j;   
        break;
      }
      else
      {
        left_line[i] = -1000;       //��߽�û�ҵ�
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
    img[i][midline[i]] = 1;       //�������
  }
}
*/