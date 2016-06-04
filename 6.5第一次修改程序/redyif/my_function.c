#include "common.h"
#include "include.h"
#include "com.h"

//用于添加一些我自己常用的函数

/*********************************************************************************
* 函 数 名          : float_char_uart
* 函数功能          : 把浮点型数据转换成字符型数据
* 输    入          : 
* 输    出          : 无
*********************************************************************************/

void float_char_uart(char *str, double number, uint8_t g, uint8_t l)
{
    static char table[]={'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint8_t i;
    int temp = number/1;
    double t2 = 0.0;
    for (i = 1; i<=g; i++)
    {
        if (temp==0)
            str[g-i] = table[0];
        else
            str[g-i] = table[temp%10];
        temp = temp/10;
    }
    *(str+g) = '.';
    temp = 0;
    t2 = number;
    for(i=1; i<=l; i++)
    {
        temp = t2*10;
        str[g+i] = table[temp%10];
        t2 = t2*10;
    }
    *(str+g+l+1) = '\0';
}





/*********************************************************************************
* 函 数 名          : int_char_uart
* 函数功能          : 把整形数据转换成字符型数据
* 输    入          : 
* 输    出          : 无
*********************************************************************************/
void int_char_uart(int num,char str[] )
{
  int sign;
  unsigned char i=0,j=0;
  unsigned char temp[10];
  sign=num;
  do
  {
    temp[i] = sign%10+'0';
    sign/=10;
    i++;
  }
  while(sign>0);
  while(i>0)
  {
    str[j] = temp[i-1];
    j++;
    i--;
  }
  str[j]='\0';
}