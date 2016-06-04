#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"

void speed_control_init(void)
{ 
  ftm_quad_init(FTM2);//FTM1 正交解码初始化（所用的管脚可查 port_cfg.h ）
  pit_init_ms(PIT1, 20);                                 //初始化PIT0，定时时间为： 1000ms
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
  enable_irq (PIT1_IRQn);                                 //使能PIT0中断  
}

/*********************************************************************************
* 函 数 名          : PIT1_IRQHandler
* 函数功能          : PIT1执行函数，速度控制
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void PIT1_IRQHandler(void)
{ 
    PIT_Flag_Clear(PIT1);             //清中断标志位
    val = ftm_quad_get(FTM2);          //获取FTM 正交解码 
    ftm_quad_clean(FTM2);
    error = speed_ideal - val;
    d_error = error - pre_error;
    pre_error = error;
    pwm = (int16)(pwm_last + PID_P*d_error + PID_I*error/1000);
    pwm_last = pwm;
    if(pwm>500)
    {
      pwm = 500;
    }
    if(pwm <0)
    {
      pwm = 0;
    }
    motor_control(pwm,pwm,speed_switch);
    var[0] = val;
    //vcan_sendware(var, sizeof(var));
    /*
    error = speed_ideal - val;
    d_error = error - pre_error;
    dd_error = pre_error - pre_pre_error;
    pre_error = error;
    pre_pre_error = d_error;
    pwm = (uint16)(pwm+ PID_P*d_error+ PID_I*error+ PID_D*dd_error);
    if(pwm>800)
    {
      pwm = 800;
    }
    if(pwm < 50)
    {
      pwm = 50;
    }
    motor_control(pwm,pwm,speed_switch);
    var[0] = val;
    //vcan_sendware(var, sizeof(var));
    */
}



/*********************************************************************************
* 函 数 名          : bt_send
* 函数功能          : 蓝牙发送函数
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void bt_send(void)
{
    char str [100];
    int error;
    memset(str,sizeof(str),0);
    //速度
    uart_putstr(VCAN_PORT,"car_speed:"); 
    int_char_uart(left_speed, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n"); 
    //停车时长
    uart_putstr(VCAN_PORT,"car_pit:"); 
    int_char_uart(n, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //舵机偏差
    uart_putstr(VCAN_PORT,"car_error:");
    if(error_duoji_final < 0)
    {
      error = 0 - error_duoji_final;
      uart_putstr(VCAN_PORT,"-");
    }
    else
    {
      error = 0 + error_duoji_final;
    }
    int_char_uart(error, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //舵机占空比
    uart_putstr(VCAN_PORT,"car_steer:"); 
    int_char_uart(steer, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //舵机p
    uart_putstr(VCAN_PORT,"servo_KP:"); 
    float_char_uart(str, PID_P, 2, 2);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //舵机d
    uart_putstr(VCAN_PORT,"servo_KD:"); 
    float_char_uart(str, PID_I, 2, 2);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    
    uart_putstr(VCAN_PORT,"\n"); 
    uart_putstr(VCAN_PORT,"\n"); 
    uart_putstr(VCAN_PORT,"\n"); 
    uart_putstr(VCAN_PORT,"\n"); 
}

/*********************************************************************************
* 函 数 名          : uart4_handler
* 函数功能          : 蓝牙中断处理函数
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void uart4_handler(void)
{
    char ch;
    if(uart_query(UART4) == 1)   //接收数据寄存器满
    {
        //用户需要处理接收数据
        uart_getchar   (UART4, &ch);                    //无限等待接受1个字节
        //开停车
        if('a' == ch)           
        {
          if(1 == speed_switch)
          {
            motor_control(0,0,speed_switch); 
            speed_switch = 0;
          }
          else
          {
            speed_switch = 1;  
            motor_control(left_speed,right_speed,speed_switch);
          }         
        }
        //kp+
        if('b' == ch)
        {
          PID_P = PID_P + 0.1;
        }
        //kp-
        if('c' == ch)
        {
          PID_P = PID_P - 0.1;
        }
        //kd+
        if('d' == ch)
        {
          PID_I = PID_I + 0.1;
        }
        //kd-
        if('e' == ch)
        {
          PID_I = PID_I - 0.1;
        }
        
        bt_send();  
    }
}



/*********************************************************************************
* 函 数 名          : spp_init
* 函数功能          : 蓝牙中断初始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void spp_init(void)
{
  set_vector_handler(UART4_RX_TX_VECTORn,uart4_handler);   // 设置中断服务函数到中断向量表里
  uart_rx_irq_en (UART4);                                 //开串口接收中断  
}


/*********************************************************************************
* 函 数 名          : dip_switch_init
* 函数功能          : 拨码开关引脚初始化并且检测拨码开关引脚状态
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void dip_switch_init(void)
{
  gpio_init(PTE6,GPO,1);          //初始化引脚
  gpio_init(PTE7,GPO,1); 
  gpio_init(PTE8,GPO,1); 
  gpio_init(PTE9,GPO,1); 
  dip_switch_1 = GPIO_GET(PTE6);  //检测引脚状态
  dip_switch_2 = GPIO_GET(PTE7);
  dip_switch_3 = GPIO_GET(PTE8);
  dip_switch_4 = GPIO_GET(PTE9);
  car_run=dip_switch_1;
}


/*********************************************************************************
* 函 数 名          : beep_off
* 函数功能          : 蜂鸣器关
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void beer_off(void)
{
  GPIO_SET(PTA14,0);
}

/*********************************************************************************
* 函 数 名          : beep_on
* 函数功能          : 蜂鸣器开
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void beer_on(void)
{
  GPIO_SET(PTA14,1);
}

/*********************************************************************************
* 函 数 名          : beep_init
* 函数功能          : 蜂鸣器端口初始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void beep_init(void)
{
  gpio_init(PTA14,GPO,0);
}




/*********************************************************************************
* 函 数 名          : key_init_all
* 函数功能          : 所有按键始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void key_init_all(void)
{
  gpio_init(PTE0,GPO,1);
  gpio_init(PTE1,GPO,1);
  gpio_init(PTE2,GPO,1);
  gpio_init(PTE3,GPO,1);
  gpio_init(PTE4,GPO,1);                  
  gpio_init(PTE5,GPO,1);                                  //初始化这些口用于检测引脚状态
  port_init(PTE0, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTE0 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
  port_init(PTE1, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTE1 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
  port_init(PTE2, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTE2 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
  port_init(PTE3, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTE3 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
  port_init(PTE4, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTE4 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
  port_init(PTE5, ALT1 | IRQ_FALLING | PULLUP );          //初始化 PTE5 管脚，复用功能为GPIO ，下降沿触发中断，上拉电阻
  set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    //设置PORTE的中断服务函数为 PORTE_IRQHandler
  enable_irq (PORTE_IRQn);                                //使能PORTE中断
}


/*********************************************************************************
* 函 数 名          : PIT0_IRQHandler
* 函数功能          : PIT0执行函数，实际是停车
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void PIT0_IRQHandler(void)
{
  motor_control(0,0,speed_switch);
  PIT_Flag_Clear(PIT0);       //清中断标志位
  while(1); 
}


/*********************************************************************************
* 函 数 名          : car_stop_init
* 函数功能          : 停车初始化函数，用了PIT0
* 输    入          : second
* 输    出          : 无
*********************************************************************************/
void car_stop_init(float second)
{
  int n;
  n = (uint16)(second * 1000.0);
  pit_init_ms(PIT0, n);                                //初始化PIT0，定时时间为： 1000ms
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //设置PIT0的中断服务函数为 PIT0_IRQHandler
  if(car_run == DIP_SWITCH_ON)
  {
    enable_irq (PIT0_IRQn);       //开启PIT0，电机开始计时
  } 
}


/*********************************************************************************
* 函 数 名          : oled_init
* 函数功能          : oled的初始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void oled_init(void)
{
  uint16 n;                     //停车时间
  LCD_Init();
  LCD_P6x8Str(0,0,"speed:");
  LCD_P6x8Str(0,1,"error:");
  LCD_P6x8Str(0,2,"steer:");
  LCD_P6x8Str(0,3,"left_lost:");
  LCD_P6x8Str(0,4,"right_lost:");
  LCD_P6x8Str(0,5,"line_uesd:");
  LCD_P6x8Str(0,6,"pit:");
  n = (uint16)(stop_second * 1000); 
  LCD_P6x8number(24,6,n);       //显示停车时间
  //LCD_P6x8number(36,1,a); 
}


/*********************************************************************************
* 函 数 名          : led_all_init
* 函数功能          : led的初始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void led_all_init(void)
{
  led_init(LED0);
  led_init(LED1);
  led_init(LED2);
  led_init(LED3);
  led_all_on();
}

/*********************************************************************************
* 函 数 名          : camera_int_init
* 函数功能          : 摄像头初始化及中断设置
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void camera_int_init()
{
  camera_init(imgbuff);                                  
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);   
  set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);   
}

/*********************************************************************************
* 函 数 名          : motor_init
* 函数功能          : 电机初始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void motor_init(void)
{
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,100);      //初始化 电机 PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,100);      //初始化 电机 PWM 
  motor_control(0,0,speed_switch);
  LCD_P6x8number(36,0,left_speed);
  LCD_P6x8number(52,0,right_speed);
}


/*********************************************************************************
* 函 数 名          : motor_control
* 函数功能          : 电机速度控制
* 输    入          : left,right
* 输    出          : 无
*********************************************************************************/
void motor_control(int16 left, int16 right, uint8 flag)
{
    if(1 == flag)
    {
      if(left<0 || right <0)
      { 
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,left);
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,right); 
      }
      else
      {
        ftm_pwm_duty(MOTOR_FTM, MOTOR1_PWM,left);
        ftm_pwm_duty(MOTOR_FTM, MOTOR2_PWM,0);
        ftm_pwm_duty(MOTOR_FTM, MOTOR3_PWM,right);
        ftm_pwm_duty(MOTOR_FTM, MOTOR4_PWM,0);          
      }

    }
}


/*********************************************************************************
* 函 数 名          : servo_init
* 函数功能          : 舵机初始化
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void servo_init(void)
{
    ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,DJ_ZZ); 
}


/*********************************************************************************
* 函 数 名          : servo_control
* 函数功能          : 舵机控制
* 输    入          : deer
* 输    出          : 无
*********************************************************************************/
void servo_control(uint16 steer)
{
  ftm_pwm_duty(S3010_FTM, S3010_CH,steer);   //140中 157左  123右
}



/*********************************************************************************
* 函 数 名          : led_all_on
* 函数功能          : led全开
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void led_all_on(void)
{
  led(LED0, LED_ON);
  led(LED1, LED_ON);
  led(LED2, LED_ON);
  led(LED3, LED_ON);  
}

/*********************************************************************************
* 函 数 名          : led_all_off
* 函数功能          : led全关
* 输    入          : 无
* 输    出          : 无
*********************************************************************************/
void led_all_off(void)
{
  led(LED0, LED_OFF);
  led(LED1, LED_OFF);
  led(LED2, LED_OFF);
  led(LED3, LED_OFF); 
}



















/*!
 *  @brief      PORTA中断服务函数
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //引脚号
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if 0             //鹰眼直接全速采集，不需要行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0中断服务函数
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}