#include "common.h"
#include "include.h"
#include "math.h"
#include "com.h"
#include "oled.h"

void speed_control_init(void)
{ 
  ftm_quad_init(FTM2);//FTM1 ���������ʼ�������õĹܽſɲ� port_cfg.h ��
  pit_init_ms(PIT1, 20);                                 //��ʼ��PIT0����ʱʱ��Ϊ�� 1000ms
  set_vector_handler(PIT1_VECTORn ,PIT1_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
  enable_irq (PIT1_IRQn);                                 //ʹ��PIT0�ж�  
}

/*********************************************************************************
* �� �� ��          : PIT1_IRQHandler
* ��������          : PIT1ִ�к������ٶȿ���
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void PIT1_IRQHandler(void)
{ 
    PIT_Flag_Clear(PIT1);             //���жϱ�־λ
    val = ftm_quad_get(FTM2);          //��ȡFTM �������� 
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
* �� �� ��          : bt_send
* ��������          : �������ͺ���
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void bt_send(void)
{
    char str [100];
    int error;
    memset(str,sizeof(str),0);
    //�ٶ�
    uart_putstr(VCAN_PORT,"car_speed:"); 
    int_char_uart(left_speed, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n"); 
    //ͣ��ʱ��
    uart_putstr(VCAN_PORT,"car_pit:"); 
    int_char_uart(n, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //���ƫ��
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
    //���ռ�ձ�
    uart_putstr(VCAN_PORT,"car_steer:"); 
    int_char_uart(steer, str);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //���p
    uart_putstr(VCAN_PORT,"servo_KP:"); 
    float_char_uart(str, PID_P, 2, 2);
    uart_putstr(VCAN_PORT,str);
    memset(str,sizeof(str),0);
    uart_putstr(VCAN_PORT,"\n");
    //���d
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
* �� �� ��          : uart4_handler
* ��������          : �����жϴ�����
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void uart4_handler(void)
{
    char ch;
    if(uart_query(UART4) == 1)   //�������ݼĴ�����
    {
        //�û���Ҫ�����������
        uart_getchar   (UART4, &ch);                    //���޵ȴ�����1���ֽ�
        //��ͣ��
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
* �� �� ��          : spp_init
* ��������          : �����жϳ�ʼ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void spp_init(void)
{
  set_vector_handler(UART4_RX_TX_VECTORn,uart4_handler);   // �����жϷ��������ж���������
  uart_rx_irq_en (UART4);                                 //�����ڽ����ж�  
}


/*********************************************************************************
* �� �� ��          : dip_switch_init
* ��������          : ���뿪�����ų�ʼ�����Ҽ�Ⲧ�뿪������״̬
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void dip_switch_init(void)
{
  gpio_init(PTE6,GPO,1);          //��ʼ������
  gpio_init(PTE7,GPO,1); 
  gpio_init(PTE8,GPO,1); 
  gpio_init(PTE9,GPO,1); 
  dip_switch_1 = GPIO_GET(PTE6);  //�������״̬
  dip_switch_2 = GPIO_GET(PTE7);
  dip_switch_3 = GPIO_GET(PTE8);
  dip_switch_4 = GPIO_GET(PTE9);
  car_run=dip_switch_1;
}


/*********************************************************************************
* �� �� ��          : beep_off
* ��������          : ��������
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void beer_off(void)
{
  GPIO_SET(PTA14,0);
}

/*********************************************************************************
* �� �� ��          : beep_on
* ��������          : ��������
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void beer_on(void)
{
  GPIO_SET(PTA14,1);
}

/*********************************************************************************
* �� �� ��          : beep_init
* ��������          : �������˿ڳ�ʼ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void beep_init(void)
{
  gpio_init(PTA14,GPO,0);
}




/*********************************************************************************
* �� �� ��          : key_init_all
* ��������          : ���а���ʼ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void key_init_all(void)
{
  gpio_init(PTE0,GPO,1);
  gpio_init(PTE1,GPO,1);
  gpio_init(PTE2,GPO,1);
  gpio_init(PTE3,GPO,1);
  gpio_init(PTE4,GPO,1);                  
  gpio_init(PTE5,GPO,1);                                  //��ʼ����Щ�����ڼ������״̬
  port_init(PTE0, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTE0 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
  port_init(PTE1, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTE1 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
  port_init(PTE2, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTE2 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
  port_init(PTE3, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTE3 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
  port_init(PTE4, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTE4 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
  port_init(PTE5, ALT1 | IRQ_FALLING | PULLUP );          //��ʼ�� PTE5 �ܽţ����ù���ΪGPIO ���½��ش����жϣ���������
  set_vector_handler(PORTE_VECTORn ,PORTE_IRQHandler);    //����PORTE���жϷ�����Ϊ PORTE_IRQHandler
  enable_irq (PORTE_IRQn);                                //ʹ��PORTE�ж�
}


/*********************************************************************************
* �� �� ��          : PIT0_IRQHandler
* ��������          : PIT0ִ�к�����ʵ����ͣ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void PIT0_IRQHandler(void)
{
  motor_control(0,0,speed_switch);
  PIT_Flag_Clear(PIT0);       //���жϱ�־λ
  while(1); 
}


/*********************************************************************************
* �� �� ��          : car_stop_init
* ��������          : ͣ����ʼ������������PIT0
* ��    ��          : second
* ��    ��          : ��
*********************************************************************************/
void car_stop_init(float second)
{
  int n;
  n = (uint16)(second * 1000.0);
  pit_init_ms(PIT0, n);                                //��ʼ��PIT0����ʱʱ��Ϊ�� 1000ms
  set_vector_handler(PIT0_VECTORn ,PIT0_IRQHandler);      //����PIT0���жϷ�����Ϊ PIT0_IRQHandler
  if(car_run == DIP_SWITCH_ON)
  {
    enable_irq (PIT0_IRQn);       //����PIT0�������ʼ��ʱ
  } 
}


/*********************************************************************************
* �� �� ��          : oled_init
* ��������          : oled�ĳ�ʼ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void oled_init(void)
{
  uint16 n;                     //ͣ��ʱ��
  LCD_Init();
  LCD_P6x8Str(0,0,"speed:");
  LCD_P6x8Str(0,1,"error:");
  LCD_P6x8Str(0,2,"steer:");
  LCD_P6x8Str(0,3,"left_lost:");
  LCD_P6x8Str(0,4,"right_lost:");
  LCD_P6x8Str(0,5,"line_uesd:");
  LCD_P6x8Str(0,6,"pit:");
  n = (uint16)(stop_second * 1000); 
  LCD_P6x8number(24,6,n);       //��ʾͣ��ʱ��
  //LCD_P6x8number(36,1,a); 
}


/*********************************************************************************
* �� �� ��          : led_all_init
* ��������          : led�ĳ�ʼ��
* ��    ��          : ��
* ��    ��          : ��
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
* �� �� ��          : camera_int_init
* ��������          : ����ͷ��ʼ�����ж�����
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void camera_int_init()
{
  camera_init(imgbuff);                                  
  set_vector_handler(PORTA_VECTORn ,PORTA_IRQHandler);   
  set_vector_handler(DMA0_VECTORn ,DMA0_IRQHandler);   
}

/*********************************************************************************
* �� �� ��          : motor_init
* ��������          : �����ʼ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void motor_init(void)
{
  ftm_pwm_init(MOTOR_FTM, MOTOR1_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR2_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR3_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM
  ftm_pwm_init(MOTOR_FTM, MOTOR4_PWM,MOTOR_HZ,100);      //��ʼ�� ��� PWM 
  motor_control(0,0,speed_switch);
  LCD_P6x8number(36,0,left_speed);
  LCD_P6x8number(52,0,right_speed);
}


/*********************************************************************************
* �� �� ��          : motor_control
* ��������          : ����ٶȿ���
* ��    ��          : left,right
* ��    ��          : ��
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
* �� �� ��          : servo_init
* ��������          : �����ʼ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void servo_init(void)
{
    ftm_pwm_init(S3010_FTM, S3010_CH,S3010_HZ,DJ_ZZ); 
}


/*********************************************************************************
* �� �� ��          : servo_control
* ��������          : �������
* ��    ��          : deer
* ��    ��          : ��
*********************************************************************************/
void servo_control(uint16 steer)
{
  ftm_pwm_duty(S3010_FTM, S3010_CH,steer);   //140�� 157��  123��
}



/*********************************************************************************
* �� �� ��          : led_all_on
* ��������          : ledȫ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void led_all_on(void)
{
  led(LED0, LED_ON);
  led(LED1, LED_ON);
  led(LED2, LED_ON);
  led(LED3, LED_ON);  
}

/*********************************************************************************
* �� �� ��          : led_all_off
* ��������          : ledȫ��
* ��    ��          : ��
* ��    ��          : ��
*********************************************************************************/
void led_all_off(void)
{
  led(LED0, LED_OFF);
  led(LED1, LED_OFF);
  led(LED2, LED_OFF);
  led(LED3, LED_OFF); 
}



















/*!
 *  @brief      PORTA�жϷ�����
 *  @since      v5.0
 */
void PORTA_IRQHandler()
{
    uint8  n = 0;    //���ź�
    uint32 flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //���жϱ�־λ

    n = 29;                                             //���ж�
    if(flag & (1 << n))                                 //PTA29�����ж�
    {
        camera_vsync();
    }
#if 0             //ӥ��ֱ��ȫ�ٲɼ�������Ҫ���ж�
    n = 28;
    if(flag & (1 << n))                                 //PTA28�����ж�
    {
        camera_href();
    }
#endif
}

/*!
 *  @brief      DMA0�жϷ�����
 *  @since      v5.0
 */
void DMA0_IRQHandler()
{
    camera_dma();
}