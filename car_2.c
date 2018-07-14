#include "common.h"
#include "include.h"
#include "math.h"

#define K               22
#define MID             4720    //舵机中值占空比3550         300HZ 5950  180HZ 3550
#define RIGHT_MAX       900     //舵机相对中值向右转最大值         1650        1000    
#define LEFT_MAX        900     //舵机相对中值向左转最大值         2100        1250    
#define NORMAL_FLAG     0       //正常运行标志位
#define CROSS_FLAG      1       //十字形弯道标志位
#define BLANCK_FLAG     2       //图像空白干扰标志位
//#define CIRCLE_FLAG     3       //圆环标志位       
#define STOP_FLAG       4       //停车标志位 
#define RIGHT_ISLAND_IN_FLAG    5       //进右环岛标志位
#define RIGHT_ISLAND_OUT_FLAG   6       //出右环岛标志位
#define LEFT_ISLAND_IN_FLAG     7       //进左环岛标志位
#define LEFT_ISLAND_OUT_FLAG    8       //出左环岛标志位
#define BARRIER_FLAG            9       //障碍物标志位

uint8 imgbuff[CAMERA_SIZE];             //定义存储接收图像的数组
uint8 img[CAMERA_H][CAMERA_W];
uint8 centerline[CAMERA_H];
uint8 center[CAMERA_H][CAMERA_W];

float Ka = 0;                   //角度倍数
float Kb = 0;                   //前瞻倍数

uint8 flag;                     //标志位
float error;                   //十字路的偏差

uint16 accelerate_num = 0;      //加速次数
uint16 angle_counter = 0;       //看到角度的次数
int16 val = 0;                  //编码器脉冲值
float motor_speed = 0;         //电机占空比设置
float feedback_speed = 0;      //反馈速度

uint8 start_flag = 0;
uint8 start_new = 0;
uint8 start_old = 0;
uint8 stop_flag = 0;
uint8 right_island_see_flag = 0;      //看到右环岛标志位
uint8 right_island_in_flag = 0;       //进右环岛标志位  
uint8 right_island_out_flag = 0;      //出右环岛标志位
uint8 left_island_see_flag = 0;
uint8 left_island_in_flag = 0;
uint8 left_island_out_flag = 0;
uint8 island_see_count = 0;
uint8 island_in_count = 0;
uint8 island_in_en_flag = 0;
uint8 island_out_en_flag = 0;

//uint16 barrier_left_black_num = 0;
//uint16 barrier_right_black_num = 0;
//uint8 barrier_counter = 0;
//uint8 barrier_flag = 0;
//uint8 left_barrier = 0;
//uint8 right_barrier = 0;
//uint8 forbid_flag = 0;        //禁止主程序控制

//uint32 time = 0;                //测试主程序用时
uint8 key1;
uint8 key2;
uint8 key3;
uint8 key4;
int blanck_duty;

float sum_x1 = 0;
float sum_y1 = 0;
uint32 sum11 = 0;
uint32 sum21 = 0;
float x_average1 = 0;
float y_average1 = 0;

float sum_x2 = 0;
float sum_y2 = 0;
uint32 sum12 = 0;
uint32 sum22 = 0;
float x_average2 = 0;
float y_average2 = 0;

float sum_x3 = 0;
float sum_y3 = 0;
uint32 sum13 = 0;
uint32 sum23 = 0;
float x_average3 = 0;
float y_average3 = 0;

float sum_x4 = 0;
float sum_y4 = 0;
uint32 sum14 = 0;
uint32 sum24 = 0;
float x_average4 = 0;
float y_average4 = 0;

//float sum_x5 = 0;
//float sum_y5 = 0;
//uint32 sum15 = 0;
//uint32 sum25 = 0;
//float x_average5 = 0;
//float y_average5 = 0;

int error1;
int error2;
int error3;
int error4;
int error5;
float all_error = 0;            //最后偏差
float forsee = 0;               //前瞻量
//float all_error_old = 0;        //偏差旧值

float b1 = 0.0;
float b2 = 0.0;
float b3 = 0.0;
float b4 = 0.0;
//float b5 = 0.0;
float angle1 = 0.0;
float angle2 = 0.0;
float angle3 = 0.0;
float angle4 = 0.0;
//float angle5 = 0.0;
float angle = 0.0;
int duty = 0;

//函数声明
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void PIT2_IRQHandler();
void PIT0_IRQHandler();
void PIT1_IRQHandler();
void find_center();
void find_centerline();
void cal_angle();
float PID_realize(float ang);
float CloseLoop_PID_realize(float speed);
void PID_init();
void CloseLoop_PID_init();
void scan();

struct _pid{
  float SetValue;      //定义设定值
  float ActualValue;   //定义实际值
  float err;           //定义偏差值
  float pre_err;       //定义上一个偏差值        
  float Kp,Kd,Ki;      //定义比例、微分、积分系数
  float duty;          //定义占空比
}pid, closeloop_pid;

void  main(void)
{
    int i,j;    
    
    DisableInterrupts;

    camera_init(imgbuff);  //摄像头初始化
    
    ftm_pwm_init(FTM1, FTM_CH0, 300, MID);      //舵机初始化，FTM1PWM精度为10000  CH0对应PTA8无效  PTA12有效  PTB0无效 CH1对应A9无效 A13有效 B1无效
                                                //比3550小 往左打轮子   左极限2300-右极限-4550  机械结构不对称
    
    ftm_pwm_init(FTM0, FTM_CH1, 10000, 200);    //电机初始化，CH1对应PTA4有效  CH0对应PTC1有效；
    
    ftm_pwm_init(FTM0, FTM_CH0, 10000, 0);    
    /********** 正交解码初始化 **********/
    
    ftm_quad_init(FTM2);                        //正交解码测速  A相---PTA10 B相---PTA11
//    port_init_NoALT(PTA10, PULLUP);             //程序内接上拉会出现有反转的情况
    pit_init_ms(PIT2, 50);
    set_vector_handler(PIT2_VECTORn, PIT2_IRQHandler);
    enable_irq(PIT2_IRQn);
    
    /********** 测试 **********/
    
//    pit_init_ms(PIT0, 10000);
//    set_vector_handler(PIT0_VECTORn , PIT0_IRQHandler);
//    enable_irq(PIT0_IRQn);
//    pit_init_ms(PIT1, 5000);
//    set_vector_handler(PIT1_VECTORn , PIT1_IRQHandler);
//    enable_irq(PIT0_IRQn);
//    enable_irq(PIT1_IRQn);
    
    /********** 测试 **********/
    
    /********** 中断优先级分组 **********/
    
    NVIC_SetPriorityGrouping(NVIC_PriorityGroup_2);     //优先级分组
    set_irq_priority(PORTA_IRQn, 0);                    //设置优先级,prio越低，则优先级越高
    set_irq_priority(DMA0_IRQn, 1);
    set_irq_priority(PIT2_IRQn, 2);
//    set_irq_priority(PIT0_IRQn, 3);
//    set_irq_priority(PIT1_IRQn, 3);
    
    //配置中断服务函数
    set_vector_handler(PORTA_VECTORn, PORTA_IRQHandler);   //设置 PORTA 的中断服务函数为 PORTA_IRQHandler
    set_vector_handler(DMA0_VECTORn, DMA0_IRQHandler);     //设置 DMA0 的中断服务函数为 PORTA_IRQHandler
    
    EnableInterrupts;
    
    //开关初始化
    gpio_init(PTD0, GPI, 0);
    gpio_init(PTD1, GPI, 0);
    gpio_init(PTD2, GPI, 0);
    gpio_init(PTD3, GPI, 0);
    key4 = gpio_get(PTD0);
    key3 = gpio_get(PTD1);
    key2 = gpio_get(PTD2);
    key1 = gpio_get(PTD3);
     
    /********** UART3,串口的TXD接C16,RXD接C17 **********/
    while(1)
    {
//      time = 0;
//      pit_time_start(PIT1);
      camera_get_img();                              //摄像头获取图像
            
       if(1)
       {
        img_extract(center, imgbuff, CAMERA_SIZE);  //把图像解压到寻中线数组center中
        
        scan();                 //行扫 列扫 得出特殊位置的标志位
        
        find_centerline();      //得出中线数组,并得到x_average,y_average等
        
        cal_angle();            //算出斜率的角度

        PID_init();             //舵机pid初始化
        
        CloseLoop_PID_init();   //电机pid初始化
        
//        printf("feedback_speed=%d\n", (int)feedback_speed);
//        printf("all_error=%d\n", (int)all_error);
//        printf("all_error_old=%d\n", (int)all_error_old);
//        printf("flag=%d\n", flag);
//        printf("error1=%d\n", error1);
//        printf("error2=%d\n", error2);
//        printf("error3=%d\n", error3);
//        printf("error4=%d\n", error4);
//        printf("all_error=%d.%d\n", (int)all_error, abs((int)((all_error - (int)all_error)*100)));
//        printf("angle=%d.%d\n", (int)angle, abs((int)((angle - (int)angle)*100)));
//        printf("angle1=%d.%d\n", (int)angle1, abs((int)((angle1 - (int)angle1)*100)));
//        printf("angle2=%d.%d\n", (int)angle2, abs((int)((angle2 - (int)angle2)*100)));
//        printf("angle3=%d.%d\n", (int)angle3, abs((int)((angle3 - (int)angle3)*100)));
//        printf("angle4=%d.%d\n", (int)angle4, abs((int)((angle4 - (int)angle4)*100)));
//        blanck_duty = (int)((( angle4 * 0.7 + angle3 * 0.3 ) * 0.4 + ( error4 / 4.5 * 0.7 + error3 / 4.5 * 0.3 ) * 0.6 ) * 27);
//        printf("blanck_duty=%d\n", blanck_duty);
        switch(flag)
        {
          case NORMAL_FLAG:
//            pid.ActualValue = all_error * 0.7 + forsee * 0.3;
            /********** 舵机控制 **********/
            if( ( all_error >= 0 && all_error < 10 ) || ( all_error > -10 && all_error < 0 ) )
            {
              pid.Kp = 0.5;
//              pid.ActualValue = all_error * 0.7 + forsee * 0.3;
            }
//            else if( ( all_error >= 10 && all_error < 15 ) || ( all_error >= -10 && all_error < -15 ) )
//            {
//              pid.Kp = 0.7;
//            }
//            else if( ( all_error >= 15 && all_error < 20 ) || ( all_error >= -15 && all_error < -20 ) )
//            {
//              pid.Kp = 0.9;
////              pid.ActualValue = all_error;
//            }
//            else if( ( all_error >= 20 && all_error < 25 ) || ( all_error >= -20 && all_error < -25 ) )
//            {
//              pid.Kp = 1.0;
////              pid.ActualValue = all_error;
//            }
            else
            {
              pid.Kp = 1.15;
            }
            duty = (int)PID_realize(0.0);   //得到实际占空比
            if(duty > LEFT_MAX)
              duty = LEFT_MAX;
            if(duty < -RIGHT_MAX)
              duty = -RIGHT_MAX;
            ftm_pwm_duty(FTM1, FTM_CH0, MID - duty); //error1为20到30行   error5为10到20行
//            all_error_old = all_error;
            break;
            
          case CROSS_FLAG:
//            add_line();
//            angle = angle5 * 0.7 + angle1 * 0.3; 
            all_error = error5 * 0.3 + error1 * 0.4 + error2 * 0.3;
            error = all_error / 4.5;
            error = error * K * pid.Kp;
//            if(error < 0)
//              error = (int)(error * 1.25);
            if(error > RIGHT_MAX)
              error = RIGHT_MAX;
            if(error < -LEFT_MAX)
              error = -LEFT_MAX;
            ftm_pwm_duty(FTM1, FTM_CH0, MID + (int)error);
            all_error = error / K / pid.Kp;
//            all_error_old = all_error;
            break;
            
          case BLANCK_FLAG:
            blanck_duty = (int)((( angle4 * 0.7 + angle3 * 0.3 ) * 0.4 + ( error4 / 4.5 * 0.7 + error3 / 4.5 * 0.3 ) * 0.6 ) * K);
//            if(blanck_duty < 0)
//              blanck_duty = (int)(blanck_duty * 1.25);
            if(blanck_duty > RIGHT_MAX)
              blanck_duty = RIGHT_MAX;
            if(blanck_duty < -LEFT_MAX)
              blanck_duty = -LEFT_MAX;
            ftm_pwm_duty(FTM1, FTM_CH0, MID + blanck_duty);
            all_error = blanck_duty / K;
//            all_error_old = all_error;
//            printf("blanck_duty=%d\n", blanck_duty);
            //ftm_pwm_duty(FTM1, FTM_CH0, 3550 + (int)(( error4 / 4.5 * 0.7 + error3 / 4.5 * 0.3 ) * 30));
            break;
            
//          case CIRCLE_FLAG:
////            ftm_pwm_duty(FTM0, FTM_CH1, 300);
////            ftm_pwm_duty(FTM0, FTM_CH0, 130); //13占空比可过圆环
//            ftm_pwm_duty(FTM1, FTM_CH0, MID - (int)( 1000 * 1.25 ));   //左转进圆环
//            all_error = 1000 / K;
//            all_error_old = all_error;
//            break;
            
          case STOP_FLAG:
            stop_flag = 1;
            break;
            
          case BARRIER_FLAG:
            all_error = error4 * 0.4 + error3 * 0.3 + error2 * 0.3;
            error = all_error / 4.5;
            error = error * 18 * pid.Kp;
//            if(error < 0)
//              error = (int)(error * 1.25);
            if(error > RIGHT_MAX)
              error = RIGHT_MAX;
            if(error < -LEFT_MAX)
              error = -LEFT_MAX;
            ftm_pwm_duty(FTM1, FTM_CH0, MID + (int)error);
            all_error = error / K / pid.Kp;
//            all_error = 0;
            break;
            
          case RIGHT_ISLAND_IN_FLAG:
//            stop_flag = 1;
////            right_island_see_flag = 0;
            ftm_pwm_duty(FTM1, FTM_CH0, MID + RIGHT_MAX * 3 / 5);       //右转进右环岛
            all_error = RIGHT_MAX * 3 / 5 / K;
////            all_error_old = all_error;
            break;
            
          case RIGHT_ISLAND_OUT_FLAG:
//            stop_flag = 1;
            ftm_pwm_duty(FTM1, FTM_CH0, MID + RIGHT_MAX * 3 / 5);       //右转出右环岛
            all_error = RIGHT_MAX * 3 / 5 / K;
//            all_error_old = all_error;
            break;
            
          case LEFT_ISLAND_IN_FLAG:
//            stop_flag = 1;
            ftm_pwm_duty(FTM1, FTM_CH0, MID - LEFT_MAX * 3 / 5);        //左转进左环岛
            all_error = LEFT_MAX * 3 / 5 / K;
//            all_error_old = all_error;
            break;
          
          case LEFT_ISLAND_OUT_FLAG:
//            stop_flag = 1;
            ftm_pwm_duty(FTM1, FTM_CH0, MID - LEFT_MAX * 3 / 5);        //左转出右环岛
            all_error = LEFT_MAX * 3 / 5 / K;
//            all_error_old = all_error;
            break;
            
          default:
            break;
        }
      }
      
      if(stop_flag == 1)             //停车
      {
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
        ftm_pwm_duty(FTM0, FTM_CH1, 0);
      }
      
      if(flag != RIGHT_ISLAND_OUT_FLAG && right_island_out_flag == 1)   //出环岛后把所有标志位清零 准备下一次进环岛
      {
        pit_close(PIT1);
        disable_irq(PIT1_IRQn);
        right_island_see_flag = 0;
        right_island_in_flag = 0;
        right_island_out_flag = 0;
        island_in_count = 0;
        island_out_en_flag = 0;
        island_see_count = 0;
        island_in_en_flag = 0;
      }
      if(flag != LEFT_ISLAND_OUT_FLAG && left_island_out_flag == 1)
      {
        pit_close(PIT1);
        disable_irq(PIT1_IRQn);
        left_island_see_flag = 0;
        left_island_in_flag = 0;
        left_island_out_flag = 0;
        island_in_count = 0;
        island_out_en_flag = 0;
        island_see_count = 0;
        island_in_en_flag = 0;
      }
    
    /********** 电机闭环控制 **********/
     if(stop_flag != 1)           //若没有停车标志位
    {
      if(key1 == 0)
      {
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
        motor_speed = 240;
        feedback_speed = CloseLoop_PID_realize(motor_speed);
        ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
      }
      if(key3 == 0)
      {
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
        motor_speed = 260;
        feedback_speed = CloseLoop_PID_realize(motor_speed);
        ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
      }
      if(key4 == 0)
      {
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
        motor_speed = 280;
        feedback_speed = CloseLoop_PID_realize(motor_speed);
        ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
      }
      if(key2 == 0)
      {
        if( ( all_error >= 0 && all_error < 10 ) || ( all_error > -10 && all_error <= 0 ) )
        {
          angle_counter = 0;
          ftm_pwm_duty(FTM0, FTM_CH0, 0);
          motor_speed = 340 + ( accelerate_num * 1 );
          if(motor_speed > 345)
            motor_speed = 345;
          feedback_speed = CloseLoop_PID_realize(motor_speed);
          ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
          accelerate_num += 1;
        }
        else if( ( all_error >= 10 && all_error < 18 ) || ( all_error > -18 && all_error <= -10 ) )
        {
          if(angle_counter <= 7)
            ftm_pwm_duty(FTM0, FTM_CH0, (int)(closeloop_pid.ActualValue / 1.3));
          else
            ftm_pwm_duty(FTM0, FTM_CH0, 0);
          accelerate_num = 0;
          motor_speed = 320;
          feedback_speed = CloseLoop_PID_realize(motor_speed);
          ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
          angle_counter += 1;
        }
        else if( ( all_error >= 18 && all_error < 25 ) || ( all_error > -25 && all_error <= -18 ) )
        {
          if(angle_counter <= 9)
            ftm_pwm_duty(FTM0, FTM_CH0, (int)(closeloop_pid.ActualValue / 2));
          else
            ftm_pwm_duty(FTM0, FTM_CH0, 0);
          accelerate_num = 0;
          motor_speed = 310;
          feedback_speed = CloseLoop_PID_realize(motor_speed);
          ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
          angle_counter += 1;
        }
        else if( all_error >= 25 || all_error <= -25 )
        {
          if(angle_counter <= 11)
            ftm_pwm_duty(FTM0, FTM_CH0, (int)(closeloop_pid.ActualValue / 2 ));
          else
            ftm_pwm_duty(FTM0, FTM_CH0, 0);
          accelerate_num = 0;
          motor_speed = 300;
          feedback_speed = CloseLoop_PID_realize(motor_speed);
          ftm_pwm_duty(FTM0, FTM_CH1, (int)(motor_speed + feedback_speed));
          angle_counter += 1;
        }
      }
    }
//      printf("angle_counter=%d\n", angle_counter);
//    printf("all_error=%d.%d\n", (int)all_error, abs((int)((all_error - (int)all_error)*100)));
    
      /********** 通过UART3把值发送到上位机显示 **********/
     if(0)
     {
        for(i=0;i<CAMERA_H;i++)
       {
           for(j=0;j<CAMERA_W;j++)
           {
            if(img[i][j] == 0xff)
              img[i][j] = '1';   //1表示白
            else if(img[i][j] == 0x00)
              img[i][j] = '0';   //0表示黑
            printf("%c", img[i][j]);
           }
            printf("    %d\n", i);
       }
        printf("\r\n");
     }
     
     /********** 把图像发送到上位机显示 **********/
     if(0)
     { 
      img_extract(img, imgbuff, CAMERA_SIZE);          //解压为灰度图像，方便发送到上位机显
      vcan_sendimg(imgbuff, sizeof(imgbuff));
     }
     
     /********** 发送有中线的图像 **********/
     if(0)
     {
      img_extract(center, imgbuff, CAMERA_SIZE);
      find_center();
      vcan_sendimg(center, CAMERA_W * CAMERA_H );
     }
//     time = pit_time_get_ms(PIT1);
//     pit_close(PIT1); 
//     printf("time=%d\n", time);
    }
}

/********** 找到中线，在图像上显示 **********/
void find_center()
{  
  int i,j,k;
   uint8 middleline,Left_Black,Left_Black_old,Left_en;
   uint8 Right_Black,Right_Black_old,Right_en;
 for(i=CAMERA_H;i>0;i--)
   {
    for(j=(CAMERA_W/2)-1;j>0;j--)  // 从中间向左边搜索，寻找黑点
    {
      Left_en = 0;
      if(center[i-1][j]==0xff && center[i-1][j-1]==0x00)
      {
        Left_Black = j;  
        Left_Black_old = Left_Black;
        break;
      }
//      else if(center[i-1][j]==0x00 && center[i-1][j-1]==0xff)  //找1黑1白
//      {
//        Left_en = 1;
//        Left_Black = j;
//        Left_Black_old = Left_Black;
//        break;
//      }
      else if(center[i-1][j]==0x00 && center[i-1][j-1]==0x00)  //找2个黑点
      {
        //printf("zuobian");
        for(k=(CAMERA_W/2)-1;k>0;k--)
        {
          if(center[i-1][k]==0x00 && center[i-1][k-1]==0xff)
          {
            Left_en = 1;
            Left_Black = k;
            break;
          }
          else
          {
            Left_Black = CAMERA_W - 1;
          }
        }
        break;
      }
      else
      {
        Left_Black = 0;  // 未找到左边黑点
        Left_Black_old = Left_Black;
      }
    }
    for(j=CAMERA_W/2-1;j<CAMERA_W-1;j++)          // 从中间向右边搜索，寻找黑点
    { 
      Right_en = 0;
      if(center[i-1][j]==0xff  && center[i-1][j+1]==0x00)
      {
        Right_Black = j;         
        Right_Black_old = Right_Black;  
        break; 
      }
//      else if(center[i-1][j]==0x00 && center[i-1][j+1]==0xff)
//      {
//        Right_en = 1;
//        Right_Black = j;         
//        Right_Black_old = Right_Black;  
//        break;
//      }
      else if(center[i-1][j]==0x00 && center[i-1][j+1]==0x00)  //找2个黑点
      {
        //printf("youbian");
        for(k=(CAMERA_W/2);k<CAMERA_W-1;k++)
        {
          if(center[i-1][k]==0x00 && center[i-1][k+1]==0xff)
          {
            Right_en = 1;
            Right_Black = k;
            break;
          }
          else
          {
            Right_Black = 0;
          }
        }
        break;
      }
      else
      {
        Right_Black = CAMERA_W-1;   //未找到右边黑点
        Right_Black_old = Right_Black;  
       }
    }
    if(Left_en == 1 && Right_en == 1)
    {
      Left_Black = Left_Black_old;
      Right_Black = Right_Black_old;
    }
    middleline=(int)((Left_Black + Right_Black)/2);
    center[i-1][middleline]=1;
    //printf("%d",center[i-1][middleline]);
//    if(i>=31 && i <=60)
//    {
//      middleline=(int)((Left_Black + Right_Black)/2);
//      //centerline[i] = middleline;  //中线数组
//      center[i-1][middleline]=1;
//     //printf("%c",center[i][middleline]);
//    }
   }
    //printf("meiyou]n");
}

/********** 找到中线数组 **********/
void find_centerline()
{  
    int i,j,k,q1,q2,q3,q4;
    uint8 middleline,Left_Black,Left_Black_old,Left_en;
    uint8 Right_Black,Right_Black_old,Right_en;
    error1 = 0;
    error2 = 0;
    error3 = 0;
    error4 = 0;
    error5 = 0;
    sum_x1 = 0;
    sum_y1 = 0;
    sum11 = 0;
    sum21 = 0;
    x_average1 = 0;
    y_average1 = 0;
    sum_x2 = 0;
    sum_y2 = 0;
    sum12 = 0;
    sum22 = 0;
    x_average2 = 0;
    y_average2 = 0;
    sum_x3 = 0;
    sum_y3 = 0;
    sum13 = 0;
    sum23 = 0;
    x_average3 = 0;
    y_average3 = 0;
    sum_x4 = 0;
    sum_y4 = 0;
    sum14 = 0;
    sum24 = 0;
    x_average4 = 0;
    y_average4 = 0;
//    sum_x5 = 0;
//    sum_y5 = 0;
//    sum15 = 0;
//    sum25 = 0;
//    x_average5 = 0;
//    y_average5 = 0;
    
  for(i=CAMERA_H;i>0;i--)
   {
    for(j=(CAMERA_W/2)-1;j>0;j--)  // 从中间向左边搜索，寻找黑点
    {
      Left_en = 0;
      if(center[i-1][j]==0xff && center[i-1][j-1]==0x00)
      {
        Left_Black = j;  
        Left_Black_old = Left_Black;
        break;
      }
//      else if(center[i-1][j]==0x00 && center[i-1][j-1]==0xff)  //找1黑1白
//      {
//        Left_en = 1;
//        Left_Black = j;
//        Left_Black_old = Left_Black;
//        break;
//      }
      else if(center[i-1][j]==0x00 && center[i-1][j-1]==0x00)  //找2个黑点
      {
        //printf("zuobian");
        for(k=(CAMERA_W/2)-1;k>0;k--)
        {
          if(center[i-1][k]==0x00 && center[i-1][k-1]==0xff)
          {
            Left_en = 1;
            Left_Black = k;
            break;
          }
          else
          {
            Left_Black = CAMERA_W - 1;
          }
        }
        break;
      }
      else
      {
        Left_Black = 0;  // 未找到左边黑点
        Left_Black_old = Left_Black;
      }
    }
    for(j=CAMERA_W/2-1;j<CAMERA_W-1;j++)          // 从中间向右边搜索，寻找黑点
    { 
      Right_en = 0;
      if(center[i-1][j]==0xff  && center[i-1][j+1]==0x00)
      {
        Right_Black = j;         
        Right_Black_old = Right_Black;  
        break; 
      }
//      else if(center[i-1][j]==0x00 && center[i-1][j+1]==0xff)
//      {
//        Right_en = 1;
//        Right_Black = j;         
//        Right_Black_old = Right_Black;  
//        break;
//      }
      else if(center[i-1][j]==0x00 && center[i-1][j+1]==0x00)  //找2个黑点
      {
        //printf("youbian");
        for(k=(CAMERA_W/2);k<CAMERA_W-1;k++)
        {
          if(center[i-1][k]==0x00 && center[i-1][k+1]==0xff)
          {
            Right_en = 1;
            Right_Black = k;
            break;
          }
          else
          {
            Right_Black = 0;
          }
        }
        break;
      }
      else
      {
        Right_Black = CAMERA_W-1;   //未找到右边黑点
        Right_Black_old = Right_Black;  
       }
    }
    if(Left_en == 1 && Right_en == 1)
    {
      Left_Black = Left_Black_old;
      Right_Black = Right_Black_old;
    }
      middleline=(int)((Left_Black + Right_Black)/2);
      centerline[i-1] = middleline;  //中线数组
      //printf("%d    %d\n", centerline[i-1], i-1);
      if(i >= 11 && i <= 20)
      {
//          q5 = i - 10;
//          sum_x5 += q5;
//          sum_y5 += centerline[i-1];
//          sum15 += (q5 * centerline[i-1]);   //i即x,centerline[i]即y
//          sum25 += (q5 * q5);
          error5 += (centerline[i-1] - 39);
      }
      if(i >= 21 && i <= 30)
      {
          q1 = i - 20;
          sum_x1 += q1;
          sum_y1 += centerline[i-1];
          sum11 += (q1 * centerline[i-1]);   //i即x,centerline[i]即y
          sum21 += (q1 * q1);
          error1 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
          //printf("%d    %d\n", error1, i-1);
      }
      if(i >= 31 && i <= 40)
      {
          q2 = i - 30;
          sum_x2 += q2;
          sum_y2 += centerline[i-1];
          sum12 += (q2 * centerline[i-1]);   //i即x,centerline[i]即y
          sum22 += (q2 * q2);
          error2 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
      }
      if(i >= 41 && i <= 50)
      {
          q3 = i - 40;
          sum_x3 += q3;
          sum_y3 += centerline[i-1];
          sum13 += (q3 * centerline[i-1]);   //i即x,centerline[i]即y
          sum23 += (q3 * q3);
          error3 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
      }
      if(i >= 51 && i <= 60)
      {
          q4 = i - 50;
          sum_x4 += q4;
          sum_y4 += centerline[i-1];
          sum14 += (q4 * centerline[i-1]);   //i即x,centerline[i]即y
          sum24 += (q4 * q4);
          error4 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
      }
   }
      x_average1 = sum_x1/10;
      y_average1 = sum_y1/10;
      x_average2 = sum_x2/10;
      y_average2 = sum_y2/10;
      x_average3 = sum_x3/10;
      y_average3 = sum_y3/10;
      x_average4 = sum_x4/10;
      y_average4 = sum_y4/10;
//      x_average5 = sum_x5/10;
//      y_average5 = sum_y5/10;
      //printf("sum_x=%d\n",sum_x);
      //printf("x_average1=%d,y_average1=%d,sum11=%d,sum21=%d\n", (int)x_average1,(int)y_average1,sum11,sum21);
      //printf("x_average2=%d,y_average2=%d,sum12=%d,sum22=%d\n", (int)x_average2,(int)y_average2,sum12,sum22);
}

void cal_angle()
{
  b1 = (sum11 - 10 * x_average1 * y_average1)/(sum21 - 10 * x_average1 * x_average1); //得出斜率
  b2 = (sum12 - 10 * x_average2 * y_average2)/(sum22 - 10 * x_average2 * x_average2);
  b3 = (sum13 - 10 * x_average3 * y_average3)/(sum23 - 10 * x_average3 * x_average3);
  b4 = (sum14 - 10 * x_average4 * y_average4)/(sum24 - 10 * x_average4 * x_average4);
//  b5 = (sum15 - 10 * x_average5 * y_average5)/(sum25 - 10 * x_average5 * x_average5);
  //printf("b1=%d.%d\n", (int)b1, abs((int)((b1 - (int)b1)*100)));
  //printf("b4=%d.%d\n", (int)b4, abs((int)((b4 - (int)b4)*100)));
  //a = y_average - b * x_average;
  if(b1 != 0)
  {
   angle1 = atan(b1) * 180.0 / PI;  //角度
   //printf("angle1=%d.%d\n", (int)angle, abs((int)((angle - (int)angle)*100)));
    if(angle1 > 0 && angle1 < 90)
    {
      angle1 = -angle1;
    }
    else if(angle1 < 0 && angle1 > -90)
    {
      angle1 = -angle1;
    } 
  }
  else if(b1 == 0)
  angle1 = 0.0;
  
  if(b2 != 0)
  {
   angle2 = atan(b2) * 180.0 / PI;  //角度
   //printf("angle1=%d.%d\n", (int)angle, abs((int)((angle - (int)angle)*100)));
    if(angle2 > 0 && angle2 < 90)
    {
      angle2 = -angle2;
    }
    else if(angle2 < 0 && angle2 > -90)
    {
      angle2 = -angle2;
    } 
  }
  else if(b2 == 0)
  angle2 = 0.0;
  
  if(b3 != 0)
  {
   angle3 = atan(b3) * 180.0 / PI;  //角度
   //printf("angle1=%d.%d\n", (int)angle, abs((int)((angle - (int)angle)*100)));
    if(angle3 > 0 && angle3 < 90)
    {
      angle3 = -angle3;
    }
    else if(angle3 < 0 && angle3 > -90)
    {
      angle3 = -angle3;
    } 
  }
  else if(b3 == 0)
  angle3 = 0.0;
  
  if(b4 != 0)
  {
   angle4 = atan(b4) * 180.0 / PI;  //角度
   //printf("angle1=%d.%d\n", (int)angle, abs((int)((angle - (int)angle)*100)));
    if(angle4 > 0 && angle4 < 90)
    {
      angle4 = -angle4;
    }
    else if(angle4 < 0 && angle4 > -90)
    {
      angle4 = -angle4;
    } 
  }
  else if(b4 == 0)
  angle4 = 0.0;
  
//  if(b5 != 0)
//  {
//   angle5 = atan(b5) * 180.0 / PI;  //角度
//   //printf("angle1=%d.%d\n", (int)angle, abs((int)((angle - (int)angle)*100)));
//    if(angle5 > 0 && angle5 < 90)
//    {
//      angle5 = -angle5;
//    }
//    else if(angle5 < 0 && angle5 > -90)
//    {
//      angle5 = -angle5;
//    } 
//  }
//  else if(b5 == 0)
//  angle5 = 0.0;
  
  angle = (angle4 * 2.55 + angle3 * 1.33 + angle2 * 0.75) / (2.55 + 1.33 + 0.75);
  all_error = (error4 * 2.55 + error3 * 1.33 + error2 * 0.75) / (2.55 + 1.33 + 0.75);  //all_error = (error4 * 2.55 + error3 * 1.33 + error2 * 0.75 + error1 * 0.1) / (2.55 + 1.33 + 0.75 + 0.1);
  //printf("all_error2=%d.%d\n", (int)all_error, abs((int)((all_error - (int)all_error)*100)));
  all_error = ( all_error / 4.5 * 0.6 + angle * 0.4 );
  forsee = (error1 * 0.8 + error5 * 0.2) / 4.5;
  all_error = all_error * 1 + forsee * 0;
//  if(all_error < 0)
//    all_error = all_error * 1.25;
}

void scan()
{
  int i,j,white_num1,white_num2,col_white,row_white1,row_white2,white_point;
  uint8 black_point[CAMERA_H],circle_num,circle_white_line,num,zebra_num,black_num1,row_black1;
  uint8 right_island_point[CAMERA_H],left_island_point[CAMERA_H],right_island_num,right_island_white_num1,right_island_white_num2,half_row_white,half_col_white,add_num;
  uint8 right_black_add_point[CAMERA_H],right_island_row_black1;
  uint8 right_black_to_white,right_white_to_black,right_island_black,rightmost_black_5;
  uint8 leftmost_black_num,rightmost_white_num,downmost_white_num,right_len_max;
  uint8 in_rightmost_white_num,in_leftmost_black_num,right_island_out_black_num;
  uint8 island_out_white,right_island_see_black,right_white_to_black1,island_out_white1;
  uint16 black_num;
  uint8 col_white1,right_island_in_white_num,right_island_in_white_line,right_white_line_num,right_island_black_counter[CAMERA_H],right_island_55_black;
  uint8 black_2_white_num,barrier_num1,right_island_row_black_line1,left_island_row_black_line1,leftmost_black_5,long_right_black_to_white,right_island_minus_num;
  uint8 white_2_black_num,barrier_num,barrier_white_point,barrier_white_line,barrier_row_white_line,island_row_black_line;
  uint8 left_white_to_black,left_black_to_white,left_island_num,left_len_max,leftmost_white_num;
  uint8 right_island_white_point,left_island_white_point,right_short_white_num,right_short_white_line,left_short_white_line,left_short_white_num;
  uint16 right_quarter_black_num,left_quarter_black_num;
  col_white = 0;
  row_white1 = 0;
  row_white2 = 0;
  row_black1 = 0;
  half_row_white = 0;
  half_col_white = 0;
  white_point = 0;
  circle_num = 0;
  right_island_num = 0;
  black_num = 0;
  circle_white_line = 0;
  zebra_num = 0;
  add_num = 0;
  right_white_to_black = 0;
  right_black_to_white = 0;
  right_island_black = 0;
  leftmost_black_num = 0;
  rightmost_white_num = 0;
  downmost_white_num = 0;
  right_len_max = 0;
  right_island_row_black1 = 0;
  rightmost_black_5 = 0;
  in_rightmost_white_num = 0;
  in_leftmost_black_num = 0;
  island_out_white = 0;
  right_island_out_black_num = 0;
  right_island_see_black = 0;
  right_white_to_black1 = 0;
  island_out_white1 = 0;
  col_white1 = 0;
  right_island_in_white_num = 0;
  right_island_in_white_line = 0;
  right_white_line_num = 0;
  right_island_55_black = 0;
  barrier_num = 0;
  right_island_white_point = 0;
  left_island_white_point = 0;
//  barrier_left_black_num = 0;
//  barrier_right_black_num = 0;
  barrier_white_line = 0;
  barrier_row_white_line = 0;
  island_row_black_line = 0;
  right_island_row_black_line1 = 0;
  left_island_row_black_line1 = 0;
  left_white_to_black = 0;
  left_black_to_white = 0;
  left_island_num = 0;
  left_len_max = 0;
  leftmost_white_num = 0;
  leftmost_black_5 = 0;
  long_right_black_to_white = 0;
  right_island_minus_num = 0;
  right_short_white_line = 0;
  left_short_white_line = 0;
  right_quarter_black_num = 0;
  left_quarter_black_num = 0;
  for(i=0;i<CAMERA_H;i++)      //行扫
  {
    num = 0;                //斑马线数目
    black_point[i] = 0;
    right_island_point[i] = 0;
    left_island_point[i] = 0;
    right_black_add_point[i] = 0;
    right_island_white_num1 = 0;
    white_num1 = 0;
    black_num1 = 0;
    right_island_black_counter[i] = 0;
    white_2_black_num = 0;
    black_2_white_num = 0;
    barrier_white_point = 0;
    barrier_num1 = 0;
    right_short_white_num = 0;
    left_short_white_num = 0;
    for(j=0;j<CAMERA_W;j++)
    {
      if(center[i][j] == 0xff) //(j == 79 && center[i][j] == 0xff && i >= 31 && i <= 50)
      {
        white_num1 += 1;
        if(i >= 55 && i <= 59 && j == 79)
          rightmost_white_num += 1;
        if(i >= 55 && i <= 59 && j == 0)
          leftmost_white_num += 1;
        if(i >= 30 && i <= 59 && j == 79)         //进右环岛最后边白点个数
          in_rightmost_white_num += 1;
        if(i == 59)
          downmost_white_num += 1;
        if(i >= 30 && i <= 59)
          right_island_white_num1 += 1;
        if(i <= CAMERA_H/2 - 1)
          white_point += 1;
        if(i >= 30 & i <= 59 && j >= 30 && j <= 79)
          right_island_in_white_num += 1;
        if(i >= 50 && i <= 59 && j >= 15 && j <= 64)
          barrier_white_point += 1;
      }
      
      if(center[i][j] == 0x00) //(j == 0 && center[i][j] == 0x00 && i >= 31 && i <= 50)
      {
        black_num1 += 1;
        if(j >= 0 && j <= 20)
          right_black_add_point[i] += 1;
        if(i >= 50 && i <= 59 && j == 0)
          leftmost_black_num += 1;
        if(i >= 30 && i <= 59 && j == 0)        //进右环岛最左边黑点个数
          in_leftmost_black_num += 1;
        if(j == 79 && i <= 59 && i >= 55)
          rightmost_black_5 += 1;
        if(j == 0 && i <= 59 && i >= 55)
          leftmost_black_5 += 1;
        if(j == 0)
          right_island_black += 1;
        if(i >= 20 && i <= 30 && j >= 25 && j <= 35)
          right_island_see_black += 1;
        if(i <= 30 && i >= 20)
          black_num += 1;
        if(i <= 50 && i >= 10) 
          black_point[i] += 1;
        if(j <= 79 && j >= 40)
          right_island_point[i] += 1;
        if(j <= 39 && j >= 0)
          left_island_point[i] += 1;
        if(i >= 30 && i <= 59 && j >= 0 && j <= 19)
          right_island_black_counter[i] += 1;
        if(i >= 55 && i <= 59 && j >= 75 && j <= 79)
          right_island_55_black += 1;
        if(i >= 30 && i <= 59 && j >= 40 && j <= 79)
          right_quarter_black_num += 1;
        if(i >= 30 && i <= 59 && j >= 0 && j <= 39)
          left_quarter_black_num += 1;
//        if(i >= 50 && i <= 54 && j >= 20 && j <= 24)
//          barrier_left_black_num += 1;
//        if(i >= 50 && i <= 54 && j >= 55 && j <= 59)
//          barrier_right_black_num += 1;
      }
      if(j >= 40 && j <= 59 && i >= 4 && i <= 15 && center[i][j] == 0xff && center[i][j+1] == 0xff)
        right_short_white_num += 1;
      
      if(j >= 20 && j <= 39 && i >= 4 && i <= 15 && center[i][j] == 0xff && center[i][j+1] == 0xff)
        left_short_white_num += 1;
      
      if( j >= 65 && j <= 79 && center[i][j] == 0xff && center[i-1][j] == 0x00 && i > 0 )
        right_white_to_black += 1;
      
      if( j >= 0 && j <= 14 && center[i][j] == 0xff && center[i-1][j] == 0x00 && i > 0 )
        left_white_to_black += 1;
      
//      if( j >= 17 && j <= 22 && i < 59 && center[i][j] == 0xff && center[i+1][j] == 0x00 )
//        right_white_to_black1 += 1;
      
      if( j >= 65 && j <= 79 && ( i < CAMERA_H - 1 ) && center[i][j] == 0x00 && center[i+1][j] == 0xff )
        right_black_to_white += 1;
      
      if( j >= 0 && j <= 14 && ( i < CAMERA_H - 1 ) && center[i][j] == 0x00 && center[i+1][j] == 0xff )
        left_black_to_white += 1;
      
      if( i <= CAMERA_H - 1 && i >= CAMERA_H - 3 && j > 0 )    //找斑马线
      {
        if( center[i][j-1] == 0xff && center[i][j] == 0x00 )
          num += 1;  //数黑白条数
      }
      if(center[i][j] == 0xff && center[i][j+1] == 0x00)
      {
        if(i >= 35 && i <= 59 && j < CAMERA_W - 1)
          white_2_black_num += 1;
      }
      if(center[i][j] == 0x00 && center[i][j+1] == 0xff)
      {
        if(i >= 35 && i <= 59 && j < CAMERA_W - 1)
          black_2_white_num += 1;
      }
    }
    if(right_short_white_num >= 15)
      right_short_white_line = 1;
    
    if(left_short_white_num >= 15)
      left_short_white_line = 1;
    
    if(num >= 6)  //数到6条以上认定为斑马线
      zebra_num += 1;
    
    if(white_2_black_num == 2)
      barrier_num += 1;
    
    if(black_2_white_num == 2)
      barrier_num1 += 1;
    
    if(barrier_white_point == 50)
      barrier_white_line += 1;
      
    if(right_island_point[i] < right_island_point[i-1] && i >= 25 && i <= 49 && center[i][79] == 0x00 && center[i-1][79] == 0x00)  //右环岛黑点递增
      right_island_num += 1;
    
//    if(right_island_point[i] > right_island_point[i-1] && i >= 14 && i <= 25 && center[i][79] == 0x00 && center[i-1][79] == 0x00)  //右环岛黑点递减
//      right_island_minus_num += 1;
    
    if(left_island_point[i] < left_island_point[i-1] && i >= 25 && i <= 49 && center[i][0] == 0x00 && center[i-1][0] == 0x00)      //左环岛黑点递增
      left_island_num += 1;
    
//    if(right_island_point[i] > right_island_point[i-1] && i >= 15 && i <= 24)  //右环岛黑点递增
//      right_island_minus_num += 1;
    
    if( right_black_add_point[i] > right_black_add_point[i-1] && i >= 30 && i <= 59)    //左边黑点递增
      add_num += 1;
    
    if(right_island_white_num1 < 80 && right_island_white_num1 >= 70)                   //一半白线条数
      half_row_white += 1;
      
    if( i <= 50 && i >= 10 )   //圆环递增
    {
      if(black_point[i] < black_point[i-1])
            circle_num += 1;
    }
    //printf("black_point=%d, i=%d\n", black_point[i], i);
    //printf("white_num1=%d,i=%d\n", white_num1, i);
//    printf("circle_num=%d,i=%d\n", circle_num, i);
    if(black_num1 == CAMERA_W)
    {
      row_black1 += 1;
      if(i >= 31 && i <= 40)
        right_island_row_black1 += 1;
    }
    
    if(black_num1 >= 70 && i >=0 && i <= 29)
        right_island_out_black_num += 1;
    
    if(white_num1 == CAMERA_W) // ( i >= 7 && i <= 74 ) && 
    {
      if(i >= 7 && i <= 52)
        island_out_white += 1;
      if(i >= 55 && i <= 59)
        circle_white_line += 1;
      if(i >= 7 && i <= 54)
        row_white1 += 1;
      if(i >= 30 && i <= 59)
        right_white_line_num += 1;
      else if(i >= 0 && i <= 6 || i >= 59 && i >= 55)
        row_white2 += 1;
    }
    
    if(white_num1 >= 70 && white_num1 <= 80 &&  i >= 31 && i <= 59)
      island_out_white1 += 1;
    
    if(right_island_in_white_num >= 40)
      right_island_in_white_line += 1;

  }
  right_len_max = right_island_point[25];
  left_len_max = left_island_point[25];
//  printf("right_island_in_white_line=%d\n", right_island_in_white_line);
  
 
  
   /********** 斑马线识别 **********/
   if(zebra_num == 3)          //数3条斑马线
    {
      start_new = 1;
      start_old = start_new;
    }
    else
      start_new = 0;
    if(start_new == 0 && start_old == 1)
      start_flag = 1;
   /*******************************/
    
//    printf("num=%d\n", num);
//    printf("start_new=%d\n", start_new);
//    printf("start_old=%d\n", start_old);
//    printf("start_flag=%d\n", start_flag);
//    printf("circle_white_line=%d\n", circle_white_line);
//    printf("black_num=%d\n", black_num);
    
  for(j=0;j<CAMERA_W;j++)    //列扫
  {
    white_num2 = 0;
    right_island_white_num2 = 0;
    for(i=0;i<CAMERA_H;i++)
    {
      if(center[i][j] == 0xff)
        white_num2 += 1;
      
      if(center[i][j] == 0xff)
        right_island_white_num2 += 1;
      
      if( i >= 30 && i <= 39 && j >= 0 && j <= 39  && center[i][j] == 0x00 && center[i][j+1] == 0xff )
        long_right_black_to_white += 1;
      
      if(center[i][j] == 0xff && j >= 60 && j <= 79 && i >= 7 && i <= 19)
        right_island_white_point += 1;
      
      if(center[i][j] == 0xff && j >= 0 && j <= 19 && i >= 7 && i <= 19)
        left_island_white_point += 1;
    }
    if(white_num2 == CAMERA_H)
    {
      col_white += 1;
      if(j >= 38 && j <= 42)
        col_white1 += 1;
    }
    if(white_num2 >= 50 && j >= 20 && j <= 59)
      barrier_row_white_line += 1;
    
    if(right_island_white_num2 >= 50 && j >= 20 && j <= 50)
      half_col_white += 1;
    
    if(white_num2 >= 51)
      island_row_black_line += 1;
    
    if(white_num2 >= 56 && j >= 10 && j <= 40) // 32  47
      right_island_row_black_line1 += 1;
    
    if(white_num2 >= 56 && j >= 39 && j <= 69)
      left_island_row_black_line1 += 1;
    
  }

  //   if( (( right_white_line_num >= 1 && (right_island_one_black == 1 && right_island_two_black == 1) || (right_island_three_black == 1 && right_island_two_black)) || right_white_line_num == 0 )
//      && right_island_num >= 5 && right_white_to_black == 20 && right_black_to_white == 20 && rightmost_white_num == 10 && downmost_white_num >= 70 && right_len_max >= 10 
//      && right_len_max <= 40 && right_island_row_black1 == 0 && rightmost_black_5 >= 3 && rightmost_black_5 <= 10 && right_white_to_black1 == 0 && right_island_55_black == 0)   //看到右环岛标志位
    if( right_white_to_black >= 25 && right_black_to_white >= 25 && right_island_num >= 15 && rightmost_white_num == 5 && right_short_white_line == 1
        && right_len_max >= 15 && right_len_max <= 40 && right_island_row_black_line1 >= 5 && right_island_row_black_line1 <= 20 && right_island_white_point >= 15 ) //island_out_white == 0 &&  && rightmost_white_num < 5   // && rightmost_white_num >= 3
    {
      // && long_right_black_to_white >= 6   && right_island_minus_num >= 0 
      right_island_see_flag = 1;   //(( right_white_to_black >= 10 && right_white_to_black <= 12 &&  right_black_to_white >= 10 && right_black_to_white <= 12) || ( right_white_to_black >= 18 && right_black_to_white >= 18 ))
//         island_see_count += 1;
    }
  
    if( left_white_to_black >= 25 && left_black_to_white >= 25 && left_island_num >= 15 && leftmost_white_num == 5 && left_short_white_line == 1
        && left_len_max >= 15 && left_len_max <= 40 && left_island_row_black_line1 >= 5 && left_island_row_black_line1 <= 20 && left_island_white_point >= 15 )  //island_out_white == 0 && && leftmost_white_num < 5   // && leftmost_white_num >= 3
    {
      left_island_see_flag = 1; 
//      island_see_count += 1;
    }
//    printf("half_row_white=%d\n", half_row_white);
//    printf("right_island_num=%d\n", right_island_num);
  if(0)
  {
//    printf("right_white_to_black=%d\n", right_white_to_black);
//    printf("right_black_to_white=%d\n", right_black_to_white);
//    printf("right_island_num=%d\n", right_island_num);
//    printf("right_island_minus_num=%d\n", right_island_minus_num);
//    printf("rightmost_white_num=%d\n", rightmost_white_num);
//    printf("long_right_black_to_white=%d\n", long_right_black_to_white);
//    printf("right_len_max=%d\n", right_len_max);
//    printf("island_row_black_line1=%d\n", island_row_black_line1);
//    printf("island_row_black_line=%d\n", island_row_black_line);
//    printf("island_out_white=%d\n\n", island_out_white);
    printf("island_in_count=%d\n", island_in_count);
    printf("island_out_en_flag=%d\n", island_out_en_flag);
    printf("col_white1=%d\n", col_white1);
    printf("island_out_white=%d\n", island_out_white);
    printf("right_island_see_flag=%d\n", right_island_see_flag);
    printf("right_island_in_flag=%d\n", right_island_in_flag);
    printf("right_island_out_flag=%d\n\n", right_island_out_flag);
//    printf("island_out_white1=%d\n", island_out_white1);
//    printf("island_row_black_line=%d\n\n", island_row_black_line);
  }

/********** 标志位判定 **********/
//  if (row_white1 >= 3 && circle_num >= 3 && black_num >= 500 && row_black1 == 0)
//    flag = CIRCLE_FLAG;
  if(start_flag == 1 && start_new == 1 && start_old == 1)
    flag = STOP_FLAG;
  else if(right_island_see_flag == 1 && right_island_in_flag == 1 && island_out_en_flag == 1 && island_out_white >= 3 && col_white1 == 0)  //right_island_out_black >= 17 // && row_black1 >= 5
  {
    flag = RIGHT_ISLAND_OUT_FLAG;
    right_island_out_flag = 1;
  }
  else if(left_island_see_flag == 1 && left_island_in_flag == 1 && island_out_en_flag == 1 && island_out_white >= 3 && col_white1 == 0)  //right_island_out_black >= 17  //row_black1 >= 5 &&
  {
    flag = LEFT_ISLAND_OUT_FLAG;
    left_island_out_flag = 1;
  }
//  else if(right_island_see_flag == 1)  
//  {
//    flag = RIGHT_ISLAND_IN_FLAG;
////    right_island_in_flag = 1;
//  }
//  else if(left_island_see_flag == 1)  
//  {
//    flag = LEFT_ISLAND_IN_FLAG;
////    right_island_in_flag = 1;
//  }
  else if(right_island_see_flag == 1 && island_out_white1 >= 10 && island_row_black_line >= 10 && right_quarter_black_num <= 80)// && right_white_to_black < 25 && right_black_to_white < 25 && rightmost_black_5 <= 1
  {
    flag = RIGHT_ISLAND_IN_FLAG;
    right_island_in_flag = 1;
    island_in_count += 1;
  }
  else if(left_island_see_flag == 1 && island_out_white1 >= 10 && island_row_black_line >= 10 && left_quarter_black_num <= 80)//ol_white1 == 0   && && add_num >= 10 && rightmost_black_5 <= 4 //( island_out_white >= 8 || island_out_white1 >= 4  ) && right_island_out_black <= 2 )
  {
    flag = LEFT_ISLAND_IN_FLAG;
    left_island_in_flag = 1;
    island_in_count += 1;
  }
  else if((barrier_num >= 3 || barrier_num1 >= 3) && barrier_row_white_line >= 10)     //&& barrier_white_line >= 8 && barrier_white_line <= 10 && 
    flag = BARRIER_FLAG;
  else if(row_white1 >= 3 && black_num < 500)   
    flag = CROSS_FLAG;
  else if(row_white2 >= 3)//(row_white2 >= 3 || white_point >= 950)
    flag = BLANCK_FLAG;
  else
    flag = NORMAL_FLAG;
  
//  if(island_see_count == 1)
//  {
//    enable_irq(PIT0_IRQn);
//  }
  if(island_in_count == 1)
  {
    pit_init_ms(PIT1, 2000);
    set_vector_handler(PIT1_VECTORn , PIT1_IRQHandler);
    set_irq_priority(PIT1_IRQn, 3);
    enable_irq(PIT1_IRQn);
    island_in_count += 1;
  }
}

void PID_init()
{
  pid.SetValue = 0.0;
  pid.ActualValue = all_error;
  pid.err = 0.0;
  pid.pre_err = 0.0;
  pid.duty = 0.0;
  pid.Kp = 1.15;        //1.0       1.2    1.15
  pid.Kd = 0.1;         //0.068     0.15      0.1
  pid.Ki = 0.0;
}

void CloseLoop_PID_init()
{
  closeloop_pid.SetValue = 0.0;
  closeloop_pid.ActualValue = val / 2;
  closeloop_pid.err = 0.0;
  closeloop_pid.pre_err = 0.0;
  closeloop_pid.duty = 0.0;
  closeloop_pid.Kp = 1.0;
  closeloop_pid.Kd = 0.0;
  closeloop_pid.Ki = 0.0;
}

float CloseLoop_PID_realize(float speed)
{
  closeloop_pid.SetValue = speed;
  closeloop_pid.err = closeloop_pid.SetValue - closeloop_pid.ActualValue;
  closeloop_pid.duty = closeloop_pid.Kp * closeloop_pid.err + closeloop_pid.Kd * (closeloop_pid.err - closeloop_pid.pre_err);
  closeloop_pid.pre_err = closeloop_pid.err;
  closeloop_pid.ActualValue = closeloop_pid.duty;
  return closeloop_pid.ActualValue;
}

float PID_realize(float ang)
{
  pid.SetValue = ang;
  pid.err = pid.SetValue - pid.ActualValue;
  pid.duty = pid.Kp * pid.err + pid.Kd * (pid.err - pid.pre_err);
  pid.pre_err = pid.err;
  pid.ActualValue = pid.duty * K;
  return pid.ActualValue;
}

void PORTA_IRQHandler()
{
    uint8  n;    //引脚号
    uint32 flag;

    while(!PORTA_ISFR);
    flag = PORTA_ISFR;
    PORTA_ISFR  = ~0;                                   //清中断标志位

    n = 29;                                             //场中断
    if(flag & (1 << n))                                 //PTA29触发中断
    {
        camera_vsync();
    }
#if ( CAMERA_USE_HREF == 1 )                            //使用行中断
    n = 28;
    if(flag & (1 << n))                                 //PTA28触发中断
    {
        camera_href();
    }
#endif


}

void PIT2_IRQHandler()
{
  val = ftm_quad_get(FTM2);
  ftm_quad_clean(FTM2);
//  if(val >= 0)
//  {
//    printf("正转:%d\n", val);
//  }
//  else
//  {
//    printf("反转:%d\n", val);
//  }
    PIT_Flag_Clear(PIT2);
}

void DMA0_IRQHandler()
{
    camera_dma();
}

//void PIT0_IRQHandler(void)
//{
//  island_in_en_flag = 1;
////  island_see_count = 0;
////  printf("PIT0\n\n");
//  PIT_Flag_Clear(PIT0);
//}

void PIT1_IRQHandler(void)
{
  island_out_en_flag = 1;
  island_in_count += 1;
//  printf("time=%d", pit_time_get_ms(PIT1));
//  island_in_count = 0;
//  disable_irq(PIT1_IRQn);
  PIT_Flag_Clear(PIT1);
//  disable_irq(PIT1_IRQn);
//  printf("PIT1\n");
//  printf("PIT1\n");
//  PIT_Flag_Clear(PIT1);
//  disable_irq(PIT1_IRQn);
}


