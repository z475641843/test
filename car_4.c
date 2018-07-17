#include "common.h"
#include "include.h"
#include "math.h"

#define MID             4720    //舵机中值占空比3550         300HZ 5950  180HZ 3550
#define RIGHT_MAX       900     //舵机相对中值向右转最大值         1650        1000    
#define LEFT_MAX        930     //舵机相对中值向左转最大值         2100        1250    
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

#define X_AVERAGE               5.5     //X平均值
#define X_AVERAGE2_10           302.5   //X平均值的平方乘10
#define X_SUM                   385     //X*X总和   

#define K                       22
#define SERVO_KP                1.15    //舵机P值  1.25   1.2
#define SERVO_KD                14      //舵机D值  0.0    8
  
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
float set_speed = 0;           //电机设定值     
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
//uint8 island_see_count = 0;
uint8 island_in_count = 0;
//uint8 island_in_en_flag = 0;
uint8 island_out_en_flag = 0;

uint32 time = 0;                //测试主程序用时
uint8 key1;
uint8 key2;
uint8 key3;
uint8 key4;

float sum_y1 = 0;
uint32 sum11 = 0;
float y_average1 = 0;

float sum_y2 = 0;
uint32 sum12 = 0;
float y_average2 = 0;

float sum_y3 = 0;
uint32 sum13 = 0;
float y_average3 = 0;

float sum_y4 = 0;
uint32 sum14 = 0;
float y_average4 = 0;

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

float pre_err = 0;              //偏差旧值        

//函数声明
void PORTA_IRQHandler();
void DMA0_IRQHandler();
void PIT2_IRQHandler();
void PIT0_IRQHandler();
void PIT1_IRQHandler();
void find_center();
void find_centerline();
void cal_angle();
void scan();

void  main(void)
{
    uint8 i,j;    
    
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
//      pit_time_start(PIT0);
      camera_get_img();                              //摄像头获取图像   8-10ms 
      
       if(1)
       {
        img_extract(center, imgbuff, CAMERA_SIZE);  //把图像解压到寻中线数组center中   不到1ms

        scan();                 //斑马线识别

//        time = 0;
//        pit_time_start(PIT0);
        find_centerline();      //得出中线数组,并得到x_average,y_average等   1-2ms
//        time = pit_time_get_ms(PIT0);
//        pit_close(PIT0); 
//        printf("time=%dms\n", time);

        cal_angle();            //算出斜率的角度


//        PID_init();             //舵机pid初始化
//        
//        CloseLoop_PID_init();   //电机pid初始化
        
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
//        if( ( all_error >= 0 && all_error < 10 ) || ( all_error > -10 && all_error < 0 ) )
//            {
//              pid.Kp = 0.3;
////              pid.ActualValue = all_error * 0.7 + forsee * 0.3;
//            }
//         else
//            {
//              pid.Kp = 1.15;
//            }

        switch(flag)
        {
          case NORMAL_FLAG: 
//            if( ( all_error >= 0 && all_error < 10 ) || ( all_error > -10 && all_error < 0 ) )
//            {
////              pid.Kp = 0.3;
//              duty = (int)( ( all_error * 0.5 + ( all_error - pre_err ) * SERVO_KD ) * K );
////              pid.ActualValue = all_error * 0.7 + forsee * 0.3;
//            }
//            else
//            {
//              duty = (int)( ( all_error * SERVO_KP + ( all_error - pre_err ) * SERVO_KD ) * K );
////              pid.Kp = 1.15;
//            }
            duty = (int)( ( all_error * SERVO_KP + ( all_error - pre_err ) * SERVO_KD ) * K );
            pre_err = all_error;
    //        duty = (int)PID_realize(0.0);   //得到实际占空比
            if(duty > RIGHT_MAX)
              duty = RIGHT_MAX;
            if(duty < -LEFT_MAX)
              duty = -LEFT_MAX;
            ftm_pwm_duty(FTM1, FTM_CH0, MID + duty); //error1为20到30行   error5为10到20行
            break;
            
          case STOP_FLAG: 
            stop_flag = 1;
            break;
            
          case RIGHT_ISLAND_IN_FLAG:
            ftm_pwm_duty(FTM1, FTM_CH0, MID + RIGHT_MAX);
            break;
            
          case LEFT_ISLAND_IN_FLAG:
            ftm_pwm_duty(FTM1, FTM_CH0, MID - LEFT_MAX);
            break;
            
          case RIGHT_ISLAND_OUT_FLAG:
            ftm_pwm_duty(FTM1, FTM_CH0, MID + RIGHT_MAX * 3 / 5);       //右转进右环岛
            break;
          
          case LEFT_ISLAND_OUT_FLAG:
            ftm_pwm_duty(FTM1, FTM_CH0, MID - LEFT_MAX * 3 / 5);       //右转进右环岛
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
        right_island_in_flag = 0;
        right_island_out_flag = 0;
        island_in_count = 0;
        island_out_en_flag = 0;
      }
      if(flag != LEFT_ISLAND_OUT_FLAG && left_island_out_flag == 1)
      {
        pit_close(PIT1);
        disable_irq(PIT1_IRQn);
        left_island_in_flag = 0;
        left_island_out_flag = 0;
        island_in_count = 0;
        island_out_en_flag = 0;
      }
    
    /********** 电机闭环控制 **********/
    if(stop_flag != 1)           //若没有停车标志位
    {
      if(key1 == 0)
      {
        set_speed = 360;
        feedback_speed = set_speed - val / 2;
        motor_speed = set_speed + feedback_speed;
        if(motor_speed > 500)
          motor_speed = 500;
        if(motor_speed < 0)
          motor_speed = 0;
        ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
      }
      if(key3 == 0)
      {
        set_speed = 380;
        feedback_speed = set_speed - val / 2;
        motor_speed = set_speed + feedback_speed;
        if(motor_speed > 500)
          motor_speed = 500;
        if(motor_speed < 0)
          motor_speed = 0;
        ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
      }
      if(key4 == 0)
      {
        set_speed = 400;
        feedback_speed = set_speed - val / 2;
        motor_speed = set_speed + feedback_speed;
        if(motor_speed > 500)
          motor_speed = 500;
        if(motor_speed < 0)
          motor_speed = 0;
        ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
        ftm_pwm_duty(FTM0, FTM_CH0, 0);
      }
//      if(key2 == 0)
//      {
//        if( ( all_error >= 0 && all_error < 10 ) || ( all_error > -10 && all_error <= 0 ) )
//        {
//          angle_counter = 0;
//          ftm_pwm_duty(FTM0, FTM_CH0, 0);
//          set_speed = 340 + ( accelerate_num * 1 );
//          if(set_speed > 345)
//            set_speed = 345;
//          feedback_speed = CloseLoop_PID_realize(set_speed);
//          motor_speed = set_speed + feedback_speed;
//          if(motor_speed > 400)
//            motor_speed = 400;
//          if(motor_speed < 0)
//            motor_speed = 0;
//          ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
//          accelerate_num += 1;
//        }
//        else if( ( all_error >= 10 && all_error < 18 ) || ( all_error > -18 && all_error <= -10 ) )
//        {
//          if(angle_counter <= 7)
//            ftm_pwm_duty(FTM0, FTM_CH0, (int)(closeloop_pid.ActualValue / 1.3));
//          else
//            ftm_pwm_duty(FTM0, FTM_CH0, 0);
//          accelerate_num = 0;
//          set_speed = 320;
//          feedback_speed = CloseLoop_PID_realize(set_speed);
//          motor_speed = set_speed + feedback_speed;
//          if(motor_speed > 400)
//            motor_speed = 400;
//          if(motor_speed < 0)
//            motor_speed = 0;
//          ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
//          angle_counter += 1;
//        }
//        else if( ( all_error >= 18 && all_error < 25 ) || ( all_error > -25 && all_error <= -18 ) )
//        {
//          if(angle_counter <= 9)
//            ftm_pwm_duty(FTM0, FTM_CH0, (int)(closeloop_pid.ActualValue / 2));
//          else
//            ftm_pwm_duty(FTM0, FTM_CH0, 0);
//          accelerate_num = 0;
//          set_speed = 310;
//          feedback_speed = CloseLoop_PID_realize(set_speed);
//          motor_speed = set_speed + feedback_speed;
//          if(motor_speed > 400)
//            motor_speed = 400;
//          if(motor_speed < 0)
//            motor_speed = 0;
//          ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
//          angle_counter += 1;
//        }
//        else if( all_error >= 25 || all_error <= -25 )
//        {
//          if(angle_counter <= 11)
//            ftm_pwm_duty(FTM0, FTM_CH0, (int)(closeloop_pid.ActualValue / 2 ));
//          else
//            ftm_pwm_duty(FTM0, FTM_CH0, 0);
//          accelerate_num = 0;
//          set_speed = 300;
//          feedback_speed = CloseLoop_PID_realize(set_speed);
//          motor_speed = set_speed + feedback_speed;
//          if(motor_speed > 400)
//            motor_speed = 400;
//          if(motor_speed < 0)
//            motor_speed = 0;
//          ftm_pwm_duty(FTM0, FTM_CH1, (int)motor_speed);
//          angle_counter += 1;
//        }
//        else
//        {
//          ftm_pwm_duty(FTM0, FTM_CH0, 0);
//          accelerate_num = 0;
//          set_speed = 340;
//          feedback_speed = CloseLoop_PID_realize(set_speed);
//          ftm_pwm_duty(FTM0, FTM_CH1, (int)(set_speed + feedback_speed));
////          angle_counter += 1;
//        }
//      }
    }
//        time = pit_time_get_us(PIT0);
//        pit_close(PIT0); 
//        printf("time=%dms\n", time);
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
      else if(center[i-1][0]==0xff && center[i-1][1]==0xff)
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
      else if(center[i-1][79]==0xff && center[i-1][78]==0xff)
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
   }
}

/********** 找到中线数组 **********/
void find_centerline()
{  
    uint8 i,j,k,q1,q2,q3,q4;
    uint8 middleline,Left_Black,Left_Black_old,Left_en;
    uint8 Right_Black,Right_Black_old,Right_en,left_in,right_in;
    uint8 right_white_num,left_white_num,right_curve,left_curve;
    error1 = 0;
    error2 = 0;
    error3 = 0;
    error4 = 0;
    error5 = 0;
    sum_y1 = 0;
    sum11 = 0;
    y_average1 = 0;
    sum_y2 = 0;
    sum12 = 0;
    y_average2 = 0;
    sum_y3 = 0;
    sum13 = 0;
    y_average3 = 0;
    sum_y4 = 0;
    sum14 = 0;
    y_average4 = 0;
    left_in = 0;
    right_in = 0;
    right_white_num = 0;
    left_white_num = 0;
    right_curve = 0;
    left_curve = 0;
    
  for( i = CAMERA_H; i > 0; i-- )
   {
    for( j = CAMERA_W / 2; j > 0; j-- )       // 从中间向左边搜索，寻找黑点
    {                    //数黑白条数
      Left_en = 0;
      if(center[i-1][j] == 0xff && center[i-1][j-1] == 0x00)
      {
        Left_Black = j; 
        if(i >= 10)
        {
          if(Left_Black - Left_Black_old >= 4)
            left_in += 1;
          if(Left_Black_old > Left_Black)
            left_curve = 1;
        }
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
      else if(center[i-1][j] == 0x00 && center[i-1][j-1] == 0x00)  //找2个黑点
      {
        //printf("zuobian");
        for( k = CAMERA_W / 2 - 1; k > 0; k-- )
        {
          if(center[i-1][k] == 0x00 && center[i-1][k-1] == 0xff)
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
      else if(center[i-1][0] == 0xff && center[i-1][1] == 0xff)
      {
        Left_Black = 0;  // 未找到左边黑点
        Left_Black_old = Left_Black;
      }
    }
//    printf("1L=%d\n", Left_Black);
//    printf("1LO=%d\n\n", Left_Black_old);
    for( j = CAMERA_W / 2 - 1; j < CAMERA_W - 1; j++ )          // 从中间向右边搜索，寻找黑点
    { 
      Right_en = 0;
      if(center[i-1][j] == 0xff  && center[i-1][j+1] == 0x00)
      {
        Right_Black = j;  
        if(i >= 10)
        {
          if(Right_Black_old - Right_Black >= 4)
            right_in += 1;
          if(Right_Black_old < Right_Black)
            right_curve = 1;
        }
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
      else if(center[i-1][j] == 0x00 && center[i-1][j+1] == 0x00)  //找2个黑点
      {
        //printf("youbian");
        for( k = CAMERA_W / 2; k < CAMERA_W - 1; k++ )
        {
          if(center[i-1][k] == 0x00 && center[i-1][k+1] == 0xff)
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
      else if(center[i-1][78] == 0xff && center[i-1][79] == 0xff)
      {
        Right_Black = CAMERA_W-1;   //未找到右边黑点
        Right_Black_old = Right_Black;  
       }
    }
    if(( Right_Black == 79 || Right_Black == 78 )&& i >= 15)
      right_white_num += 1;
    if(( Left_Black == 0 || Left_Black == 1 ) && i >= 15)
      left_white_num += 1;
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
          q4 = i - 20;
          sum_y4 += centerline[i-1];
          sum14 += (q4 * centerline[i-1]);   //i即x,centerline[i]即y
          error4 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
          //printf("%d    %d\n", error1, i-1);
      }
      if(i >= 31 && i <= 40)
      {
          q3 = i - 30;
          sum_y3 += centerline[i-1];
          sum13 += (q3 * centerline[i-1]);   //i即x,centerline[i]即y
          error3 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
      }
      if(i >= 41 && i <= 50)
      {
          q2 = i - 40;
          sum_y2 += centerline[i-1];
          sum12 += (q2 * centerline[i-1]);   //i即x,centerline[i]即y
          error2 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
      }
      if(i >= 51 && i <= 60)
      {
          q1 = i - 50;
          sum_y1 += centerline[i-1];
          sum11 += (q1 * centerline[i-1]);   //i即x,centerline[i]即y
          error1 += (centerline[i-1] - 39);
          //printf("%d    %d\n", centerline[i-1], k);
      }
   }
  /********** 标志位判定 **********/
  if(start_flag == 1 && start_new == 1 && start_old == 1)
    flag = STOP_FLAG;
  else if(left_island_in_flag == 1 && island_out_en_flag == 1 && right_white_num >= 20 && left_white_num >= 20)
  {
    flag = LEFT_ISLAND_OUT_FLAG;
    left_island_out_flag = 1;
  }
  else if(right_island_in_flag == 1 && island_out_en_flag == 1 && right_white_num >= 20 && left_white_num >= 20)
  {
    flag = RIGHT_ISLAND_OUT_FLAG;
    right_island_out_flag = 1;
  }
  
  else if(left_in == 0 && right_in >= 1 && right_white_num >= 20 && right_curve == 0 )
  {
    flag = RIGHT_ISLAND_IN_FLAG;
    right_island_in_flag = 1;
    island_in_count += 1;
  }
  else if(left_in >= 1 && right_in == 0 && left_white_num >= 20 && left_curve == 0 )
  {
    flag = LEFT_ISLAND_IN_FLAG;
    left_island_in_flag = 1;
    island_in_count += 1;
  }
  else
    flag = NORMAL_FLAG;
  if(island_in_count == 1)
  {
    pit_init_ms(PIT1, 1000);
    set_vector_handler(PIT1_VECTORn , PIT1_IRQHandler);
    set_irq_priority(PIT1_IRQn, 3);
    enable_irq(PIT1_IRQn);
    island_in_count += 1;
  }
  /*******************************/
      y_average1 = sum_y1/10;
      y_average2 = sum_y2/10;
      y_average3 = sum_y3/10;
      y_average4 = sum_y4/10;
}

void cal_angle()
{
  b1 = (sum11 - 10 * X_AVERAGE * y_average1)/(X_SUM - X_AVERAGE2_10); //得出斜率
  b2 = (sum12 - 10 * X_AVERAGE * y_average2)/(X_SUM - X_AVERAGE2_10);
  b3 = (sum13 - 10 * X_AVERAGE * y_average3)/(X_SUM - X_AVERAGE2_10);
  b4 = (sum14 - 10 * X_AVERAGE * y_average4)/(X_SUM - X_AVERAGE2_10);
  if(b1 != 0)
  {
    angle1 = atan(b1) * 180.0 / PI;  //角度
  }
  else
    angle1 = 0.0;
  
  if(b2 != 0)
  {
    angle2 = atan(b2) * 180.0 / PI;
  }
  else
    angle2 = 0.0;
  
  if(b3 != 0)
  {
    angle3 = atan(b3) * 180.0 / PI;
  }
  else
    angle3 = 0.0;
  
  if(b4 != 0)
  {
    angle4 = atan(b4) * 180.0 / PI;
  }
  else
    angle4 = 0.0;
  
  angle = -(angle1 * 2.55 + angle2 * 1.33 + angle3 * 0.75) / (2.55 + 1.33 + 0.75);
  all_error = (error1 * 2.55 + error2 * 1.33 + error3 * 0.75) / (2.55 + 1.33 + 0.75);  //all_error = (error4 * 2.55 + error3 * 1.33 + error2 * 0.75 + error1 * 0.1) / (2.55 + 1.33 + 0.75 + 0.1);
  //printf("all_error2=%d.%d\n", (int)all_error, abs((int)((all_error - (int)all_error)*100)));
  all_error = ( all_error / 4.5 * 0.6 + angle * 0.4 );
//  forsee = (error1 * 0.8 + error5 * 0.2) / 4.5;
//  all_error = all_error * 1 + forsee * 0;
//  if(all_error < 0)
//    all_error = all_error * 1.25;
}
 
void scan()
{
  uint8 i,j,zebra_num,num;
  zebra_num = 0;
  for( i = CAMERA_H - 1; i <= CAMERA_H - 3; i-- )
  {
    num = 0;
    for( j = 0; j < CAMERA_W; j++)
    {
      if( center[i][j-1] == 0xff && center[i][j] == 0x00 && j > 0 )
        num += 1;                       //数黑白条数
    }
    if(num >= 6)                        //数到6条以上认定为斑马线
      zebra_num += 1;
  }
  /********** 斑马线识别 **********/
  if(zebra_num == 3)                    //数3条斑马线
  {
    start_new = 1;
    start_old = start_new;
  }
  else
    start_new = 0;
  if(start_new == 0 && start_old == 1)
    start_flag = 1;
  /*******************************/
  if(start_flag == 1 && start_new == 1 && start_old == 1)
    stop_flag = 1;
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
  PIT_Flag_Clear(PIT1);
}


