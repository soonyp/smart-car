/*********************************************************************************************************************
* CH32V307VCT6 Opensourec Library 即（CH32V307VCT6 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是CH32V307VCT6 开源库的一部分
*
* CH32V307VCT6 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          MounRiver Studio V1.8.1
* 适用平台          CH32V307VCT6
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期                                      作者                             备注
* 2022-09-15        大W            first version
********************************************************************************************************************/
#include "zf_common_headfile.h"
#include "Ourcode_headfile.h"

extern int cnttt;
extern uint8 stoppp;
uint8 image[60][60];
uint8 startflag;
double k;

void Init(void) {
    tft180_set_dir(TFT180_CROSSWISE_180);
    tft180_init();     //初始化屏幕

    mt9v03x_init();

    JumpInit();

}

int main(void) {
    clock_init(SYSTEM_CLOCK_120M);      // 初始化芯片时钟 工作频率为 120MHz

    // 此处编写用户代码 例如外设初始化代码等
    Init();
    Motor_Init();
    // exti_init(B10,EXTI_TRIGGER_FALLING);
    //  while(startflag==0);//按键启动
    pit_ms_init(TIM6_PIT, 10);//中断回调函数在isr.c里面
    PID_init();

    // 此处编写用户代码 例如外设初始化代码等
//    if(startflag==1)
//    {
//        gpio_init(A15, GPO, 1, GPIO_PIN_CONFIG);
//        system_delay_ms(50);
//        gpio_init(A15, GPO, 0, GPIO_PIN_CONFIG);

    while (1) {
        // 此处编写需要循环执行的代码
        if (mt9v03x_finish_flag) {
            ImgPreprocess();
            mt9v03x_finish_flag = 0;
        }
        /*显示--解析图*/

        tft180_displayimage03x(mt9v03x_image[0], 60, 60);//原始图像
        tft180_show_gray_image(60, 60, &BinaryImg_CDM[0][0], IMG_COL, IMG_ROW, 60, 60, 1);

        for (int i = 0; i < 60; i++) {
            for (int j = 0; j < 60; j++) {
                image[i][j] = BinaryImg_CDM[i][j];
            }
        }
        image_scan(image);
        Strait_Detect();
        Cross_Detect();
        cheku_Detect();
        //                 obstacles_Detect();


        //       error = cal_error();

        //     Straight_line_accelerate_judge();
        //    motorPidSetSpeed(1800,1800);//这里面计算了error
        //     Motor_Set(2000,2000);

        if (Cross_Flag == 1) {

            gpio_init(A15, GPO, 1, GPIO_PIN_CONFIG);
            system_delay_ms(40);
            gpio_init(A15, GPO, 0, GPIO_PIN_CONFIG);
            k = (centerline[59] - centerline[top]) / (59 - top);
            for (int i = 58; i > top; i--) {
                centerline[i] = centerline[i + 1] - k;
            }
        }


        for (int i = 59; i > top; i--) {
            for (int j = 0; j < 60; j++) {

                //                                 if (leftline[i] == j)
                //                                 {
                //                                     image[i][j]=5;
                //                                 }
                //                                 else if (rightline[i] == j)
                //                                 {
                //                                     image[i][j]=5;
                //                                 }
                if (centerline[i] == j) {
                    image[i][j] = 5;
                }
            }
        }
        tft180_show_gray_image(0, 60, &image[0][0], IMG_COL, IMG_ROW, 60, 60, 5);
//                  if(garage_flag==1)
//                  {
//                      gpio_init(A15, GPO, 1, GPIO_PIN_CONFIG);
//                      system_delay_ms(50);
//                      gpio_init(A15, GPO, 0, GPIO_PIN_CONFIG);
//                  }
        //   tft180_show_int (60, 0,StraitFlag, 4);
        //      tft180_show_int (90, 0,vari, 4);
        //       tft180_show_int (60, 20,top, 2);

        //       tft180_show_int(60, 20,Left_Up_Find, 2);
        //       tft180_show_int(80, 20,Right_Up_Find, 2);
        //       tft180_show_int(60, 40,Left_Down_Find, 2);
        //       tft180_show_int(80, 40,Right_Down_Find, 2);
        //      tft180_show_int (60, 0,garage_flag, 2);
        //    tft180_show_int (60, 0,obstacles_flag, 2);
//                   tft180_show_int (60, 0,obstacles_flag,3);
//                   tft180_show_int (60, 0,obstacles_flag,2);
//                   tft180_show_int (60, 20,top,2);
//                   tft180_show_int (60, 40,stoppp,2);
        //      tft180_show_int (60, 0,Straight_Flag, 2);
        //      tft180_show_int (60, 0,error, 4);
        //
//
//                   tft180_show_int(60, 20,g_motor_PID_Out1, 4);
//                   tft180_show_int(60, 40,g_motor_PID_Out2, 4);

//                   tft180_show_int(80, 0,Left_Up_Find, 2);
//                   tft180_show_int(100, 0,Right_Up_Find, 2);
//                   tft180_show_int(80, 20,Left_Down_Find, 2);
//                   tft180_show_int(100, 20,Right_Down_Find, 2);
        //       tft180_show_int(60, 0,Left_Lost_Time, 2);
        //       tft180_show_int(80, 0,Right_Lost_Time, 2);
        //       tft180_show_int(80, 40,Both_Lost_Time, 2);
        // 此处编写需要循环执行的代码


    }

}






