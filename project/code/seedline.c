/*
 * seedline.c
 *
 *  Created on: 2023��9��27��
 *      Author: Universe
 */

#include "seedline.h"
#include "My_Math.h"
#include "math.h"
#include "motor.h"
#include "cam_preprocess.h"

uint8 leftline[130];  //������߽�
uint8 rightline[130];  //�����ұ߽�
uint8 centerline[130];

extern uint8 image[60][60];
int top, interval;
int16 Both_Lost_Time, Left_Lost_Time, Right_Lost_Time;
int16 lost_start;

int16 Left_Up_Find, Right_Up_Find, Left_Down_Find, Right_Down_Find;//�ĸ��յ����

uint8 Cross_Flag = 0;
uint8 garage_flag = 0;
uint8 obstacles_flag = 0;
uint8 Straight_Flag = 0;
uint8 Left_Lost_flag;
uint8 Right_Lost_flag;
float aver = 0;
float vari;
int times;


/*ɨ���ұ��ߵõ�����*/
void image_scan(uint8 image_deal[Length][Width]) {

    int16 list = 0;
    int16 line = 0;
    int16 seed = 0;
    uint8 cntblack = 0;//�����ж�ĳһ��Ϊ���У������ó�top

    Both_Lost_Time = 0;

    Left_Lost_Time = 0;
    Right_Lost_Time = 0;

    lost_start = 0;
    uint8 lost_start_flag = 1;//����lost_start��־λ
    Left_Lost_flag = 0;
    Right_Lost_flag = 0;
//    for(uint8 i=0;i<70;i++)
//    {
//        leftline[i]=0;
//        rightline[i]=0;
//        centerline[i]=0;
//    }



    for (list = 30; list < 60; list++) {
        if ((image_deal[59][list - 2] == white) && (image_deal[59][list - 1] == black)
            && (image_deal[59][list] == black)) {
            rightline[59] = list - 1;
            break;
        } else rightline[59] = 59;
    }

    for (list = 30; list >= 0; list--) {
        if ((image_deal[59][list] == black) && (image_deal[59][list + 1] == black)
            && (image_deal[59][list + 2] == white)) {
            leftline[59] = list + 1;
            break;
        } else leftline[59] = 0;
    }

    centerline[59] = (rightline[59] + leftline[59]) / 2;
    seed = centerline[59]; //���ҵ�60�м�image_deal[59][]�����ߵ㣬��Ϊseed

    for (line = 58; line >= 0; line--) {

        for (list = seed; list < 60; list++) {
            if ((image_deal[line][list - 2] == white) && (image_deal[line][list - 1] == black)
                && (image_deal[line][list] == black)) {
                rightline[line] = list - 1;
                Right_Lost_flag = 1;
                break;
            } else Right_Lost_flag = 0;


        }
        if (Right_Lost_flag == 0) {
            rightline[line] = 59;
            Right_Lost_Time++;
        }

        for (list = seed; list >= 0; list--) {
            if ((image_deal[line][list] == black) && (image_deal[line][list + 1] == black)
                && (image_deal[line][list + 2] == white)) {
                leftline[line] = list + 1;
                Left_Lost_flag = 1;
                break;
            } else Left_Lost_flag = 0;

        }
        if (Left_Lost_flag == 0) {
            leftline[line] = 0;
            Left_Lost_Time++;
        }


        seed = centerline[line] = (rightline[line] + leftline[line]) / 2;
        top = line;

        for (uint8 i = 1; i <= 57; i += 2)//�ж�top����
        {
            if (image_deal[line][i] == 0) {
                cntblack++;
            }
        }

        if (cntblack >= 28) {
            top = line;
            break;//�ҵ�topֱ��break����� forѭ��
        }
        cntblack = 0;
        if (rightline[line] == 59 && leftline[line] == 0) {
            Both_Lost_Time++;

            if (lost_start_flag == 1) {
                lost_start = line;
                lost_start_flag = 0;
            }
        }


    }
}

//ʮ��·�ڼ��
void Cross_Detect(void) {
//    int down_search_start=0;
//
////    Left_Down_Find = 0;
////    Right_Down_Find = 0;
////    if(Both_Lost_Time >=3)
////    {
//////    Left_Up_Find =Find_Left_UP_Point(lost_start,top);
//// //   Left_Down_Find =Find_Left_Down_Point(54,lost_start-5);
//////    Right_Up_Find=Find_Right_UP_Point(lost_start,top);
////  //  Right_Down_Find=Find_Right_Down_Point(54,lost_start-5);
//// //   //�½ǵ�������ʼ��
////  //  Cross_Flag=0;
////    }
//    Left_Up_Find=0;
//    Right_Up_Find=0;
//    Left_Down_Find=0;
//    Right_Down_Find=0;
//    int down_search_start=0;


    if (Both_Lost_Time >= 3)//ʮ�ֱض���˫�߶��ߣ�����˫�߶��ߵ�������ٿ�ʼ�ҽǵ�
    {
        Cross_Flag = 1;//ȷ����Ӧ��־λ�����ڸ�Ԫ�ػ����

    } else {
        Cross_Flag = 0;
    }
//    Find_Up_Point( lost_start+2, top);
//    if(Left_Up_Find!=0&&Right_Up_Find!=0)//�ҵ������ϵ㣬����Ϊ�ҵ�ʮ����
//    {
////       // Cross_Flag=1;//ȷ����Ӧ��־λ�����ڸ�Ԫ�ػ����
//            down_search_start=Left_Up_Find>Right_Up_Find?Left_Up_Find:Right_Up_Find;//�������Ϲյ����꿿������Ϊ�µ����������
////
//            Find_Down_Point(54,down_search_start+5);//���Ϲյ���2����Ϊ�½ǵ�Ľ�ֹ��
////
//            if(Left_Down_Find<=Left_Up_Find)
//            {
//                Left_Down_Find=0;//�µ㲻���ܱ��ϵ㻹����
//            }
//            if(Right_Down_Find<=Right_Up_Find)
//            {
//                Right_Down_Find=0;//�µ㲻���ܱ��ϵ㻹����
//            }
//            if( Left_Down_Find!=0&&Right_Down_Find!=0&&abs(Left_Down_Find-Right_Down_Find<=3) )
//            {//�ĸ��㶼�ڣ��������ߣ����������Ȼ����
//                Left_Add_Line (leftline [Left_Up_Find ],Left_Up_Find ,leftline [Left_Down_Find ] ,Left_Down_Find);
//                Right_Add_Line(rightline[Right_Up_Find],Right_Up_Find,rightline[Right_Down_Find],Right_Down_Find);
//            }
//////            else if(Left_Down_Find==0&&Right_Down_Find!=0)//11//����ʹ�õ���б�ʲ���
//////            {//������                                     //01
//////                Lengthen_Left_Boundry(Left_Up_Find-1,59);
//////                Right_Add_Line(rightline[Right_Up_Find],Right_Up_Find,rightline[Right_Down_Find],Right_Down_Find);
//////            }
//////            else if(Left_Down_Find!=0&&Right_Down_Find==0)//11
//////            {//������                                      //10
//////                Left_Add_Line (leftline [Left_Up_Find ],Left_Up_Find ,leftline [Left_Down_Find ] ,Left_Down_Find);
//////
//////                Lengthen_Right_Boundry(Right_Up_Find-1,59);
//////            }
////            else if(Left_Down_Find==0&&Right_Down_Find==0)//11
////            {//�����ϵ�                                    //00
////                Lengthen_Left_Boundry (Left_Up_Find-1,59);
////                Lengthen_Right_Boundry(Right_Up_Find-1,59);
////            }
//    }


    //�ǵ���ر�����debugʹ��
//    int record1=0;
//    int record2=0;
//    for(int j=30;j<=35;j++)
//       {
//         for(int i=59;i>0;i++)
//          {
//             if(image[i][j]==0&&i<30)
//             record1=j;
//             break;
//          }
//        }
//
//    for(int j=30;j>=25;j--)
//          {
//            for(int i=59;i>0;i++)
//             {
//                  if(image[i][j]==0&&i<30)
//                  record2=j;
//                  break;
//             }
//           }


}


void cheku_Detect(void) {

    int cnt = 0;
    int black_blocks = 0;
    times = 0;
    for (int i = 40; i < 43; i++)//35,43
    {
        black_blocks = 0;
        cnt = 0;
        for (int j = 5; j < 55; j++) {
            if (BinaryImg_CDM[i][j] == 0) {
                cnt++;
            } else {
                if (cnt >= 2 && cnt <= 10) {
                    black_blocks++;
                    cnt = 0;
                } else {
                    cnt = 0;
                }
            }
        }

        if (black_blocks >= 3 && black_blocks <= 9)
            times++;

    }
    if (times >= 2) {
        garage_flag = 1;
    } else {
        garage_flag = 0;
    }

}

void obstacles_Detect(void) {
    //   (rightline[top]+leftline[top])/2>=28&&(rightline[top]+leftline[top])/2<=32
    if (top >= 25 && top <= 30 && Left_Lost_Time <= 1 && Right_Lost_Time <= 1) {

        obstacles_flag = 1;
//            Motor_Set(3000,200);
//            system_delay_ms(500);
//            Motor_Set(100,2000);
//            pit_ms_init(TIM8_PIT,10);

    }
//    else
//    {
//        obstacles_flag=0;
//
//    }

}

void Strait_Detect(void) {
    int i;
    float sum1 = 0, sum2 = 0;
    for (i = 40; i > top; i--) {
        sum1 += centerline[i];
    }
    aver = sum1 / (40 - i);
    for (i = 40; i > top; i--) {
        sum2 += pow(centerline[i] - aver, 2);
    }
    vari = sum2 / (40 - i);
    if (vari <= 1 && Left_Lost_Time <= 1 && Right_Lost_Time <= 1)
        Straight_Flag = 1;
    else if (vari > 19 && (Left_Lost_Time >= 2 || Right_Lost_Time >= 2))
        Straight_Flag = 2;
    else
        Straight_Flag = 0;
}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     ���½ǵ���
  @param     ��ʼ�У���ֹ��
  @return    ���ؽǵ����ڵ��������Ҳ�������0
  Sample     left_down_guai[0]=Find_Left_Down_Point(MT9V03X_H-1,20);
  @note      �ǵ�����ֵ�ɸ���ʵ��ֵ����
-------------------------------------------------------------------------------------------------------------------*/
uint8 Find_Left_Down_Point(int start, int end)//�����½ǵ㣬����ֵ�ǽǵ����ڵ����� //param ��54  ��top
{
    int i = 0;
    int left_down_line = 0;

//    if(Left_Lost_Time>=0.9*MT9V03X_H)//�󲿷ֶ����ߣ�û�йյ��жϵ�����
//       return left_down_line;

    if (start >= 54)//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start = 54;//��һ���棬���жϵ�i��ʱ������ʵ�i+3��i-4�У���ֹԽ��

    if (end <= 5)
        end = 5;
    for (i = start; i >= end; i--) {
        if (left_down_line == 0 &&//ֻ�ҵ�һ�����������ĵ�
            abs(leftline[i] - leftline[i + 1]) <= 5 &&//�ǵ����ֵ���Ը���
            abs(leftline[i + 1] - leftline[i + 2]) <= 5 &&
            abs(leftline[i + 2] - leftline[i + 3]) <= 5 &&
            (leftline[i] - leftline[i - 3]) >= 5 &&
            (leftline[i] - leftline[i - 4]) >= 8 &&
            (leftline[i] - leftline[i - 5]) >= 8) {
            left_down_line = i;//��ȡ��������
            break;
        }
    }
    return left_down_line;
}

/*----YOU���½ǹյ���---------------------------------------------------------------------------------------------------------*/
uint8 Find_Right_Down_Point(int start, int end)//�����½ǵ㣬����ֵ�ǽǵ����ڵ�����
{
    int right_down_line = 0;
    int y0 = rightline[59], j = 0;
    for (int i = 59; i > 0; i--) {
        for (j = y0; j > 30; j--) {
//              if (j >= 59)
//                  break;
            if (image[i][j - 1] == 1)
                break;
        }
        y0 = j + 1;
        if (image[i - 1][j] > 0 && image[i - 1][j + 5] > 0) {
            right_down_line = i;
//              D.y = j - 1;
            break;
        }
        right_down_line = 0;
//          D.y = Y - 1;
    }

    return right_down_line;

//    int i=0;
//    int right_down_line=0;
////    if(Left_Lost_Time>=0.9*MT9V03X_H)//�󲿷ֶ����ߣ�û�йյ��жϵ�����
////       return left_down_line;
//
//    if(start>=54)//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
//        start=54;//��һ���棬���жϵ�i��ʱ������ʵ�i+3��i-4�У���ֹԽ��
//
//    if(end<=5)
//       end=5;
//    for(i=start;i>=end;i--)
//    {
//        if(right_down_line==0&&//ֻ�ҵ�һ�����������ĵ�
//                abs(rightline[i]-rightline[i+1])<=5&&//�ǵ����ֵ���Ը���
//                abs(rightline[i+1]-rightline[i+2])<=5&&
//                abs(rightline[i+2]-rightline[i+3])<=5&&
//               (rightline[i]-rightline[i-3])>=5&&
//               (rightline[i]-rightline[i-4])>=8&&
//               (rightline[i]-rightline[i-5])>=8)
//
//        {
//            right_down_line=i;//��ȡ��������
//            break;
//        }
//    }


}


//�Ϲյ�Ӧ�ô�top->lost_startѰ�ҡ���������topλ�õ�̫������
/*----ZUO���Ͻǹյ���---------------------------------------------------------------------------------------------------------*/

uint8 Find_Left_UP_Point(int start, int end)//param ��top    lost_start
{
    int i = 0;
    int left_up_line = 0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)//�󲿷ֶ����ߣ�û�йյ��жϵ�����
//       return left_down_line;
    if (start >= 54)//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start = 54;//��һ���棬���жϵ�i��ʱ������ʵ�i+3��i-4�У���ֹԽ��

    if (end <= 5)
        end = 5;
    for (i = start - 5; i >= end + 5; i--) {
        if (left_up_line == 0 &&//ֻ�ҵ�һ�����������ĵ�
            abs(leftline[i] - leftline[i - 1]) <= 5 &&//�ǵ����ֵ���Ը���
            abs(leftline[i - 1] - leftline[i - 2]) <= 5 &&
            abs(leftline[i - 2] - leftline[i - 3]) <= 5 &&
            abs(leftline[i] - leftline[i + 2]) >= 5 &&
            abs(leftline[i] - leftline[i + 3]) >= 8 &&
            abs(leftline[i] - leftline[i + 4]) >= 8) {
            left_up_line = i;//��ȡ��������
            break;
        }
    }
    return left_up_line;
}

/*----YOU�Ͻǹյ���---------------------------------------------------------------------------------------------------------*/

uint8 Find_Right_UP_Point(int start, int end) {
    int i = 0;
    int right_up_line = 0;
//    if(Left_Lost_Time>=0.9*MT9V03X_H)//�󲿷ֶ����ߣ�û�йյ��жϵ�����
//       return left_down_line;
    if (start >= 54)//����5������5�����ݲ��ȶ���������Ϊ�߽�����жϣ�����
        start = 54;//��һ���棬���жϵ�i��ʱ������ʵ�i+3��i-4�У���ֹԽ��

    if (end <= 5)
        end = 5;
    for (i = start - 5; i >= end + 2; i--) {
        if (right_up_line == 0 &&//ֻ�ҵ�һ�����������ĵ�
            abs(rightline[i] - rightline[i - 1]) <= 5 &&//�ǵ����ֵ���Ը���
            abs(rightline[i - 1] - rightline[i - 2]) <= 5 &&
            abs(rightline[i - 2] - rightline[i - 3]) <= 5 &&
            abs(rightline[i] - rightline[i + 2]) >= 5 &&
            abs(rightline[i] - rightline[i + 3]) >= 8 &&
            abs(rightline[i] - rightline[i + 4]) >= 8) {
            right_up_line = i;//��ȡ��������
            break;
        }
    }
    return right_up_line;
}


void Find_Up_Point(int start, int end) {
    Left_Up_Find = Find_Left_UP_Point(start, end);
    Right_Up_Find = Find_Right_UP_Point(start, end);

}


void Find_Down_Point(int start, int end) {
    Left_Down_Find = Find_Left_Down_Point(start, end);
    Right_Down_Find = Find_Right_Down_Point(start, end);

}


/*-------------------------------------------------------------------------------------------------------------------
  @brief     ����
  @param     ���ߵ���㣬�յ�
  @return    null
  Sample     Left_Add_Line(int x1,int y1,int x2,int y2);
  @note      ����ֱ���Ǳ߽磬������ǿ��Ŷȸߵ�,��Ҫ�Ҳ�
-------------------------------------------------------------------------------------------------------------------*/
void Left_Add_Line(int x1, int y1, int x2, int y2)//����,�����Ǳ߽�
{
    int i, max, a1, a2;
    int hx;
    if (x1 >= 59)//��ʼ��λ��У�����ų�����Խ��Ŀ���
        x1 = 59;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= 59)
        y1 = 59;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= 59)
        x2 = 59;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= 59)
        y2 = 59;
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;
    if (a1 > a2)//���껥��
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }
    for (i = a1; i <= a2; i++)//����б�ʲ��߼���
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= 59)
            hx = 59;
        else if (hx <= 0)
            hx = 0;
        leftline[i] = hx;
    }
}


void Right_Add_Line(int x1, int y1, int x2, int y2)//�ұ߲���,�����Ǳ߽�
{
    int i, max, a1, a2;
    int hx;
    if (x1 >= 59)//��ʼ��λ��У�����ų�����Խ��Ŀ���
        x1 = 59;
    else if (x1 <= 0)
        x1 = 0;
    if (y1 >= 59)
        y1 = 59;
    else if (y1 <= 0)
        y1 = 0;
    if (x2 >= 59)
        x2 = 59;
    else if (x2 <= 0)
        x2 = 0;
    if (y2 >= 59)
        y2 = 59;
    else if (y2 <= 0)
        y2 = 0;
    a1 = y1;
    a2 = y2;
    if (a1 > a2)//���껥��
    {
        max = a1;
        a1 = a2;
        a2 = max;
    }
    for (i = a1; i <= a2; i++)//����б�ʲ��߼���
    {
        hx = (i - y1) * (x2 - x1) / (y2 - y1) + x1;
        if (hx >= 59)
            hx = 59;
        else if (hx <= 0)
            hx = 0;
        rightline[i] = hx;
    }
}

/*-------------------------------------------------------------------------------------------------------------------
  @brief     �ұ߽��ӳ�
  @param     �ӳ���ʼ�������ӳ���ĳ��
  @return    null
  Sample     Lengthen_Right_Boundry(int start,int end)��
  @note      ����ʼ��������3���㣬���б�ʣ������ӳ���ֱ��������
-------------------------------------------------------------------------------------------------------------------*/
void Lengthen_Right_Boundry(int start, int end)//start<end
{
    int i, t;
    float k = 0;
    if (start >= 59)//��ʼ��λ��У�����ų�����Խ��Ŀ���
        start = 59;
    else if (start <= 0)
        start = 0;
    if (end >= 59)
        end = 59;
    else if (end <= 0)
        end = 0;
    if (end < start)//++���ʣ����껥��
    {
        t = end;
        end = start;
        start = t;
    }

    if (start <= 5)//��Ϊ��Ҫ�ڿ�ʼ��������3���㣬������ʼ����ڿ��ϣ��������ӳ���ֻ��ֱ������
    {
        Right_Add_Line(rightline[start], start, rightline[end], end);//�Ե�
    } else {
        k = (float) (rightline[start] - rightline[start - 4]) / 5.0;//�����k��1/б��
        for (i = start; i <= end; i++) {
            rightline[i] = (int) (i - start) * k + rightline[start];//(x=(y-y1)*k+x1),��бʽ����
            if (rightline[i] >= 59) {
                rightline[i] = 59;
            } else if (rightline[i] <= 0) {
                rightline[i] = 0;
            }
        }
    }
}

//
void Lengthen_Left_Boundry(int start, int end)//start<end
{
    int i, t;
    float k = 0;
    if (start >= 59)//��ʼ��λ��У�����ų�����Խ��Ŀ���
        start = 59;
    else if (start <= 0)
        start = 0;
    if (end >= 59)
        end = 59;
    else if (end <= 0)
        end = 0;
    if (end < start)//++���ʣ����껥��
    {
        t = end;
        end = start;
        start = t;
    }

    if (start <= 5)//��Ϊ��Ҫ�ڿ�ʼ��������3���㣬������ʼ����ڿ��ϣ��������ӳ���ֻ��ֱ������
    {
        Left_Add_Line(leftline[start], start, leftline[end], end);//�Ե�
    } else {
        k = (float) (leftline[start] - leftline[start - 4]) / 4.0;//�����k��1/б��
        for (i = start; i <= end; i++) {
            leftline[i] = (int) (i - start) * k + leftline[start];//(x=(y-y1)*k+x1),��бʽ����
            if (leftline[i] >= 59) {
                leftline[i] = 59;
            } else if (leftline[i] <= 0) {
                leftline[i] = 0;
            }
        }
    }
}






//void GET_straight_flag(uint8 arr[top])
//{
//       Straight_Flag=0;
//       if(top>=45)//��ֹ�к�Զ
//       {
//
//               if(abs(error)<=50)//����С
//               {
//                   Straight_Flag=1;//��Ϊ��ֱ��
//               }
//        }
//   Straight_Flag=0;
//}
//
//void Straight_line_accelerate_judge(void)
//{
//    GET_straight_flag(centerline);
//
//    if( Straight_Flag == 1)
//    {
//        motorPidSetSpeed(2500,2500);
//
//    }
//    if( Straight_Flag == 0)
//       {
//        motorPidSetSpeed(2000,2000);
//       }
//
//
//}





/*Զ���ж˸�Ȩ�ؼ������*/
float cal_error(void) {
    float error1 = 0, error2 = 0, error3 = 0;
    float k1 = 1.5;  //����2
    float k2 = 2;  //�ж�1.5
    float k3 = 1.6;   //Զ��2.1
    interval = (59 - top) / 3;   //Ӧ�ã�59-top��/3

    for (int i = 59; i > 59 - interval; i--)
        error1 += centerline[i] - 30;

    for (int i = 59 - interval; i > 59 - 2 * interval; i--)
        error2 += centerline[i] - 30;

    for (int i = 59 - 2 * interval; i > top; i--)
        error3 += centerline[i] - 30;

    return error1 * k1 + error2 * k2 + error3 * k3;
}




//void Speedup(void)
//{
//        g_motor_PID_Out1 = 2000-g_PID_Out;
//        g_motor_PID_Out2 = 2000+g_PID_Out;
//        Motor_Set(g_motor_PID_Out1,g_motor_PID_Out2);
//}
//
//void Slowdown(void)
//{
//    g_motor_PID_Out1 = 1500-g_PID_Out;
//    g_motor_PID_Out2 = 1500+g_PID_Out;
//    Motor_Set(g_motor_PID_Out1,g_motor_PID_Out2);
//}



