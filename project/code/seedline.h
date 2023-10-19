/*
 * seedline.h
 *
 *  Created on: 2023年9月27日
 *      Author: Universe
 */

#ifndef SEEDLINE_H_
#define SEEDLINE_H_

#include "zf_common_headfile.h"

#define Length 60
#define Width  60
#define white  1
#define black  0


extern uint8 leftline[130];  //赛道左边界
extern uint8 rightline[130];  //赛道右边界
extern uint8 centerline[130];
extern int top, interval;
extern int16 Left_Lost_Time, Right_Lost_Time, Both_Lost_Time;
extern int16 lost_start;
extern int16 Left_Up_Find, Right_Up_Find, Left_Down_Find, Right_Down_Find;
extern uint8 Cross_Flag;
extern uint8 garage_flag;
extern uint8 obstacles_flag;
extern uint8 Straight_Flag;

void image_scan(uint8 image_deal[Length][Width]);

void Cross_Detect(void);

void cheku_Detect(void);

void obstacles_Detect(void);

void Strait_Detect(void);

//void Speedup(void);
//void Slowdown(void);
float cal_error(void);


uint8 Find_Left_Down_Point(int start, int end);

uint8 Find_Right_Down_Point(int start, int end);

uint8 Find_Left_UP_Point(int start, int end);

uint8 Find_Right_UP_Point(int start, int end);

void Find_Up_Point(int start, int end);

void Find_Down_Point(int start, int end);

void Left_Add_Line(int x1, int y1, int x2, int y2);

void Right_Add_Line(int x1, int y1, int x2, int y2);

void Lengthen_Right_Boundry(int start, int end);

void Lengthen_Left_Boundry(int start, int end);

//void GET_straight_flag(uint8 arr[top]);
//void Straight_line_accelerate_judge(void);
extern int times;


#endif /* SEEDLINE_H_ */
