#ifndef _TRACK_H__
#define _TRACK_H__


/****************外部变量声明*****************/
extern int D_AD_VALUE; 		//确定左右传感器差值
extern int LEFT_MAX;   	    //左传感器峰值
extern int MID_MAX;  		//中间传感器峰值
extern int RIGHT_MAX;  	    //右传感器峰值
extern int LEFT_THERSH;	    //左传感器阈值
extern int RIGHT_THERSH;	//右传感器阈值
extern int LEFT_SPAN;		//传感器向左移动跳跃差值   
extern int RIGHT_SPAN;	    //传感器向右移动跳跃差值


extern int SensorCalFlag;//定义全局变量传感器校准标志位，为1开始校准，为2校准完成，为0校准结束 
extern char PosFlag;//定义传感器位置标志位，2认为传感器在黑线偏左位置，1认为小车在传感器偏右位置 0认为小车在中间位置

/******************函数声明*******************/
extern void Track_Init(void);
extern int GetTraceDate(void);
extern void GetParament(void);

#endif

