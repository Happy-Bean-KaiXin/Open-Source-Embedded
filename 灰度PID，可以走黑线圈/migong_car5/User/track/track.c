/**
  ******************************************************************************
  * @文件名 ：track.c
  * @文件说明：此文件为关于红外循迹传感器输出和校准的函数
  * @版本：V1.2.1
  * @日期：2020-12-12 
  * @魔石科技@
  * @淘宝店铺链接：https://shop418595371.taobao.com
  *  或者直接淘宝搜索店铺魔石电子科技工作室
  ******************************************************************************/
#include "track.h"
#include "bsp_oled.h"
#include "myadc.h"
#include "stdio.h"
#include "stdlib.h"

char PosFlag = 0;//定义传感器位置标志位，0认为传感器在黑线偏左位置，1认为小车在传感器偏右位置

int SensorCalFlag = 0;//定义全局变量传感器校准标志位 

static int PosFlagValue;//定义阈值变量


int D_AD_VALUE   = -115; 	//确定左右传感器差值
int LEFT_MAX     = 3748;   	//左传感器峰值
int RIGHT_MAX    = 3688;  	//右传感器峰值
int MID_MAX      = 3688;  		//中间传感器峰值
int LEFT_THERSH  = 1867;	//左传感器阈值
int RIGHT_THERSH = 1937;	//右传感器阈值
int LEFT_SPAN    = 5992;		//传感器向左移动跳跃差值   //790
int RIGHT_SPAN   = -6032;	//传感器向右移动跳跃差值   //1023 


/**
  * @brief  红外循迹传感器初始化函数
  * @param  None
  * @return None
  */
void Track_Init(void)
{	
	PosFlagValue=(int)((LEFT_MAX+RIGHT_MAX-LEFT_THERSH-RIGHT_THERSH)/3.0f);
}


/**
 *@brief: 获取循迹传感器输出函数
 * @param: None
 * @return: int类型，范围循迹传感器数据，根据此值来调节小车舵机角度,
 *			一般根据循迹场地输出-10000到10000之间，输出30000表示遇到
 *          停止线，即横放的黑线
 */
int GetTraceDate(void)
{
	char buf[22];
	int Data_Out;//定义数据输出变量
	int Left_AD ,Right_AD, Mid_AD;//定义左右中传感器AD值变量
//	static char PosFlag = 0;//定义传感器位置标志位，0认为传感器在黑线偏左位置，1认为小车在传感器偏右位置
	 	
	Right_AD = -Get_Adc(5) + 4096;	  //左传感器获取的AD值
	Mid_AD   = -Get_Adc(4) + 4096;		//中间传感器获取的AD值
	Left_AD  = -Get_Adc(1) + 4096;		//右传感器获取的AD值
	
	if((Left_AD >= (LEFT_MAX*1.0f)) && (Mid_AD>=(MID_MAX*1.0f)) && (Right_AD>=(RIGHT_MAX*1.0f)))
	return 30000;
//	printf("L:%d M:%d R:%d\r\n",Left_AD,Mid_AD,Right_AD);
	Data_Out=(Left_AD-Right_AD+D_AD_VALUE);
//	printf("Data_Out：%d\r\n",Data_Out);
	if(Data_Out > PosFlagValue)
	{
		PosFlag=1;
	}
	else if(Data_Out < -PosFlagValue)
	{
		PosFlag=0;
	}
	
	if(Mid_AD < LEFT_THERSH)
	{	
		if(Data_Out > PosFlagValue)
		{
			Data_Out = (2*LEFT_MAX-Left_AD)*2-LEFT_SPAN;
		}
		else if((Data_Out<PosFlagValue)&&(PosFlag == 0))
		{
			Data_Out=abs((2*LEFT_MAX-Left_AD)*2-LEFT_SPAN);
		}
		
	} 
	
	if(Mid_AD<RIGHT_THERSH)
	{	
		if(Data_Out<-PosFlagValue)
		{
			Data_Out=(Right_AD-2*RIGHT_MAX)*2-RIGHT_SPAN;
		}
		else if((Data_Out>-PosFlagValue)&&(PosFlag == 1))
		{
			Data_Out=-abs((Right_AD-2*RIGHT_MAX)*2-RIGHT_SPAN);
		}
	}
	
	sprintf(buf, (const char* )"PosFlag:%d ",PosFlag);
	OLED_ShowStr(0, 6, (unsigned char* )buf, 1);
	
	return Data_Out;
}


/*@brief: 此函数为循迹传感器校准函数
 * @param: None
 * @return: None
 */
void GetParament(void)
{
	char buf[22];
	int DValue=0;
	int Left_AD,Right_AD,Mid_AD;//定义左右中传感器AD值变量
	
	static int LeftMax=0;
	static int RightMax=0;
	static int MidMax=0;
	static int Left_Thersh=0;
	static int Right_Thersh=0;
	static int Left_Span=0;
	static int Right_Span=0;
	
	Right_AD = -Get_Adc(5) + 4096;	  //左传感器获取的AD值
	Mid_AD   = -Get_Adc(4) + 4096;		//中间传感器获取的AD值
	Left_AD  = -Get_Adc(1) + 4096;		//右传感器获取的AD值
	
	if(Left_AD>LeftMax)	
	{
		LeftMax=Left_AD;
		Left_Thersh=Mid_AD;
		Left_Span=(2*LeftMax-Left_AD)*2-(Left_AD-Right_AD+D_AD_VALUE);
		
	}
	if(Right_AD>RightMax)
	{
		RightMax=Right_AD;
		Right_Thersh=Mid_AD;
		Right_Span=(Right_AD-2*RightMax)*2-(Left_AD-Right_AD+D_AD_VALUE);	
	}	

	if(Mid_AD>MidMax)
	{
		MidMax=Mid_AD;
	}		
	DValue=Right_AD-Left_AD;//差值，右传感器减左传感器
	
	if(SensorCalFlag==2)//如果按下校准完成按键，则将校准的数据储存到EEROM中并使用新校准的数据
    {
			D_AD_VALUE=DValue;
			LEFT_MAX=LeftMax;
			RIGHT_MAX=RightMax;
			LEFT_THERSH=Left_Thersh;
			RIGHT_THERSH=Right_Thersh;
			LEFT_SPAN=Left_Span;
			RIGHT_SPAN=Right_Span;   
			SensorCalFlag = 0;
    }
		// 读取ADC的值
//		sprintf(buf, (const char* )"RigAD:%d ",Right_AD);
//		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
//		sprintf(buf, (const char* )" Mid_AD:%d ",Mid_AD);
//		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
//		sprintf(buf, (const char* )"LeftAD:%d ",Left_AD);
//		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		
		// 校准显示值
		sprintf(buf, (const char* )"RigAD:%d MiAD:%d ",Right_AD,Mid_AD);
		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
		sprintf(buf, (const char* )"LAD:%d LThe:%d ",Left_AD, Left_Thersh);
		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
		sprintf(buf, (const char* )"D_AD_VALUE:%d ",DValue);
		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		sprintf(buf, (const char* )"LeftMax:%d ",LeftMax);
		OLED_ShowStr(0, 3, (unsigned char* )buf, 1);
		sprintf(buf, (const char* )"MidMax:%d ",MidMax);
		OLED_ShowStr(0, 4, (unsigned char* )buf, 1);
		sprintf(buf, (const char* )"RightMax:%d ",MidMax);
		OLED_ShowStr(0, 5, (unsigned char* )buf, 1);
		sprintf(buf, (const char* )"Right_Thersh:%d ",Right_Thersh);
		OLED_ShowStr(0, 6, (unsigned char* )buf, 1);
		sprintf(buf,"LSpan:%dRSpan:%d ",Left_Span,Right_Span);
		OLED_ShowStr(0, 7, (unsigned char* )buf, 1);
		
		printf("Right_AD:%d-Mid_AD:%d-Left_AD:%d\n",Right_AD,Mid_AD,Left_AD);
		printf("D_AD_VALUE:%d-LeftMax:%d-MidMax:%d-RightMax:%d\n  Left_Thersh:%d-Right_Thersh:%d-Left_Span:%d-Right_Span:%d\n", \
																								DValue,LeftMax,MidMax,RightMax,Left_Thersh,Right_Thersh,Left_Span,Right_Span);
		
}




