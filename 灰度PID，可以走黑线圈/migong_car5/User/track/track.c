/**
  ******************************************************************************
  * @�ļ��� ��track.c
  * @�ļ�˵�������ļ�Ϊ���ں���ѭ�������������У׼�ĺ���
  * @�汾��V1.2.1
  * @���ڣ�2020-12-12 
  * @ħʯ�Ƽ�@
  * @�Ա��������ӣ�https://shop418595371.taobao.com
  *  ����ֱ���Ա���������ħʯ���ӿƼ�������
  ******************************************************************************/
#include "track.h"
#include "bsp_oled.h"
#include "myadc.h"
#include "stdio.h"
#include "stdlib.h"

char PosFlag = 0;//���崫����λ�ñ�־λ��0��Ϊ�������ں���ƫ��λ�ã�1��ΪС���ڴ�����ƫ��λ��

int SensorCalFlag = 0;//����ȫ�ֱ���������У׼��־λ 

static int PosFlagValue;//������ֵ����


int D_AD_VALUE   = -115; 	//ȷ�����Ҵ�������ֵ
int LEFT_MAX     = 3748;   	//�󴫸�����ֵ
int RIGHT_MAX    = 3688;  	//�Ҵ�������ֵ
int MID_MAX      = 3688;  		//�м䴫������ֵ
int LEFT_THERSH  = 1867;	//�󴫸�����ֵ
int RIGHT_THERSH = 1937;	//�Ҵ�������ֵ
int LEFT_SPAN    = 5992;		//�����������ƶ���Ծ��ֵ   //790
int RIGHT_SPAN   = -6032;	//�����������ƶ���Ծ��ֵ   //1023 


/**
  * @brief  ����ѭ����������ʼ������
  * @param  None
  * @return None
  */
void Track_Init(void)
{	
	PosFlagValue=(int)((LEFT_MAX+RIGHT_MAX-LEFT_THERSH-RIGHT_THERSH)/3.0f);
}


/**
 *@brief: ��ȡѭ���������������
 * @param: None
 * @return: int���ͣ���Χѭ�����������ݣ����ݴ�ֵ������С������Ƕ�,
 *			һ�����ѭ���������-10000��10000֮�䣬���30000��ʾ����
 *          ֹͣ�ߣ�����ŵĺ���
 */
int GetTraceDate(void)
{
	char buf[22];
	int Data_Out;//���������������
	int Left_AD ,Right_AD, Mid_AD;//���������д�����ADֵ����
//	static char PosFlag = 0;//���崫����λ�ñ�־λ��0��Ϊ�������ں���ƫ��λ�ã�1��ΪС���ڴ�����ƫ��λ��
	 	
	Right_AD = -Get_Adc(5) + 4096;	  //�󴫸�����ȡ��ADֵ
	Mid_AD   = -Get_Adc(4) + 4096;		//�м䴫������ȡ��ADֵ
	Left_AD  = -Get_Adc(1) + 4096;		//�Ҵ�������ȡ��ADֵ
	
	if((Left_AD >= (LEFT_MAX*1.0f)) && (Mid_AD>=(MID_MAX*1.0f)) && (Right_AD>=(RIGHT_MAX*1.0f)))
	return 30000;
//	printf("L:%d M:%d R:%d\r\n",Left_AD,Mid_AD,Right_AD);
	Data_Out=(Left_AD-Right_AD+D_AD_VALUE);
//	printf("Data_Out��%d\r\n",Data_Out);
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


/*@brief: �˺���Ϊѭ��������У׼����
 * @param: None
 * @return: None
 */
void GetParament(void)
{
	char buf[22];
	int DValue=0;
	int Left_AD,Right_AD,Mid_AD;//���������д�����ADֵ����
	
	static int LeftMax=0;
	static int RightMax=0;
	static int MidMax=0;
	static int Left_Thersh=0;
	static int Right_Thersh=0;
	static int Left_Span=0;
	static int Right_Span=0;
	
	Right_AD = -Get_Adc(5) + 4096;	  //�󴫸�����ȡ��ADֵ
	Mid_AD   = -Get_Adc(4) + 4096;		//�м䴫������ȡ��ADֵ
	Left_AD  = -Get_Adc(1) + 4096;		//�Ҵ�������ȡ��ADֵ
	
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
	DValue=Right_AD-Left_AD;//��ֵ���Ҵ��������󴫸���
	
	if(SensorCalFlag==2)//�������У׼��ɰ�������У׼�����ݴ��浽EEROM�в�ʹ����У׼������
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
		// ��ȡADC��ֵ
//		sprintf(buf, (const char* )"RigAD:%d ",Right_AD);
//		OLED_ShowStr(0, 0, (unsigned char* )buf, 1);
//		sprintf(buf, (const char* )" Mid_AD:%d ",Mid_AD);
//		OLED_ShowStr(0, 1, (unsigned char* )buf, 1);
//		sprintf(buf, (const char* )"LeftAD:%d ",Left_AD);
//		OLED_ShowStr(0, 2, (unsigned char* )buf, 1);
		
		// У׼��ʾֵ
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




