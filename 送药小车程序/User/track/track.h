#ifndef _TRACK_H__
#define _TRACK_H__


/****************�ⲿ��������*****************/
extern int D_AD_VALUE; 		//ȷ�����Ҵ�������ֵ
extern int LEFT_MAX;   	    //�󴫸�����ֵ
extern int MID_MAX;  		//�м䴫������ֵ
extern int RIGHT_MAX;  	    //�Ҵ�������ֵ
extern int LEFT_THERSH;	    //�󴫸�����ֵ
extern int RIGHT_THERSH;	//�Ҵ�������ֵ
extern int LEFT_SPAN;		//�����������ƶ���Ծ��ֵ   
extern int RIGHT_SPAN;	    //�����������ƶ���Ծ��ֵ


extern int SensorCalFlag;//����ȫ�ֱ���������У׼��־λ��Ϊ1��ʼУ׼��Ϊ2У׼��ɣ�Ϊ0У׼���� 
extern char PosFlag;//���崫����λ�ñ�־λ��2��Ϊ�������ں���ƫ��λ�ã�1��ΪС���ڴ�����ƫ��λ�� 0��ΪС�����м�λ��

/******************��������*******************/
extern void Track_Init(void);
extern int GetTraceDate(void);
extern void GetParament(void);

#endif

