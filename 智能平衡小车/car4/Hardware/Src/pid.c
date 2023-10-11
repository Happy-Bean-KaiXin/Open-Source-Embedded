#include "pid.h"
#include "usart.h"
#include "tim.h"
#include "bsp_oled.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "mpu6050.h"


PIDTypedef VerTical;
PIDTypedef VeloCity;

extern uint8_t fx;

/**
��������PDֱ���� + ��������PI�ٶȻ���
ֱ����Ϊ��
�ٶȻ�Ϊ��
ת��
*/

/**
 * @function: int Vertical(float Med, float Angle, float gyro_y) 
 * @description: ֱ����PD����
 * �����ĵ��PWM=7200Ϊ��������ռ�ձ�100%��
 * ���kp=720���Ƕ�ƫ��10�ȣ�����͵�����ת�ˡ�
 * ���ǲ�������С��ƫ��10�����ϣ�����kp�ķ�Χ��0 - 720�������Ǿ�������������
 * ��ֱ�����У�PD������ڲ���ΪС������̬���Լ���̬�Ƕ�Ӧ�Ľ��ٶ�
 * @param1 float Med, �����Ƕ�
 * @param2 float Angle, ��ʵ�Ƕ�
 * @return ֱ�������
 */
int Vertical(float Med, float Angle, float gyro_y) {
	int PWM_out;
	
	PWM_out = VerTical.Kp * (Angle - Med) + VerTical.Kd * (gyro_y - 0);
	
	return PWM_out;
}

/**
 * @function: 
 * @description: �ٶȻ�PI����
 * @param1 int Moto_Left, �����ٶ�
 * @param2 int Moto_Right �ҵ���ٶ�
 * @return 
 */
int Velocity(int Moto_Left, int Moto_Right) {
	static int PWM_out, ENco_Err_LowOut, ENco_Err_LowOut_Last,\
		Integral, Error;
	
	float a = 0.7; 
	
	// 1.�����ٶ�ƫ��
	Error = (Moto_Left + Moto_Right) - 0;
	// 2.���ٶȽ��е�ͨ�˲� low_out = (1-a)*Ek + a*low_out_last
	ENco_Err_LowOut = (1-a)*Error + a*ENco_Err_LowOut_Last;
	ENco_Err_LowOut_Last = ENco_Err_LowOut;
	// 3.���ٶ�ƫ����л���λ��
	Integral += ENco_Err_LowOut;
	LIMIT(Integral, -10000, 10000);
	
	// 4.����رպ��������
	if(pitch < -40 || pitch > 40)
		Integral = 0;
	
	// 4.���ٶȻ��������
	PWM_out = (int)(VeloCity.Kp * ENco_Err_LowOut + VeloCity.Ki * Integral);
	
	return PWM_out;
}

/**
 * @function: 
 * @description: ת��*z����ٶ�
 * @param: 
 * @return 
 */
int Turn(int gyro_z) {
	int PWM_out;
	
	PWM_out = (-0.25)*gyro_z;
	
	return PWM_out;
}

void print(UART_HandleTypeDef* huart, const char* buf, ...) {
  const char *p = buf;
  char str[255] = {0};
  va_list v;
  va_start(v, buf);
  vsprintf(str, buf, v); //ʹ�ÿɱ�������ַ�����ӡ������sprintf
  HAL_UART_Transmit(huart, (uint8_t* )str, strlen(str), 0xFF);  // ����ʽ����
  va_end(v);
}

//��ʼ��pid����
void PID_Init(void) {
	// ֱ����PD  ����������
	VerTical.Kp = 300.0f;//400
	VerTical.Kd = 1.3;
	
	// �ٶȻ�PI  Kp��Ki�����Թ�ϵ Ki = ��1/200��* Kp
	VeloCity.Kp = -59.0f;  // 60
	VeloCity.Ki = VeloCity.Kp/299.0;
	
}

unsigned char Key_Scan(void)  //���з���ֵ�İ���ɨ�躯��
{
	static unsigned char ucKey_value;	//����һ������ֵ�ı���
	
	if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin) == GPIO_PIN_RESET)	//����B1���¼�ֵ����Ϊ1
		ucKey_value=1;
	else if(HAL_GPIO_ReadPin(KEY2_GPIO_Port,KEY2_Pin) == GPIO_PIN_RESET)	//����B2���¼�ֵ����Ϊ2
		ucKey_value=3;
	else
		ucKey_value=0;	//�ް������¼�ֵ����Ϊ0
	
	return ucKey_value;
}

void Key_Proc(void)	//���������������԰����ļ򵥿���
{
	/*����uwTick�ı������巽ʽ����δ�ʱ������������*/
	static __IO uint32_t uwTick_KEY_Speed;

	unsigned char ucKey_value,ucKey_Dwon,ucKey_Up;
	static unsigned char ucKey_Old = 0; //��̬������һ�εļ�ֵ
	
	//���õδ��ʱ�����尴��ɨ��ʱ��100ms
	if(uwTick-uwTick_KEY_Speed < 50) return;
	uwTick_KEY_Speed = uwTick;
	
	ucKey_value = Key_Scan();
	ucKey_Dwon = ucKey_value & ( ucKey_value ^ ucKey_Old );	//����Ϊ����ֵ������Ϊ0
	ucKey_Up = ~ucKey_value & ( ucKey_value ^ ucKey_Old );	//����Ϊ̧��ǰ�İ���ֵ������Ϊ0 
	ucKey_Old = ucKey_value;
	
	switch(ucKey_Dwon) {
		case 1: fx += 10; OLED_Clear(); break;
		case 3: fx -= 10; OLED_Clear(); break;
	}
}


/**
 * @function: int GetMotorPulse(TIM_HandleTypeDef* htim)
 * @description: ʹ��STM32������ģʽ����ȡ����������������ֵ
 * @param {*} 
 * @return {*}
 */
int GetMotorPulse(TIM_HandleTypeDef* htimx)
{
	static short encoder_counter = 0;//STM32������ģʽ��ȡ����������
	
	encoder_counter = (short) __HAL_TIM_GET_COUNTER(htimx); // ��ȡ��������ֵ
	__HAL_TIM_SET_COUNTER(&htim3,0);                         // ����������
	
	return encoder_counter;
}
