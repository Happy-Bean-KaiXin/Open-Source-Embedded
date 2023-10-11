#include "car.h"
#include "string.h"

TARGET_t target;

/**
 *@brief: ��ת��ľ��룬ת��Ϊ�Ƕ�
 * @param: None
 * @return: С��ת��ľ���
 */
float Car_Turn(int16_t angle) {
	float Circle;
	switch(angle) {
		case 15:
			Circle = (15.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -15:
			Circle = -(15.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 20:
			Circle = (20.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -20:
			Circle = -(20.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 30:
			Circle = (30.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -30:
			Circle = -(30.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 35:
			Circle = (35.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -35:
			Circle = -(35.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 40:
			Circle = (40.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -40:
			Circle = -(40.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 45:
			Circle = (45.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -45:
			Circle = -(45.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 60:
			Circle = (60.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -60:
			Circle = -(60.0/90.0)*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 90:    // ����90��Ϊ��ת
			Circle = (pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -90:   // ��90��Ϊ��ת
			Circle = -(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case 180:
			Circle = 2.0*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		case -180:
			Circle = -2.0*(pi*CarWidth/4.0)/(pi*WheelDiameter);
			break;
		default: break;
	}
	return Circle;
}

// ����ֵ��ȡ����
uint8_t Key_Scan(void) {
	
	uint8_t Key_Value;
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == 0) {
		Key_Value = 1;
	}
	else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 0) {
		Key_Value = 2;
	}
	else {
		Key_Value = 0;
	}
	return Key_Value;
}

// ���������� 
void Key_Proc(void)
{
	static __IO uint32_t uwTick_KEY_Speed;
		
	if(uwTick - uwTick_KEY_Speed<50) return;	//���ٺ���
		uwTick_KEY_Speed = uwTick;	            //ÿ50msִ��һ��
	
	uint8_t key_value,key_up,key_down;
	static uint8_t key_old;
	
	key_value = Key_Scan();
	key_up = ~key_value & (key_old ^ key_value);
	
	key_down = key_value & (key_old ^ key_value);
	key_old = key_value;
	
	if(key_down) {
		switch(key_down) {
			case 1:
				Flag.Is_Medicine = 1;      // ����1ģ��װҩ
				break;
			case 2:
				Flag.Is_Medicine = 0;      // ����2ж��ҩƷ
				Red_ReSet;
				break;
			default: break;
		}
	}
	
}

// ���ڻص�����

uint8_t rec;
uint8_t rec_buf[20];

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{ 
	static int rec_cnt = 0;
 
	if(huart->Instance == USART2)
	{ 
		rec_buf[rec_cnt++] = rec; 
		if(rec == '\n') {
			rec_cnt = 0;
			target.number = rec_buf[0]-'0';
			if(Flag.Is_Num_Recongnize) {
				if(strstr((const char*)rec_buf,"LEFT") != NULL)
				{
					target.direction = LEFT;
					Flag.Is_NumOne = 1;
				}
				else if(strstr((const char*)rec_buf,"RIGHT") != NULL)
				{  
					target.direction = RIGHT; 
					Flag.Is_NumTwo = 1;
				}
				// ��Զ�˴��ں�������
				else if(strstr((const char* )rec_buf, "Seven") != NULL) {
						Flag.Is_DIs_MID_FAR = 1;      // ����Զ�˱�־λ��Ϊ1
						Flag.Is_NumSeven = 1;
					}
				else if(strstr((const char* )rec_buf, "Six") != NULL) {
					Flag.Is_DIs_MID_FAR = 1;      // ����Զ�˱�־λ��Ϊ1
					Flag.Is_NumSix = 1;
				}
				else if(strstr((const char* )rec_buf, "Five") != NULL) {
					Flag.Is_DIs_MID_FAR = 1;      // ����Զ�˱�־λ��Ϊ1
					Flag.Is_NumFive = 1;
				}
				else if(strstr((const char* )rec_buf, "Four") != NULL) {
					Flag.Is_DIs_MID_FAR = 1;      // ����Զ�˱�־λ��Ϊ1
					Flag.Is_NumFour = 1;
				}
				else if(strstr((const char* )rec_buf, "Three") != NULL) {
					Flag.Is_DIs_MID_FAR = 1;      // ����Զ�˱�־λ��Ϊ1
					Flag.Is_NumThree = 1;
				}
				else  {
					Flag.Is_NumOne = 0;
					Flag.Is_NumTwo = 0;
					Flag.Is_NumThree = 0;
					Flag.Is_NumFour = 0;
					Flag.Is_NumFive = 0;
					Flag.Is_NumSix = 0;
					Flag.Is_NumSeven = 0;
					
					target.direction = 0;
				}
			}
		}
	}
	HAL_UART_Receive_IT(&huart2,&rec,1);

}
