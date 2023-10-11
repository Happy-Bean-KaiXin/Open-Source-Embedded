#include "key.h"

uint8_t Key_Scan(void) {
	
	uint8_t Key_Value;
	if(HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == 0) {
		Key_Value = 1;
	}
	else if(HAL_GPIO_ReadPin(K2_GPIO_Port, K2_Pin) == 0) {
		Key_Value = 2;
	}
	else if(HAL_GPIO_ReadPin(K3_GPIO_Port, K3_Pin) == 0) {
		Key_Value = 3;
	}
	else if(HAL_GPIO_ReadPin(K4_GPIO_Port, K4_Pin) == 0) {
		Key_Value = 4;
	}
	else {
		Key_Value = 0;
	}
	return Key_Value;
}

void Key_Proc() {
	
	//***按键处理子函数相关变量定义
	uint8_t key_value, key_up, key_down;
	static uint8_t key_old;
	static __IO uint32_t uwTick_Key_Speed;
	
	if(uwTick - uwTick_Key_Speed < 10) return;	// 控制函数执行的频率
		uwTick_Key_Speed = uwTick;	              // 每50ms执行一次
		
	
	key_value = Key_Scan();
	key_up = ~ key_value & (key_old ^ key_value);
	key_down = key_value & (key_old ^ key_value);
	key_old = key_value;
	
	if(key_down)	//当有按键按下时
	{
		switch(key_down) {
			case 1: 
				Flag.Is_Mode_Set = 0;    // 复位模式
				Mode1_SET;
				break;
			case 2: 
				Flag.TEST++;				// 识别红点模式
				Flag.Is_Mode_Set = 1;  // 开启模式1
				K2_Driver();
				break;
			case 3: 
				Flag.TEST++;
				Flag.Is_Mode_Set = 2;  // 开启模式2
				K3_Driver();             // 暂停启动键          
				break;
			case 4: 
				Flag.TEST++;
				K4_Driver();             // 读取红点坐标数据按键
				break;
			
		}
	}
}

void K2_Driver(void) {
	static uint8_t K2_State;
	K2_State = 1;
	switch(K2_State) {
		case 1: 
			OLED_Clear();
			Mode1_SET;
			Flag.Is_Mode_Set = 1;  // 开启模式1
			Flag.Is_Uart_Rec = 0;  // 开启识别红点  串口接收
			Flag.Is_OLED_Face = 1;
			break;
	}
}

void K3_Driver(void) {
	static uint8_t K3_State;
	K3_State = 1;
	switch(K3_State) {
		case 1: 
			OLED_Clear();
			Mode2_SET;
			Flag.Is_Mode_Set = 2;
			Flag.Is_Uart_Rec = 1;  // 开启识别矩形  串口接收
			Flag.Is_OLED_Face = 5;
			break;
	}
}

void K4_Driver(void) {
	static uint8_t K4_State;
	K4_State++;
	switch(K4_State) {
		case 1:
			OLED_Clear();
			Flag.Is_Uart_Rec = 2;        // 开启矩形矫正  串口接收
			Flag.Is_OLED_Face = 4;
			Mode3_SET;
			break;
		case 2: 
			Identify_Upleft;
			break;
		case 3: 
			Identify_Upright;
			break;
		case 4: 
			Identify_Lowright;
			break;
		case 5: 
			Identify_Lowleft;
			break;
		case 6: 
			Identify_Midpoint;
			break;
	}
	K4_State = K4_State % 6;
}

