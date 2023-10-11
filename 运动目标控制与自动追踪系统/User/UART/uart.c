#include "uart.h" 
/*
 *
 * 使用说明：
 * 在stm32f1xx_it.c找到所需要要‘使用的串口’中断服务函数，在中断服务函数中加入以下代码。
 * 例如下面是针对于串口1使用所举的例子
void USART1_IRQHandler(void)
{
  * USER CODE BEGIN USART1_IRQn 0 *
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //获取IDLE标志位
    if((tmp_flag != RESET))//idle标志被置位
    {
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);//清除标志位
      HAL_UART_DMAStop(&huart1);
      temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// 获取DMA中未传输的数据个数

      rx_len_1 =  BUFFER_SIZE - temp; //总计数减去未传输的数据个数，得到已经接收的数据个数
      recv_end_flag_1 = 1;  // 接受完成标志位置1
    }
  HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,BUFFER_SIZE);//重新打开DMA接收
  * USER CODE END USART1_IRQn 0 *
  HAL_UART_IRQHandler(&huart1);
  * USER CODE BEGIN USART1_IRQn 1 *

  * USER CODE END USART1_IRQn 1 *
}
 * 除此之外：
 * 在main函数中的主循环开始前加入使能语句
 *	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //使能IDLE中断
 *	HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,BUFFER_SIZE);
 *
 *
 *
 * */

#include "stdlib.h"
#include "string.h"

int None = 1; 

uint8_t uart2_rx_buffer[200]; // 串口接收数据暂存数组
uint8_t BUFFER_SIZE = 200;    // 串口接收字符总数
uint8_t rx_len_1;             // 接收到的数据个数
uint8_t recv_end_flag_1;      // 串口接收完成标志位
uint8_t sti_buff[64];         // 发送数据暂存数组


/**
 * 函数名:USART_data_processing
 * 描述:处理接收到的串口数据
 * 输入:void
 * 输出:void
 */
void USART_data_processing(void)
{
	if(recv_end_flag_1==1)//串口中断接收标志位
	{
			if(Flag.Is_Uart_Rec == 0) {
        // 解析字符串，查找"x="和"y="的位置
				const char*px = strstr((const char*)uart2_rx_buffer, "X=");
				const char*py = strstr((const char*)uart2_rx_buffer, "Y=");
				const char*pn = strstr((const char*)uart2_rx_buffer,"None");
				if(pn != NULL)  None = 1;
				else if(px && py)
				{
					None = 0;
					Flag.X_AXIS = atoi(px + 2); // +2是为了跳过"x="
					Flag.Y_AXIS = atoi(py + 2); // +2是为了跳过"y="
					memset(uart2_rx_buffer, 0, 200);
				}
			}
			else if(Flag.Is_Uart_Rec == 1) {		
          const char*px1 = strstr((const char*)uart2_rx_buffer, "p1");
          const char*py1 = strstr((const char*)uart2_rx_buffer, "p2");
          const char*px2 = strstr((const char*)uart2_rx_buffer, "p3");
          const char*py2 = strstr((const char*)uart2_rx_buffer, "p4");
          const char*px3 = strstr((const char*)uart2_rx_buffer, "p5");
          const char*py3 = strstr((const char*)uart2_rx_buffer, "p6");
          const char*px4 = strstr((const char*)uart2_rx_buffer, "p7");
          const char*py4 = strstr((const char*)uart2_rx_buffer, "p8");

          if(px1&&py1&&px2&&py2&&px3&&py3&&px4&&py4)
          {

						Black_Retanx[0] = atoi(px1 + 2);
            Black_Retany[0] = atoi(py1 + 2);
            Black_Retanx[1] = atoi(px2 + 2);
            Black_Retany[1] = atoi(py2 + 2);

            Black_Retanx[2] = atoi(px3 + 2);
            Black_Retany[2] = atoi(py3 + 2);
            Black_Retanx[3] = atoi(px4 + 2);
            Black_Retany[3] = atoi(py4 + 2);
						
            memset(uart2_rx_buffer,0,200);     
						Flag.Cnt++;
						if(Flag.Cnt >= 3) {
							Flag.Cnt = 0;
							Flag.Is_Uart_Rec = 0;
						}
          }
					
        }
				else if(Flag.Is_Uart_Rec == 2) {		
          const char*pane1x = strstr((const char*)uart2_rx_buffer, "r1");
          const char*pane1y = strstr((const char*)uart2_rx_buffer, "r2");
          const char*pane2x = strstr((const char*)uart2_rx_buffer, "r3");
          const char*pane2y = strstr((const char*)uart2_rx_buffer, "r4");
          const char*pane3x = strstr((const char*)uart2_rx_buffer, "r5");
          const char*pane3y = strstr((const char*)uart2_rx_buffer, "r6");
          const char*pane4x = strstr((const char*)uart2_rx_buffer, "r7");
          const char*pane4y = strstr((const char*)uart2_rx_buffer, "r8");
					const char*pane5x = strstr((const char*)uart2_rx_buffer, "r9");
          const char*pane5y = strstr((const char*)uart2_rx_buffer, "r0");
					
          if(pane1x && pane1y && pane2x && pane2y && pane3x &&
						pane3y && pane4x && pane4y && pane5x && pane5y)
          {
            RetangleX[0] = atoi(pane1x + 2);
            RetangleY[0] = atoi(pane1y + 2);
            RetangleX[1] = atoi(pane2x + 2);
            RetangleY[1] = atoi(pane2y + 2);

            RetangleX[2] = atoi(pane3x + 2);
            RetangleY[2] = atoi(pane3y + 2);
            RetangleX[3] = atoi(pane4x + 2);
            RetangleY[3] = atoi(pane4y + 2);
						
						Flag.x_centry = atoi(pane5x + 2);
						Flag.y_centry = atoi(pane5y + 2);
						
            memset(uart2_rx_buffer,0,200);				    
          }
					
        }
      }
		recv_end_flag_1 = 0; //标志位置零等待下一次接收
		rx_len_1 = 0;//清除接收到数据的个数

		memset(uart2_rx_buffer,0,sizeof(uart2_rx_buffer)); //初始化接收数组
}

// 进行对帧头帧尾数据判断 
int data_test(int* data) {
	if(data[0] != 0xa3) return 0;  // 帧头
	if(data[1] != 0xb3) return 0;  // 帧头
	if(data[5] != 0xc3) return 0;  // 帧尾
	
	return 1;
}

// 发送端：0xa3, 0xb3, 数据, 0xc3
void Send_Data(const uint8_t* ptr) {
	int i = 0;
	int len = strlen((const char* )ptr) + 3;   // 加上帧头帧尾的数据
	uint8_t* data;
	
//	data[0] = 0xa3;
//	data[1] = 0xb3;
//	(data + 2) = ptr;
	while(i < len) {
		HAL_UART_Transmit_DMA(&huart2, &data[i], 1);
		i++;
	}
}
