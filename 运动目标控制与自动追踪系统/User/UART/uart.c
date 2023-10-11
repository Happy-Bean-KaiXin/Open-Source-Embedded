#include "uart.h" 
/*
 *
 * ʹ��˵����
 * ��stm32f1xx_it.c�ҵ�����ҪҪ��ʹ�õĴ��ڡ��жϷ����������жϷ������м������´��롣
 * ��������������ڴ���1ʹ�����ٵ�����
void USART1_IRQHandler(void)
{
  * USER CODE BEGIN USART1_IRQn 0 *
    uint32_t tmp_flag = 0;
    uint32_t temp;
    tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
    if((tmp_flag != RESET))//idle��־����λ
    {
      __HAL_UART_CLEAR_IDLEFLAG(&huart1);//�����־λ
      HAL_UART_DMAStop(&huart1);
      temp  =  __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);// ��ȡDMA��δ��������ݸ���

      rx_len_1 =  BUFFER_SIZE - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
      recv_end_flag_1 = 1;  // ������ɱ�־λ��1
    }
  HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,BUFFER_SIZE);//���´�DMA����
  * USER CODE END USART1_IRQn 0 *
  HAL_UART_IRQHandler(&huart1);
  * USER CODE BEGIN USART1_IRQn 1 *

  * USER CODE END USART1_IRQn 1 *
}
 * ����֮�⣺
 * ��main�����е���ѭ����ʼǰ����ʹ�����
 *	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); //ʹ��IDLE�ж�
 *	HAL_UART_Receive_DMA(&huart1,uart1_rx_buffer,BUFFER_SIZE);
 *
 *
 *
 * */

#include "stdlib.h"
#include "string.h"

int None = 1; 

uint8_t uart2_rx_buffer[200]; // ���ڽ��������ݴ�����
uint8_t BUFFER_SIZE = 200;    // ���ڽ����ַ�����
uint8_t rx_len_1;             // ���յ������ݸ���
uint8_t recv_end_flag_1;      // ���ڽ�����ɱ�־λ
uint8_t sti_buff[64];         // ���������ݴ�����


/**
 * ������:USART_data_processing
 * ����:������յ��Ĵ�������
 * ����:void
 * ���:void
 */
void USART_data_processing(void)
{
	if(recv_end_flag_1==1)//�����жϽ��ձ�־λ
	{
			if(Flag.Is_Uart_Rec == 0) {
        // �����ַ���������"x="��"y="��λ��
				const char*px = strstr((const char*)uart2_rx_buffer, "X=");
				const char*py = strstr((const char*)uart2_rx_buffer, "Y=");
				const char*pn = strstr((const char*)uart2_rx_buffer,"None");
				if(pn != NULL)  None = 1;
				else if(px && py)
				{
					None = 0;
					Flag.X_AXIS = atoi(px + 2); // +2��Ϊ������"x="
					Flag.Y_AXIS = atoi(py + 2); // +2��Ϊ������"y="
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
		recv_end_flag_1 = 0; //��־λ����ȴ���һ�ν���
		rx_len_1 = 0;//������յ����ݵĸ���

		memset(uart2_rx_buffer,0,sizeof(uart2_rx_buffer)); //��ʼ����������
}

// ���ж�֡ͷ֡β�����ж� 
int data_test(int* data) {
	if(data[0] != 0xa3) return 0;  // ֡ͷ
	if(data[1] != 0xb3) return 0;  // ֡ͷ
	if(data[5] != 0xc3) return 0;  // ֡β
	
	return 1;
}

// ���Ͷˣ�0xa3, 0xb3, ����, 0xc3
void Send_Data(const uint8_t* ptr) {
	int i = 0;
	int len = strlen((const char* )ptr) + 3;   // ����֡ͷ֡β������
	uint8_t* data;
	
//	data[0] = 0xa3;
//	data[1] = 0xb3;
//	(data + 2) = ptr;
	while(i < len) {
		HAL_UART_Transmit_DMA(&huart2, &data[i], 1);
		i++;
	}
}
