#ifndef __UART_H
#define __UART_H

#include "contorl.h"

extern int None; 

extern uint8_t uart2_rx_buffer[200];//����1���������ݴ�����
extern uint8_t BUFFER_SIZE;//���ڽ����ַ�����
extern uint8_t rx_len_1;//���յ������ݸ���
extern uint8_t recv_end_flag_1;//����1������ɱ�־λ
extern uint8_t sti_buff[64];////���������ݴ�����

void USART_data_processing(void);
void Send_Data(const uint8_t* ptr);

#endif
