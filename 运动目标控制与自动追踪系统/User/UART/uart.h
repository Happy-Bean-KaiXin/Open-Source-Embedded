#ifndef __UART_H
#define __UART_H

#include "contorl.h"

extern int None; 

extern uint8_t uart2_rx_buffer[200];//串口1接收数据暂存数组
extern uint8_t BUFFER_SIZE;//串口接收字符总数
extern uint8_t rx_len_1;//接收到的数据个数
extern uint8_t recv_end_flag_1;//串口1接收完成标志位
extern uint8_t sti_buff[64];////发送数据暂存数组

void USART_data_processing(void);
void Send_Data(const uint8_t* ptr);

#endif
