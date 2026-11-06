#include "main.h"
#include "NRF24L01_Reg.h"
#include "NRF24L01.h"
#include "MySPI.h"
#include "Delay.h"

#include <stdint.h>

uint8_t T_ADDR[5]={0x11,0x22,0x33,0x44,0x55};
uint8_t R_ADDR[5]={0x11,0x22,0x33,0x44,0x55};

void NRF24L01_GPIO_Init(void)
{
    /* 引脚模式由 MX_GPIO_Init() 配置，这里只设置初始电平 */
    MySPI_W_CE(0);      /* 暂停无线收发 */
    MySPI_W_CSN(1);     /* 取消 SPI 片选 */
    MySPI_W_SCLK(0);    /* SPI 时钟空闲为低 */
    MySPI_W_MOSI(0);
}

uint8_t NRF24L01_R_IRQ(void)
{
    /* IRQ 低电平表示有事件，保持原来的返回值语义 */
    return (uint8_t)HAL_GPIO_ReadPin(NRF_IRQ_GPIO_Port,
                                   NRF_IRQ_Pin);
}

//写入NRF24L01寄存器一个数据的操作
void Write_NRF24L01_Val(uint8_t Reg,uint8_t Value)
{
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	MySPI_SwapData(Value);								//数据
	MySPI_W_CSN(1);												//停止选中从机
}

//读NRF24L01寄存器的操作
uint8_t Read_NRF24L01_Val(uint8_t Reg)
{
	uint8_t value;
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	value=MySPI_SwapData(NOP);						//数据
	MySPI_W_CSN(1);												//停止选中从机
	return value;
}

//写入NRF24L01寄存器一个数组的操作
void Write_NRF24L01_Buf(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	for(i=0;i<Len;i++)
	{
		MySPI_SwapData(Buf[i]);
	}
	MySPI_W_CSN(1);												//停止选中从机
}

//接收NRF24L01寄存器一个数组的操作
void Receive_NRF24L01_Buf(uint8_t Reg,uint8_t *Buf,uint8_t Len)
{
	uint8_t i;
	MySPI_W_CSN(0);												//选中从机
	MySPI_SwapData(Reg);									//指令
	for(i=0;i<Len;i++)
	{
		Buf[i]=MySPI_SwapData(NOP);
	}
	MySPI_W_CSN(1);												//停止选中从机
}

//NRF24L01初始�?
void NRF24L01_Init(void)
{
	NRF24L01_GPIO_Init();
	MySPI_W_CE(0);
	
	Write_NRF24L01_Buf(W_REGISTER+TX_ADDR,T_ADDR,5);//配置发送地址
	Write_NRF24L01_Buf(W_REGISTER+RX_ADDR_P0,R_ADDR,5);//配置接收地址
	Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0B);//RX, 1-byte CRC, powered up
	Write_NRF24L01_Val(W_REGISTER+EN_AA,0x01);//配置自动应答模式
	Write_NRF24L01_Val(W_REGISTER+SETUP_AW,0x03);//地址宽度5字节
	Write_NRF24L01_Val(W_REGISTER+RF_CH,0x02);//平衡车遥控器固定信道
	Write_NRF24L01_Val(W_REGISTER+RF_SETUP,0x0E);//2Mbps
	Write_NRF24L01_Val(W_REGISTER+RX_PW_P0,32);//固定32字节负载
	Write_NRF24L01_Val(W_REGISTER+EN_RXADDR,0x01);//使能接通通道0
	Write_NRF24L01_Val(W_REGISTER+SETUP_RETR,0x03);//自动应答重试设置
	Write_NRF24L01_Val(FLUSH_RX,NOP);
	
	HAL_Delay(2); // 等待退出掉电状态
	MySPI_W_CE(1);
}

//接收数据
uint8_t Receive_NRF24L01_Data(uint8_t*Buf)
{
	uint8_t Status; 																	//定义状态量
	Status=Read_NRF24L01_Val(R_REGISTER+STATUS);			//获取当前接收状态量
	if(Status & RX_OK)																//获取当前接收状态量，若为真则说明接收消息成�?
	{																				
		Receive_NRF24L01_Buf(R_RX_PAYLOAD,Buf,32);  		//将收到的消息存入Buff数组
		Write_NRF24L01_Val(FLUSH_RX,NOP);						    //清除接收寄存�?
		Write_NRF24L01_Val(W_REGISTER+STATUS,Status);   //清除中断
		Delay_us(150);
		return 1;
	}	
	return 0;
}

//send data, 32-byte payload, 1-byte CRC to match the paired remote
uint8_t Send(uint8_t *Buf)
{
	uint8_t Status;																		//status register
	uint32_t timeout;

	Write_NRF24L01_Val(FLUSH_TX,NOP);              //flush any stale TX payload
	Write_NRF24L01_Buf(W_TX_PAYLOAD,Buf,32);   	//write full 32-byte payload (remote RX_PW_P0 is 32)

	MySPI_W_CE(0);
	Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0A);    //TX mode: PWR_UP+EN_CRC, CRCO=0 for 1-byte CRC
	MySPI_W_CE(1);

	timeout=200000;                                 //bounded wait, a few ms at 72MHz
	while(NRF24L01_R_IRQ()==1)               	   //wait for IRQ pin to go low
	{
		if((--timeout)==0)                         //timeout: drop this frame, never block control loop
		{
			Write_NRF24L01_Val(FLUSH_TX,NOP);              //clear TX FIFO
			Status=Read_NRF24L01_Val(R_REGISTER+STATUS);
			Write_NRF24L01_Val(W_REGISTER+STATUS,Status);  //clear interrupt flags
			Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0B);    //restore RX mode, 1-byte CRC
			MySPI_W_CE(1);
			return 0;
		}
	}
	Status=Read_NRF24L01_Val(R_REGISTER+STATUS);
	if(Status & MAX_TX)                         			//max retries reached
	{
		Write_NRF24L01_Val(FLUSH_TX,NOP);              //clear TX FIFO
		Write_NRF24L01_Val(W_REGISTER+STATUS,Status);	  //clear interrupt
		Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0B);    //restore RX mode, 1-byte CRC
		MySPI_W_CE(1);
		return MAX_TX;
	}
	if(Status & TX_OK)                        			  //send success
	{
		Write_NRF24L01_Val(W_REGISTER+STATUS,Status);   //clear interrupt
		Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0B);    //restore RX mode, 1-byte CRC
		MySPI_W_CE(1);
		return TX_OK;
	}
	Write_NRF24L01_Val(W_REGISTER+CONFIG,0x0B);    //restore RX mode on every exit path
	MySPI_W_CE(1);
	return 0;
}




