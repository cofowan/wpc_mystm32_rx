/**
  ******************************************************************************
  * File Name          : USART.c
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdio.h"
#include "string.h"
extern osSemaphoreId UART_ReceiveBinarySemHandle;
extern uint8_t uart_receive_data ;
uint8_t RxData[20];

extern EventGroupHandle_t xEventGrop_LED;
extern __IO uint8_t over_vol_tx_flag;
extern __IO uint8_t ble_connect_flag;
extern EventGroupHandle_t xControlUartEventGroup;
/* USER CODE END 0 */

UART_HandleTypeDef huart3;

/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();
  
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();
  
    /**USART3 GPIO Configuration    
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
int fputc(int ch,FILE *f)
{ /* 
	特别注意，用哪个串口就用对应的串口，不然的话，
	可能一开机就死机了，因有机会调会不对应的串口就会卡死！
	另外，就是对应了串口号，若串口没有初始化，也同样会锁死！
	*/
	
	while((USART3->SR & 0X40)==0){}
    	USART3->DR = (uint8_t) ch;      
	return(ch);
	
	//HAL_UART_Transmit(&huart3,(uint8_t*)&ch,1,0xFFFF);
	
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
	BaseType_t xHigherPriorityTaskWoken, xResult;
	/* xHigherPriorityTaskWoken must be initialized to pdFALSE. */
	xHigherPriorityTaskWoken = pdFALSE;
	static uint16_t Index; 
	uint16_t temp;
	if(UartHandle == &huart3)
	{
		RxData[Index] = uart_receive_data;
		Index++;
		if(RxData[Index -1] == '\n' || RxData[Index -1] == '\r' || ( Index >= 20 ) )
		{
			ble_connect_flag = 1;
			//接收到数据进行处理！可以用来判断是否蓝牙断开！
			HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin); //这里设这个，在正常传输数据时，led快速闪烁，表明在快速传送数据中，非常好！！
			if( strncmp( ( const char* )RxData, "ok", 2 ) == 0 ) //判断是否接收到ok的信号
			{
				//HAL_UART_Transmit_IT(&huart3,(uint8_t *)"yes\n",4); //经调试ok
			
				xResult = xEventGroupSetBitsFromISR(
				xControlUartEventGroup, 
				BIT_RX_OKN, 
				&xHigherPriorityTaskWoken );
				if( xResult != pdFAIL )
				{			
					portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
				}
			}
			
			Index=0;
		}
		HAL_UART_Receive_IT(&huart3, &uart_receive_data, 1);
	}
}
/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
