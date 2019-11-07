/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "stdio.h"
#include "string.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
int32_t com(const void*a,const void*b)
{
	/* ��С��������  */
	//return *(int*)a-*(int*)b;
	/* �Ӵ�С����  */
	//return *(int*)b-*(int*)a;
	/* ��С��������  */
	return *(int32_t *)a - *(int32_t *)b;
}

uint16_t crc16_compute(uint8_t const * p_data, uint32_t size, uint16_t const * p_crc)
{
    uint16_t crc = (p_crc == NULL) ? 0xFFFF : *p_crc;

    for (uint32_t i = 0; i < size; i++)
    {
        crc  = (uint8_t)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (uint8_t)(crc & 0xFF) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xFF) << 4) << 1;
    }

    return crc;
}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
__IO static uint8_t over_load_rx = 0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId UART_RxTaskHandle;
osThreadId Get_ADC_TaskHandle;
osThreadId Interrupt_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
extern uint8_t uart_receive_data ;
extern uint8_t RxData[100];

extern UART_HandleTypeDef huart3;
extern __IO wireless_charge_data_t myWC_dat;
extern uint32_t myADC[2];
extern __IO uint8_t over_vol_tx_flag;
__IO uint8_t ble_connect_flag = 0;
extern 	__IO SignalData mydata ;

led_blinky_t mBliky = {
			.mode = LED_NULL, 
			.gpioport = LED_GPIO_Port, 
			LED_Pin };

/* Declare a variable to hold the created event group. */
EventGroupHandle_t xControlUartEventGroup = NULL;
vol_cur_t *pData = NULL;
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void UART_Receive_TaskHandle(void const * argument);
void Get_ADC_Task_Handle(void const * argument);
void Interrupt_task_handle(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    xControlUartEventGroup = xEventGroupCreate();
	/* Was the event group created successfully? */
	if( xControlUartEventGroup == NULL )
	{
		
	}
	else
	{
	/* The event group was created. */
	}
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of UART_RxTask */
 // osThreadDef(UART_RxTask, UART_Receive_TaskHandle, osPriorityAboveNormal, 0, 512);
 // UART_RxTaskHandle = osThreadCreate(osThread(UART_RxTask), NULL);

  /* definition and creation of Get_ADC_Task */
  osThreadDef(Get_ADC_Task, Get_ADC_Task_Handle, osPriorityAboveNormal, 0, 768);
  Get_ADC_TaskHandle = osThreadCreate(osThread(Get_ADC_Task), NULL);

  /* definition and creation of Interrupt_task */
 // osThreadDef(Interrupt_task, Interrupt_task_handle, osPriorityHigh, 0, 512);
 // Interrupt_taskHandle = osThreadCreate(osThread(Interrupt_task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
 
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
    
    
    
    
    

  /* USER CODE BEGIN StartDefaultTask */
	EventBits_t uxBits;
  /* Infinite loop */
  for(;;)
  {
    uxBits = xEventGroupWaitBits(
		xControlUartEventGroup, /* The event group being tested. */
		BIT_ADC_OK | BIT_RX_OKN, /* The bits within the event group to wait for. */
		pdTRUE, /* BIT_0 and BIT_4 should be cleared before returning. */
		//pdFALSE, /* Don't wait for both bits, either bit will do. */
	   pdTRUE,
		portMAX_DELAY );/* Wait a maximum of 100ms for either bit to be set. */
	 if( ( uxBits & ( BIT_ADC_OK | BIT_RX_OKN ) ) == ( BIT_ADC_OK | BIT_RX_OKN ) )
	{
		//HAL_UART_Transmit_IT(&huart3,(uint8_t *)"yes\n",4); //������ok
		HAL_UART_Transmit_IT( &huart3, (uint8_t *)pData, sizeof(vol_cur_t) );
	}
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_UART_Receive_TaskHandle */
/**
* @brief Function implementing the UART_RxTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART_Receive_TaskHandle */
void UART_Receive_TaskHandle(void const * argument)
{
  /* USER CODE BEGIN UART_Receive_TaskHandle */
	
  /* Infinite loop */
  for(;;)
  {
	osDelay(10);
  }
  /* USER CODE END UART_Receive_TaskHandle */
}

/* USER CODE BEGIN Header_Get_ADC_Task_Handle */
/**
* @brief Function implementing the Get_ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Get_ADC_Task_Handle */
void Get_ADC_Task_Handle(void const * argument)
{
  /* USER CODE BEGIN Get_ADC_Task_Handle */
	BaseType_t xHigherPriorityTaskWoken, xResult;
	
	#define SAMPLE_CNT 59
	#define SAMPLE_MIDDLE_VALUE ( SAMPLE_CNT / 2 )
	static uint32_t adc_cur[SAMPLE_CNT] = {0};
	static uint32_t adc_vol[SAMPLE_CNT] = {0};
	static uint8_t index = 0;
	static uint32_t cnt = 0;
	uint32_t ulNotifycationValue;
	pData = (vol_cur_t *)calloc(1,sizeof(vol_cur_t));
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 8 );
	vTaskDelay(700); //�տ�ʼʱ��ʱ500ms��ȡ��
  /* Infinite loop */
  for(;;)
  {
	  ulNotifycationValue = ulTaskNotifyTake( pdFALSE, xMaxBlockTime );//�ȴ�adc�жϲ�������֪ͨ
	  if( ulNotifycationValue == 1 )//ת�����
	  { //����ļ��Լ1ms
		 
		  if( index < SAMPLE_CNT )
		  {
			  adc_cur[index] = myADC[0];
			  adc_vol[index] = myADC[1];
			  index++;
		  }
		  else if( index >= SAMPLE_CNT )
		  { //ȡ�м�ֵ�㷨
			  index = 0;
			  qsort( adc_cur, SAMPLE_CNT, sizeof( uint32_t ), com ); //��С��������
			  qsort( adc_vol, SAMPLE_CNT, sizeof( uint32_t ), com );	
			  pData->data.vol = adc_vol[SAMPLE_MIDDLE_VALUE];
			  pData->data.cur = adc_cur[SAMPLE_MIDDLE_VALUE];
			  pData->ch = '\n';
			  if( ble_connect_flag == 1) //ֻ�л���BLE,�ŷ��ͣ��������
			  {		
				  // HAL_UART_Transmit_IT( &huart3, (uint8_t *)pData, sizeof(vol_cur_t) ); //ͨ�����ڷ��ͳ�ȥ
			  }
				xResult = xEventGroupSetBits(
				xControlUartEventGroup, 
				BIT_ADC_OK );
	
			  
		  }
		 
		  
		  if(cnt++ > 200) //200ms ���һ��,����������״̬
		  {
			  cnt = 0;
			  if(ble_connect_flag == 1) //�ڴ����ж���1�����жϱ�������������
			  {
				  //ble_connect_flag = 0;
				  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);//����LED
			  }
			  else //�����������ӣ���Ϩled
			  {
				  HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);//����LED 
			  }
		  }
		 
		   
	  }
	  else //��ʱ
	  {
		  //��ʱ����
	  }
  }
  /* USER CODE END Get_ADC_Task_Handle */
}

/* USER CODE BEGIN Header_Interrupt_task_handle */
/**
* @brief Function implementing the Interrupt_task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Interrupt_task_handle */
void Interrupt_task_handle(void const * argument)
{
  /* USER CODE BEGIN Interrupt_task_handle */
	uint32_t times_limit = 0;
	uint32_t ulNotificationValue;
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
	
  /* Infinite loop */
  for(;;)
  {
   
		ulNotificationValue = ulTaskNotifyTake( pdFALSE, xMaxBlockTime ); //�ȴ�����֪ͨ
		if( ulNotificationValue == 1 ) //������ѹ�ж�
		{
			// The transmission ended as expected. 
			//������⵽10���ж��ź�ʱ����ȷ�ϣ�ÿ50ms����ȷ��һ��
			if( times_limit >= 2 ) //ֻ��50ms���ټ�⣡�����ٶȻᵼ����������,Ҳ���Ƿ����жϺ󣬱���50ms�������жϣ�
			{
				 over_load_rx = 1;
			}
			times_limit++; //ֻ��45ms���ټ�⣡�����ٶȻᵼ����������
		}
		else //45ms��û���ֹ�ѹ�ж�
		{
			// The call to ulTaskNotifyTake() timed out. 
			times_limit = 0; //ֻ��200ms���ټ�⣡�����ٶȻᵼ����������
			 
		}
		
  }
  /* USER CODE END Interrupt_task_handle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
