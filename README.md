# wpc_mystm32_rx
# 一个任务在每隔1ms读adc的值，一个中断接收串口信息“OK\N",但如何合作一起协同工作呢？最好的办法就是使用事件组来解决。
# 一个事件组，等侍两个位： 它们的位分别在各自的事件设定，在下面的任务中阻塞，两个位都置位时，则发送一次数据，从而完美解决，一开机就不断发信息给ble,造成ble几秒后锁死或断开重连锁死的问题。
  #define BIT_RX_OKN ( 1 << 0 )
  #define BIT_ADC_OK ( 1 << 1 )
	EventBits_t uxBits;
  for(;;)
  {
    uxBits = xEventGroupWaitBits(
		  xControlUartEventGroup,       /* The event group being tested. */
		  BIT_ADC_OK | BIT_RX_OKN,      /* The bits within the event group to wait for. */
		  pdTRUE,                       /* BIT_0 and BIT_4 should be cleared before returning. */
	    pdTRUE,
		  portMAX_DELAY );              /* Wait a maximum of 100ms for either bit to be set. */
	    if( ( uxBits & ( BIT_ADC_OK | BIT_RX_OKN ) ) == ( BIT_ADC_OK | BIT_RX_OKN ) )
	    {
		    HAL_UART_Transmit_IT( &huart3, (uint8_t *)pData, sizeof(vol_cur_t) );
	    }
  }
