#include "dma.h"


DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart6_rx;

//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx)
{ 
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
	}else 
	{
        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
	}
    
    __HAL_LINKDMA(&huart1,hdmarx,hdma_usart1_rx);    //将DMA与USART1联系起来(接收DMA)

	
	 //Rx DMA配置   除优先级是低等优先级，他是中等优先级，其他已配置成和正点原子论坛上帖子相同的模式。
    hdma_usart1_rx.Instance=DMA_Streamx;                            //数据流选择 DMA2_Stream2   串口2是DMA1_Stream5
    hdma_usart1_rx.Init.Channel=chx;                                //通道选择 DMA_CHANNEL_4    串口2是DMA_CHANNEL_4
    hdma_usart1_rx.Init.Direction=DMA_PERIPH_TO_MEMORY;             //外设到存储器
    hdma_usart1_rx.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
    hdma_usart1_rx.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
    hdma_usart1_rx.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
    hdma_usart1_rx.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
    hdma_usart1_rx.Init.Mode=DMA_CIRCULAR;                          //循环模式
    hdma_usart1_rx.Init.Priority=DMA_PRIORITY_VERY_HIGH;            //非常高等优先级
    hdma_usart1_rx.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
    hdma_usart1_rx.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_1QUARTERFULL;     
    hdma_usart1_rx.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
    hdma_usart1_rx.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
	  if(HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }
		 
    HAL_DMA_DeInit(&hdma_usart1_rx);   
    HAL_DMA_Init(&hdma_usart1_rx);
		
	  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
		
} 


//开启一次DMA传输
//huart:串口句柄
//pData：传输的数据指针
//Size:传输的数据量
void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
    HAL_DMA_Start(huart->hdmatx, (u32)pData, (uint32_t)&huart->Instance->DR, Size);//开启DMA传输
    
    huart->Instance->CR3 |= USART_CR3_DMAT;//使能串口DMA发送
}	  

void DMA2_Stream2_IRQHandler(void)
{

  HAL_DMA_IRQHandler(&hdma_usart1_rx);
	
} 

////开启一次DMA传输
////DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7 
////ndtr:数据传输量  
//void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
//{
// 
//	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
//	
//	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
//		
//	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
// 
//	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
//}	  



void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
