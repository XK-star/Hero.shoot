
/* Includes ------------------------------------------------------------------*/
#include "can.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;


/* CAN1 init function */
void MX_CAN1_Init(void)
{
	static CanTxMsgTypeDef  		Tx1Message;
	static CanRxMsgTypeDef      Rx1Message;
	CAN_FilterConfTypeDef  sFilterConfig;
	hcan1.Instance = CAN1;
	hcan1.Init.Prescaler = 3;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.SJW = CAN_SJW_1TQ;
	hcan1.Init.BS1 = CAN_BS1_9TQ;
	hcan1.Init.BS2 = CAN_BS2_4TQ;
	hcan1.Init.TTCM = DISABLE;
	hcan1.Init.ABOM = ENABLE;
	hcan1.Init.AWUM = DISABLE;
	hcan1.Init.NART = ENABLE;
	hcan1.Init.RFLM = DISABLE;
	hcan1.Init.TXFP = DISABLE;
	if (HAL_CAN_Init(&hcan1) != HAL_OK)
	{
	_Error_Handler(__FILE__, __LINE__);
	}
  
	/*CAN过滤器初始化*/
	sFilterConfig.FilterNumber = 0;                    //过滤器组0
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //工作在标识符屏蔽位模式
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //过滤器位宽为单个32位。
	/* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
	sFilterConfig.FilterIdHigh         = 0;                                //要过滤的ID高位 
	sFilterConfig.FilterIdLow          = 0; 								//要过滤的ID低位 
	sFilterConfig.FilterMaskIdHigh     = 0;                        //过滤器高16位每位必须匹配
	sFilterConfig.FilterMaskIdLow      = 0;                        //过滤器低16位每位必须匹配
	sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;           //过滤器被关联到FIFO 0
	sFilterConfig.FilterActivation = ENABLE;          				//使能过滤器
	sFilterConfig.BankNumber = 0;
	HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig);

	hcan1.pRxMsg = &Rx1Message;
	hcan1.pTxMsg = &Tx1Message;
	__HAL_CAN_ENABLE_IT(&hcan1, CAN_IT_FMP0);
	  
}
void MX_CAN2_Init(void)
{
	static CanTxMsgTypeDef  				Tx2Message;
	static CanRxMsgTypeDef         Rx2Message;
	
	
	hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SJW = CAN_SJW_1TQ;
  hcan2.Init.BS1 = CAN_BS1_7TQ;
  hcan2.Init.BS2 = CAN_BS2_6TQ;
  hcan2.Init.TTCM = DISABLE;
  hcan2.Init.ABOM = ENABLE;
  hcan2.Init.AWUM = DISABLE;
  hcan2.Init.NART = ENABLE;
  hcan2.Init.RFLM = DISABLE;
  hcan2.Init.TXFP = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
  /*CAN过滤器初始化*/
  CAN_FilterConfTypeDef sFilterConfig;
  sFilterConfig.FilterNumber = 14;                    //过滤器组2
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  //工作在标识符屏蔽位模式
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; //过滤器位宽为单个32位。
  /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
  sFilterConfig.FilterIdHigh = 0;                        //要过滤的ID高位
  sFilterConfig.FilterIdLow = 0;                         //要过滤的ID低位
  sFilterConfig.FilterMaskIdHigh = 0;                    //过滤器高16位每位必须匹配
  sFilterConfig.FilterMaskIdLow = 0x0000;                     //过滤器低16位每位必须匹配
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0; //过滤器被关联到FIFO 0
  sFilterConfig.FilterActivation = ENABLE;               //使能过滤器
  sFilterConfig.BankNumber = 14;
	HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig);
	
	hcan2.pTxMsg = &Tx2Message;
  hcan2.pRxMsg = &Rx2Message;
  __HAL_CAN_ENABLE_IT(&hcan2, CAN_IT_FMP0);
	HAL_CAN_Receive_IT(&hcan2,CAN_FIFO0);
}

void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* Peripheral clock enable */
      __HAL_RCC_CAN1_CLK_ENABLE();
  
    /**CAN1 GPIO Configuration    
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
	else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* Peripheral clock enable */
		__HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_CAN2_CLK_ENABLE();
    
  
    /**CAN2 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }

}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PB8     ------> CAN1_RX
    PB9     ------> CAN1_TX 
    */
    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_0|GPIO_PIN_1);
	
	HAL_NVIC_DisableIRQ(CAN1_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);

  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(hcan->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
  
    /**CAN1 GPIO Configuration    
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);
	
	  HAL_NVIC_DisableIRQ(CAN2_TX_IRQn);
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);

  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }

}

