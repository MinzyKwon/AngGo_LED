/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define led_num 10
//led ?ïò?Çò?ãπ 16Î∞îÏù¥?ä∏ ?Ç¨?ö© (4bit*8(=?ïò?Çò?ùò ?Éâ)*RGBW=16Î∞îÏù¥?ä∏),+4?äî Î¶¨ÏÖã?ùÑ ?úÑ?ï®
#define led_spi_buffer_bytes led_num*4*4+4

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;
DMA_HandleTypeDef hdma_spi2_tx;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
struct sk_RGBW
{
	uint8_t red;
	uint8_t green;
	uint8_t blue;
	uint8_t white;
};
struct sk_RGBW sk6812_led[led_num]={0};
//?Ç¨?ö©?ï† LED Í∞??àòÎßåÌÅº Íµ¨Ï°∞Ï≤? Î≥??àò ?Éù?Ñ±
//ex) sk6812_led[0]?ùò Í≤ΩÏö∞, Ï≤´Î≤àÏß? ?àú?Ñú?ùò LEDÍ∞? Í∞?Ïß? ?Éâ?ÉÅ?ì§?ùÑ ?ùòÎØ?
uint8_t led_buffer[led_spi_buffer_bytes]={0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
void set_color(struct sk_RGBW *led_arr, uint8_t led_index)
{
	uint8_t i;
	uint32_t led_temp_buff[4]={0};
    //?ïò?Çò?ùò Î∞∞Ïó¥?ãπ ?ïò?Çò?ùò ?Éâ?ùÑ ?ã¥?ãπ ex)led_temp_buff[0]=Green, led_temp_buff[1]=Red
    //LED ?ç∞?ù¥?Ñ∞ ?ïò?Çò?ùò ÎπÑÌä∏Î•? ?ëú?òÑ?ïòÍ∏? ?úÑ?ï¥ 4Í∞úÏùò H, L ÎπÑÌä∏Í∞? ?ïÑ?öî?ïòÍ≥? Ï¥? 8?ûêÎ¶¨Ïùò LED ?ç∞?ù¥?Ñ∞Í∞? ?ïÑ?öî?ïòÎØ?Î°?
    //unsigned int 32bits Î∞∞Ïó¥ ?Ç¨?ö© (4*8=32)

    //LED ÎπÑÌä∏ 0?ùÑ ?ëú?òÑ?ïòÍ∏? ?úÑ?ï¥?Ñ† 1000=0x8, 1?ùÑ ?ëú?òÑ?ïòÍ∏? ?úÑ?ï¥?Ñ† 1100=0xc
    //?òà) Red=128?ùº ?ïå(1000 0000), led_temp_buff[1]=1100 1000 1000 ... 1000

    /*
    MSBÎ∂??Ñ∞ Ï∂úÎ†•?êòÎØ?Î°? led_temp_buff?ùò MSBÎ∂??Ñ∞ ?†Ñ?ã¨Î∞õÏ? ?Éâ ?ç∞?ù¥?Ñ∞?ùò 0, 1?ùÑ Íµ¨Î∂Ñ?ïú ?í§ Ï±ÑÏõå?ÇòÍ∞?
    ?†Ñ?ã¨Î∞õÏ? ?è¨?ù∏?Ñ∞ Íµ¨Ï°∞Ï≤? Ï£ºÏÜåÎ•? Ï∞∏Ï°∞?ï¥ LED Î∞∞Ïó¥?ùò MSBÎ∂??Ñ∞ ?ç∞?ù¥?Ñ∞Î•? Ï±ÑÏõå?ÇòÍ∞?
    ?òà) led_arr->green=1100 1000?ùº ?ïå (Green=200),
    Ï≤´Î≤àÏß?(i=0) Î£®ÌîÑ?óê?Ñú 0ÎßåÌÅº MSB Î∞©Ìñ•?úºÎ°? ?†ÑÏ≤? ÎπÑÌä∏Î•? ?ãú?îÑ?ä∏?ïòÍ≥?
    1000 0000?ùÑ Í≥±Ìïú ?í§, ?Çò?ò® Í∞íÏù¥ 0?ù¥?Éê 1?ù¥?Éê?óê ?î∞?ùº 0x8(0?ùº ?ïå) or 0xC(1?ùº ?ïå)Î•?
    (7-0)*4ÎßåÌÅº MSB Î∞©Ìñ•?úºÎ°? ?ãú?îÑ?ä∏ ?ãú?Ç® ?í§ 32ÎπÑÌä∏ Î∞∞Ïó¥?óê ???û•
    i=0?ùº ?ïå, led_temp_buff[0]=(31,MSB) 1100 0000 0000 .... 0000 (0,LSB)
    i=1?ùº ?ïå, led_temp_buff[0]=(31,MSB) 1100 1100 0000 .... 0000 (0,LSB)
    i=7?ùº ?ïå, led_temp_buff[0]=(31,MSB) 1100 1100 1000 1000 1100 1000 1000 1000 (0,LSB)

    SK6812?äî ?ç∞?ù¥?Ñ∞Î•? RGBW?àú?ù¥ ?ïÑ?ãå GRBW?àú?úºÎ°? ?ù∏?ãù?ï®
    */
	for(i=0;i<8;i++)
	{
		if((led_arr->green<<i)&0x80)	//Green
			led_temp_buff[0]+=(0xc<<(7-i)*4);
		else
			led_temp_buff[0]+=(0x8<<(7-i)*4);
		if((led_arr->red<<i)&0x80)	//Red
			led_temp_buff[1]+=(0xc<<(7-i)*4);
		else
			led_temp_buff[1]+=(0x8<<(7-i)*4);
		if((led_arr->blue<<i)&0x80)	//Blue
			led_temp_buff[2]+=(0xc<<(7-i)*4);
		else
			led_temp_buff[2]+=(0x8<<(7-i)*4);
		if((led_arr->white<<i)&0x80)	//WHITE
			led_temp_buff[3]+=(0xc<<(7-i)*4);
		else
			led_temp_buff[3]+=(0x8<<(7-i)*4);
	}


	/*
    led_index?äî ?ã§?†ú LED?ùò ?àú?ÑúÎ•? ?ùòÎØ?
    ex) led_index=0 : MOSI?óê ?ó∞Í≤∞Îêú Ï≤´Î≤àÏß? LED

    led_index*16?óê?Ñú *16?? ?ïò?Çò?ùò LED?ãπ 16Î∞îÏù¥?ä∏?ùò ?Éâ?ÉÅ ?ç∞?ù¥?Ñ∞Î•? ?Ç¨?ö©?ïòÍ∏? ?ïåÎ¨?
    ex) led_index=2?ùº Í≤ΩÏö∞(?ã§?†úÎ°úÎäî ?Ñ∏Î≤àÏß∏?óê ?úÑÏπòÌïú LED),
    ?†Ñ?ó≠Î≥??àòÎ°? ?Ñ†?ñ∏?êú led_buffer[] Î∞∞Ïó¥ Í∞? Ï§ëÏóê?Ñú led_index[32]~led_index[47]ÍπåÏ??ùò Î∞∞Ïó¥Îß? Í∞íÏù¥ Î∞îÎ?åÍ≥†
    ?ÇòÎ®∏Ï? Î∞∞Ïó¥?ì§?ùò Í∞íÏ? ?ú†Ïß??êú Ï±ÑÎ°ú Ï∂úÎ†•?ê®

	led_buffer Î∞∞Ïó¥?óê LED ?ç∞?ù¥?Ñ∞Î•? ?ûÖ?†•?ïú ?õÑ, DMAÎ•? ?ù¥?ö©?ï¥ LED?óê Ï∂úÎ†•
    Í∞? ?ÉâÍπîÎ≥ÑÎ°? 4(LOW+HIGH)*8(LED ?ç∞?ù¥?Ñ∞(=LED Í∞ïÎèÑ))=32bits=4bytes?ùò ?ç∞?ù¥?Ñ∞ ?Å¨Í∏∞Î?? Í∞ñÏùå
    ex) GREEN=200=1100 1000=1100 1100 1000 1000 ... 1000 ?ùº ?ïå,
    led_buffer[0]=led_temp_buff[0] 31~24 ÎπÑÌä∏Í∞? 1100 1100
    led_buffer[1]=led_temp_buff[0] 24~16 ÎπÑÌä∏Í∞? 1000 1000
    led_buffer[2]=led_temp_buff[0] 15~8 ÎπÑÌä∏Í∞? 1100 1000
    led_buffer[3]=led_temp_buff[0] 7~0 ÎπÑÌä∏Í∞? 1000 1000
    (MSBÎ∂??Ñ∞ Ï∂úÎ†•?êòÎØ?Î°? MSBÎ•? [0]Î≤? Î∞∞Ïó¥?óê ???û•)

    led_buffer[4]~[7]=RED, led_buffer[8]~[11]=BLUE, led_buffer[12]~[15]=WHITE
    */

	for(i=0;i<4;i++)
	{
		led_buffer[(i+led_index*16)]=(led_temp_buff[0]>>(3-i)*8);	//GREEN
		led_buffer[(i+led_index*16+4)]=(led_temp_buff[1]>>(3-i)*8);	//RED
		led_buffer[(i+led_index*16+8)]=(led_temp_buff[2]>>(3-i)*8);	//BLUE
		led_buffer[(i+led_index*16+12)]=(led_temp_buff[3]>>(3-i)*8);	//WHITE
	}
	HAL_SPI_Transmit_DMA(&hspi2,led_buffer,led_spi_buffer_bytes);
    //DMAÎ•? ?ù¥?ö©?ï¥ ?ç∞?ù¥?Ñ∞ Î≥¥ÎÉÑ, SPI2 ?Ç¨?ö©, led_buffer Î≥¥ÎÇº ?ç∞?ù¥?Ñ∞, led_spi_buffer_bytes ?†Ñ?Ü°?ï† byte?ùò ?Å¨Í∏?
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void set_rgb_value(struct sk_RGBW *current_rgb,uint8_t red,uint8_t green,uint8_t blue,uint8_t white)
{
	current_rgb->red=red;
	current_rgb->green=green;
	current_rgb->blue=blue;
	current_rgb->white=white;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */
  uint8_t i;
  uint8_t red, green, blue, white;

  red = 255;
  green = 0;
  blue = 0;
  white = 0;

  	  	  	  	  	  	  	  	  	  	  	  	  	  /* clear */

  set_rgb_value(&sk6812_led[0],0,0,0,0);	//Íµ¨Ï°∞Ï≤? Î∞∞Ïó¥ Î≥??àò(sk6812_led[0])?óê RGBWÍ∞? ?Ñ§?†ï
  for(i=0;i<led_num;i++)
  {
  	  set_color(&sk6812_led[0],i);	//?Ñ§?†ï?êú LED Í∞??àòÎßåÌÅº sk6812_led[0]?óê ???û•?êú RGBWÍ∞íÏùÑ Ï∂úÎ†•
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


	  	  	  	  	  	  	  	  	  	  	  	  	  	 /* Continuous Rainbow */

	  for (int i = 0; i < led_num; i++)
	  {
		  set_rgb_value(&sk6812_led[i], red, green, blue, white);
		  set_color(&sk6812_led[i], i);
	  }

	  HAL_Delay(15);  // color changing speed

	  if (red > 0 && green < 255 && blue == 0 && white == 0)
	  {
		  red--;
		  green++;
	  }
	  // Transition from green to blue
	  else if (red == 0 && green > 0 && blue < 255 && white == 0)
	  {
		  green--;
		  blue++;
	  }
	  // Transition from blue to white
	  else if (red == 0 && green == 0 && blue > 0 && white < 255)
	  {
		  blue--;
		  white++;
	  }
	  // Transition from white to red (restart the cycle)
	  else if (red < 255 && green == 0 && blue == 0 && white > 0)
	  {
		  white--;
		  red++;
	  }




	  	  	  	  	  	  	  	  	  	  	  	  	  	  	  /* LED filling */

//	  set_rgb_value(&sk6812_led[0],0,0,0,30); //example
//	  set_rgb_value(&sk6812_led[1],0,0,0,0); //off
//
//	  for(i = 0; i < led_num; i++){
//		  set_color(&sk6812_led[0],i);
//		  HAL_Delay(500);
//	  }
//
//	  for(i = 0; i < led_num; i++)
//	  {
//		  set_color(&sk6812_led[1],i);
//	  }




	  	  	  	  	  	  	  	  	  	  	  	  	  	 /* Rainbow color on each LED */


//	  set_rgb_value(&sk6812_led[0],255,0,0,0);
//	  set_rgb_value(&sk6812_led[1],255,165,0,0);
//	  set_rgb_value(&sk6812_led[2],255,255,0,0);
//	  set_rgb_value(&sk6812_led[3],165,255,0,0);
//	  set_rgb_value(&sk6812_led[4],0,255,0,0);
//	  set_rgb_value(&sk6812_led[5],0,165,255,0);
//	  set_rgb_value(&sk6812_led[6],0,0,255,0);
//	  set_rgb_value(&sk6812_led[7],65,0,140,0);
//	  set_rgb_value(&sk6812_led[8],148,0,211,0);
//	  set_rgb_value(&sk6812_led[9],0,0,0,50);
//
//	  for (i = 0; i < led_num; i++){
//		  set_color(&sk6812_led[i],i);
//	  }
//	  HAL_Delay(1000);



      /*
      sk6812_led[1]?óê ???û•?êú RGBWÍ∞íÏùÑ 2Î≤àÏß∏ LED?óê ?†Ñ?ã¨
      led_indexÍ∞íÏù¥ 0Î∂??Ñ∞ ?ãú?ûë?êòÎØ?Î°? led_index=1?? ?ëêÎ≤àÏß∏ LEDÎ•? ?ùòÎØ?
      sk6812_led[1]?ùò Í∞íÏùÑ ?ëêÎ≤àÏß∏ LED?óê ?ï†?ãπ?ïú ?ù¥?ú†?äî ?ã®?àú?ûà
      sk6812_led[0]?? Ï≤´Î≤àÏß? LED, sk6812_led[1]?? ?ëêÎ≤àÏß∏ LED?ùò Í∞íÏùÑ Í∞ñÎäî?ã§?äî ?àú?ÑúÎ•? ÎßûÏ∂∞Ï§?Í≤ÉÏùºÎø?
      Ï§ëÏöî?ïú Î∂?Î∂ÑÏ? set_color(&sk6812_led[1],1); ?óê?Ñú LED ?àú?ÑúÎ•? ?ùòÎØ∏Ìïò?äî led_index=1
      */
//	  HAL_Delay(1000);

//	  set_rgb_value(&sk6812_led[1],0,50,0,0);	//sk6812_led[1]?óê GREEN=255, ?ÇòÎ®∏Ï?=0 ?ûÖ?†•
//	  set_color(&sk6812_led[0],1);
//	  HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV6;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
