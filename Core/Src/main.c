/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <OLED_Fonts.h>
#include <OLED_Icons.h>
#include <OLED.h>
#include <eeprom.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NUMAMOSTRAS 50
#define ADREF 4095
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/**********************************************************************************/
/*          ESTRUTURAS DE CONSTANTES PARA IDENTIFICAÇÃO DE TECLAS                 */
/**********************************************************************************/
typedef enum{
	OK = 1,
	BCK,
	UP,
	DW,
}tecla;

/*---------UINT8_T---------*/
uint8_t sentido,
/*----*/flagADC,
/*----*/flagUART,
/*----*/uartvpot,
/*----*/uartvpot_old,
/*----*/flagtransmit,
/*----*/flag_com[4] = {0},
/*----*/initp = 0,
/*----*/pacote_completo,
/*----*/rx_init,
/*----*/remote,
/*----*/byte_cont,
/*----*/power,
/*----*/potencia,
/*----*/pacote_completo,
/*----*/cmd_completo,
/*----*/flag_bt,
/*----*/deboucing,
/*----*/flagespera = 0;


/*---------UINT16_T---------*/
uint16_t register_cont,
/*-----*/adc_buffer[50] = {0},
/*-----*/timer_teste[4] = {0},
/*-----*/cont[8] = {0};

/*---------UINT32_T---------*/
/*uint32_t analog_pot = 0;*/ /*VIROU VAR LOCAL*/

/*---------CHAR----------*/
char data_rx_buff[50];
char rcv_byte;
char stt;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C3_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_ADC1_Init(void);
void trata_teclas(void);
void trata_tela(void);
void beep(void);
uint8_t mediaADC(uint32_t n);
void transmite_UART(void);
void initdisp(void);
void init_VARS(void);
void trata_rx(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
	flagADC = 1;												//QUANDO ACABAR UMA CONVERSÃO A A INTERRUPÇÃO IR�? DIRECIONAR PARA ESSE CALLBACK
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART3) {
		if(rcv_byte == '(' || rcv_byte == '['){					/**VERIFICA PRIMEIRO BYTE RECEBIDO, SE É OU NOA DE UM PACOTE PADRÃO**/
			rx_init = 1;
			byte_cont = 0;
			memset(data_rx_buff, 0x00, sizeof(data_rx_buff));
		}
		if(rcv_byte == ')'){									/*VERIFICA SE CHEGOU AO FIM DO PACOTE*/
			pacote_completo = 1;
		}
		if(rcv_byte == ']'){									/*VERIFICA SE CHEGOU AO FIM DO PACOTE*/
			cmd_completo = 1;
		}
		if(byte_cont >= 45)										/*VERIFICA SE O PACOTE PASSOU DO LIMITE ESPERADO*/
			byte_cont = 45;

		if(rx_init)												/*SE O PACOTE É VALIDO, ALOCA OS BITS NO BUFFER*/
		{
			data_rx_buff[byte_cont] = rcv_byte;
			byte_cont++;

			if(rcv_byte == ')' || rcv_byte == ']')				/*SE O PACOTE TERMINOU RESETA A VAR DO INDEXADOR DO BUFFER*/
				rx_init = 0;
		}
		/*TRIGGER RX*/
		if(!pacote_completo || !cmd_completo){
			HAL_UART_Receive_IT(&huart3, (uint8_t*)&rcv_byte, sizeof(rcv_byte));	/*RECEBE UM BYTE*/
		}
	}
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef * htim){//ENDEREÇO DE HTIM COMO PARAMETRO
	if(htim->Instance == TIM3){
		timer_teste[0]++;
		cont[0]++;
		if(flag_com[0])cont[6]++;
		if(timer_teste[0]>=250){
			//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
			timer_teste[0] = 0;
		}
		if(cont[0]>=1500 && !flagUART){
			flagtransmit = 1;
			cont[0] = 0;
		}
		if(cont[6]>2500){
			cont[6] = 0;
			flag_com[0] = 0;
		}
	}
}
void trata_teclas(void){
	/**********************************************************************************/
	/*                           TRATA EVENTO TECLA CONF                              */
	/**********************************************************************************/
	if(!HAL_GPIO_ReadPin(TCF_GPIO_Port, TCF_Pin)){
		deboucing = 50;										//CONTADOR DEBOUCING
		cont[2]=0x00;
		while(deboucing>0){
			deboucing--;									//DECREMENTO DO DEBOUCING
		}
		if(!flagespera)
			flag_bt = OK;									//SETA FLAG DE BOTÃO ACIONADO
	}
	if(flag_bt == OK)beep();
	/**********************************************************************************/
	/*                              TRATA TECLA CONF                                  */
	/**********************************************************************************/


	/**********************************************************************************/
	/*                           TRATA EVENTO TECLA VOLTA                             */
	/**********************************************************************************/
	if(!HAL_GPIO_ReadPin(TVL_GPIO_Port, TVL_Pin)){
		deboucing = 50;							//CONTADOR DEBOUCING
		cont[2]=0x00;
		while(deboucing>0){
			deboucing--;						//DECREMENTO DO DEBOUCING
		}
		if(!flagespera)
			flag_bt = BCK;							//SETA FLAG DE BOTÃO ACIONADO
	}
	if(flag_bt == BCK)beep();
	/**********************************************************************************/
	/*                               TRATA TECLA VOLTA                                */
	/**********************************************************************************/
	if(flag_bt==BCK){
		flag_bt = 0;
		flagUART = 1;
		stt = 'D';
		transmite_UART();
		flagUART = 0;
	}

	/**********************************************************************************/
	/*                           TRATA EVENTO TECLA CIMA                              */
	/**********************************************************************************/
	if(!HAL_GPIO_ReadPin(TCM_GPIO_Port, TCM_Pin)){
		deboucing = 50;												//CONTADOR DEBOUCING
		cont[2]=0x00;
		while(deboucing>0){
			deboucing--;											//DECREMENTO DO DEBOUCING
		}
		if(!flagespera)
			flag_bt = UP;												//SETA FLAG DE BOTÃO ACIONADO
	}
	if(flag_bt==UP)beep();
	/**********************************************************************************/
	/*                              TRATA TECLA CIMA                                  */
	/**********************************************************************************/
	if(flag_bt==UP){
		flag_bt = 0;
		flagUART = 1;
		stt = 'H';
		transmite_UART();
		trata_tela();
		while(!HAL_GPIO_ReadPin(TCM_GPIO_Port, TCM_Pin)){				//SEGURA O PROGRAMA ENQUANTO O BOTÃO
				//asm("nop");												//CONTINUAR PRESSIONADO
		}
		stt = 'D';
		transmite_UART();
		flagUART = 0;
	}
	/**********************************************************************************/
	/*                           TRATA EVENTO TECLA BAIXO                             */
	/**********************************************************************************/
	if(!HAL_GPIO_ReadPin(TBX_GPIO_Port, TBX_Pin)){
		deboucing = 50;							//CONTADOR DEBOUCING
		cont[2]=0x00;
		while(deboucing>0){
			deboucing--;						//DECREMENTO DO DEBOUCING
		}
		if(!flagespera)
			flag_bt = DW;							//SETA FLAG DE BOTÃO ACIONADO
	}
	if(flag_bt==DW)beep();
	/**********************************************************************************/
	/*                               TRATA TECLA BAIXO                                */
	/**********************************************************************************/
	if(flag_bt==DW){
		flag_bt = 0;
		flagUART = 1;
		stt = 'A';
		transmite_UART();
		trata_tela();
		while(!HAL_GPIO_ReadPin(TBX_GPIO_Port, TBX_Pin)){	//SEGURA O PROGRAMA ENQUANTO O BOTÃO
			//asm("nop");								//CONTINUAR PRESSIONADO
		}
		stt = 'D';
		transmite_UART();
		flagUART = 0;
	}
}
void trata_tela(void){
	char temp[4] = "00%";

	OLED_Clear(0);
	FontSet(Lucida_12);
	OLED_DrawStr("SENTIDO:", CENTER, 0, 1);
	if(stt == 'H'){
		FontSet(Lucida_12);
		OLED_DrawStr("HORARIO", CENTER, 16, 1);
	}
	if(stt == 'A'){
		FontSet(Lucida_12);
		OLED_DrawStr("ANTI-HORARIO", CENTER, 16, 1);
	}
	if(stt == 'D'){
		FontSet(Lucida_12);
		OLED_DrawStr("DESLIGADO", CENTER, 16, 1);
	}
	FontSet(Lucida_12);
	OLED_DrawStr("VELOCIDADE:", CENTER, 33, 1);
	sprintf(temp,"%2d%%", uartvpot);
	OLED_DrawStr(temp, CENTER, 49, 1);
	OLED_UpdateScreen();
}
void beep(void){
		HAL_GPIO_WritePin(BZ1_GPIO_Port, BZ1_Pin, GPIO_PIN_SET);
		HAL_Delay(25);
		HAL_GPIO_WritePin(BZ1_GPIO_Port, BZ1_Pin, GPIO_PIN_RESET);
}
uint8_t mediaADC(uint32_t n){
	uint32_t analog_pot = 0;
	uint8_t perc = 0;

	for(uint8_t i = 0; i<n; i++){			//FAZ A MEDIA DAS AMOSTRAS NO VETOR QUE O DMA ARMAZENA OS DADOS DO ADC
		analog_pot += adc_buffer[i];
	}

	analog_pot/=n;
	perc = (analog_pot*100)/ADREF;			//TIRAMOS A PORCENTAGEM DO AD.

	return perc;
}
void transmite_UART(void){
	char temp[16] = "[CPWM01,P=99,D]";
	uint8_t vpot = 0;

	vpot  = uartvpot;
	sprintf(temp,"[CPWM01,P=%2d,%c]", vpot, stt);
	HAL_GPIO_TogglePin(LED_TX_GPIO_Port, LED_TX_Pin);
	HAL_UART_Transmit(&huart3, (uint8_t*)&temp, sizeof(temp), HAL_MAX_DELAY);
	HAL_UART_Receive_IT(&huart3, (uint8_t*)&rcv_byte, sizeof(rcv_byte));
	HAL_Delay(50);
	//trata_rx();
}
void initdisp(void){
	OLED_Clear(0);
	OLED_DrawXBM(0 , 0, logov2);/*LOGO AUTOBAN*/
	OLED_UpdateScreen();
	HAL_Delay(3500);
	OLED_Clear(0);
	OLED_UpdateScreen();
}
void init_VARS(void){
	stt = 'D';
}
void trata_rx(void){
	if(data_rx_buff[1] != 'P' && data_rx_buff[2] != 'W' && data_rx_buff[3] != 'M' && data_rx_buff[4] != '-' && data_rx_buff[5] != 'O' && data_rx_buff[6] != 'K'){
		flag_com[0] = 1;
	}
	else
		flag_com[0] = 0;
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
  MX_I2C3_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
	/*CONFIGS PERIFERICOS*/
	HAL_TIM_Base_Start_IT(&htim3); //INCIA TIMER3 COM INTERRUPÇÃO NO OVERFLOW
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 50); /*INICIA O AD CO DMA NO MODO CIRCULAR*/
	HAL_Delay(500);
	/*CONFIG DISP*/
	OLED_Init(&hi2c3); //INICIANDO DISPLAY
	OLED_Clear(11);
	OLED_UpdateScreen();
	/*-----------*/
	/*CONFIGS EEPROM*/
	/*--------------*/
	/*INICIA AS VARIAVEIS DO SISTEMA*/
	initdisp(); //IMPRIMINDO O BITMAP
	init_VARS();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if(flagADC){
			uartvpot = mediaADC(NUMAMOSTRAS);
			if(uartvpot>99)
				uartvpot=99;
			flagADC = 0;
		}
		/*if(cmd_completo){
			trata_rx();

		}*/
		trata_teclas();
		if(flag_com[0]){
			OLED_Clear(0);
			FontSet(Lucida_12);
			OLED_DrawStr("FALHA NA", CENTER, 0, 1);
			OLED_DrawStr("COMUNICACAO", CENTER, 16, 1);
			OLED_UpdateScreen();
		}
		else
			trata_tela();
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.GainCompensation = 0;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_247CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00602173;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x00602173;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 239;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RF_SET_Pin|BZ1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_TX_GPIO_Port, LED_TX_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : RF_SET_Pin */
  GPIO_InitStruct.Pin = RF_SET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RF_SET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TVL_Pin TBX_Pin */
  GPIO_InitStruct.Pin = TVL_Pin|TBX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : TCF_Pin TCM_Pin */
  GPIO_InitStruct.Pin = TCF_Pin|TCM_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_TX_Pin */
  GPIO_InitStruct.Pin = LED_TX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(LED_TX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BZ1_Pin */
  GPIO_InitStruct.Pin = BZ1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(BZ1_GPIO_Port, &GPIO_InitStruct);

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
