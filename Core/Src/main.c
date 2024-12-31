/*
 * main.c
 *
 *  Created on: Dec 18, 2024
 *      Author: ariat
 */
/*
 * PF1 - PF4 are LED Pins
 * PD0 and PD1 are CAN pins
 *
 */

#include "main.h"
#include"string.h"

void UART2_Init(void);
void Error_handler(void);
void GPIO_Init(void);
void CAN_Tx(void);
void CAN1_Init(void);
void SystemClock_Config_HSE(uint8_t clkFreq);
void CAN_Rx(void);
void CAN_Filter_Config(void);
void TIMER6_Init(void);
void CAN_TransmitLED(void);
void GPIO_Init(void);
void LED_Manager(uint8_t rcvd_msg);
void Send_response(void);

TIM_HandleTypeDef htim6;
UART_HandleTypeDef huart2;
CAN_HandleTypeDef hcan1;
uint32_t mailbox;
uint32_t FLatency=0;



int main(void){



	HAL_Init();
	srand(1);
	//clock config should be second always
	SystemClock_Config_HSE(SYS_CLK_FREQ_50_MHZ);
	UART2_Init();
	CAN1_Init();
	//TIMER6_Init();
	GPIO_Init();



	CAN_Filter_Config();

	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_TX_MAILBOX_EMPTY |CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF);

	if(HAL_CAN_Start(&hcan1) != HAL_OK){
			Error_handler();
		}


	while(1);

	return 0;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	//send LED_NUM
	//CAN_TransmitLED();


}
void LED_Manager(uint8_t rcvd_msg){

	if(rcvd_msg == 1){

		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);

	}
	else if(rcvd_msg == 2){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);



	}
	else if(rcvd_msg == 3){
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);



	}
	else{
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_SET);




	}



}

void GPIO_Init(void){

	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();

	GPIO_InitTypeDef ledgpio;
	ledgpio.Pin = GPIO_PIN_13;
	ledgpio.Pull = GPIO_NOPULL;
	ledgpio.Mode = GPIO_MODE_IT_FALLING;
	HAL_GPIO_Init(GPIOC, &ledgpio);

	GPIO_InitTypeDef LEDsPins;
	LEDsPins.Pin = GPIO_PIN_1;
	LEDsPins.Pull = GPIO_NOPULL;
	LEDsPins.Mode = GPIO_MODE_OUTPUT_PP;
	HAL_GPIO_Init(GPIOF, &LEDsPins);

	LEDsPins.Pin = GPIO_PIN_2;
	HAL_GPIO_Init(GPIOF, &LEDsPins);


	LEDsPins.Pin = GPIO_PIN_3;
	HAL_GPIO_Init(GPIOF, &LEDsPins);



	LEDsPins.Pin = GPIO_PIN_4;
	HAL_GPIO_Init(GPIOF, &LEDsPins);

	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_4, GPIO_PIN_RESET);




	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan){



}

void Send_response(void){

	CAN_TxHeaderTypeDef canh;


		char *our_msg = "Hi";

		canh.DLC = 2;
		canh.StdId = 0x67a;
		canh.IDE = CAN_ID_STD;
		canh.RTR = CAN_RTR_DATA;
		if(HAL_CAN_AddTxMessage(&hcan1, &canh, our_msg, &mailbox)!= HAL_OK){
			Error_handler();
		}



}

void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan){



}
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan){


}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan){

	CAN_RxHeaderTypeDef canRx;

			uint8_t rcvd_msg[8];
			char msg[50];


			if(HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &canRx, rcvd_msg) != HAL_OK){
				Error_handler();
			}


			if(canRx.StdId == 0x65D && canRx.RTR == 0){

				//This is a data frame
				LED_Manager(rcvd_msg[0]);
				sprintf(msg, "Message received: #%x\r\n",rcvd_msg[0]);


			}
			else if(canRx.StdId == 0x65D && canRx.RTR == 1){
				//A remote Frame
				Send_response();
				return;
			}







}
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan){




}

void TIMER6_Init(void){

	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 4999;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 10000-1 ;
	if(HAL_TIM_Base_Init(&htim6) != HAL_OK){

		Error_handler();
	}




}
void CAN_TransmitLED(void){

		CAN_TxHeaderTypeDef canh ={0};


		uint8_t our_msg = {0};

		canh.DLC = 1;
		canh.StdId = 0x65a;
		canh.IDE = CAN_ID_STD;
		canh.RTR = CAN_RTR_DATA;

		uint8_t random_number = (rand() % 4) ;
		our_msg = random_number;

		if(HAL_CAN_AddTxMessage(&hcan1, &canh, &our_msg, &mailbox)!= HAL_OK){
			Error_handler();


}
}

void SystemClock_Config_HSE(uint8_t clkFreq){

	RCC_OscInitTypeDef osc_init;
	RCC_ClkInitTypeDef clk_init;

	osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	osc_init.HSEState = RCC_HSE_BYPASS;
	osc_init.PLL.PLLState = RCC_PLL_ON;
	osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	switch(clkFreq)
	{
	case SYS_CLK_FREQ_180_MHZ: {



				//enable clock for pwr controller

				__HAL_RCC_PWR_CLK_ENABLE();

				//set voltage scalar so we run on max clock
				__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

				//turn on overdrive
				__HAL_PWR_OVERDRIVE_ENABLE();

		        osc_init.PLL.PLLM = 8;
				osc_init.PLL.PLLP = 1;
				osc_init.PLL.PLLN = 360;
				osc_init.PLL.PLLQ = 2;
				osc_init.PLL.PLLR = 2;

				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
				FLatency = FLASH_ACR_LATENCY_5WS;



		break;
	}
	case SYS_CLK_FREQ_50_MHZ:{
		osc_init.PLL.PLLM = 4;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLN = 100;
		osc_init.PLL.PLLQ = 7;
		osc_init.PLL.PLLR = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV2;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		FLatency = FLASH_ACR_LATENCY_1WS;

		break;
	}

	case SYS_CLK_FREQ_84_MHZ:{

				osc_init.PLL.PLLM = 8;
				osc_init.PLL.PLLP = 2;
				osc_init.PLL.PLLN = 168;
				osc_init.PLL.PLLQ = 7;
				osc_init.PLL.PLLR = 2;


				clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
				clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
				clk_init.AHBCLKDivider = RCC_SYSCLK_DIV2;
				clk_init.APB1CLKDivider = RCC_HCLK_DIV2;
				clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
				FLatency = FLASH_ACR_LATENCY_2WS;

				break;

	}

	case SYS_CLK_FREQ_120_MHZ:{

		osc_init.PLL.PLLM = 8;
		osc_init.PLL.PLLP = 2;
		osc_init.PLL.PLLN = 240;
		osc_init.PLL.PLLQ = 2;
		osc_init.PLL.PLLR = 2;

		clk_init.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
		clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;// look at the clock tree for more detail
		clk_init.AHBCLKDivider = RCC_SYSCLK_DIV1;
		clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
		clk_init.APB2CLKDivider = RCC_HCLK_DIV2;
		FLatency = FLASH_ACR_LATENCY_3WS;
				break;

	}
	default:
		return;

	}

	if(HAL_RCC_OscConfig(&osc_init) != HAL_OK){

		Error_handler();
	}
	if(HAL_RCC_ClockConfig(&clk_init, FLatency)!= HAL_OK){
		Error_handler();
	}

	//SYSTICK config
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);


}

void CAN_Filter_Config(void){

	CAN_FilterTypeDef can_filter;

	can_filter.FilterActivation = ENABLE;
	can_filter.FilterBank = 0;
	can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
	can_filter.FilterIdHigh = 0x0;
	can_filter.FilterIdLow = 0x0;
	can_filter.FilterMaskIdHigh = 0x00;
	can_filter.FilterMaskIdLow = 0x0;
	can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
	can_filter.FilterScale = CAN_FILTERSCALE_32BIT;

	if ( HAL_CAN_ConfigFilter(&hcan1, &can_filter) != HAL_OK){
		Error_handler();
	}

}
void CAN1_Init(void){

	hcan1.Instance = CAN1;
	hcan1.Init.AutoBusOff =  DISABLE;
	hcan1.Init.Mode = CAN_MODE_NORMAL;
	hcan1.Init.AutoRetransmission = ENABLE;
	hcan1.Init.AutoWakeUp = DISABLE;
	hcan1.Init.ReceiveFifoLocked = DISABLE;
	hcan1.Init.TimeTriggeredMode = DISABLE;
	hcan1.Init.TransmitFifoPriority = DISABLE;

	 hcan1.Init.Prescaler = 5;
	 hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
	 hcan1.Init.TimeSeg1 = CAN_BS1_8TQ;
	    hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;

	if(HAL_CAN_Init(&hcan1)!= HAL_OK){
		Error_handler();
	}




}

void UART2_Init(){

	huart2.Instance = USART3;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength =  UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	if(HAL_UART_Init(&huart2)!= HAL_OK ){

		Error_handler();
	}
}

void Error_handler(void){

}



