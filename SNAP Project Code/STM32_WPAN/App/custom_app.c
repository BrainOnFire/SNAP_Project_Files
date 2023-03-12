/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
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
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app_ble.h"
#include "sht31.h"
#include <stdio.h>
#include "stm32_seq.h"
#include "fonts.h"
#include "ssd1306.h"
#include "usb_device.h"
#include "custom_app.h"
#include "stm32_lpm.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* SNAP_SVC */
  /* USER CODE BEGIN CUSTOM_APP_Context_t */
  uint8_t				BT_SW_Status;
  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[247];
uint8_t NotifyCharData[247];

/* USER CODE BEGIN PV */
uint8_t verificador1, amp_counter, chemical_compound, activador_ble, envio_finalizado;
uint8_t menu_counter = 0;
uint8_t selected_main = 0;
uint8_t selected_second = 0;
uint8_t entered_second = 0;
uint8_t entered_main = 0;
uint8_t done_1 = 0;
uint8_t done_2 = 0;
uint32_t adc_value[2];

const float A = 3.1725417;
const float B = 0.73272727;
float humidity, temperature;
float batt_percentage = 0.0;
float val_lineal = 0.0;
float val_log = 0.0;
char buf[10];

char *options_menu[3] = {
		"Elegir opcion",
		"Medir quimico",
		"Medir temperatura y humedad"
		//"Activar Bluetooth"
};

char *options_elements[5] = {
		"Elegir opcion",
		"Nitrogeno",
		"Fosforo",
		"Potasio",
		"Retornar a menu"
};

sht3x_handle_t sht3x_handle = {
    .i2c_handle = &hi2c1,
    .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SNAP_SVC */

/* USER CODE BEGIN PFP */
//void BLE_Function(void);
//void ADC_BATT_CH6(void);
//void ADC_OPAMP_CH12(void);
void TempHum_Function(void);
void ADCCheck_Function(void);
void WaitUser_Function(void);
void Battery_Percentage(void);
void DisplayMenu_Function(void);
//void RealSample_Function(void);
void ChemicalError_Function(void);
void Amplification_Function(void);
void Sample_Function(char* str, uint8_t sample_type);
void MeasureChemical_Function(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin);

void task_main(void)
{
	//Initialize temp/hum sensor
	sht3x_init(&sht3x_handle);

	//Check if battery was done charging
	if(battery_completed == 1)
	{


		SSD1306_Clear();
		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Bateria cargada", &Font_7x10, 1);
		SSD1306_GotoXY(5, 15);
		SSD1306_Puts("Desconectar cable USB", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		HAL_Delay(3000);

		SSD1306_Clear();
		HAL_GPIO_WritePin(GPIOA, RGB_GREEN_Pin, GPIO_PIN_RESET);
		battery_completed = 0;
	}

	if(menu_counter == 0)
	{
		DisplayMenu_Function();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
	}

	else if(menu_counter == 1)
	{
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN_2, CFG_SCH_PRIO_0);
	}

	else if(menu_counter == 2)
	{
		HAL_Delay(500);
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_READ_TEMP_HUM, CFG_SCH_PRIO_0);
	}

	/*
	else if(menu_counter == 3)
	{
		//Advertising ON
		ADV_Start();
	}
	*/
}
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* SNAP_SVC */
    case CUSTOM_STM_ABS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ABS_READ_EVT */

      /* USER CODE END CUSTOM_STM_ABS_READ_EVT */
      break;

    case CUSTOM_STM_REF_VOL_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_REF_VOL_READ_EVT */

      /* USER CODE END CUSTOM_STM_REF_VOL_READ_EVT */
      break;

    case CUSTOM_STM_TEMP_HUM_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TEMP_HUM_READ_EVT */

      /* USER CODE END CUSTOM_STM_TEMP_HUM_READ_EVT */
      break;

    case CUSTOM_STM_DATE_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DATE_READ_EVT */

      /* USER CODE END CUSTOM_STM_DATE_READ_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
	  //Advertising OFF
	  ADV_Stop();

	  //Check if battery has sufficient charge
	  Battery_Percentage();

	  //Task 1
	  UTIL_SEQ_RegTask( 1<< CFG_TASK_MAIN, UTIL_SEQ_RFU, task_main);

	  //Task 2
	  UTIL_SEQ_RegTask( 1<< CFG_TASK_READ_TEMP_HUM, UTIL_SEQ_RFU, TempHum_Function);

	  //Task 3
	  UTIL_SEQ_RegTask( 1<< CFG_TASK_MAIN_2, UTIL_SEQ_RFU, Amplification_Function);

  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void DisplayMenu_Function(void)
{
	//Se presiona boton 1
	if((HAL_GPIO_ReadPin(BUTT_2_GPIO_Port, BUTT_2_Pin) == GPIO_PIN_RESET))
	{
		SSD1306_Clear();
		if (selected_main >= 2)
		{
			selected_main = 0;
		}
		else
		{
			//HAL_Delay(100);
			selected_main++;
		}
	}

	//Se presiona boton 2
	if((HAL_GPIO_ReadPin(BUTT_1_GPIO_Port, BUTT_1_Pin) == GPIO_PIN_RESET))
	{
		entered_main = selected_main;
	}

	//No se presiona ningun boton y solo se muestra el menu inicial
	if (entered_main == 0)
	{
		SSD1306_GotoXY(1, 0);
		SSD1306_Puts("SNAP Menu", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		for (uint8_t i = 0; i <= 2; i++)
		{
			if (i == selected_main)
			{
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(options_menu[i], &Font_7x10, 0);
				SSD1306_UpdateScreen();
			}
		}
	}

	//Ingresa a la funcion de amplificacion
	else if (entered_main == 1)
	{
		menu_counter = 1;
		entered_second = 0;
		selected_second = 0;
	}

	//Ingresa a la funcion de obtencion de datos de temperatura y humedad
	else if (entered_main == 2)
	{
		menu_counter = 2;
	}
}

void Amplification_Function(void)
{
	//Se presiona boton 1
	if((HAL_GPIO_ReadPin(BUTT_2_GPIO_Port, BUTT_2_Pin) == GPIO_PIN_RESET))
	{
		SSD1306_Clear();
		if (selected_second >= 4)
		{
			selected_second = 0;
		}
		else
		{
			//HAL_Delay(100);
			selected_second++;
		}
	}

	//Se presiona boton 2
	if((HAL_GPIO_ReadPin(BUTT_1_GPIO_Port, BUTT_1_Pin) == GPIO_PIN_RESET))
	{
		entered_second = selected_second;
	}

	if (entered_second == 0)
	{
		SSD1306_GotoXY (1, 0);
		SSD1306_Puts("Elegir quimico", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		for (uint8_t i = 0; i <= 4; i++)
		{
			if (i == selected_second)
			{
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(options_elements[i], &Font_7x10, 0);
				SSD1306_UpdateScreen();
			}
		}
	}

	//Nitrogen
	if (entered_second == 1)
	{
		MeasureChemical_Function(GPIOB, LED_4261_EN_Pin);
	}

	//Phosphorus
	else if (entered_second == 2)
	{
		MeasureChemical_Function(GPIOA, LED_WP7_EN_Pin);
	}

	//Potasium
	else if (entered_second == 3)
	{
		MeasureChemical_Function(GPIOB, LED_4273_EN_Pin);
	}

	//Return
	else if (entered_second == 4)
	{
		menu_counter = 0;
		entered_main = 0;
		selected_main = 0;
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		return;
	}

	UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN_2, CFG_SCH_PRIO_0);
	return;
}

void TempHum_Function(void)
{
	if(HAL_GPIO_ReadPin(BUTT_1_GPIO_Port, BUTT_1_Pin) == GPIO_PIN_RESET)
	{

		entered_main = 0;
		selected_main = 0;
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		menu_counter = 0;
		SSD1306_Clear();

	}

	else
	{
		//Informacion de la pantalla
		SSD1306_GotoXY(1, 0);
		SSD1306_Puts("Back", &Font_7x10, 0);
		SSD1306_GotoXY(29,0);
		SSD1306_Puts("Temperatura", &Font_7x10, 1);
		SSD1306_GotoXY(35,35);
		SSD1306_Puts("Humedad", &Font_7x10, 1);

		//Lectura del sensor e impresion en la pantalla
		sht3x_read_temperature_and_humidity(&sht3x_handle, &temperature, &humidity);
		SSD1306_GotoXY(1, 20);
		gcvt(temperature, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_GotoXY(1, 45);
		gcvt(humidity, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		//Guardar informacion en memoria BLE
		UpdateCharData[1] = (uint8_t) temperature;
		Custom_STM_App_Update_Char(CUSTOM_STM_TEMP_HUM, &UpdateCharData[1]);
		UpdateCharData[2] = (uint8_t) humidity;
		Custom_STM_App_Update_Char(CUSTOM_STM_TEMP_HUM, &UpdateCharData[2]);

		//Repeats Task
		UTIL_SEQ_SetTask(1 << CFG_TASK_READ_TEMP_HUM, CFG_SCH_PRIO_0);
	}
}

//void BLE_Function(void)
//{
	//activador_ble = 1;
//}

void MeasureChemical_Function(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
	//Wait for user input
	WaitUser_Function();

	//Turn on selected LED
	HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_SET);
	HAL_Delay(100);

	//Check if selected LED is in correct position
	ADCCheck_Function();

	if(adc_value[0] > 500)
	{
		//Funcion que pide al usuario colocar la prueba blanca en la ranura
		//BlankSample_Function();
		Sample_Function("Blanca", 1);

		//Funcion que pide al usuario colocar la prueba real en la ranura
		//RealSample_Function();
		Sample_Function("Real", 2);

		//Turn off selected LED & reset variables for returning to Menu
		HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
		entered_second = 0;
		menu_counter = 0;

		//TODO:Mostrar el valor en pantalla y guardar en memoria
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		return;
	}

	else
	{
		//Turn off selected LED
		HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);

		//Error al escoger led
		ChemicalError_Function();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		return;
	}

	return;
}

void WaitUser_Function(void)
{
	SSD1306_Clear();
	HAL_Delay(500);
	while(HAL_GPIO_ReadPin(GPIOB, BUTT_1_Pin) == GPIO_PIN_SET){
		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Mover palanca", &Font_7x10, 1);
		SSD1306_GotoXY(15, 15);
		SSD1306_Puts("al LED escogido", &Font_7x10, 1);
		SSD1306_GotoXY(15, 25);
		SSD1306_Puts("Presionar 2", &Font_7x10, 1);
		SSD1306_GotoXY(15, 35);
		SSD1306_Puts("Para continuar", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
}

void ChemicalError_Function(void)
{
	//Show on screen
	SSD1306_Clear();
	SSD1306_GotoXY(15, 15);
	SSD1306_Puts("LED Incorrecto", &Font_7x10, 1);
	SSD1306_GotoXY(15, 45);
	SSD1306_Puts("Cambiar LED", &Font_7x10, 1);
	SSD1306_UpdateScreen();

	HAL_Delay(1000);
	entered_second = 4;
}

void Sample_Function(char* str, uint8_t sample_type)
{
	SSD1306_Clear();
	HAL_Delay(500);

	while(HAL_GPIO_ReadPin(GPIOB, BUTT_1_Pin) == GPIO_PIN_SET)
	{
		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Colocar muestra", &Font_7x10, 1);
		SSD1306_GotoXY(15, 15);
		switch(sample_type)
		{
		case 1:
			SSD1306_Puts(str, &Font_7x10, 1);
			break;

		case 2:
			SSD1306_Puts(str, &Font_7x10, 1);
			break;
		}
		//SSD1306_Puts(str, &Font_7x10, 1);
		SSD1306_GotoXY(15, 25);
		SSD1306_Puts("Presionar 2", &Font_7x10, 1);
		SSD1306_GotoXY(15, 35);
		SSD1306_Puts("Para continuar", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}

	//PWM voltage to 1.5V
	TIM2->CCR1 = 34492;

	//Start ADC and get ADC value
	//ADC_OPAMP_CH12();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2);
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, 1);
	//adc_pht = HAL_ADC_GetValue(&hadc1);

	//Find ideal PWM value
	while (adc_value[0] > 2200 || adc_value[0] < 2100)
	{
		if (adc_value[0] <= 2101)
		{
			//PWM++
			TIM2->CCR1 += 100;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			if(TIM2->CCR1 > 65565)
			{
				done_1 = 1;
			}
		}
		else if(adc_value[0] >= 2199)
		{
			//PWM--
			TIM2->CCR1 -= 100;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			if(TIM2->CCR1 < 200)
			{
				done_1 = 1;
			}
		}
		//for(uint8_t i = 0; i < 34674; i++){

		//HAL_ADC_Start(&hadc1);
		//HAL_ADC_PollForConversion(&hadc1, 1);
		//adc_pht = HAL_ADC_GetValue(&hadc1);

		if(done_1 == 1)
		{
			break;
		}
	}

	//Voltage calculation
	val_lineal = 2.99*adc_value[0]/4095;

	//Turn off ADC and PWM
	//HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

	switch(sample_type)
	{
	case 1:
		//Linear formula
		val_lineal = (A * val_lineal) - B; //TODO Corregir
		break;

	case 2:
		//Log formula
		//TODO: Agregar
		break;
	}

	return;
}

/*
void RealSample_Function(void)

{
	SSD1306_Clear();
	HAL_Delay(500);
	while(HAL_GPIO_ReadPin(GPIOB, BUTT_1_Pin) == GPIO_PIN_SET)
	{
		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Colocar muestra", &Font_7x10, 1);
		SSD1306_GotoXY(15, 15);
		SSD1306_Puts("real", &Font_7x10, 1);
		SSD1306_GotoXY(15, 25);
		SSD1306_Puts("Presionar 2", &Font_7x10, 1);
		SSD1306_GotoXY(15, 35);
		SSD1306_Puts("Para continuar", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}

	//PWM voltage to 1.5V
	TIM2->CCR1 = 34492;

	//Start ADC and get ADC value
	ADC_OPAMP_CH12();
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	adc_pht = HAL_ADC_GetValue(&hadc1);

	//Find ideal PWM value
	while (adc_pht > 2200 || adc_pht < 2100)
	{
		if (adc_pht <= 2101)
		{
			//PWM++
			TIM2->CCR1 += 100;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			if(TIM2->CCR1 > 65565)
			{
				done_2 = 1;
			}
		}
		else if(adc_pht >= 2199)
		{
			//PWM--
			TIM2->CCR1 -= 100;
			HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
			if(TIM2->CCR1 < 200)
			{
				done_2 = 1;
			}

		}

		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 1);
		adc_pht = HAL_ADC_GetValue(&hadc1);

		if(done_2 == 1)
		{
			break;
		}
	}

	//Voltage calculation
	val_log = 2.99*(adc_pht/4095);

	//Logaritmic formula
	//TODO Calculo de la funcion log

	//Turn off ADC and PWM
	HAL_ADC_Stop(&hadc1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

	entered_second = 0;
	menu_counter = 0;

	return;
}
*/

void ADCCheck_Function(void)
{
	//Change ADC channel
	//ADC_OPAMP_CH12();

	//Start PWM, PHT & OPAMP
	TIM2->CCR1 = 65535;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2);
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, 1000);
	//adc_pht = HAL_ADC_GetValue(&hadc1);

	//Turn off PWM and ADC
	//HAL_ADC_Stop(&hadc1);
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void Battery_Percentage(void)
{
	//Obtain battery value by ADC
	//ADC_BATT_CH6();
	//HAL_ADC_Start(&hadc1);
	//HAL_ADC_PollForConversion(&hadc1, 1000);
	//adc_batt = HAL_ADC_GetValue(&hadc1);
	//HAL_ADC_Stop(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2);
	HAL_Delay(100);

	//Voltage calculation
	batt_percentage = 2.99*adc_value[1]/4095;

	//Show value on screen

	if(batt_percentage <= 1.64)
	{
		SSD1306_Clear();
		HAL_GPIO_WritePin(GPIOA, RGB_RED_Pin, GPIO_PIN_SET);
		SSD1306_Clear();
		SSD1306_GotoXY(15, 15);
		SSD1306_Puts("Bateria baja", &Font_7x10, 1);
		SSD1306_GotoXY(15, 45);
		SSD1306_Puts("Conectar cargador", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		battery_completed = 1;
		HAL_Delay(2000);
	}

	return;
}
/*
void ADC_OPAMP_CH12(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_12;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

void ADC_BATT_CH6(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};

	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_24CYCLES_5;

	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
	Error_Handler();
	}
}

*/

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SNAP_SVC */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
