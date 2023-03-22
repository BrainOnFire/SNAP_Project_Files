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
#include <math.h>
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
const float B = 3.663636;
float humidity, temperature;
float batt_percentage = 0.0;
float voltage_lin = 0.0;
float voltage_log = 0.0;
float val_lineal = 0.0;
float val_log = 0.0;
float absorbance = 0.0;
char buf[10];

char *options_menu[2] = {
		"Medir quimico",
		"Medir temperatura y humedad"
		//"Activar Bluetooth"
};

char *options_elements[4] = {
		"Fosforo",
		"Potasio",
		"Nitrogeno",
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
void TempHum_Function(void);
void ADCCheck_Function(void);
void WaitUser_Function(void);
void Battery_Percentage(void);
void ShowValues_Function(void);
void DisplayMenu_Function(void);
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
	  //ADV_Stop();

	  //Check if battery has sufficient charge
	  Battery_Percentage();

	  //Task 1
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_MAIN, UTIL_SEQ_RFU, task_main);

	  //Task 2
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_READ_TEMP_HUM, UTIL_SEQ_RFU, TempHum_Function);

	  //Task 3
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_MAIN_2, UTIL_SEQ_RFU, Amplification_Function);

	  //Task 4
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_SHOW_VALUES, UTIL_SEQ_RFU, ShowValues_Function);


  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */
void DisplayMenu_Function(void)
{
	//Se presiona boton 1
	if((HAL_GPIO_ReadPin(BUTT_2_GPIO_Port, BUTT_2_Pin) == GPIO_PIN_RESET))
	{
		//SSD1306_Clear();
		if (selected_main >= 1)
		{
			SSD1306_Clear();
			selected_main = 0;
		}
		else
		{
			HAL_Delay(500);
			selected_main++;
		}
	}

	//Se presiona boton 2
	if((HAL_GPIO_ReadPin(BUTT_1_GPIO_Port, BUTT_1_Pin) == GPIO_PIN_RESET))
	{
		HAL_Delay(500);
		entered_main = selected_main + 1;
	}

	//No se presiona ningun boton y solo se muestra el menu inicial
	if (entered_main == 0)
	{
		SSD1306_GotoXY(1, 0);
		SSD1306_Puts("SNAP Menu", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		for (uint8_t i = 0; i <= 1; i++)
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
		//SSD1306_Clear();
		if (selected_second >= 3)
		{
			SSD1306_Clear();
			selected_second = 0;
		}
		else
		{
			HAL_Delay(500);
			selected_second++;
		}
	}

	//Se presiona boton 2
	if((HAL_GPIO_ReadPin(BUTT_1_GPIO_Port, BUTT_1_Pin) == GPIO_PIN_RESET))
	{
		HAL_Delay(500);
		entered_second = selected_second + 1;
	}

	if (entered_second == 0)
	{
		SSD1306_GotoXY (1, 0);
		SSD1306_Puts("Elegir quimico", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		for (uint8_t i = 0; i <= 3; i++)
		{
			if (i == selected_second)
			{
				SSD1306_GotoXY(1, 30);
				SSD1306_Puts(options_elements[i], &Font_7x10, 0);
				SSD1306_UpdateScreen();
			}
		}
	}

	//Phosphorus
	if (entered_second == 1)
	{
		MeasureChemical_Function(GPIOA, LED_WP7_EN_Pin);
	}

	//Potasium
	else if (entered_second == 2)
	{
		MeasureChemical_Function(GPIOB, LED_4273_EN_Pin);
	}

	//Nitrogen
	else if (entered_second == 3)
	{
		MeasureChemical_Function(GPIOB, LED_4261_EN_Pin);
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
		HAL_Delay(1000);

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
		Sample_Function("Blanca", 1);

		//Funcion que pide al usuario colocar la prueba real en la ranura
		Sample_Function("Real", 2);

		//Turn off selected LED & reset variables for returning to Menu
		HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
		entered_main = 0;
		selected_main = 0;
		selected_second = 0;
		entered_second = 0;
		//menu_counter = 0;

		//TODO:Mostrar el valor en pantalla y guardar en memoria
		UTIL_SEQ_SetTask(1 << CFG_TASK_SHOW_VALUES, CFG_SCH_PRIO_0);
		return;
	}

	else
	{
		//Turn off selected LED
		HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);

		//Error al escoger led
		ChemicalError_Function();
		entered_main = 0;
		selected_main = 0;
		selected_second = 0;
		entered_second = 0;
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		return;
	}

	return;
}

void ShowValues_Function(void){
	if(HAL_GPIO_ReadPin(BUTT_1_GPIO_Port, BUTT_1_Pin) == GPIO_PIN_RESET)
	{
		entered_main = 0;
		selected_main = 0;

		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Valores guardados", &Font_7x10, 1);
		SSD1306_GotoXY(5, 15);
		SSD1306_Puts("en memoria", &Font_7x10, 1);
		SSD1306_UpdateScreen();

		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		menu_counter = 0;
		SSD1306_Clear();
		HAL_Delay(3000);

	}

	else
	{
		//Clear screen
		SSD1306_Clear();

		//Absorbance & lineal information showed on screen
		SSD1306_GotoXY(1, 1);
		SSD1306_Puts("Back", &Font_7x10, 0);
		SSD1306_GotoXY(0,10);
		SSD1306_Puts("Absorbancia y voltaje", &Font_7x10, 1);
		SSD1306_GotoXY(0,20);
		SSD1306_Puts("Lineal y voltaje", &Font_7x10, 1);

		SSD1306_GotoXY(1, 15);
		gcvt(absorbance, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);

		SSD1306_GotoXY(1, 25);
		gcvt(voltage_log, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);

		SSD1306_GotoXY(30, 25);
		gcvt(voltage_lin, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		//Delay & clear
		HAL_Delay(2000);
		SSD1306_Clear();

		//Temp/Hum information showed on screen
		SSD1306_GotoXY(1, 1);
		SSD1306_Puts("Back", &Font_7x10, 0);
		SSD1306_GotoXY(29,0);
		SSD1306_Puts("Temperatura", &Font_7x10, 1);
		SSD1306_GotoXY(35,35);
		SSD1306_Puts("Humedad", &Font_7x10, 1);

		sht3x_read_temperature_and_humidity(&sht3x_handle, &temperature, &humidity);
		SSD1306_GotoXY(1, 20);
		gcvt(temperature, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_GotoXY(1, 45);
		gcvt(humidity, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		//Store information on screen
		UpdateCharData[1] = (uint8_t) temperature;
		Custom_STM_App_Update_Char(CUSTOM_STM_TEMP_HUM, &UpdateCharData[1]);
		UpdateCharData[2] = (uint8_t) humidity;
		Custom_STM_App_Update_Char(CUSTOM_STM_TEMP_HUM, &UpdateCharData[2]);

		//Repeats Task
		UTIL_SEQ_SetTask(1 << CFG_TASK_SHOW_VALUES, CFG_SCH_PRIO_0);
	}
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

	HAL_Delay(3000);
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

	//Start TIM, ADC and get ADC value[0]
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2);

	//Find ideal PWM value
	while (adc_value[0] > 2200 || adc_value[0] < 2100)
	{
		HAL_ADC_Start_DMA(&hadc1, adc_value, 2);
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

		if(done_1 == 1)
		{
			break;
		}
	}

	//Turn off ADC and PWM
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

	switch(sample_type)
	{
	case 1:
		//Voltage calculation
		voltage_lin = 2.99*adc_value[0]/4095;
		//Linear formula
		val_lineal = (A * voltage_lin) - B; //TODO Corregir
		break;

	case 2:
		//Voltage calculation
		voltage_log = 2.99*adc_value[0]/4095;
		//Log formula
		absorbance = log(voltage_log / val_lineal);

		//Store absorbance values in memory
		UpdateCharData[1] = (uint8_t) absorbance;
		Custom_STM_App_Update_Char(CUSTOM_STM_ABS, &UpdateCharData[1]);
		break;
	}

	return;
}

void ADCCheck_Function(void)
{
	//Start PWM, PHT & OPAMP
	TIM2->CCR1 = 65535;
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_Delay(100);
	HAL_ADC_Start_DMA(&hadc1, adc_value, 2);

	//Turn off PWM and ADC
	HAL_ADC_Stop_DMA(&hadc1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
}

void Battery_Percentage(void)
{
	//Obtain battery value by ADC
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

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SNAP_SVC */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
