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
#include <stdlib.h>
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
  /* SNAP_SVC_2 */
  /* SNAP_SVC_3 */
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
uint8_t total_measures = 1;
uint8_t acknowledge = 0;
uint8_t menu_counter = 0;
uint8_t selected_main = 0;
uint8_t selected_second = 0;
uint8_t entered_second = 0;
uint8_t entered_main = 0;
uint8_t done_1 = 0;
uint8_t done_2 = 0;
uint8_t id_nut = 0;
uint32_t adc_value[2];

double array_values[15][6];

int **resultsArray = NULL;
int numRowsArray = 0;
const int numColsArray = 10;

const float A = 3.1725417;
const float B = 3.663636;
float humidity = 85;
float temperature = 27;
float batt_percentage = 0.0;
float voltage_lin = 0.0;
float voltage_log = 0.0;
float val_lineal = 0.0;
float val_log = 0.0;
float absorbance = 0.0;
float voltage_sensor;
float current_sensor;
char  buf[10];

uint8_t device_ID[12] = {'S', 'N', 'A', 'P', '2', '3', '0', '6', '6', '0', '0', '1'};

char *options_menu[3] = {
		"Measure nutrient",
		"Measure temp/hum",
		"Send data to APP"
};

char *options_elements[4] = {
		"Phosphorus",
		"Potasium",
		"Nitrogen",
		"Return to menu"
};

sht3x_handle_t sht3x_handle = {
    .i2c_handle = &hi2c1,
    .device_address = SHT3X_I2C_DEVICE_ADDRESS_ADDR_PIN_LOW
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* SNAP_SVC */
/* SNAP_SVC_2 */
/* SNAP_SVC_3 */

/* USER CODE BEGIN PFP */
void addRow();
void deleteRow();
void clearMemory();
void TempHum_Function(void);
void ADCCheck_Function(void);
void WaitUser_Function(void);
void Battery_Percentage(void);
void ShowValues_Function(void);
void SendValues_Function(void);
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
		SSD1306_Puts("Battery full", &Font_7x10, 1);
		SSD1306_GotoXY(5, 15);
		SSD1306_Puts("Unplug USB", &Font_7x10, 1);
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

	else if(menu_counter == 3)
	{
		HAL_Delay(500);
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_VALUES, CFG_SCH_PRIO_0);
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
    case CUSTOM_STM_VOL_SEN_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_VOL_SEN_READ_EVT */

      /* USER CODE END CUSTOM_STM_VOL_SEN_READ_EVT */
      break;

    case CUSTOM_STM_CU_SEN_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_CU_SEN_READ_EVT */

      /* USER CODE END CUSTOM_STM_CU_SEN_READ_EVT */
      break;

    case CUSTOM_STM_TEMP_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_TEMP_READ_EVT */

      /* USER CODE END CUSTOM_STM_TEMP_READ_EVT */
      break;

    case CUSTOM_STM_HUM_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_HUM_READ_EVT */

      /* USER CODE END CUSTOM_STM_HUM_READ_EVT */
      break;

    /* SNAP_SVC_2 */
    case CUSTOM_STM_NUM_VAR_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NUM_VAR_READ_EVT */

      /* USER CODE END CUSTOM_STM_NUM_VAR_READ_EVT */
      break;

    case CUSTOM_STM_ABS_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ABS_READ_EVT */

      /* USER CODE END CUSTOM_STM_ABS_READ_EVT */
      break;

    case CUSTOM_STM_ID_NUT_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ID_NUT_READ_EVT */

      /* USER CODE END CUSTOM_STM_ID_NUT_READ_EVT */
      break;

    case CUSTOM_STM_ID_EXTRA_READ_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ID_EXTRA_READ_EVT */

      /* USER CODE END CUSTOM_STM_ID_EXTRA_READ_EVT */
      break;

    /* SNAP_SVC_3 */
    case CUSTOM_STM_ACK_WRITE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_ACK_WRITE_EVT */

      /* USER CODE END CUSTOM_STM_ACK_WRITE_EVT */
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
	  //Check if battery has sufficient charge
	  Battery_Percentage();

	  //Task 1
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_MAIN, UTIL_SEQ_RFU, task_main);

	  //Task 2
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_READ_TEMP_HUM, UTIL_SEQ_RFU, TempHum_Function);

	  //Task 3
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_MAIN_2, UTIL_SEQ_RFU, Amplification_Function);

	  //Task 4
	  UTIL_SEQ_RegTask( 1 << CFG_TASK_SEND_VALUES, UTIL_SEQ_RFU, SendValues_Function);

	  //Task 5
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
		if (selected_main >= 2)
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

	//Ingresa a la funcion de enviar datos a APP
	else if (entered_main == 3)
	{
		menu_counter = 3;
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
		SSD1306_Puts("Choose nutrient", &Font_7x10, 1);
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
		SSD1306_Puts("Temperature", &Font_7x10, 1);
		SSD1306_GotoXY(35,35);
		SSD1306_Puts("Humidity", &Font_7x10, 1);

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
		//array_values[0][3] = temperature;
		//Custom_STM_App_Update_Char(CUSTOM_STM_TEMP, &UpdateCharData[1]);
		//array_values[0][4] = humidity;
		//Custom_STM_App_Update_Char(CUSTOM_STM_HUM , &UpdateCharData[2]);

		//Repeats Task
		UTIL_SEQ_SetTask(1 << CFG_TASK_READ_TEMP_HUM, CFG_SCH_PRIO_0);
	}
}

void MeasureChemical_Function(GPIO_TypeDef *GPIO_Port, uint16_t GPIO_Pin)
{
	if(total_measures < 15){
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
			Sample_Function("White", 1);

			//Funcion que pide al usuario colocar la prueba real en la ranura
			Sample_Function("Real", 2);

			//Turn off selected LED & reset variables for returning to Menu
			HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
			entered_main = 0;
			selected_main = 0;
			selected_second = 0;
			entered_second = 0;

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
	}

	else{

		SSD1306_GotoXY(0,10);
		SSD1306_Puts("Memory full", &Font_7x10, 1);
		SSD1306_UpdateScreen();
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
		SSD1306_Puts("Data saved", &Font_7x10, 1);
		SSD1306_GotoXY(5, 15);
		SSD1306_Puts("in memory", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		HAL_Delay(3000);

		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		menu_counter = 0;
		total_measures++;
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
		SSD1306_Puts("Abs & voltage", &Font_7x10, 1);
		SSD1306_GotoXY(0,20);
		SSD1306_Puts("Current", &Font_7x10, 1);

		SSD1306_GotoXY(1, 15);
		gcvt(absorbance, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);

		SSD1306_GotoXY(1, 25);
		gcvt(voltage_sensor, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);

		SSD1306_GotoXY(30, 25);
		gcvt(current_sensor, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		//Delay & clear
		HAL_Delay(2000);
		SSD1306_Clear();

		//Temp/Hum information showed on screen
		SSD1306_GotoXY(1, 1);
		SSD1306_Puts("Back", &Font_7x10, 0);
		SSD1306_GotoXY(29,0);
		SSD1306_Puts("Temperature", &Font_7x10, 1);
		SSD1306_GotoXY(35,35);
		SSD1306_Puts("Humidity", &Font_7x10, 1);

		//Reads temp/hum
		sht3x_read_temperature_and_humidity(&sht3x_handle, &temperature, &humidity);
		SSD1306_GotoXY(1, 20);
		gcvt(temperature, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_GotoXY(1, 45);
		gcvt(humidity, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		//Store information from temp to flash
		array_values[total_measures][0] = voltage_sensor;
		array_values[total_measures][1] = current_sensor;
		array_values[total_measures][2] = temperature;
		array_values[total_measures][3] = humidity;
		array_values[total_measures][4] = absorbance;
		array_values[total_measures][5] = id_nut;

		//Repeats Task
		UTIL_SEQ_SetTask(1 << CFG_TASK_SHOW_VALUES, CFG_SCH_PRIO_0);
	}
}

void SendValues_Function(void){
	//Verify if user has measure data
	if(total_measures > 0){
		//Verify if all values have been read
		if(acknowledge == 1){

			//Show on screen
			SSD1306_Clear();
			SSD1306_GotoXY(0,30);
			SSD1306_Puts("Data sent!", &Font_7x10, 1);
			SSD1306_UpdateScreen();
			memset(array_values, 0, sizeof array_values);

			HAL_Delay(2000);

			total_measures = 0;
			menu_counter = 0;
			entered_main = 0;
			selected_main = 0;
			SSD1306_Clear();
			UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
			return;
		}

		else{
			//Mostrar en pantalla que se esta mandando la info
			SSD1306_GotoXY(0,30);
			SSD1306_Puts("Sending...", &Font_7x10, 1);
			SSD1306_UpdateScreen();

			//Pasar por todas las mediciones disponibles
			//for(uint8_t i = total_measures; i > 0; i--){  //Changed i to 0 in array

				//Pasar info temporal al BT (siete variables)
				UpdateCharData[0] = (uint8_t) array_values[0][1]*100; 	//voltage_sensor
				Custom_STM_App_Update_Char(CUSTOM_STM_VOL_SEN, &UpdateCharData[0]);
				UpdateCharData[1] = (uint8_t) array_values[0][2]*100; 	//current_sensor
				Custom_STM_App_Update_Char(CUSTOM_STM_CU_SEN, &UpdateCharData[1]);
				UpdateCharData[2] = (uint8_t) array_values[0][3];		//temp
				Custom_STM_App_Update_Char(CUSTOM_STM_TEMP, &UpdateCharData[2]);
				UpdateCharData[3] = (uint8_t) array_values[0][4];		//hum
				Custom_STM_App_Update_Char(CUSTOM_STM_HUM, &UpdateCharData[3]);
				UpdateCharData[4] = (uint8_t) array_values[0][5]*100;	//abs
				Custom_STM_App_Update_Char(CUSTOM_STM_ABS, &UpdateCharData[4]);
				//UpdateCharData[5] = (uint8_t) device_ID;				//deviceID
				Custom_STM_App_Update_Char(CUSTOM_STM_ID_EXTRA, device_ID);
				UpdateCharData[6] = (uint8_t) array_values[0][6];		//id_nut
				Custom_STM_App_Update_Char(CUSTOM_STM_ID_NUT, &UpdateCharData[6]);
				UpdateCharData[7] = (uint8_t) total_measures;			//measures remaining
				Custom_STM_App_Update_Char(CUSTOM_STM_NUM_VAR, &UpdateCharData[7]);

				//total_measures -- esto me permite que la app cambie de valores
				HAL_Delay(2000);
			//}

			//Repeats task
			UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_VALUES, CFG_SCH_PRIO_0);
			return;
		}
	}

	else{
		//No data stored on memory; return to main menu
		SSD1306_GotoXY(0,30);
		SSD1306_Puts("No data on memory", &Font_7x10, 1);
		SSD1306_UpdateScreen();

		HAL_Delay(2000);

		menu_counter = 0;
		entered_main = 0;
		selected_main = 0;
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
		return;
	}
}

void WaitUser_Function(void)
{
	SSD1306_Clear();
	HAL_Delay(500);
	while(HAL_GPIO_ReadPin(GPIOB, BUTT_1_Pin) == GPIO_PIN_SET){
		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Move lever", &Font_7x10, 1);
		SSD1306_GotoXY(15, 15);
		SSD1306_Puts("to chosen LED", &Font_7x10, 1);
		SSD1306_GotoXY(15, 25);
		SSD1306_Puts("Press 2", &Font_7x10, 1);
		SSD1306_GotoXY(15, 35);
		SSD1306_Puts("to continue", &Font_7x10, 1);
		SSD1306_UpdateScreen();
	}
}

void ChemicalError_Function(void)
{
	//Show on screen
	SSD1306_Clear();
	SSD1306_GotoXY(25, 15);
	SSD1306_Puts("Wrong LED", &Font_7x10, 1);
	SSD1306_GotoXY(10, 45);
	SSD1306_Puts("Try again please", &Font_7x10, 1);
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
		SSD1306_Puts("Put sample", &Font_7x10, 1);
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
		SSD1306_Puts("Press 2", &Font_7x10, 1);
		SSD1306_GotoXY(15, 35);
		SSD1306_Puts("to resume", &Font_7x10, 1);
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
	TIM2->CCR1 = 65535; //Max Value
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //PWM_VREF on
	//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); //PWM_VREF off
	//TIM2->CCR1 = 0; //Min Value

	//TIM2->CCR2 = 54612; //Max Value
	TIM2->CCR2 = 65535; //Max Value
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //PWM_LEDS on
	//HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); //PWM_LEDS off
	//TIM2->CCR2 = 0; //Min Value

	HAL_ADC_Start_DMA(&hadc1, adc_value, 2); //Read value

	//Depuracion delay de 100 segundos
	HAL_Delay(100000);

	//Turn off PWM and ADC
	HAL_ADC_Stop_DMA(&hadc1);

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
		SSD1306_Puts("Low battery", &Font_7x10, 1);
		SSD1306_GotoXY(15, 45);
		SSD1306_Puts("Plug charger", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		battery_completed = 1;
		HAL_Delay(2000);
	}

	return;
}

void addRow(){
	numRowsArray++;
	resultsArray = realloc(resultsArray, numRowsArray * sizeof(int *));
	resultsArray[numRowsArray - 1] = malloc(numColsArray * sizeof(int));
}

void deleteRow(int row) {
	if (row < 0 || row >= numRowsArray){
		//Mostrar error en pantalla
		return;
	}

	free(resultsArray[row]);

	for (int i = row; i < numRowsArray - 1; i++){
		resultsArray[i] = resultsArray[i + 1];
	}

	numRowsArray--;
	resultsArray = realloc(resultsArray, numRowsArray * sizeof(int *));
}

void clearMemory(){
	//Free the memory
	for (int i = 0; i < numRowsArray; i++){
		free(resultsArray[i]);
	}

	free(resultsArray);
}
/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* SNAP_SVC */
/* SNAP_SVC_2 */
/* SNAP_SVC_3 */

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
