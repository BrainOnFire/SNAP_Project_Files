/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  Luciano Zagastizabal Granadino (UTEC)
  * @brief   SNAP Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  * Please refer to the following files for better understanding of the code:
  *
  * 	SNAP_Project_Code -> Core -> Scr -> main.c
  * 	SNAP_Project_Code -> Core -> Inc -> main.h
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_app.c  <- Currently in this file
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_app.h
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_stm.c
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_stm.h
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
uint8_t total_measures = 1; //Cambiar a 0 para entrega final; deberia aumentar automaticamente luego de una medicion
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

//Edit this value when programming different SNAP devices
uint8_t device_ID[12] = {'S', 'N', 'A', 'P', '2', '3', '0', '6', '6', '0', '0', '3'};

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

/* By default, this is the first task to run. It shows the three possible choices that can be selected by the user. It also checks
 * in the background if the battery has charged completely. This first task jumps to the DisplayFunction() to print the SNAP Menu
 * and repeats the task_main automatically.
 * */
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

	//Task repeats itself
	if(menu_counter == 0)
	{
		DisplayMenu_Function();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
	}

	//First choice was selected -> enters the task to select the nutrient
	else if(menu_counter == 1)
	{
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN_2, CFG_SCH_PRIO_0);
	}

	//Second choice was selected -> enters to the task for reading the temperature and humidity values
	else if(menu_counter == 2)
	{
		HAL_Delay(500);
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_READ_TEMP_HUM, CFG_SCH_PRIO_0);
	}

	//Third choice was selected -> enters to the task to send the values
	else if(menu_counter == 3)
	{
		HAL_Delay(500);
		SSD1306_Clear();
		UTIL_SEQ_SetTask(1 << CFG_TASK_SEND_VALUES, CFG_SCH_PRIO_0);
	}
}
/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* We don't use this funtion, please do not edit
 * */
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

/* We don't use this funtion, please do not edit
 * */
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

/* This functions is the app initialization which checks the current state of the battery first, then creates the following tasks
 * that run with the use of the scheduler. Every task has a name and a priority. For this case every task has a different name, but
 * the same priority, so they don't interfiere with eachother. Do not edit this function!
*/
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
/* Function that gets called by the main menu and runs automatically. It prints in the screen the SNAP Menu.
 * */
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

/* This function gets called when selecting the first choice in the main menu. The following task shows the four possible choices that
 * can be selected by the user. This choices are the type of nutrient to be selected for the measurement. The fourth choice is the one
 * that allows the user to return to the main menu.
 * */
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

/* This functions gets called when selecting the second choice in the SNAP Menu. It enters to the "CFG_TASK_READ_TEMP_HUM" task to
 * read the temperature and himidity of the environment.
 * */
void TempHum_Function(void)
{
	//If pressed the enter button, it exits this function and enters to the main menu task
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
		//Prints the information to the scren
		SSD1306_GotoXY(1, 0);
		SSD1306_Puts("Back", &Font_7x10, 0);
		SSD1306_GotoXY(29,0);
		SSD1306_Puts("Temperature", &Font_7x10, 1);
		SSD1306_GotoXY(35,35);
		SSD1306_Puts("Humidity", &Font_7x10, 1);

		//Reading of the temperature and humidity values
		sht3x_read_temperature_and_humidity(&sht3x_handle, &temperature, &humidity);
		SSD1306_GotoXY(1, 20);
		gcvt(temperature, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_GotoXY(1, 45);
		gcvt(humidity, 3, buf);
		SSD1306_Puts(buf, &Font_7x10, 1);
		SSD1306_UpdateScreen();

		//Repeats Task
		UTIL_SEQ_SetTask(1 << CFG_TASK_READ_TEMP_HUM, CFG_SCH_PRIO_0);
	}
}

/* This function gets called when the user has selected one of the nutrients before starting a measurement.
 * */
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
			//Function that asks the user to insert the blank sample in the slot
			Sample_Function("White", 1);

			//Function that asks the user to insert the real sample in the slot
			Sample_Function("Real", 2);

			//Turn off selected LED & reset menu variables
			HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);
			entered_main = 0;
			selected_main = 0;
			selected_second = 0;
			entered_second = 0;

			//Enters to the task that shows the values measured in the screen and saves them in the system memory
			UTIL_SEQ_SetTask(1 << CFG_TASK_SHOW_VALUES, CFG_SCH_PRIO_0);
			return;
		}

		else
		{
			//Turn off selected LED
			HAL_GPIO_WritePin(GPIO_Port, GPIO_Pin, GPIO_PIN_RESET);

			//Wrongly selected LED
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

		//If the memory is full the function exits this task and returns to the main menu
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

		SSD1306_GotoXY(15, 5);
		SSD1306_Puts("Data saved", &Font_7x10, 1);
		SSD1306_GotoXY(5, 15);
		SSD1306_Puts("in memory", &Font_7x10, 1);
		SSD1306_UpdateScreen();
		HAL_Delay(3000);

		menu_counter = 0;
		entered_main = 0;
		selected_main = 0;
		total_measures++;
		SSD1306_Clear();

		UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
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

/*
 * Function to send values using BT to APP
 * First it checks if a measure has been made usign the device. If the device has not made any measurements, it will show on screen
 * that there is no data saved on memory. Once a measurement is made, the device will start passing the array values too the BT cache.
 * Once the APP returns a ACK, the screen will show a "data sent" message and the task scheduler will return to the main
 * menu.
*/
void SendValues_Function(void){
	//Verify if user has measure data
	if(total_measures > 0 || acknowledge == 1){
		//Verify if all values have been read
		if(acknowledge == 1){

			//Show on screen info has been sent
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
			HAL_Delay(5000);
			SSD1306_Clear();
			UTIL_SEQ_SetTask(1 << CFG_TASK_MAIN, CFG_SCH_PRIO_0);
			return;
		}

		else{
			//Show on screen BT is being used to send the data
			SSD1306_GotoXY(0,30);
			SSD1306_Puts("Sending...", &Font_7x10, 1);
			SSD1306_UpdateScreen();

			//Pasar por todas las mediciones disponibles (sin usar, para futuras pruebas)
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

/* /Function to show on screen to moce the lever to chosen LED
 * */
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

/* Function to show on screen that selected LED has wrongfully been selected
 * */
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

//Funcion para obtener los valores de voltaje y corriente y las absorbacias (falta probar y corregir probablemente toda la funcion)///
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
//Funcion para obtener los valores de voltaje y corriente y las absorbacias (falta probar y corregir probablemente toda la funcion)///

/*
 * This function tests if the LEDs are working properly by tunrning ON the PWM of both the PWM_VREF and PWM_LEDs. Then it turns on the
 * ADC for a brief period an finally turns OFF both PWMs. The value of the ADC is stored in adc_value[0].
 * */
void ADCCheck_Function(void)
{
	//Start PWM, PHT & OPAMP
	TIM2->CCR1 = 65535; //Max Value
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); //PWM_VREF on

	TIM2->CCR2 = 65535; //Max Value
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //PWM_LEDs on

	HAL_ADC_Start_DMA(&hadc1, adc_value, 2); //Read value to check LED

	//100 seconds delay for debugging purposes
	//HAL_Delay(100000);

	//Turn off PWM and ADC
	HAL_ADC_Stop_DMA(&hadc1);

	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1); //PWM_VREF OFF
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_2); //PWM_LEDs OFF
}

/*
 * This function will test if the battery has sufficient battery remaining for the device to work properly. First, the ADC starts and
 * checks the value. If this value is greater than 1.64 it means there is enough battery left for the device to work properly. IF this
 * value is lower than 1.64 the screen will show "Low battery, Plug charger" to indicate to the user that there is no more battery left.
 * The device will remain on hold until the value changes to be greater than 1.64
 * */
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
