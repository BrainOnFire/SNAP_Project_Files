/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.h
  * @author  MCD Application Team
  * @brief   Header for custom_stm.c module.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef CUSTOM_STM_H
#define CUSTOM_STM_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
  /* SNAP_SVC */
  CUSTOM_STM_VOL_SEN,
  CUSTOM_STM_CU_SEN,
  CUSTOM_STM_TEMP,
  CUSTOM_STM_HUM,
  /* SNAP_SVC_2 */
  CUSTOM_STM_NUM_VAR,
  CUSTOM_STM_ABS,
  CUSTOM_STM_ID_NUT,
  CUSTOM_STM_ID_EXTRA,
  /* SNAP_SVC_3 */
  CUSTOM_STM_ACK,
} Custom_STM_Char_Opcode_t;

typedef enum
{
  /* VOLTAGE_SENSOR */
  CUSTOM_STM_VOL_SEN_READ_EVT,
  /* CURRENT_SENSOR */
  CUSTOM_STM_CU_SEN_READ_EVT,
  /* TEMP */
  CUSTOM_STM_TEMP_READ_EVT,
  /* HUM */
  CUSTOM_STM_HUM_READ_EVT,
  /* NUMBER_VAR */
  CUSTOM_STM_NUM_VAR_READ_EVT,
  /* ABSORBANCE */
  CUSTOM_STM_ABS_READ_EVT,
  /* ID_NUTRIENT */
  CUSTOM_STM_ID_NUT_READ_EVT,
  /* ID_DISP_EXTRA */
  CUSTOM_STM_ID_EXTRA_READ_EVT,
  /* Acknowledge */
  CUSTOM_STM_ACK_WRITE_EVT,

  CUSTOM_STM_BOOT_REQUEST_EVT
} Custom_STM_Opcode_evt_t;

typedef struct
{
  uint8_t * pPayload;
  uint8_t   Length;
} Custom_STM_Data_t;

typedef struct
{
  Custom_STM_Opcode_evt_t       Custom_Evt_Opcode;
  Custom_STM_Data_t             DataTransfered;
  uint16_t                      ConnectionHandle;
  uint8_t                       ServiceInstance;
} Custom_STM_App_Notification_evt_t;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
extern uint8_t SizeVol_Sen;
extern uint8_t SizeCu_Sen;
extern uint8_t SizeTemp;
extern uint8_t SizeHum;
extern uint8_t SizeNum_Var;
extern uint8_t SizeAbs;
extern uint8_t SizeId_Nut;
extern uint8_t SizeId_Extra;
extern uint8_t SizeAck;

/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */
extern uint8_t acknowledge;
extern uint8_t total_measures;
/* USER CODE END EV */

/* Exported macros -----------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions ------------------------------------------------------- */
void SVCCTL_InitCustomSvc(void);
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification);
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode,  uint8_t *pPayload);
/* USER CODE BEGIN EF */

/* USER CODE END EF */

#ifdef __cplusplus
}
#endif

#endif /*CUSTOM_STM_H */
