/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_stm.c
  * @author  MCD Application Team
  * @brief   Custom Example Service.
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
  * This file has the asynchronus data received by Bluetooth. In line 179 there is more
  * information regarding the SNAP BT communication.
  *
  * Please refer to the following files for better understanding of the code:
  *
  * 	SNAP_Project_Code -> Core -> Scr -> main.c
  * 	SNAP_Project_Code -> Core -> Inc -> main.h
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_app.c
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_app.h
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_stm.c  <- Currently in this file
  * 	SNAP_Project_Code -> STM32_WPAN -> App -> custom_stm.h
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "common_blesvc.h"
#include "custom_stm.h"

/* USER CODE BEGIN Includes */
#include "main.h"
#include "custom_app.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct{
  uint16_t  CustomSnapHdle;                    /**< SNAP_SVC handle */
  uint16_t  CustomVol_SenHdle;                  /**< VOLTAGE_SENSOR handle */
  uint16_t  CustomCu_SenHdle;                  /**< CURRENT_SENSOR handle */
  uint16_t  CustomTempHdle;                  /**< TEMP handle */
  uint16_t  CustomHumHdle;                  /**< HUM handle */
  uint16_t  CustomSnap_1Hdle;                    /**< SNAP_SVC_2 handle */
  uint16_t  CustomNum_VarHdle;                  /**< NUMBER_VAR handle */
  uint16_t  CustomAbsHdle;                  /**< ABSORBANCE handle */
  uint16_t  CustomId_NutHdle;                  /**< ID_NUTRIENT handle */
  uint16_t  CustomId_ExtraHdle;                  /**< ID_DISP_EXTRA handle */
  uint16_t  CustomSnap_2Hdle;                    /**< SNAP_SVC_3 handle */
  uint16_t  CustomAckHdle;                  /**< Acknowledge handle */
}CustomContext_t;

/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private defines -----------------------------------------------------------*/
#define UUID_128_SUPPORTED  1

#if (UUID_128_SUPPORTED == 1)
#define BM_UUID_LENGTH  UUID_TYPE_128
#else
#define BM_UUID_LENGTH  UUID_TYPE_16
#endif

#define BM_REQ_CHAR_SIZE    (3)

/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros ------------------------------------------------------------*/
#define CHARACTERISTIC_DESCRIPTOR_ATTRIBUTE_OFFSET         2
#define CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET              1
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
uint8_t SizeVol_Sen = 1;
uint8_t SizeCu_Sen = 1;
uint8_t SizeTemp = 1;
uint8_t SizeHum = 1;
uint8_t SizeNum_Var = 1;
uint8_t SizeAbs = 1;
uint8_t SizeId_Nut = 1;
uint8_t SizeId_Extra = 12;
uint8_t SizeAck = 1;

/**
 * START of Section BLE_DRIVER_CONTEXT
 */
static CustomContext_t CustomContext;

/**
 * END of Section BLE_DRIVER_CONTEXT
 */

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *pckt);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
/* USER CODE BEGIN PFD */

/* USER CODE END PFD */

/* Private functions ----------------------------------------------------------*/

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
    uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
    uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
    uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
/*
 The following 128bits UUIDs have been generated from the random UUID
 generator:
 D973F2E0-B19E-11E2-9E96-0800200C9A66: Service 128bits UUID
 D973F2E1-B19E-11E2-9E96-0800200C9A66: Characteristic_1 128bits UUID
 D973F2E2-B19E-11E2-9E96-0800200C9A66: Characteristic_2 128bits UUID
 */
#define COPY_SNAP_SVC_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_VOLTAGE_SENSOR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x41,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_CURRENT_SENSOR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x42,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_TEMP_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x43,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_HUM_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x44,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SNAP_SVC_2_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_NUMBER_VAR_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x41,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_ABSORBANCE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x42,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_ID_NUTRIENT_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x43,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_ID_DISP_EXTRA_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x00,0x00,0x01,0x44,0x8e,0x22,0x45,0x41,0x9d,0x4c,0x21,0xed,0xae,0x82,0xed,0x19)
#define COPY_SNAP_SVC_3_UUID(uuid_struct)          COPY_UUID_128(uuid_struct,0x00,0x00,0x02,0x40,0xcc,0x7a,0x48,0x2a,0x98,0x4a,0x7f,0x2e,0xd5,0xb3,0xe5,0x8f)
#define COPY_ACKNOWLEDGE_UUID(uuid_struct)    COPY_UUID_128(uuid_struct,0x02,0x41,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01)

/* USER CODE BEGIN PF */

/* USER CODE END PF */

/**
 * @brief  Event handler
 * @param  Event: Address of the buffer holding the Event
 * @retval Ack: Return whether the Event has been managed or not
 */
static SVCCTL_EvtAckStatus_t Custom_STM_Event_Handler(void *Event)
{
  SVCCTL_EvtAckStatus_t return_value;
  hci_event_pckt *event_pckt;
  evt_blecore_aci *blecore_evt;
  aci_gatt_attribute_modified_event_rp0 *attribute_modified;
  Custom_STM_App_Notification_evt_t     Notification;
  /* USER CODE BEGIN Custom_STM_Event_Handler_1 */

  /* USER CODE END Custom_STM_Event_Handler_1 */

  return_value = SVCCTL_EvtNotAck;
  event_pckt = (hci_event_pckt *)(((hci_uart_pckt*)Event)->data);

  switch (event_pckt->evt)
  {
    case HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE:
      blecore_evt = (evt_blecore_aci*)event_pckt->data;
      switch (blecore_evt->ecode)
      {
        case ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_BEGIN */
          attribute_modified = (aci_gatt_attribute_modified_event_rp0*)blecore_evt->data;
          if (attribute_modified->Attr_Handle == (CustomContext.CustomAckHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))
          {
            return_value = SVCCTL_EvtAckFlowEnable;
            /* USER CODE BEGIN CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
            /* The following lines represent the moment the APP has send an ackowledge to the device. Once the device has
             * received the ACK it turns on a green LED and also lowers the amount of measurements remaining to be read by the APP.
             * At this moment the device can only send one measurement with no problem. However it is possible to send multiple
             * measurements by testing and debugging. Only this section of the code has been changed for SNAP.
             * */

            HAL_GPIO_TogglePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin);		//Turn on Green LED
            HAL_Delay(500);

            acknowledge = 1; 	//ACK was received, the value of the variable acknowledge changes to 1

            total_measures--; 	//Total value of measures decreases

			Custom_STM_App_Update_Char(CUSTOM_STM_NUM_VAR, &total_measures);	//Save value of remaining measurements to BT memory
            HAL_GPIO_TogglePin(RGB_GREEN_GPIO_Port, RGB_GREEN_Pin);				//Turn off Green LED
            /* USER CODE END CUSTOM_STM_Service_3_Char_1_ACI_GATT_ATTRIBUTE_MODIFIED_VSEVT_CODE */
          } /* if (attribute_modified->Attr_Handle == (CustomContext.CustomAckHdle + CHARACTERISTIC_VALUE_ATTRIBUTE_OFFSET))*/
          /* USER CODE BEGIN EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */

          /* USER CODE END EVT_BLUE_GATT_ATTRIBUTE_MODIFIED_END */
          break;

        case ACI_GATT_READ_PERMIT_REQ_VSEVT_CODE :
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_READ_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_READ_PERMIT_REQ_END */
          break;

        case ACI_GATT_WRITE_PERMIT_REQ_VSEVT_CODE:
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_BEGIN */
          /* USER CODE BEGIN EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */

          /* USER CODE END EVT_BLUE_GATT_WRITE_PERMIT_REQ_END */
          break;
        /* USER CODE BEGIN BLECORE_EVT */

        /* USER CODE END BLECORE_EVT */
        default:
          /* USER CODE BEGIN EVT_DEFAULT */

          /* USER CODE END EVT_DEFAULT */
          break;
      }
      /* USER CODE BEGIN EVT_VENDOR*/

      /* USER CODE END EVT_VENDOR*/
      break; /* HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE */

      /* USER CODE BEGIN EVENT_PCKT_CASES*/

      /* USER CODE END EVENT_PCKT_CASES*/

    default:
      /* USER CODE BEGIN EVENT_PCKT*/

      /* USER CODE END EVENT_PCKT*/
      break;
  }

  /* USER CODE BEGIN Custom_STM_Event_Handler_2 */

  /* USER CODE END Custom_STM_Event_Handler_2 */

  return(return_value);
}/* end Custom_STM_Event_Handler */

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Service initialization
 * @param  None
 * @retval None
 */
void SVCCTL_InitCustomSvc(void)
{

  Char_UUID_t  uuid;
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN SVCCTL_InitCustomSvc_1 */

  /* USER CODE END SVCCTL_InitCustomSvc_1 */

  /**
   *  Register the event handler to the BLE controller
   */
  SVCCTL_RegisterSvcHandler(Custom_STM_Event_Handler);

  /**
   *          SNAP_SVC
   *
   * Max_Attribute_Records = 1 + 2*4 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for SNAP_SVC +
   *                                2 for VOLTAGE_SENSOR +
   *                                2 for CURRENT_SENSOR +
   *                                2 for TEMP +
   *                                2 for HUM +
   *                              = 9
   */

  COPY_SNAP_SVC_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             9,
                             &(CustomContext.CustomSnapHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: SNAP, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: SNAP \n\r");
  }

  /**
   *  VOLTAGE_SENSOR
   */
  COPY_VOLTAGE_SENSOR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnapHdle,
                          UUID_TYPE_128, &uuid,
                          SizeVol_Sen,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_AUTHOR_READ,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomVol_SenHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : VOL_SEN, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : VOL_SEN \n\r");
  }
  /**
   *  CURRENT_SENSOR
   */
  COPY_CURRENT_SENSOR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnapHdle,
                          UUID_TYPE_128, &uuid,
                          SizeCu_Sen,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_AUTHOR_READ,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomCu_SenHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : CU_SEN, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : CU_SEN \n\r");
  }
  /**
   *  TEMP
   */
  COPY_TEMP_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnapHdle,
                          UUID_TYPE_128, &uuid,
                          SizeTemp,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_AUTHOR_READ,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomTempHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : TEMP, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : TEMP \n\r");
  }
  /**
   *  HUM
   */
  COPY_HUM_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnapHdle,
                          UUID_TYPE_128, &uuid,
                          SizeHum,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_AUTHOR_READ,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomHumHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : HUM, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : HUM \n\r");
  }

  /**
   *          SNAP_SVC_2
   *
   * Max_Attribute_Records = 1 + 2*4 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for SNAP_SVC_2 +
   *                                2 for NUMBER_VAR +
   *                                2 for ABSORBANCE +
   *                                2 for ID_NUTRIENT +
   *                                2 for ID_DISP_EXTRA +
   *                              = 9
   */

  COPY_SNAP_SVC_2_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             9,
                             &(CustomContext.CustomSnap_1Hdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: SNAP_1, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: SNAP_1 \n\r");
  }

  /**
   *  NUMBER_VAR
   */
  COPY_NUMBER_VAR_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnap_1Hdle,
                          UUID_TYPE_128, &uuid,
                          SizeNum_Var,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomNum_VarHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : NUM_VAR, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : NUM_VAR \n\r");
  }
  /**
   *  ABSORBANCE
   */
  COPY_ABSORBANCE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnap_1Hdle,
                          UUID_TYPE_128, &uuid,
                          SizeAbs,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomAbsHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : ABS, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : ABS \n\r");
  }
  /**
   *  ID_NUTRIENT
   */
  COPY_ID_NUTRIENT_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnap_1Hdle,
                          UUID_TYPE_128, &uuid,
                          SizeId_Nut,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomId_NutHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : ID_NUT, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : ID_NUT \n\r");
  }
  /**
   *  ID_DISP_EXTRA
   */
  COPY_ID_DISP_EXTRA_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnap_1Hdle,
                          UUID_TYPE_128, &uuid,
                          SizeId_Extra,
                          CHAR_PROP_READ,
                          ATTR_PERMISSION_NONE,
                          GATT_DONT_NOTIFY_EVENTS,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomId_ExtraHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : ID_EXTRA, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : ID_EXTRA \n\r");
  }

  /**
   *          SNAP_SVC_3
   *
   * Max_Attribute_Records = 1 + 2*1 + 1*no_of_char_with_notify_or_indicate_property + 1*no_of_char_with_broadcast_property
   * service_max_attribute_record = 1 for SNAP_SVC_3 +
   *                                2 for Acknowledge +
   *                              = 3
   */

  COPY_SNAP_SVC_3_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_service(UUID_TYPE_128,
                             (Service_UUID_t *) &uuid,
                             PRIMARY_SERVICE,
                             3,
                             &(CustomContext.CustomSnap_2Hdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_service command: SNAP_2, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_service command: SNAP_2 \n\r");
  }

  /**
   *  Acknowledge
   */
  COPY_ACKNOWLEDGE_UUID(uuid.Char_UUID_128);
  ret = aci_gatt_add_char(CustomContext.CustomSnap_2Hdle,
                          UUID_TYPE_128, &uuid,
                          SizeAck,
                          CHAR_PROP_WRITE,
                          ATTR_PERMISSION_NONE,
                          GATT_NOTIFY_ATTRIBUTE_WRITE,
                          0x10,
                          CHAR_VALUE_LEN_CONSTANT,
                          &(CustomContext.CustomAckHdle));
  if (ret != BLE_STATUS_SUCCESS)
  {
    APP_DBG_MSG("  Fail   : aci_gatt_add_char command   : ACK, error code: 0x%x \n\r", ret);
  }
  else
  {
    APP_DBG_MSG("  Success: aci_gatt_add_char command   : ACK \n\r");
  }

  /* USER CODE BEGIN SVCCTL_InitCustomSvc_2 */

  /* USER CODE END SVCCTL_InitCustomSvc_2 */

  return;
}

/**
 * @brief  Characteristic update
 * @param  CharOpcode: Characteristic identifier
 * @param  Service_Instance: Instance of the service to which the characteristic belongs
 *
 */
tBleStatus Custom_STM_App_Update_Char(Custom_STM_Char_Opcode_t CharOpcode, uint8_t *pPayload)
{
  tBleStatus ret = BLE_STATUS_INVALID_PARAMS;
  /* USER CODE BEGIN Custom_STM_App_Update_Char_1 */

  /* USER CODE END Custom_STM_App_Update_Char_1 */

  switch (CharOpcode)
  {

    case CUSTOM_STM_VOL_SEN:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnapHdle,
                                       CustomContext.CustomVol_SenHdle,
                                       0, /* charValOffset */
                                       SizeVol_Sen, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value VOL_SEN command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value VOL_SEN command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_1*/
      break;

    case CUSTOM_STM_CU_SEN:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnapHdle,
                                       CustomContext.CustomCu_SenHdle,
                                       0, /* charValOffset */
                                       SizeCu_Sen, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value CU_SEN command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value CU_SEN command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_2*/
      break;

    case CUSTOM_STM_TEMP:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnapHdle,
                                       CustomContext.CustomTempHdle,
                                       0, /* charValOffset */
                                       SizeTemp, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value TEMP command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value TEMP command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_3*/
      break;

    case CUSTOM_STM_HUM:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnapHdle,
                                       CustomContext.CustomHumHdle,
                                       0, /* charValOffset */
                                       SizeHum, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value HUM command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value HUM command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_1_Char_4*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_1_Char_4*/
      break;

    case CUSTOM_STM_NUM_VAR:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnap_1Hdle,
                                       CustomContext.CustomNum_VarHdle,
                                       0, /* charValOffset */
                                       SizeNum_Var, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value NUM_VAR command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value NUM_VAR command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_1*/
      break;

    case CUSTOM_STM_ABS:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnap_1Hdle,
                                       CustomContext.CustomAbsHdle,
                                       0, /* charValOffset */
                                       SizeAbs, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value ABS command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value ABS command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_2*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_2*/
      break;

    case CUSTOM_STM_ID_NUT:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnap_1Hdle,
                                       CustomContext.CustomId_NutHdle,
                                       0, /* charValOffset */
                                       SizeId_Nut, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value ID_NUT command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value ID_NUT command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_3*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_3*/
      break;

    case CUSTOM_STM_ID_EXTRA:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnap_1Hdle,
                                       CustomContext.CustomId_ExtraHdle,
                                       0, /* charValOffset */
                                       SizeId_Extra, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value ID_EXTRA command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value ID_EXTRA command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_2_Char_4*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_2_Char_4*/
      break;

    case CUSTOM_STM_ACK:
      ret = aci_gatt_update_char_value(CustomContext.CustomSnap_2Hdle,
                                       CustomContext.CustomAckHdle,
                                       0, /* charValOffset */
                                       SizeAck, /* charValueLen */
                                       (uint8_t *)  pPayload);
      if (ret != BLE_STATUS_SUCCESS)
      {
        APP_DBG_MSG("  Fail   : aci_gatt_update_char_value ACK command, result : 0x%x \n\r", ret);
      }
      else
      {
        APP_DBG_MSG("  Success: aci_gatt_update_char_value ACK command\n\r");
      }
      /* USER CODE BEGIN CUSTOM_STM_App_Update_Service_3_Char_1*/

      /* USER CODE END CUSTOM_STM_App_Update_Service_3_Char_1*/
      break;

    default:
      break;
  }

  /* USER CODE BEGIN Custom_STM_App_Update_Char_2 */

  /* USER CODE END Custom_STM_App_Update_Char_2 */

  return ret;
}
