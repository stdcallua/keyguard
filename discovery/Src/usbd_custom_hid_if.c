/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2017 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"
/* USER CODE BEGIN INCLUDE */
/* USER CODE END INCLUDE */
/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */

/** @defgroup USBD_CUSTOM_HID 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_TYPES */
/* USER CODE END PRIVATE_TYPES */ 
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Defines
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_DEFINES */
/* USER CODE END PRIVATE_DEFINES */
  
/**
  * @}
  */ 

/** @defgroup USBD_CUSTOM_HID_Private_Macros
  * @{
  */ 
/* USER CODE BEGIN PRIVATE_MACRO */
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */ 

/** @defgroup USBD_AUDIO_IF_Private_Variables
 * @{
 */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
	0x06, 0x00, 0xff,              // USAGE_PAGE (Vendor Defined Page 1)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x95, 0x40,                    // REPORT_COUNT (64)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x81, 0x02,                    //   INPUT (Data,Var,Abs)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x15, 0x00,                    // LOGICAL_MINIMUM (0)
    0x26, 0xff, 0x00,              //   LOGICAL_MAXIMUM (255)
    0x95, 0x40,                    //   REPORT_COUNT (64)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x91, 0x02,                    //   OUTPUT (Data,Var,Abs)
    0xc0                           //         END_COLLECTION
	
	
	/*
	0x06, 0x00, 0xFF,            // (GLOBAL) USAGE_PAGE         0xFF00 Vendor-defined 
	0x09, 0x01,                  //   (LOCAL)  USAGE              0xFF000001  
  0xA1, 0x01,                  // (MAIN)   COLLECTION         0x01 Application (Usage=0x0: Page=, Usage=, Type=) <-- Warning: USAGE type should be CA (Application)
	0x85, 0x01,                  //   (GLOBAL) REPORT_ID          0x01 (1) 
	0x09, 0x01,                  //   (LOCAL)  USAGE              0xFF000001  
  0x15, 0x00,                  //   (GLOBAL) LOGICAL_MINIMUM    0x00 (0) <-- Redundant: LOGICAL_MINIMUM is already 0
  0x26, 0xFF, 		             //   (GLOBAL) LOGICAL_MAXIMUM    0x00FF (255) 
  0x75, 0x08,                  //   (GLOBAL) REPORT_SIZE        0x08 (8) Number of bits per field 
  0x95, 0x40,                  //   (GLOBAL) REPORT_COUNT       0x40 (64) Number of fields 
  0x81, 0x82,                  //   (MAIN)   INPUT              0x00000002 (64 fields x 8 bits) 0=Data 1=Variable 0=Absolute 0=NoWrap 0=Linear 0=PrefState 0=NoNull 0=NonVolatile 0=Bitmap 
  0xC0                         // (MAIN)   END_COLLECTION     Application
	*/
	/*
		0x06, 0x00, 0xff,              // 	USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // 	USAGE (Vendor Usage 1)
    // System Parameters
    0xa1, 0x01,                    // 	COLLECTION (Application)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x40, 	                     //   REPORT_COUNT (4)
    0xb1, 0x82,                    //   FEATURE (Data,Var,Abs,Vol)
    0x85, 0x01,                    //   REPORT_ID (1)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
    0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)
	
    0x85, 0x02,                    //   REPORT_ID (2)
    0x09, 0x02,                    //   USAGE (Vendor Usage 2)
    0x75, 0x08,                    //   REPORT_SIZE (8)
    0x95, 0x40, 	                     //   REPORT_COUNT (4)
    0x81, 0x82,                    //   INPUT (Data,Var,Abs,Vol)
		0xC0    */
   
}; 

/* USER CODE BEGIN PRIVATE_VARIABLES */
/* USER CODE END PRIVATE_VARIABLES */
/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_IF_Exported_Variables
  * @{
  */ 
  extern USBD_HandleTypeDef hUsbDeviceFS;
	extern uint8_t dataToSend[65];
/* USER CODE BEGIN EXPORTED_VARIABLES */
/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */ 
  
/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @{
  */
static int8_t CUSTOM_HID_Init_FS     (void);
static int8_t CUSTOM_HID_DeInit_FS   (void);
static int8_t CUSTOM_HID_OutEvent_FS (uint8_t event_idx, uint8_t state);

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS = 
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS,
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  CUSTOM_HID_Init_FS
  *         Initializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{ 
  /* USER CODE BEGIN 4 */ 
  return (0);
  /* USER CODE END 4 */ 
}

/**
  * @brief  CUSTOM_HID_DeInit_FS
  *         DeInitializes the CUSTOM HID media low layer
  * @param  None
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */ 
  return (0);
  /* USER CODE END 5 */ 
}

/**
  * @brief  CUSTOM_HID_OutEvent_FS
  *         Manage the CUSTOM HID class events       
  * @param  event_idx: event index
  * @param  state: event state
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */

uint8_t SendBuffer[65];
uint8_t ReceiveBuffer[65];


static int8_t CUSTOM_HID_OutEvent_FS  (uint8_t event_idx, uint8_t state)
{ 
  /* USER CODE BEGIN 6 */ 
	USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;  
	HAL_Delay(30);
	for (uint8_t i = 0; i < 65; i++)
  {
    ReceiveBuffer[i] = hhid->Report_buf[i];
  }
	HAL_Delay(30);
//  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_6);
	//uint8_t dataToSend[65];
	SendBuffer[0] = ReceiveBuffer[0]++;
  SendBuffer[1] = ReceiveBuffer[1]++;
	SendBuffer[2] = ReceiveBuffer[2]++;
	SendBuffer[3] = 0xff;
	SendBuffer[4] = ReceiveBuffer[4]++;
	
	
	uint8_t dataToSend[65];
	dataToSend[0] = 1;
  dataToSend[1] = 2;
  dataToSend[2] = 3;
  dataToSend[3] = 4;
	dataToSend[4] = 5;
	
	//USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, &dataToSend[0], 16);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
	HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
  return (0);
  /* USER CODE END 6 */ 
}

/* USER CODE BEGIN 7 */ 
/**
  * @brief  USBD_CUSTOM_HID_SendReport_FS
  *         Send the report to the Host       
  * @param  report: the report to be sent
  * @param  len: the report length
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
/*  
static int8_t USBD_CUSTOM_HID_SendReport_FS ( uint8_t *report,uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len); 
}
*/
/* USER CODE END 7 */ 

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */ 

/**
  * @}
  */  
	
	
	
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
