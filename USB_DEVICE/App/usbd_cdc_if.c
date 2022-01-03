/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v2.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device library.
  * @{
  */

/** @addtogroup USBD_CDC_IF
  * @{
  */

/** @defgroup USBD_CDC_IF_Private_TypesDefinitions USBD_CDC_IF_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Defines USBD_CDC_IF_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

#define HL_RX_BUFFER_SIZE 256 // Can be larger if desired

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Macros USBD_CDC_IF_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_Variables USBD_CDC_IF_Private_Variables
  * @brief Private variables.
  * @{
  */
/* Create buffer for reception and transmission           */
/* It's up to user to redefine and/or remove those define */
/** Received data over USB are stored in this buffer      */
uint8_t UserRxBufferFS[APP_RX_DATA_SIZE];

/** Data to send over USB CDC are stored in this buffer   */
uint8_t UserTxBufferFS[APP_TX_DATA_SIZE];

/* USER CODE BEGIN PRIVATE_VARIABLES */

uint8_t lcBuffer[7]; // Line coding buffer
volatile uint8_t rxBuffer[HL_RX_BUFFER_SIZE]; // Receive buffer
volatile uint16_t rxBufferHeadPos = 0; // Receive buffer write position
volatile uint16_t rxBufferTailPos = 0; // Receive buffer read position

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Exported_Variables USBD_CDC_IF_Exported_Variables
  * @brief Public variables.
  * @{
  */

extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CDC_IF_Private_FunctionPrototypes USBD_CDC_IF_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CDC_Init_FS(void);
static int8_t CDC_DeInit_FS(void);
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length);
static int8_t CDC_Receive_FS(uint8_t* pbuf, uint32_t *Len);

/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */

/**
  * @}
  */

USBD_CDC_ItfTypeDef USBD_Interface_fops_FS =
{
  CDC_Init_FS,
  CDC_DeInit_FS,
  CDC_Control_FS,
  CDC_Receive_FS
};

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initializes the CDC media low layer over the FS USB IP
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Init_FS(void)
{
  /* USER CODE BEGIN 3 */
  /* Set Application Buffers */
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, UserTxBufferFS, 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, UserRxBufferFS);

  // https://stackoverflow.com/a/26925578
  uint32_t baudrate = 9600;
  lcBuffer[0] = (uint8_t)(baudrate);
  lcBuffer[1] = (uint8_t)(baudrate >> 8);
  lcBuffer[2] = (uint8_t)(baudrate >> 16);
  lcBuffer[3] = (uint8_t)(baudrate >> 24);
  lcBuffer[4] = 0; // 1 Stop bit
  lcBuffer[5] = 0; // No parity
  lcBuffer[6] = 8; // 8 data bits

  return (USBD_OK);
  /* USER CODE END 3 */
}

/**
  * @brief  DeInitializes the CDC media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_DeInit_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  Manage the CDC class requests
  * @param  cmd: Command code
  * @param  pbuf: Buffer containing command data (request parameters)
  * @param  length: Number of data to be sent (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  switch(cmd)
  {
    case CDC_SEND_ENCAPSULATED_COMMAND:

    break;

    case CDC_GET_ENCAPSULATED_RESPONSE:

    break;

    case CDC_SET_COMM_FEATURE:

    break;

    case CDC_GET_COMM_FEATURE:

    break;

    case CDC_CLEAR_COMM_FEATURE:

    break;

    /*******************************************************************************/
    /* Line Coding Structure                                                       */
    /*-----------------------------------------------------------------------------*/
    /* Offset | Field       | Size | Value  | Description                          */
    /* 0      | dwDTERate   |   4  | Number |Data terminal rate, in bits per second*/
    /* 4      | bCharFormat |   1  | Number | Stop bits                            */
    /*                                        0 - 1 Stop bit                       */
    /*                                        1 - 1.5 Stop bits                    */
    /*                                        2 - 2 Stop bits                      */
    /* 5      | bParityType |  1   | Number | Parity                               */
    /*                                        0 - None                             */
    /*                                        1 - Odd                              */
    /*                                        2 - Even                             */
    /*                                        3 - Mark                             */
    /*                                        4 - Space                            */
    /* 6      | bDataBits  |   1   | Number Data bits (5, 6, 7, 8 or 16).          */
    /*******************************************************************************/
    case CDC_SET_LINE_CODING:
      lcBuffer[0] = pbuf[0];
      lcBuffer[1] = pbuf[1];
      lcBuffer[2] = pbuf[2];
      lcBuffer[3] = pbuf[3];
      lcBuffer[4] = pbuf[4];
      lcBuffer[5] = pbuf[5];
      lcBuffer[6] = pbuf[6];
    break;

    case CDC_GET_LINE_CODING:
      pbuf[0] = lcBuffer[0];
      pbuf[1] = lcBuffer[1];
      pbuf[2] = lcBuffer[2];
      pbuf[3] = lcBuffer[3];
      pbuf[4] = lcBuffer[4];
      pbuf[5] = lcBuffer[5];
      pbuf[6] = lcBuffer[6];

      // Get line coding is invoked when the host connects, clear the RxBuffer when this occurs
      CDC_FlushRxBuffer_FS();
    break;

    case CDC_SET_CONTROL_LINE_STATE:

    break;

    case CDC_SEND_BREAK:

    break;

  default:

    break;
  }

  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Data received over USB OUT endpoint are sent over CDC interface
  *         through this function.
  *
  *         @note
  *         This function will issue a NAK packet on any OUT packet received on
  *         USB endpoint until exiting this function. If you exit this function
  *         before transfer is complete on CDC interface (ie. using DMA controller)
  *         it will result in receiving more data while previous ones are still
  *         not sent.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);

  uint8_t len = (uint8_t) *Len; // Get length
  uint16_t tempHeadPos = rxBufferHeadPos; // Increment temp head pos while writing, then update main variable when complete

  for (uint32_t i = 0; i < len; i++) {
    rxBuffer[tempHeadPos] = Buf[i];
  	tempHeadPos = (uint16_t)((uint16_t)(tempHeadPos + 1) % HL_RX_BUFFER_SIZE);
    if (tempHeadPos == rxBufferTailPos) {
      return USBD_FAIL;
    }
  }

  rxBufferHeadPos = tempHeadPos;
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  
  return (USBD_OK);
  /* USER CODE END 6 */
}

/**
  * @brief  CDC_Transmit_FS
  *         Data to send over USB IN endpoint are sent over CDC interface
  *         through this function.
  *         @note
  *
  *
  * @param  Buf: Buffer of data to be sent
  * @param  Len: Number of data to be sent (in bytes)
  * @retval USBD_OK if all operations are OK else USBD_FAIL or USBD_BUSY
  */
uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 7 */
  USBD_CDC_HandleTypeDef *hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
  if (hcdc->TxState != 0){
    return USBD_BUSY;
  }
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, Buf, Len);
  result = USBD_CDC_TransmitPacket(&hUsbDeviceFS);
  /* USER CODE END 7 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
uint8_t CDC_ReadRxBuffer_FS(uint8_t* Buf, uint16_t Len)
{
  uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();

  if (bytesAvailable < Len)
    return USB_CDC_RX_BUFFER_NO_DATA;

  for (uint8_t i = 0; i < Len; i++) {
    Buf[i] = rxBuffer[rxBufferTailPos];
    rxBufferTailPos = (uint16_t)((uint16_t)(rxBufferTailPos + 1) % HL_RX_BUFFER_SIZE);
  }

  return USB_CDC_RX_BUFFER_OK;
}

uint8_t CDC_PeekRxBuffer_FS(uint8_t* Buf, uint16_t Len)
{
  uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();

  if (bytesAvailable < Len)
    return USB_CDC_RX_BUFFER_NO_DATA;

  for (uint8_t i = 0; i < Len; i++) {
    Buf[i] = rxBuffer[(rxBufferTailPos + i) % HL_RX_BUFFER_SIZE]; // Get data without incrementing the tail position
  }

  return USB_CDC_RX_BUFFER_OK;
}

uint16_t CDC_GetRxBufferBytesAvailable_FS()
{
  return (uint16_t)(rxBufferHeadPos - rxBufferTailPos) % HL_RX_BUFFER_SIZE;
}

void CDC_FlushRxBuffer_FS() {
  for (int i = 0; i < HL_RX_BUFFER_SIZE; i++) {
    rxBuffer[i] = 0;
  }

  rxBufferHeadPos = 0;
  rxBufferTailPos = 0;
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
