# BluePill USB-CDC Test

## Overview

Simple serial transmit and recieve using BluePill (STM32F103C8T6). This also includes setting the line coding and forcing the host to re-enumerate USB on reset (otherwise it is not possible to reconnect to the COM port from the host after it the device has been reset).

Only `CDC_Transmit_FS` is defined in `usbd_cdc_if.h`, which can be called from `main.c` (or used internally in `usbd_cdc_if.c`). The code below is a trivial implementation of a recieve buffer, plus a exported functions which can be called from `main.c`: `CDC_GetRxBufferBytesAvailable_FS`, `CDC_ReadRxBuffer_FS` and `CDC_FlushRxBuffer_FS`;

## Steps to develop from scratch

### Project configuration (.ioc file)

- Create new STM32CubeIDE project, selecting appropriate controller
- Select SYS->Debug->Serial Wire to Enable debug output
- Select USB->Device(FS)
- Select (Middleware) USB_DEVICE->Virtual COM Port
- Set PC13 as GPIO Output, with a user label of `LED`, and an internal pull-up
- If external crystal is used select RCC->Crystal/Ceramic Resonator then go to clock configuration and set up as appropriate (for BluePill, HSE is 8MHz). Ensure that USB frequency equates to 48MHz, but HCLK (MHz) can be higher.
- Save to generate code

### Code Changes
---
#### USB_Device/App/usbd_cdc_if.h
```C
/* USER CODE BEGIN EXPORTED_TYPES */

typedef enum
{
  USB_CDC_READ_RX_BUFFER_OK   = 0U,
  USB_CDC_READ_RX_BUFFER_NO_DATA
} USB_CDC_READ_RX_BUFFER_StatusTypeDef;

/* USER CODE END EXPORTED_TYPES */

```

```C
/* USER CODE BEGIN EXPORTED_FUNCTIONS */

uint8_t CDC_ReadRxBuffer_FS(uint8_t* Buf, uint16_t Len);
uint16_t CDC_GetRxBufferBytesAvailable_FS();
void CDC_FlushRxBuffer_FS();

/* USER CODE END EXPORTED_FUNCTIONS */
```
---
#### USB_Device/App/usbd_cdc_if.c

```C
/* USER CODE BEGIN PRIVATE_DEFINES */

#define HL_RX_BUFFER_SIZE 256 // Can be larger if desired

/* USER CODE END PRIVATE_DEFINES */
```

```C
/* USER CODE BEGIN PRIVATE_VARIABLES */

uint8_t lcBuffer[7]; // Line coding buffer
uint8_t rxBuffer[HL_RX_BUFFER_SIZE]; // Receive buffer
volatile uint16_t rxBufferHeadPos = 0; // Receive buffer write position
volatile uint16_t rxBufferTailPos = 0; // Receive buffer read position

/* USER CODE END PRIVATE_VARIABLES */
```

```C
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
```

Edit `CDC_Control_FS`. Host invokes GET and SET multiple times during USB enumeration and when connecting to a serial port. Seems to update baud rate, stop bit, parity and data bits settings separately. Doesnt seem to matter what these are set to, however I have set sensible defaults in the `CDC_Init_FS` method which are then modified here as the host sets them when connecting to the port. 

```C
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
```

```C
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  uint8_t len = (uint8_t) *Len; // Get length

    uint16_t tempHeadPos = rxBufferHeadPos; // Increment temp head pos while writing, then update main variable when complete

    for (uint32_t i = 0; i < len; i++) {
  	  rxBuffer[tempHeadPos] = Buf[i];

  	  // Compact position increment logic
  	  tempHeadPos = (uint16_t)((uint16_t)(tempHeadPos + 1) % HL_RX_BUFFER_SIZE);

  	  /*
  	  // Simple but more verbose version if preferred
  	  tempHeadPos++;
  	  if (tempHeadPos == HL_RX_BUFFER_SIZE) {
  		  tempHeadPos = 0;
  	  }
  	  */

  	  if (tempHeadPos == rxBufferTailPos) {
  		  return USBD_FAIL;
  	  }

    }

    rxBufferHeadPos = tempHeadPos;

  return (USBD_OK);
  /* USER CODE END 6 */
}
```

```C
/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

uint8_t CDC_ReadRxBuffer_FS(uint8_t* Buf, uint16_t Len) {
	uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();

	if (bytesAvailable < Len)
		return USB_CDC_READ_RX_BUFFER_NO_DATA;

	// Update this to use memcpy in future?
	for (uint8_t i = 0; i < Len; i++) {
		Buf[i] = rxBuffer[rxBufferTailPos];
		rxBufferTailPos = (uint16_t)((uint16_t)(rxBufferTailPos + 1) % HL_RX_BUFFER_SIZE);
		/*
		rxBufferTailPos++;
		if (rxBufferTailPos == HL_RX_BUFFER_SIZE) {
			rxBufferTailPos = 0;
		}
		*/
	}

	return USB_CDC_READ_RX_BUFFER_OK;
}


uint16_t CDC_GetRxBufferBytesAvailable_FS() {

	// Compact version
    return (uint16_t)(rxBufferHeadPos - rxBufferTailPos) % HL_RX_BUFFER_SIZE;
	
    /*
	// Simple, more verbose version if preferred
	
	// Take snapshot of head and tail pos to prevent head position changing in 
	// CDC_Receive_FS after if statement but before calculation
	uint16_t headPos = rxBufferHeadPos;
	uint16_t tailPos = rxBufferTailPos;

	if (headPos >= tailPos)
		return headPos - tailPos;
	else
		return HL_RX_BUFFER_SIZE - tailPos + headPos;
	*/
}

void CDC_FlushRxBuffer_FS() {
    for (int i = 0; i < HL_RX_BUFFER_SIZE; i++) {
    	rxBuffer[i] = 0;
    }

    rxBufferHeadPos = 0;
    rxBufferTailPos = 0;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
```
---
#### usb_device.c

**This forces the host to re-enumerate the device as it is akin to unplugging the USB cable and plugging it back in. Only needed if there is a pullup resistor on the USB_DP line - as is the case with the BluePill** 

```C
void MX_USB_DEVICE_Init(void)
{
  /* USER CODE BEGIN USB_DEVICE_Init_PreTreatment */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(100);
  /* USER CODE END USB_DEVICE_Init_PreTreatment */
```
---
#### main.c

```C
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
/* USER CODE END Includes */
```

```C
/* Private user code ---------------------------------------------------------*/
  /* USER CODE BEGIN 2 */

  uint8_t rxData[8];
  memset(rxData, 0, 8);

  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  HAL_Delay(500);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* USER CODE END 2 */
```

```C
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Echo data
    uint16_t bytesAvailable = CDC_GetRxBufferBytesAvailable_FS();
    if (bytesAvailable > 0) {
        uint16_t bytesToRead = bytesAvailable >= 8 ? 8 : bytesAvailable;
	    if (CDC_ReadRxBuffer_FS(rxData, bytesToRead) == USB_CDC_READ_RX_BUFFER_OK) {
	        while (CDC_Transmit_FS(rxData, bytesToRead) == USBD_BUSY);
        }
    }
  }
  /* USER CODE END 3 */
}
```