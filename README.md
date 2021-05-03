# BluePill USB-CDC Test

## Overview

Simple serial transmit and recieve using BluePill (STM32F103C8T6). This also includes setting the line coding and forcing the host to re-enumerate USB on reset (otherwise it is not possible to reconnect to the COM port from the host after it the device has been reset).

Only `CDC_Transmit_FS` is defined in `usbd_cdc_if.h`, which can be called from `main.c` (or used internally in `usbd_cdc_if.c`). The code below is a trivial implementation of a recieve buffer, plus a exported functions which can be called from `main.c`: `CDC_ReadRxBuffer_FS` and `CDC_FlushRxBuffer_FS`;

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
#### USB_Device/App/usbd_cdc_if.c

```C
/* USER CODE BEGIN PRIVATE_DEFINES */

#define HL_RX_BUFFER_SIZE 256

/* USER CODE END PRIVATE_DEFINES */
```

```C
/* USER CODE BEGIN PRIVATE_VARIABLES */

uint8_t lcBuffer[7]; // Line coding buffer
uint8_t rxBuffer[256]; // Receive buffer
uint16_t rxBufferWritePos = 0; // Receive buffer write position
uint16_t rxBufferBytesAvailable = 0; // Receive buffer bytes available
uint16_t rxBufferReadPos = 0; // Receive buffer read position

/* USER CODE END PRIVATE_VARIABLES */
```

```C
/* USER CODE BEGIN PRIVATE_FUNCTIONS_DECLARATION */

void LockRxBuffer();
void UnlockRxBuffer();

/* USER CODE END PRIVATE_FUNCTIONS_DECLARATION */
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
    break;
```

```C
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  uint8_t len = (uint8_t) *Len; // Get length

  LockRxBuffer();

  // Update this to use memcpy in future
  for (uint32_t i = 0; i < len; i++) {
	  rxBuffer[rxBufferWritePos] = Buf[i];
	  rxBufferWritePos = (uint8_t)((rxBufferWritePos + 1) % HL_RX_BUFFER_SIZE);
  }

  rxBufferBytesAvailable += (uint16_t)len;

  UnlockRxBuffer();

  return (USBD_OK);
  /* USER CODE END 6 */
}
```

```C
/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

uint8_t CDC_ReadRxBuffer_FS(uint8_t* Buf, uint16_t Len) {

	if (rxBufferBytesAvailable < Len)
		return 0;

	LockRxBuffer();

	// Update this to use memcpy in future
	for (uint8_t i = 0; i < Len; i++) {
		Buf[i] = rxBuffer[rxBufferReadPos];
		rxBufferReadPos = (uint16_t)((rxBufferReadPos + 1) % HL_RX_BUFFER_SIZE);
	}

	rxBufferBytesAvailable -= (uint16_t)Len;

	UnlockRxBuffer();

	return 1;
}

void CDC_FlushRxBuffer_FS() {
    memset(rxBuffer, 0, HL_RX_BUFFER_SIZE);
    rxBufferWritePos = 0;
    rxBufferReadPos = 0;
    rxBufferBytesAvailable = 0;
}

void LockRxBuffer() {
    while (rxBufferLock != 0) {
        HAL_Delay(1);
    }
    rxBufferLock = 1;
}

void UnlockRxBuffer() {
    rxBufferLock = 0;
}

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
```
---
#### usbd_csc_if.h
```C
/* USER CODE BEGIN EXPORTED_FUNCTIONS */

uint8_t CDC_ReadRxBuffer_FS(uint8_t* Buf, uint16_t Len);
void CDC_FlushRxBuffer_FS();

/* USER CODE END EXPORTED_FUNCTIONS */
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
#include "string.h"
/* USER CODE END Includes */
```

```C
/* Private user code ---------------------------------------------------------*/
  /* USER CODE BEGIN 2 */

  char *txData = "Hello from device!\n";
  uint8_t rxData[8];
  uint32_t lastInterval = HAL_GetTick();

  memset(rxData, 0, 8);

  /* USER CODE END 2 */
```

```C
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	uint32_t currentInterval = HAL_GetTick();
	if (currentInterval - lastInterval > 2000) {
		lastInterval = currentInterval;
		uint8_t result = CDC_Transmit_FS((uint8_t *) txData, strlen(txData));
		if (result == USBD_OK) { // result is USBD_FAIL if host is not connected
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		}
	}
	else {
		if (CDC_ReadRxBuffer_FS(rxData, 8)) {
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			HAL_Delay(100);
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		}
	}
  }
  /* USER CODE END 3 */
```