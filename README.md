# BluePill USB-CDC Test

## Overview

Simple serial transmit and recieve using BluePill (STM32F103C8T6). This also includes setting the line coding and forcing the host to re-enumerate USB on reset (otherwise it is not possible to reconnect to the COM port from the host after it the device has been reset).

Key points:
- Only `CDC_Transmit_FS` is defined in `usbd_cdc_if.h`, which can be called from main.c (or used internally in `usbd_cdc_if.c`)
- The `rxBuffer` which is defined in `usbd_cdc_if.c` is updated the in the `CDC_Receive_FS` function (which has been modified from the default). While not necessary, this variable is referenced in `main.c` - meaning that while debugging, it is possible to hit a breakpoint in the main while loop and inspect the contents of `rxBuffer`.

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

#### USB_Device/App/usbd_cdc_if.c

```C
/* USER CODE BEGIN PRIVATE_TYPES */
uint8_t rxBuffer[64];
/* USER CODE END PRIVATE_TYPES */
```

```C
static int8_t CDC_Receive_FS(uint8_t* Buf, uint32_t *Len)
{
  /* USER CODE BEGIN 6 */
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, &Buf[0]);
  USBD_CDC_ReceivePacket(&hUsbDeviceFS);

  // Copy data to buffer
  memset(rxBuffer, '\0', 64); // Clear buffer
  uint8_t len = (uint8_t) *Len;
  memcpy(rxBuffer, Buf, len); // Copy data to buffer
  return (USBD_OK);
  /* USER CODE END 6 */
}
```

```C
static int8_t CDC_Control_FS(uint8_t cmd, uint8_t* pbuf, uint16_t length)
{
  /* USER CODE BEGIN 5 */
  uint32_t baudrate = 9600;

  switch(cmd)
  {
```

Then further down in the same function:

```C
    case CDC_GET_LINE_CODING:
	  // https://stackoverflow.com/a/26925578
	  pbuf[0] = (uint8_t)(baudrate);
	  pbuf[1] = (uint8_t)(baudrate >> 8);
	  pbuf[2] = (uint8_t)(baudrate >> 16);
	  pbuf[3] = (uint8_t)(baudrate >> 24);
	  pbuf[4] = 0;
	  pbuf[5] = 0;
	  pbuf[6] = 8;
    break;
```

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

#### main.c

```C
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "string.h"
/* USER CODE END Includes */
```

```C
/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

char *data = "Hello from device!\n";
extern uint8_t rxBuffer[64]; // Reference to buffer defined in "usb_cdc_if.h"

/* USER CODE END 0 */
```

```C
/* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	uint8_t result = CDC_Transmit_FS((uint8_t *) data, strlen(data));
	if (result == USBD_OK) {
	    HAL_Delay(500); // Blinks faster if host connected and receiving data
	}
	else {
	    // Other response types: USBD_FAIL or USBD_BUSY
		HAL_Delay(1000); // Blinks slower if host not connected
	}
	HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }
  /* USER CODE END 3 */
```