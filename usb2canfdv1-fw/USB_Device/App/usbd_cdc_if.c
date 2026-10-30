/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_cdc_if.c
  * @version        : v3.0_Cube
  * @brief          : Usb device for Virtual Com Port.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "usbd_cdc_if.h"

/* USER CODE BEGIN INCLUDE */
#include "buffer.h"
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
static int8_t CDC_TransmitCplt_FS(uint8_t *pbuf, uint32_t *Len, uint8_t epnum);

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
  CDC_Receive_FS,
  CDC_TransmitCplt_FS
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
  USBD_CDC_SetTxBuffer(&hUsbDeviceFS, (uint8_t *)buf_cdc_tx.data[buf_cdc_tx.tail], 0);
  USBD_CDC_SetRxBuffer(&hUsbDeviceFS, (uint8_t *)buf_cdc_rx.data[buf_cdc_rx.head]);
  // The library arms the OUT endpoint on the head buffer after this call, even if the
  // reception was paused (see CDC_Receive_FS).
  buf_cdc_rx.rx_state = BUF_CDC_RX_RUNNING;
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
  UNUSED(length);
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

    break;

    case CDC_GET_LINE_CODING:

    pbuf[0] = (uint8_t)(115200);
    pbuf[1] = (uint8_t)(115200 >> 8);
    pbuf[2] = (uint8_t)(115200 >> 16);
    pbuf[3] = (uint8_t)(115200 >> 24);
    pbuf[4] = 0; // stop bits (1)
    pbuf[5] = 0; // parity (none)
    pbuf[6] = 8; // number of bits (8)
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
  UNUSED(Buf);
  buf_cdc_rx.msglen[buf_cdc_rx.head] = *Len;

  uint32_t new_head = (buf_cdc_rx.head + 1) % BUF_CDC_RX_NUM_BUFS;
  if (new_head == buf_cdc_rx.tail)
  {
    // No room to move to the next buffer. The endpoint is re-armed only when there is
    // room, so this happens only if the class is re-initialized (e.g. by a USB reset)
    // while the reception is paused: the library then re-arms the endpoint by itself.
    // Hold the packet in the current buffer and leave the endpoint NAK.
    // CDC_Resume_Receive_FS moves to the next buffer once a buffer is free.
    buf_cdc_rx.rx_state = BUF_CDC_RX_PAUSED_HELD;
    return (USBD_OK);
  }

  // Move to next buffer. The received one will be processed in main loop.
  buf_cdc_rx.head = new_head;

  // Start listening on next buffer if the one after it is also free, so that the next
  // packet can be kept. Otherwise leave the endpoint NAK, which holds off the host,
  // until CDC_Resume_Receive_FS is called after a buffer is processed.
  if ((buf_cdc_rx.head + 1) % BUF_CDC_RX_NUM_BUFS != buf_cdc_rx.tail)
  {
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, (uint8_t *)buf_cdc_rx.data[buf_cdc_rx.head]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
  else
  {
    buf_cdc_rx.rx_state = BUF_CDC_RX_PAUSED;
  }
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

/**
  * @brief  CDC_TransmitCplt_FS
  *         Data transmitted callback
  *
  *         @note
  *         This function is IN transfer complete callback used to inform user that
  *         the submitted Data is successfully sent over USB.
  *
  * @param  Buf: Buffer of data to be received
  * @param  Len: Number of data received (in bytes)
  * @retval Result of the operation: USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CDC_TransmitCplt_FS(uint8_t *Buf, uint32_t *Len, uint8_t epnum)
{
  uint8_t result = USBD_OK;
  /* USER CODE BEGIN 13 */
  UNUSED(Buf);
  UNUSED(Len);
  UNUSED(epnum);
  uint32_t new_tail = (uint32_t)((buf_cdc_tx.tail + 1) % BUF_CDC_TX_NUM_BUFS);
  if (new_tail != buf_cdc_tx.head)
  {
    if (CDC_Transmit_FS((uint8_t *)buf_cdc_tx.data[new_tail], buf_cdc_tx.msglen[new_tail]) == USBD_OK)
    {
      buf_cdc_tx.tail = new_tail;
    }
    else
    {
      // If the transmission completes while interrupts are disabled in the main loop.
      result = USBD_FAIL;
    }
  }
  /* USER CODE END 13 */
  return result;
}

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @brief  CDC_Resume_Receive_FS
  *         Resumes the reception paused by CDC_Receive_FS once a buffer is free.
  *
  *         @note
  *         Call this function with interrupts disabled, after the main loop has
  *         processed a buffer (moved buf_cdc_rx.tail).
  *
  * @retval None
  */
void CDC_Resume_Receive_FS(void)
{
  // A packet held in the current buffer (see CDC_Receive_FS): move to next buffer first.
  if (buf_cdc_rx.rx_state == BUF_CDC_RX_PAUSED_HELD
      && (buf_cdc_rx.head + 1) % BUF_CDC_RX_NUM_BUFS != buf_cdc_rx.tail)
  {
    buf_cdc_rx.head = (buf_cdc_rx.head + 1) % BUF_CDC_RX_NUM_BUFS;
    buf_cdc_rx.rx_state = BUF_CDC_RX_PAUSED;
  }

  if (buf_cdc_rx.rx_state == BUF_CDC_RX_PAUSED
      && (buf_cdc_rx.head + 1) % BUF_CDC_RX_NUM_BUFS != buf_cdc_rx.tail)
  {
    buf_cdc_rx.rx_state = BUF_CDC_RX_RUNNING;
    USBD_CDC_SetRxBuffer(&hUsbDeviceFS, (uint8_t *)buf_cdc_rx.data[buf_cdc_rx.head]);
    USBD_CDC_ReceivePacket(&hUsbDeviceFS);
  }
}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */

/**
  * @}
  */

/**
  * @}
  */

