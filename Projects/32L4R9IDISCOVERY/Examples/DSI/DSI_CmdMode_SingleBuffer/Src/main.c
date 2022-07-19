/**
  ******************************************************************************
  * @file    DSI/DSI_CmdMode_SingleBuffer/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use LCD DSI to display
  *          an RGB image (390x390) using the STM32L4xx HAL API and BSP.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2017 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gfxmmu_lut_390x390_24bpp.h"
#include "image_390x390_rgb888.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*
 * @Harvey Zhang
 *
 * The original example is optimized for 390*390 round display with display controller RM67162.
 *
 * As pixels outside of the circle will never be displayed,
 * those pixel data don't have to be stored in the frame buffer nor have to be transfer to the display controller.
 * So it needs about 20% less buffer when using the round display.
 * It's done with the help of gfxmmu.
 *
 * In our case, we are using a 320*320 square display with display controller RM69032.
 *
 * There are several modifications required.
 * 1. The gfxmmu can't help to save memory when using square display, we don't need it anymore.
 *
 * 2. We will need a full size frame buffer. The size is 320 * 320 * 3 / 4 = 76800 words.
 *
 * 3. The display controller initialization code has to be changed according to RM69032.
 *    3.1 The panel of the display is powered by TPS65631(ELVDD/ELVSS).
 *        TPS65631 can be programmed from it CTRL pin to supply different voltage on ELVSS.
 *        The CTRL pin of TPS65631 can be routed to SWRIE pin of the display controller
 *        or it can be routed to the host.
 *        So either the host or the display controller needs to program TPS65631.
 *        In the current adapter board, TPS65631 CTRL pin is routed to PA8 of the host.
 *        The code controlling PA8 only turns on TPS65631 for now. So ELVSS is -4V which might impact the life span of the display.
 *        If you want to program TPS65631 to output -3.3V, you will need to generate 0x0C pulses on PA8(See the data sheet of TPS65631).
 *
 *    3.2 The initialization of RM69032 has been done under LP mode.
 *        You can send instructions under HS mode but it looks like those instructions don't have any effect.(Just a guess)
 *
 *    3.3 During initialization of RM69032, you can't send SET_TEAR_ON otherwise all the following instructions will fail.
 *        If you need enable TE signal from the display, probably you have send SET_TEAR_ON after all other initialization instructions.
 *
 * 4. The display controller is powered by VDD/IOVDD. After turning on VDD/IOVDD, make sure assert the DSI_RESET correctly.
 *    The power on sequence is important, make sure to follow the sequence otherwise the display controller may not get a complete reset.
 *
 * 5. On the discovery board, USART2 of the host is routed to the STLINK virtual com port. I use USART2 to output debug information.
 *    Very useful.
 *
 */
#define DISPLAY_DRIVER_RM69032

/** @addtogroup STM32L4xx_HAL_Examples
  * @{
  */

/** @addtogroup DSI_CmdMode_SingleBuffer
  * @{
  */

/* Physical frame buffer for background and foreground layers */
#if defined (DISPLAY_DRIVER_RM69032)
#if defined ( __ICCARM__ )  /* IAR Compiler */
  #pragma data_alignment = 16
uint32_t              PhysFrameBuffer[76800];
#elif defined (__GNUC__)    /* GNU Compiler */
uint32_t              PhysFrameBuffer[76800] __attribute__ ((aligned (16)));
#else                       /* ARM Compiler */
__align(16) uint32_t  PhysFrameBuffer[76800];
#endif
#else
/* 390*390 pixels with 24bpp - 20% (with GFXMMU) */
#if defined ( __ICCARM__ )  /* IAR Compiler */
  #pragma data_alignment = 16
uint32_t              PhysFrameBuffer[91260];
#elif defined (__GNUC__)    /* GNU Compiler */
uint32_t              PhysFrameBuffer[91260] __attribute__ ((aligned (16)));
#else                       /* ARM Compiler */
__align(16) uint32_t  PhysFrameBuffer[91260];
#endif
#endif

/* Peripheral handles */
DMA2D_HandleTypeDef  Dma2dHandle;
DSI_HandleTypeDef    DsiHandle;
LTDC_HandleTypeDef   LtdcHandle;
UART_HandleTypeDef   Usart2Handle;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/*
 * VIDEO timing settings has been adjusted
 * LTDC settings are calculated from the following numbers.
 * DO NOT FORGET to recalculate/update LTDC settings if you change these.
 */
#define VSYNC               8
#define VBP                 10
#define VFP                 36
#define VACT                320
#define HSYNC               8
#define HBP                 40
#define HFP                 40
#define HACT                320

#define LAYER_ADDRESS       GFXMMU_VIRTUAL_BUFFER0_BASE

#define BRIGHTNESS_MIN      100
#define BRIGHTNESS_NORMAL   200

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
static __IO int32_t pending_buffer = -1;

/* Private function prototypes -----------------------------------------------*/
static void SystemClock_Config(void);
static void Error_Handler(void);
static uint8_t LCD_Config(void);
static void LCD_PowerOff(void);
static void LCD_PowerOn(void);

/*
 * USART2 is routed to STLINK VCP, you can use it to output debug information.
 */
static void MX_USART2_UART_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK) {
	Error_Handler();
  }

  /* Peripheral clock enable */
  __HAL_RCC_USART2_CLK_ENABLE();

  __HAL_RCC_GPIOA_CLK_ENABLE();
  /**USART2 GPIO Configuration
  PA3     ------> USART2_RX
  PA2     ------> USART2_TX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  Usart2Handle.Instance = USART2;
  Usart2Handle.Init.BaudRate = 115200;
  Usart2Handle.Init.WordLength = UART_WORDLENGTH_8B;
  Usart2Handle.Init.StopBits = UART_STOPBITS_1;
  Usart2Handle.Init.Parity = UART_PARITY_NONE;
  Usart2Handle.Init.Mode = UART_MODE_TX_RX;
  Usart2Handle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  Usart2Handle.Init.OverSampling = UART_OVERSAMPLING_16;
  Usart2Handle.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  Usart2Handle.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  Usart2Handle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&Usart2Handle) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&Usart2Handle, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&Usart2Handle, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&Usart2Handle) != HAL_OK)
  {
    Error_Handler();
  }
}

size_t DebugPrintf(const char * format, ...) {
  char buffer[1024];
  va_list args;
  va_start(args, format);
  vsprintf(buffer, format, args);
  size_t len = strlen(buffer);
  HAL_UART_Transmit(&Usart2Handle, (const uint8_t *)buffer, len, 1000);
  va_end(args);
  return len;
}

/**
  * @brief  Copy an input RGB888 buffer to output RGB888 with output offset
  * @param  pSrc: Pointer to source buffer
  * @param  pDst: Output color
  * @param  x: Start x position
  * @param  y: Start y position
  * @param  xsize: width
  * @param  ysize: height
  * @param  ColorMode: Input color mode
  * @retval None
  */
static void WriteFrameBuffer(uint32_t *pSrc, uint32_t *pDst, uint16_t x, uint16_t y, uint16_t xsize, uint16_t ysize)
{
  uint32_t destination = (uint32_t)pDst + (y * 320 + x) * 4;
  uint32_t source      = (uint32_t)pSrc;

  if (HAL_DMA2D_Start(&Dma2dHandle, source, destination, xsize, ysize) == HAL_OK) {
    /* Polling For DMA transfer */
    HAL_DMA2D_PollForTransfer(&Dma2dHandle, 100);
  }
}

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Error Handler.
  * @retval None
  */
static void Error_Handler(void)
{
  BSP_LED_On(LED1);
  while(1) { ; } /* Blocking on error */
}

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  //uint32_t  brightness = BRIGHTNESS_NORMAL;

  /* STM32L4xx HAL library initialization:
       - Configure the Flash prefetch, Instruction cache, Data cache
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
     */
  HAL_Init();

  /* Configure the System clock to have a frequency of 120 MHz */
  SystemClock_Config();

  MX_USART2_UART_Init();
  DebugPrintf("A MIPI DSI example for RM69032!\r\n");

  __HAL_RCC_GPIOA_CLK_ENABLE();

  GPIO_InitTypeDef  GPIO_InitStructure;
  GPIO_InitStructure.Pin   = GPIO_PIN_8;
  GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull  = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;

  /*
   * Turn off TPS65631 output so the power supply to ELVDD/ELVSS will be off
   */
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /* Initialize used Leds */
  BSP_LED_Init(LED1);
  BSP_LED_Init(LED2);

  /* Initialize the LCD   */
  if(LCD_Config() != LCD_OK)
  {
    Error_Handler();
  }

  /* Infinite loop */
  uint8_t flag = 0;
  while(1)
  {
    BSP_LED_Toggle(LED2);
    if (flag == 0) {
      WriteFrameBuffer((uint32_t *)color_square_320x320_rgb888, PhysFrameBuffer, 0, 0, 320, 320);
      flag = 1;
    }
    else {
      WriteFrameBuffer((uint32_t *)bw_square_320x320_rgb888, PhysFrameBuffer, 0, 0, 320, 320);
      flag = 0;
    }
    pending_buffer = 1;

    /*Refresh the LCD display*/
#if 0
    uint8_t columnAddr[4]= {0x00, 0x00, 0x01, 0x3F};
    uint8_t pageAddr[4]= {0x00, 0x00, 0x01, 0x3F};
    HAL_DSI_LongWrite(&DsiHandle, 0, DSI_DCS_LONG_PKT_WRITE, 4, DSI_SET_COLUMN_ADDRESS, columnAddr);
    HAL_DSI_LongWrite(&DsiHandle, 0, DSI_DCS_LONG_PKT_WRITE, 4, DSI_SET_PAGE_ADDRESS, pageAddr);
#endif
    HAL_DSI_Refresh(&DsiHandle);

    while (pending_buffer == 1);

    //BSP_LED_On(LED2);

    HAL_Delay(2000);
#if 0
    brightness = BRIGHTNESS_NORMAL;
    HAL_DSI_ShortWrite(&DsiHandle, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x51, brightness);
    while(brightness > BRIGHTNESS_MIN) {
      brightness -= 1;
      HAL_DSI_ShortWrite(&DsiHandle, 0, DSI_DCS_SHORT_PKT_WRITE_P1, 0x51, brightness);
      HAL_Delay(100);
    }
#endif
  }
}

/**
  * @brief  End of Refresh DSI callback.
  * @param  hdsi: pointer to a DSI_HandleTypeDef structure that contains
  *               the configuration information for the DSI.
  * @retval None
  */
void HAL_DSI_EndOfRefreshCallback(DSI_HandleTypeDef *hdsi)
{
  if(pending_buffer > 0)
  {
    pending_buffer = 0;
  }
}

/** @defgroup Private_Functions Private Functions
  * @{
  */

static uint8_t DisplayController_Config(void) {
/*************************/
/* LCD POWER ON SEQUENCE */
/*************************/
	#define INIT_OP_ELVSS_ON 0xFFFFFFFE
	#define INIT_OP_DELAY 0xFFFFFFFF

	  typedef struct {
	    uint32_t hdr_type;
	    uint32_t delay_time;
	    uint8_t cmd;
	    uint8_t payload_size;
	    uint8_t payload[10];
	  } INIT_OP;

	  const INIT_OP rm69032_init_ops[] = {
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xF0, .payload_size = 5, .payload = {0x55, 0xAA, 0x52, 0x08, 0x00}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xBD, .payload_size = 5, .payload = {0x03, 0x20, 0x14, 0x4B, 0x00}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xBE, .payload_size = 5, .payload = {0x03, 0x20, 0x14, 0x4B, 0x01}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xBF, .payload_size = 5, .payload = {0x03, 0x20, 0x14, 0x4B, 0x00}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xBB, .payload_size = 3, .payload = {0x07, 0x07, 0x07}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xC7, .payload_size = 1, .payload = {0x40}},

	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xF0, .payload_size = 5, .payload = {0x55, 0xAA, 0x52, 0x08, 0x02}},
	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P1,	.cmd = 0xEB, .payload_size = 1, .payload = {0x02}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xFE, .payload_size = 2, .payload = {0x08, 0x50}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xC3, .payload_size = 3, .payload = {0xF2, 0x95, 0x04}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xE9, .payload_size = 3, .payload = {0x00, 0x36, 0x38}},
	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P1,	.cmd = 0xCA, .payload_size = 1, .payload = {0x04}},

	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xF0, .payload_size = 5, .payload = {0x55, 0xAA, 0x52, 0x08, 0x01}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB0, .payload_size = 3, .payload = {0x03, 0x03, 0x03}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB1, .payload_size = 3, .payload = {0x05, 0x05, 0x05}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB2, .payload_size = 3, .payload = {0x01, 0x01, 0x01}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB4, .payload_size = 3, .payload = {0x07, 0x07, 0x07}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB5, .payload_size = 3, .payload = {0x03, 0x03, 0x03}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB6, .payload_size = 3, .payload = {0x55, 0x55, 0x55}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB7, .payload_size = 3, .payload = {0x36, 0x36, 0x36}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB8, .payload_size = 3, .payload = {0x23, 0x23, 0x23}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xB9, .payload_size = 3, .payload = {0x03, 0x03, 0x03}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xBA, .payload_size = 3, .payload = {0x03, 0x03, 0x03}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xBE, .payload_size = 3, .payload = {0x32, 0x30, 0x70}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xCF, .payload_size = 7, .payload = {0xFF, 0xD4, 0x95, 0xE8, 0x4F, 0x00, 0x04}},
	    // SET_TEAR_ON ?
	    // {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P1,	.cmd = 0x35, .payload_size = 1, .payload = {0x00}},
	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P0, .cmd = 0x34, .payload_size = 0},
	    // SET_ADDRESS_MODE ?
	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P1,	.cmd = 0x36, .payload_size = 1, .payload = {0x00}},
	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P1,	.cmd = 0xC0, .payload_size = 1, .payload = {0x20}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xC2, .payload_size = 6, .payload = {0x17, 0x17, 0x17, 0x17, 0x17, 0x0B}},
	    // {.hdr_type = 0x32},

	    // {.hdr_type = INIT_OP_DELAY, .delay_time = 100},

        // {.hdr_type = DSI_DCS_LONG_PKT_WRITE,    .cmd = 0x51, .payload_size = 1, .payload = {0x00}},

#if 0
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xF0, .payload_size = 5, .payload = {0x55, 0xAA, 0x52, 0x08, 0x02}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xED, .payload_size = 8, .payload = {0x48, 0x00, 0xFF, 0x13, 0x08, 0x30, 0x0C, 0x00}},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 20},
#endif

	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P0,	.cmd = 0x11, .payload_size = 0},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 300},

#if 0
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xF0, .payload_size = 5, .payload = {0x55, 0xAA, 0x52, 0x08, 0x02}},
	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xED, .payload_size = 8, .payload = {0x48, 0x00, 0xFE, 0x13, 0x08, 0x30, 0x0C, 0x00}},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 20},

	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xED, .payload_size = 8, .payload = {0x48, 0x00, 0xE6, 0x13, 0x08, 0x30, 0x0C, 0x00}},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 20},

	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xED, .payload_size = 8, .payload = {0x48, 0x00, 0xE2, 0x13, 0x08, 0x30, 0x0C, 0x00}},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 20},

	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xED, .payload_size = 8, .payload = {0x48, 0x00, 0xE0, 0x13, 0x08, 0x30, 0x0C, 0x00}},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 20},

	    {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xED, .payload_size = 8, .payload = {0x48, 0x00, 0xE0, 0x13, 0x08, 0x00, 0x0C, 0x00}},
#endif

	    {.hdr_type = DSI_DCS_SHORT_PKT_WRITE_P0,	.cmd = 0x29, .payload_size = 0},

	    // {.hdr_type = DSI_DCS_LONG_PKT_WRITE,		.cmd = 0xF0, .payload_size = 5, .payload = {0x55, 0xAA, 0x52, 0x08, 0x00}},

	    {.hdr_type = INIT_OP_DELAY, .delay_time = 10},

	    {.hdr_type = INIT_OP_ELVSS_ON}
	  };

	  HAL_StatusTypeDef rc;
	  for (int i = 0; i < sizeof(rm69032_init_ops) / sizeof(INIT_OP); i ++) {
	    const INIT_OP *op = &rm69032_init_ops[i];
	    switch (op->hdr_type) {
	      case DSI_DCS_SHORT_PKT_WRITE_P0:
	        //do {
	          rc = HAL_DSI_ShortWrite(&DsiHandle, 0, op->hdr_type, op->cmd, 0);
	          if (HAL_OK != rc) {
	            DebugPrintf("[%d]DSI_DCS_SHORT_PKT_WRITE_P0 returned[%d] DSIErrCode[0x%x] cmd[0x%x]\r\n", i, rc, HAL_DSI_GetError(&DsiHandle), op->cmd);
	          }
	        //} while(rc != HAL_OK);
	        break;
	      case DSI_DCS_SHORT_PKT_WRITE_P1:
	        //do {
	          rc = HAL_DSI_ShortWrite(&DsiHandle, 0, op->hdr_type, op->cmd, op->payload[0]);
	          if (HAL_OK != rc) {
	          	DebugPrintf("[%d]DSI_DCS_SHORT_PKT_WRITE_P1 returned[%d] DSIErrCode[0x%x] cmd[0x%x] p1[0x%x]\r\n", i, rc, HAL_DSI_GetError(&DsiHandle), op->cmd, op->payload[0]);
	          }
	        //} while (rc != HAL_OK);
	        break;
	      case DSI_DCS_LONG_PKT_WRITE:
	        //do {
	          rc = HAL_DSI_LongWrite(&DsiHandle, 0, op->hdr_type, op->payload_size, op->cmd, (uint8_t *)op->payload);
	          if (HAL_OK != rc) {
	            DebugPrintf("[%d]DSI_DCS_LONG_PKT_WRITE returned[%d] DSIErrCode[0x%x] cmd[0x%x] size[%d]\r\n", i, rc, HAL_DSI_GetError(&DsiHandle), op->cmd, op->payload_size);
	          }
	        //} while (rc != HAL_OK);
	        break;
	      case INIT_OP_ELVSS_ON:
	        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	        break;
	      case INIT_OP_DELAY:
	        HAL_Delay(op->delay_time);
	        break;
	      default:
	        break;
	    }
	  }
	  return LCD_OK;
}
/**
  * @brief  Initializes the DSI LCD.
  * The initialization is done as below:
  *     - Power recycle the display controller
  *     - DMA2D initialization
  *     - DSI PLL initialization
  *     - DSI initialization
  *     - LTDC initialization
  *     - RM69032 LCD Display IC Driver initialization
  * @param  None
  * @retval LCD state
  */
static uint8_t LCD_Config(void)
{
  LTDC_LayerCfgTypeDef    LayerCfg;
  DSI_PLLInitTypeDef      dsiPllInit;
  DSI_PHY_TimerTypeDef    PhyTimings;
  DSI_HOST_TimeoutTypeDef HostTimeouts;
  DSI_LPCmdTypeDef        LPCmd;
  DSI_CmdCfgTypeDef       CmdCfg;

  /* Power reset LCD */
  LCD_PowerOff();
  LCD_PowerOn();

  Dma2dHandle.Instance          = DMA2D;

  /*##-1- Configure the DMA2D Mode, Color Mode and output offset #############*/
  Dma2dHandle.Init.Mode           = DMA2D_M2M_PFC;
  Dma2dHandle.Init.ColorMode      = DMA2D_OUTPUT_RGB888;
  Dma2dHandle.Init.OutputOffset   = 0;
  Dma2dHandle.Init.AlphaInverted  = DMA2D_REGULAR_ALPHA;  /* No Output Alpha Inversion */
  Dma2dHandle.Init.RedBlueSwap    = DMA2D_RB_REGULAR;     /* No Output Red & Blue swap */
  Dma2dHandle.Init.BytesSwap      = DMA2D_BYTES_REGULAR;  /* Regular output byte order */
  Dma2dHandle.Init.LineOffsetMode = DMA2D_LOM_PIXELS;     /* Pixel mode                */

  /*##-2- Foreground Configuration ###########################################*/
  Dma2dHandle.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB888;
  Dma2dHandle.LayerCfg[1].InputOffset    = 0;
  Dma2dHandle.LayerCfg[1].AlphaMode      = DMA2D_NO_MODIF_ALPHA;
  Dma2dHandle.LayerCfg[1].InputAlpha     = 0xFF;                /* Not used */
  Dma2dHandle.LayerCfg[1].RedBlueSwap    = DMA2D_RB_SWAP; //DMA2D_RB_REGULAR;    /* No ForeGround Red/Blue swap */
  Dma2dHandle.LayerCfg[1].AlphaInverted  = DMA2D_REGULAR_ALPHA; /* No ForeGround Alpha inversion */

  if(HAL_DMA2D_Init(&Dma2dHandle) != HAL_OK) {
    return LCD_ERROR;
  }
  if(HAL_DMA2D_ConfigLayer(&Dma2dHandle, 1) != HAL_OK) {
    return LCD_ERROR;
  }

  /**********************/
  /* LTDC CONFIGURATION */
  /**********************/

  /* LTDC initialization */
  __HAL_LTDC_RESET_HANDLE_STATE(&LtdcHandle);
  LtdcHandle.Instance = LTDC;
  LtdcHandle.Init.HSPolarity         = LTDC_HSPOLARITY_AL;
  LtdcHandle.Init.VSPolarity         = LTDC_VSPOLARITY_AL;
  LtdcHandle.Init.DEPolarity         = LTDC_DEPOLARITY_AL;
  LtdcHandle.Init.PCPolarity         = LTDC_PCPOLARITY_IPC;
  /*
   * LTDC video timing settings calculated against HSYNC/VSYNC/HBP/HFP/VBP/VFP
   */
  LtdcHandle.Init.HorizontalSync     = 7;   /* HSYNC width - 1 */
  LtdcHandle.Init.VerticalSync       = 7;   /* VSYNC width - 1 */
  LtdcHandle.Init.AccumulatedHBP     = 47;   /* HSYNC width + HBP - 1 */
  LtdcHandle.Init.AccumulatedVBP     = 17;   /* VSYNC width + VBP - 1 */
  LtdcHandle.Init.AccumulatedActiveW = 367; /* HSYNC width + HBP + Active width - 1 */
  LtdcHandle.Init.AccumulatedActiveH = 337; /* VSYNC width + VBP + Active height - 1 */
  LtdcHandle.Init.TotalWidth         = 407; /* HSYNC width + HBP + Active width + HFP - 1 */
  LtdcHandle.Init.TotalHeigh         = 373; /* VSYNC width + VBP + Active height + VFP - 1 */
  LtdcHandle.Init.Backcolor.Red      = 0;   /* Not used default value */
  LtdcHandle.Init.Backcolor.Green    = 0;   /* Not used default value */
  LtdcHandle.Init.Backcolor.Blue     = 0;   /* Not used default value */
  LtdcHandle.Init.Backcolor.Reserved = 0xFF;
  if(HAL_LTDC_Init(&LtdcHandle) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  /* LTDC layer 1 configuration */
  LayerCfg.WindowX0        = 0;
  LayerCfg.WindowX1        = 320;
  LayerCfg.WindowY0        = 0;
  LayerCfg.WindowY1        = 320;
  LayerCfg.PixelFormat     = LTDC_PIXEL_FORMAT_RGB888;
  LayerCfg.Alpha           = 0xFF; /* NU default value */
  LayerCfg.Alpha0          = 0; /* NU default value */
  LayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_PAxCA; /* Not Used: default value */
  LayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_PAxCA; /* Not Used: default value */
  LayerCfg.FBStartAdress   = (uint32_t)PhysFrameBuffer;
  LayerCfg.ImageWidth      = 320;
  LayerCfg.ImageHeight     = 320;
  LayerCfg.Backcolor.Red   = 0; /* Not Used: default value */
  LayerCfg.Backcolor.Green = 0; /* Not Used: default value */
  LayerCfg.Backcolor.Blue  = 0; /* Not Used: default value */
  LayerCfg.Backcolor.Reserved = 0xFF;
  if(HAL_LTDC_ConfigLayer(&LtdcHandle, &LayerCfg, LTDC_LAYER_1) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  /*********************/
  /* DSI CONFIGURATION */
  /*********************/

  /* DSI initialization */
  /* DSI data lane 500Mbps
   * TX escape clock 15.625Mhz
   */
  __HAL_DSI_RESET_HANDLE_STATE(&DsiHandle);
  DsiHandle.Instance = DSI;
  DsiHandle.Init.AutomaticClockLaneControl = DSI_AUTO_CLK_LANE_CTRL_DISABLE;
  DsiHandle.Init.TXEscapeCkdiv             = 4;
  DsiHandle.Init.NumberOfLanes             = DSI_ONE_DATA_LANE;
  dsiPllInit.PLLNDIV = 125;
  dsiPllInit.PLLIDF  = DSI_PLL_IN_DIV4;
  dsiPllInit.PLLODF  = DSI_PLL_OUT_DIV1;
  if(HAL_DSI_Init(&DsiHandle, &dsiPllInit) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  HostTimeouts.TimeoutCkdiv                 = 1;
  HostTimeouts.HighSpeedTransmissionTimeout = 0;
  HostTimeouts.LowPowerReceptionTimeout     = 0;
  HostTimeouts.HighSpeedReadTimeout         = 0;
  HostTimeouts.LowPowerReadTimeout          = 0;
  HostTimeouts.HighSpeedWriteTimeout        = 0;
  HostTimeouts.HighSpeedWritePrespMode      = 0;
  HostTimeouts.LowPowerWriteTimeout         = 0;
  HostTimeouts.BTATimeout                   = 0;
  if(HAL_DSI_ConfigHostTimeouts(&DsiHandle, &HostTimeouts) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  PhyTimings.ClockLaneHS2LPTime  = 33; /* Tclk-post + Tclk-trail + Ths-exit = [(60ns + 52xUI) + (60ns) + (300ns)]/16ns */
  PhyTimings.ClockLaneLP2HSTime  = 30; /* Tlpx + (Tclk-prepare + Tclk-zero) + Tclk-pre = [150ns + 300ns + 8xUI]/16ns */
  PhyTimings.DataLaneHS2LPTime   = 11; /* Ths-trail + Ths-exit = [(60ns + 4xUI) + 100ns]/16ns */
  PhyTimings.DataLaneLP2HSTime   = 21; /* Tlpx + (Ths-prepare + Ths-zero) + Ths-sync = [150ns + (145ns + 10xUI) + 8xUI]/16ns */
  PhyTimings.DataLaneMaxReadTime = 0;
  PhyTimings.StopWaitTime        = 7;
  if(HAL_DSI_ConfigPhyTimer(&DsiHandle, &PhyTimings) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  if(HAL_DSI_ConfigFlowControl(&DsiHandle, DSI_FLOW_CONTROL_BTA) != HAL_OK)
  {
    return(LCD_ERROR);
  }
#if 0
  if (HAL_DSI_SetLowPowerRXFilter(&DsiHandle, 10000) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DSI_ConfigErrorMonitor(&DsiHandle, HAL_DSI_ERROR_NONE) != HAL_OK)
  {
    Error_Handler();
  }
#endif

  /*
   * RM69032 has to be initialized in LP mode
   * Enable all LP mode commands
   */
  LPCmd.LPGenShortWriteNoP  = DSI_LP_GSW0P_ENABLE;
  LPCmd.LPGenShortWriteOneP = DSI_LP_GSW1P_ENABLE;
  LPCmd.LPGenShortWriteTwoP = DSI_LP_GSW2P_ENABLE;
  LPCmd.LPGenShortReadNoP   = DSI_LP_GSR0P_ENABLE;
  LPCmd.LPGenShortReadOneP  = DSI_LP_GSR1P_ENABLE;
  LPCmd.LPGenShortReadTwoP  = DSI_LP_GSR2P_ENABLE;
  LPCmd.LPGenLongWrite      = DSI_LP_GLW_ENABLE;
  LPCmd.LPDcsShortWriteNoP  = DSI_LP_DSW0P_ENABLE;
  LPCmd.LPDcsShortWriteOneP = DSI_LP_DSW1P_ENABLE;
  LPCmd.LPDcsShortReadNoP   = DSI_LP_DSR0P_ENABLE;
  LPCmd.LPDcsLongWrite      = DSI_LP_DLW_ENABLE;
  LPCmd.LPMaxReadPacket     = DSI_LP_MRDP_DISABLE;
  LPCmd.AcknowledgeRequest  = DSI_ACKNOWLEDGE_DISABLE;
  if(HAL_DSI_ConfigCommand(&DsiHandle, &LPCmd) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  CmdCfg.VirtualChannelID      = 0;
  CmdCfg.ColorCoding           = DSI_RGB888;
  CmdCfg.CommandSize           = 320;
  CmdCfg.TearingEffectSource   = DSI_TE_DSILINK;
  CmdCfg.TearingEffectPolarity = DSI_TE_FALLING_EDGE;
  CmdCfg.HSPolarity            = DSI_HSYNC_ACTIVE_LOW;
  CmdCfg.VSPolarity            = DSI_VSYNC_ACTIVE_LOW;
  CmdCfg.DEPolarity            = DSI_DATA_ENABLE_ACTIVE_HIGH;
  CmdCfg.VSyncPol              = DSI_VSYNC_FALLING;
  CmdCfg.AutomaticRefresh      = DSI_AR_DISABLE;
  CmdCfg.TEAcknowledgeRequest  = DSI_TE_ACKNOWLEDGE_ENABLE;
  if(HAL_DSI_ConfigAdaptedCommandMode(&DsiHandle, &CmdCfg) != HAL_OK)
  {
    return(LCD_ERROR);
  }

  if (HAL_DSI_SetGenericVCID(&DsiHandle, 0) != HAL_OK)
  {
    Error_Handler();
  }

  /* Disable the Tearing Effect interrupt activated by default on previous function */
  __HAL_DSI_DISABLE_IT(&DsiHandle, DSI_IT_TE);

  /* Enable DSI */
  __HAL_DSI_ENABLE(&DsiHandle);

  //HAL_DSI_EnterULPMData(&DsiHandle);
  if (LCD_OK != DisplayController_Config()) {
    return LCD_ERROR;
  }

  //HAL_DSI_ExitULPMData(&DsiHandle);

  /* Enable DSI Wrapper */
  __HAL_DSI_WRAPPER_ENABLE(&DsiHandle);

  return LCD_OK;
}

/**
  * @brief  LCD power off
  *         Power off LCD.
  */
static void LCD_PowerOff(void)
{
  BSP_IO_Init();

  /* Configure the GPIO connected to DSI_RESET signal */
  BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);
  /* Activate DSI_RESET */
  BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

  /* Wait at least 5 ms */
  HAL_Delay(5);

  /* Set DSI_POWER_ON to analog mode only if psram is not currently used */
  /* Disable first DSI_1V8_PWRON then DSI_3V3_PWRON */
  BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_ANALOG);
  BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_ANALOG);
}

/**
  * @brief  LCD power on
  *         Power on LCD.
  */
static void LCD_PowerOn(void)
{
  BSP_IO_Init();

  /* Configure the GPIO connected to DSI_RESET signal */
  BSP_IO_ConfigPin(IO_PIN_10, IO_MODE_OUTPUT);

  /* Activate DSI_RESET */
  BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_RESET);

  /* Configure the GPIO connected to DSI_3V3_POWERON signal as output low */
  /* to activate 3V3_LCD. VDD_LCD is also activated if VDD = 3,3V */
  BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_SET);
  BSP_IO_ConfigPin(IO_PIN_8, IO_MODE_OUTPUT);
  BSP_IO_WritePin(IO_PIN_8, GPIO_PIN_RESET);

  /* Wait at least 1ms before enabling 1V8_LCD */
  HAL_Delay(10);

  /* Configure the GPIO connected to DSI_1V8_POWERON signal as output low */
  /* to activate 1V8_LCD. VDD_LCD is also activated if VDD = 1,8V */
  BSP_IO_WritePin(AGPIO_PIN_2, GPIO_PIN_SET);
  BSP_IO_ConfigPin(AGPIO_PIN_2, IO_MODE_OUTPUT);
  BSP_IO_WritePin(AGPIO_PIN_2, GPIO_PIN_RESET);

  /* Wait at least 15 ms (minimum reset low width is 10ms and add margin for 1V8_LCD ramp-up) */
  HAL_Delay(15);

  /* Deactivate DSI_RESET */
  BSP_IO_WritePin(IO_PIN_10, GPIO_PIN_SET);

  /* Wait reset complete time (maximum time is 5ms when LCD in sleep mode and 120ms when LCD is not in sleep mode) */
  HAL_Delay(120);
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI)
  *            SYSCLK(Hz)                     = 120000000
  *            HCLK(Hz)                       = 120000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 4000000
  *            PLL_M                          = 1
  *            PLL_N                          = 60
  *            PLL_Q                          = 2
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            Flash Latency(WS)              = 5
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};

  /* Enable voltage range 1 boost mode for frequency above 80 Mhz */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  __HAL_RCC_PWR_CLK_DISABLE();

  /* Enable MSI Oscillator and activate PLL with MSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 60;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLP = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* To avoid undershoot due to maximum frequency, select PLL as system clock source */
  /* with AHB prescaler divider 2 as first step */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* AHB prescaler divider at 1 as second step */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

