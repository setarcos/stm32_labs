/**
  ******************************************************************************
  * @file    USB_Host/MSC_Standalone/Src/main.c
  * @author  MCD Application Team
  * @brief   USB host Mass storage demo main file
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

/* Includes ----------------------------------------------------------------- */
#include "main.h"

/* Private typedef ---------------------------------------------------------- */
/* Private define ----------------------------------------------------------- */
/* Private macro ------------------------------------------------------------ */
/* Private variables -------------------------------------------------------- */
UART_HandleTypeDef huart3;
USBH_HandleTypeDef hUSBHost;
MSC_ApplicationTypeDef Appli_state = APPLICATION_IDLE;
FATFS USBH_fatfs;
char USBDISKPath[4];            /* USB Host logical drive path */
int last_tick = 0;
FIL MyFile;
const char* name = "stm32.txt";
const uint8_t wtext[] = "Hello world!\n";
char uart_buf[100];
int uart_len;
int fillbuf;
//extern const Diskio_drvTypeDef  USBH_Driver;
/* Private function prototypes ---------------------------------------------- */
static void SystemClock_Config(void);
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id);
static void MSC_InitApplication(void);
static void Error_Handler(void);
static void ExplreDir(int view);
/* Private functions -------------------------------------------------------- */

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  /* STM32F469xx HAL library initialization  */
  HAL_Init();

  /* Configure the System clock to have a frequency of 180 MHz */
  SystemClock_Config();

  /* Init MSC Application */
  MSC_InitApplication();

  /* Init Host Library */
  USBH_Init(&hUSBHost, USBH_UserProcess, 0);

  /* Add Supported Class */
  USBH_RegisterClass(&hUSBHost, USBH_MSC_CLASS);

  /* Start Host Process */
  USBH_Start(&hUSBHost);

  /* Run Application (Blocking mode) */
  while (1)
  {
    /* USB Host Background task */
    USBH_Process(&hUSBHost);

    if (Appli_state != APPLICATION_READY) continue;

    /* Got key pressed */
    if ((HAL_GetTick() - last_tick > 200) && (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == GPIO_PIN_SET)) {
      last_tick = HAL_GetTick();
      /*Create a file*/
      if (f_open(&MyFile, name, FA_OPEN_ALWAYS | FA_WRITE) != FR_OK ) {
        /* Creation failed */
        HAL_UART_Transmit(&huart3, (uint8_t *)"E: Open!\r\n", 10, 1000);
      } else {
        UINT bytesWritten;
        HAL_UART_Transmit(&huart3, (uint8_t *)"I: Write!\r\n", 11, 1000);
        f_lseek(&MyFile, f_size(&MyFile));
        /*write message to the file. Use variable wtext, bytesWritten*/
        f_write(&MyFile, wtext, sizeof(wtext) - 1, &bytesWritten);
        /*close the file*/
        f_close(&MyFile);
      }
    }

    if ((USART3->SR & USART_SR_RXNE) != 0) { /* Got command */
      char c = USART3->DR;
      static int idx;
      if (fillbuf == 1) {
        USART3->DR = c;
        uart_buf[idx++] = c;
        if (c == '\n') {
          uart_buf[idx - 1] = 0;
          if (f_open(&MyFile, uart_buf, FA_OPEN_APPEND | FA_WRITE) != FR_OK ) {
            HAL_UART_Transmit(&huart3, (uint8_t *)"E: Open!\r\n", 10, 1000);
            fillbuf = 0;
          } else
            fillbuf = 2;
          idx = 0;
        }
        continue;
      }
      if (fillbuf == 2) {
        USART3->DR = c;
        uart_buf[idx++] = c;
        if (c == '\n') {
          if (idx == 1) {
            f_close(&MyFile);
            idx = 0;
            fillbuf = 0;
          } else {
            UINT bytesWritten;
            f_write(&MyFile, uart_buf, idx, &bytesWritten);
            idx = 0;
          }
        }
        continue;
      }
      if (c == 'l') { /* List files */
        ExplreDir(0);
      }
      if (c >= '0' && c <= '9') { /* View file */
        ExplreDir(c - '0');
      }
      if (c == 'w') {
        fillbuf = 1;
        HAL_UART_Transmit(&huart3, (uint8_t *)"File Name: ", 11, 1000);
        idx = 0;
      }
    }
  }
}

void ExplreDir(int view)
{
  if (view == 0) HAL_UART_Transmit(&huart3, (uint8_t *)"File List:\r\n", 12, 1000);
  DIR dir;
  FRESULT res = FR_OK;
  FILINFO fno;
  res = f_opendir(&dir, USBDISKPath);
  if (res != FR_OK) {
    HAL_UART_Transmit(&huart3, (uint8_t *)"E: OpDIR!\r\n", 11, 1000);
    return;
  }
  int idx = 0;
  while (USBH_MSC_IsReady(&hUSBHost)) {
    idx++;
    if (idx > 9) break;
    res = f_readdir(&dir, &fno);
    if (res != FR_OK || fno.fname[0] == 0) break;
    if (fno.fname[0] == '.') continue;
    if (view == 0) {
      uart_len = sprintf(uart_buf, "%d: %s\r\n", idx, fno.fname);
      HAL_UART_Transmit(&huart3, (uint8_t*)uart_buf, uart_len, 1000);
    }
    if (view == idx) {
      if (f_open(&MyFile, fno.fname, FA_READ) == FR_OK ) {
        UINT br;
        while (1) {
          f_read(&MyFile, uart_buf, 10, &br);
          HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, br, 1000);
          if (br < 10) break;
        }
        f_close(&MyFile);
      }
      break;
    }
  }
  f_closedir(&dir);
}

/**
  * @brief  MSC application Init.
  * @param  None
  * @retval None
  */
static void MSC_InitApplication(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_USART3_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /**USART3 GPIO Configuration
  PB10     ------> USART3_TX
  PB11     ------> USART3_RX
  */
  GPIO_InitStruct.Pin = GPIO_PIN_10 | GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct); // Wake up key

  huart3.Instance        = USART3;

  huart3.Init.BaudRate     = 115200;
  huart3.Init.WordLength   = UART_WORDLENGTH_8B;
  huart3.Init.StopBits     = UART_STOPBITS_1;
  huart3.Init.Parity       = UART_PARITY_NONE;
  huart3.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  huart3.Init.Mode         = UART_MODE_TX_RX;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if(HAL_UART_Init(&huart3) != HAL_OK) {
    Error_Handler();
  }
}

/**
  * @brief  User Process
  * @param  phost: Host Handle
  * @param  id: Host Library user message ID
  * @retval None
  */
static void USBH_UserProcess(USBH_HandleTypeDef *phost, uint8_t id)
{
  switch(id) {
  case HOST_USER_SELECT_CONFIGURATION:
    break;

  case HOST_USER_DISCONNECTION:
    Appli_state = APPLICATION_DISCONNECT;
    if (f_mount(NULL, "", 0) != FR_OK) {
      HAL_UART_Transmit(&huart3, (uint8_t *)"E: DeInit!\r\n", 12, 1000);
    }
    FATFS_UnLinkDriver(USBDISKPath);
    break;

  case HOST_USER_CLASS_ACTIVE:
    Appli_state = APPLICATION_READY;
    HAL_UART_Transmit(&huart3, (uint8_t *)"I: Ready!\r\n", 11, 1000);
    if (f_mount(&USBH_fatfs, "", 0) != FR_OK)
      HAL_UART_Transmit(&huart3, (uint8_t *)"E: Mount!\r\n", 11, 1000);
    break;

  case HOST_USER_CONNECTION:
    Appli_state = APPLICATION_START;
    /* FATFS_LinkDriver will be called with MX_FATFS_Init if using STM32CubeIDE */
    FATFS_LinkDriver(&USBH_Driver, USBDISKPath);
    uart_len = sprintf(uart_buf, "I: path=%s\r\n", USBDISKPath);
    HAL_UART_Transmit(&huart3, (uint8_t *)uart_buf, uart_len, 1000);
    break;

  default:
    break;
  }
}

/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSE)
  *            SYSCLK(Hz)                     = 180000000
  *            HCLK(Hz)                       = 180000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 4
  *            APB2 Prescaler                 = 2
  *            HSE Frequency(Hz)              = 8000000
  *            PLL_M                          = 8
  *            PLL_N                          = 360
  *            PLL_P                          = 2
  *            PLL_Q                          = 7
  *            PLL_R                          = 2
  *            VDD(V)                         = 3.3
  *            Main regulator output voltage  = Scale1 mode
  *            Flash Latency(WS)              = 5
  *         The USB clock configuration from PLLSAI:
  *            PLLSAIM                        = 8
  *            PLLSAIN                        = 384
  *            PLLSAIP                        = 8
  * @param  None
  * @retval None
  */
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
#if defined(USE_STM32469I_DISCO_REVA)
  RCC_OscInitStruct.PLL.PLLM = 25;
#else
  RCC_OscInitStruct.PLL.PLLM = 8;
#endif /* USE_STM32469I_DISCO_REVA */
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Enable the OverDrive to reach the 180 Mhz Frequency */
  if(HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLLSAI output as USB clock source */
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 7;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 384;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV8;
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CK48;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CK48CLKSOURCE_PLLSAIP;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
static void Error_Handler(void)
{
  /* User may add here some code to deal with this error */
  while(1)
  {
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
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif
