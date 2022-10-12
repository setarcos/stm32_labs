/**
  ******************************************************************************
  * @file    USB_Host/MSC_Standalone/Inc/main.h
  * @author  MCD Application Team
  * @brief   Header for main.c module
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
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "stdio.h"
#include "usbh_core.h"
#include "usbh_msc.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "usbh_diskio_dma.h"

typedef enum {
  APPLICATION_IDLE = 0,
  APPLICATION_READY,
  APPLICATION_START,
  APPLICATION_DISCONNECT,
}MSC_ApplicationTypeDef;

#define WAKEUP_Pin GPIO_PIN_0
#define WAKEUP_GPIO_Port GPIOA

#endif /* __MAIN_H */
