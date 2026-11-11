/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */
#define SCAN_KEY(port, pin, code) do{if(HAL_GPIO_ReadPin(port, pin)==GPIO_PIN_RESET) return code;}while(0)

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED2_Pin|LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : BEEP_Pin */
  GPIO_InitStruct.Pin = BEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BEEP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : RELAY_Pin LED2_Pin LED1_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin|LED2_Pin|LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : OK_Pin KEY_UP_Pin KEY_DOWN_Pin KEY_RIGHT_Pin
                           KEY_LEFT_Pin */
  GPIO_InitStruct.Pin = OK_Pin|KEY_UP_Pin|KEY_DOWN_Pin|KEY_RIGHT_Pin
                          |KEY_LEFT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : KEY_ESC_Pin */
  GPIO_InitStruct.Pin = KEY_ESC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(KEY_ESC_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void ledControl(uint8_t device, uint8_t cmd) {
	if(device == LED1) {
		if(cmd == ON) {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
		}
	} else if(device == LED2) {
		if(cmd == ON) {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
		}
	}
}

void beepControl(uint8_t cmd) {
	if(cmd == ON) {
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);
	}
}

void relayControl(uint8_t cmd) {
	if(cmd == ON) {
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);
	}
}

void relayTest(void) {
//	relayControl(ON);
//	HAL_Delay(5000);
	relayControl(OFF);
	HAL_Delay(5000);
}

uint8_t keyScan(void) {
    SCAN_KEY(KEY_UP_GPIO_Port, KEY_UP_Pin, KEY_UP);
    SCAN_KEY(KEY_DOWN_GPIO_Port, KEY_DOWN_Pin, KEY_DOWN);
    SCAN_KEY(KEY_LEFT_GPIO_Port, KEY_LEFT_Pin, KEY_LEFT);
    SCAN_KEY(KEY_RIGHT_GPIO_Port, KEY_RIGHT_Pin, KEY_RIGHT);
    SCAN_KEY(OK_GPIO_Port, OK_Pin, KEY_OK);
    SCAN_KEY(KEY_ESC_GPIO_Port, KEY_ESC_Pin, KEY_ESC);
    return 0;
}

void keyTest(void) {
	uint8_t key = keyScan();
	if(key == KEY_UP) {
		ledControl(LED1,ON);
	}else if(key == KEY_DOWN) {
		ledControl(LED1,OFF);
	}
}

/* USER CODE END 2 */
