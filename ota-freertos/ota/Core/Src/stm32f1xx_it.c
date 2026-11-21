/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "gpio.h"
#include <stdio.h>
#include "usart.h"
#include "modbus.h"
#include "fifo.h"
#include "wifi4g.h"
#include "mqtt.h"
#include "FreeRTOS.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
// 串口接收调试计数：FE/NE/ORE 为 USART3 错误次数，drop 为入队时队列已满丢弃的字节数
__IO uint32_t gRx3FeCnt = 0;
__IO uint32_t gRx3NeCnt = 0;
__IO uint32_t gRx3OreCnt = 0;
__IO uint32_t gRx3DropCnt = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
void USER_UART_IRQHandler(UART_HandleTypeDef *huart);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_usart3_rx;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M3 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  // 调试用：打印故障原因和出错的任务名，确认问题后可删除
  printf("\r\n[FAULT] HardFault CFSR=%08lx HFSR=%08lx BFAR=%08lx task=%s\r\n",
    (unsigned long)SCB->CFSR, (unsigned long)SCB->HFSR, (unsigned long)SCB->BFAR,
    (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) ? pcTaskGetName(NULL) : "none");
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Prefetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 channel3 global interrupt.
  */
void DMA1_Channel3_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */

  /* USER CODE END DMA1_Channel3_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */

  /* USER CODE END DMA1_Channel3_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  // 在 HAL 处理错误之前读 SR，统计接收错误（HAL 处理后会清掉标志）
  if (huart3.Instance->SR & USART_SR_FE) {
    gRx3FeCnt++;
  }
  if (huart3.Instance->SR & USART_SR_NE) {
    gRx3NeCnt++;
  }
  if (huart3.Instance->SR & USART_SR_ORE) {
    gRx3OreCnt++;
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	USER_UART_IRQHandler(&huart3);       //新添加的函数，用来处理串口空闲中断
  /* USER CODE END USART3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/*
 * 把 DMA 缓冲区中「已写入、还没入队」的数据全部入队。
 * 用 DMA 实时位置计算长度，所以半满、满、空闲三处调用哪一处，结果都一样，
 * 也不依赖中断的先后顺序。队列满时 Enqueue 返回 -1，数据丢弃（与原来一致）。
 */
static void RX3_PullToQueue(void) {
	uint32_t pos = (DMA_BUF_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx)) % DMA_BUF_SIZE;
	uint32_t len = (pos + DMA_BUF_SIZE - RX3_Offset) % DMA_BUF_SIZE;
	uint32_t k;

	if (len == 0) {
		return;
	}

	// 逐字节入队：RX3_Buf 是环形的，一段数据可能跨过缓冲区末尾，不能整段直接拷贝
	for (k = 0; k < len; k++) {
		if (Enqueue(&gQueueUart3, RX3_Buf[(RX3_Offset + k) % DMA_BUF_SIZE]) < 0) {
			gRx3DropCnt++;   // 队列已满，这个字节被丢弃
		}
	}

	RX3_Offset = (uint8_t)pos;
	gWifi4G.rxPending = 1;
	wifiRxNotifyFromISR();                  // 唤醒通信任务（要求本中断优先级数值 >= 5）
	// 注意：这里不刷新 lastIdleTick。半满/满中断只是数据还在流，不代表一帧结束；
	// 空闲时间戳只在空闲中断里刷新（见 USER_UART_IRQHandler），与旧代码的 TIM4 逻辑一致。
}

// DMA 接收到一半的中断
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		RX3_PullToQueue();
	}
}

// DMA传输完成中断   , 就是接收满了的时候 触发中断
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {
		RX3_PullToQueue();
	}
}

// 用户自定义的函数 ， 处理串口空闲中断 以及 帧错误/噪声错误
void USER_UART_IRQHandler(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART3) {                       // 判断是否是串口3
		if ((__HAL_UART_GET_FLAG(huart, UART_FLAG_FE) ||
			__HAL_UART_GET_FLAG(huart, UART_FLAG_NE)) == SET) { // 判断是否是帧错误或者噪音中断
			__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_FE);
			__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_NE);

			RX3_PullToQueue();                          // 先把已收到的数据入队，再重启DMA
			huart->ErrorCode &= ~HAL_UART_ERROR_FE;     // 清除错误标志位
			huart->ErrorCode &= ~HAL_UART_ERROR_NE;     // 清除错误标志位
			HAL_UART_DMAStop(huart);                    // 关闭DMA
			RX3_Offset = 0;                             // DMA 从缓冲区起点重新开始，偏移同步清零
			HAL_UART_Receive_DMA(huart, RX3_Buf, DMA_BUF_SIZE); // 重新启动DMA
		}
		if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET) { // 判断是否是空闲中断
			__HAL_UART_CLEAR_IDLEFLAG(huart);    // 清除空闲中断标志（否则会一直不断进入中断）

			// 一帧结束：把剩余数据入队，并刷新空闲时间戳（主循环在空闲满10ms后再处理）
			RX3_PullToQueue();
			gWifi4G.lastIdleTick = HAL_GetTick();   // 只有空闲中断才刷新时间戳
		}
	}
}

/* USER CODE END 1 */
