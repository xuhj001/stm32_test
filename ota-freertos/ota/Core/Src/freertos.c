/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wifi4g.h"
#include "mqtt.h"
#include "modbus.h"
#include "oled.h"
#include "i2c.h"
#include "iwdg.h"
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 任务栈大小的单位是「字」（4 字节）
#define COMM_TASK_STACK      512                      // 通信任务：AT 命令、MQTT、cJSON，局部缓冲区较大
#define COMM_TASK_PRIO       (tskIDLE_PRIORITY + 1)
#define SENSOR_TASK_STACK    256                      // 传感器任务：I2C、ADC、浮点换算
#define SENSOR_TASK_PRIO     (tskIDLE_PRIORITY + 2)   // 最高：擦写忙等时传感器仍要更新
#define OTA_TASK_STACK       512                      // OTA 任务：下载状态机、CRC、W25Q
#define OTA_TASK_PRIO        (tskIDLE_PRIORITY + 0)   // 低于 Comm：擦写等待已让出，Comm 更高作双保险

#define COMM_POLL_MS         10       // 没有新数据时，最多等这么久也要解析一次
#define UPLOAD_PERIOD_MS     5000     // 传感器数据上报周期（ms）
#define SENSOR_PERIOD_MS     500      // 传感器刷新周期（ms）
#define SENSOR_POLL_MS       20       // 传感器任务轮询周期（ms）：读取温湿度结果、刷新 LED/蜂鸣器/继电器
#define OTA_START_DELAY_MS   1000     // 收到 OTA 信息后，等这么久再开始下载（ms）
#define MQTT_RECONNECT_MS    60000    // MQTT 断开后的重连间隔（ms）
#define MQTT_ERR_LIMIT       3        // 已连接状态下连续收到 ERROR 的次数达到该值，判定为掉线
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
// 业务任务从 FreeRTOS 堆动态创建（见 configTOTAL_HEAP_SIZE）
static TaskHandle_t commTaskHandle = NULL;
static TaskHandle_t sensorTaskHandle = NULL;
static TaskHandle_t otaTaskHandle = NULL;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void CommTask(void *argument);
static void SensorTask(void *argument);
static void OtaTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
	I2C1_LockInit();   // OLED、AHT20、INA226 共用 I2C1
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	if (xTaskCreate(CommTask, "comm", COMM_TASK_STACK, NULL,
			COMM_TASK_PRIO, &commTaskHandle) != pdPASS) {
		printf("[ERR] create CommTask fail\r\n");
		Error_Handler();
	}
	wifiBindTask(commTaskHandle);   // 串口中断收到数据后通知这个任务
	if (xTaskCreate(SensorTask, "sensor", SENSOR_TASK_STACK, NULL,
			SENSOR_TASK_PRIO, &sensorTaskHandle) != pdPASS) {
		printf("[ERR] create SensorTask fail\r\n");
		Error_Handler();
	}
	if (xTaskCreate(OtaTask, "ota", OTA_TASK_STACK, NULL,
			OTA_TASK_PRIO, &otaTaskHandle) != pdPASS) {
		printf("[ERR] create OtaTask fail\r\n");
		Error_Handler();
	}
	printf("> tasks created\r\n");
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
	// 业务都在 CommTask / SensorTask / OtaTask 里，CubeMX 生成的这个任务直接删除自己
	vTaskDelete(NULL);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 通信任务：独占串口接收解析（wifiPoll），负责连接、遥测、RPC；OTA 下载交给 OtaTask
static void CommTask(void *argument) {
	TickType_t now = 0;
	TickType_t uploadTick = 0;
	TickType_t retryTick = 0;
	TickType_t otaWaitTick = 0;
	uint8_t otaWaiting = 0;   // OTA 已请求，正在等待 OTA_START_DELAY_MS 后通知 OtaTask

	(void)argument;
	printf("> comm task start\r\n");
	mqttConnect();            // 最长阻塞 180 秒（只阻塞本任务）
	printf("> MQTT %s\r\n", gWifi4G.connected ? "connected" : "not connected");
	retryTick = xTaskGetTickCount();
	uploadTick = retryTick;

	for (;;) {
		wifiWaitRx(COMM_POLL_MS);
		wifiPoll();
		now = xTaskGetTickCount();

		if ((now - uploadTick) >= pdMS_TO_TICKS(UPLOAD_PERIOD_MS)) {
			uploadTick = now;
			if (gWifi4G.connected && !gOtaBusy) {
				mqttSendData((uint8_t *)PUB1, MQTT_MSG_TELEMETRY, 0);
			}
		}
		if (gWifi4G.connected && gWifi4G.errCount >= MQTT_ERR_LIMIT) {
			gWifi4G.connected = 0;
			gWifi4G.errCount = 0;
			retryTick = now;
			printf("> MQTT offline (ERROR x%d)\r\n", MQTT_ERR_LIMIT);
			OLED_ShowStr(0, 3, (unsigned char *)"MQTTServer... --", OLED_FONT_8X16);
		}
		if (!gOtaBusy && !gWifi4G.connected &&
			(now - retryTick) >= pdMS_TO_TICKS(MQTT_RECONNECT_MS)) {
			retryTick = now;
			printf("> MQTT reconnect...\r\n");
			mqttConnect();
			printf("> MQTT %s\r\n", gWifi4G.connected ? "connected" : "not connected");
		}
		if (gOtaFlag && !otaWaiting && !gOtaBusy) {
			gOtaFlag = 0;
			otaWaiting = 1;
			otaWaitTick = now;
			printf("> OTA request, start in %d ms\r\n", OTA_START_DELAY_MS);
		}
		if (otaWaiting && (now - otaWaitTick) >= pdMS_TO_TICKS(OTA_START_DELAY_MS)) {
			otaWaiting = 0;
			printf("> OTA notify OtaTask\r\n");
			if (otaTaskHandle != NULL) {
				xTaskNotifyGive(otaTaskHandle);
			}
		}
	}
}

// 传感器任务：读 AHT20、INA226、ADC，按 REG_HOLD_BUF[0] 刷新 LED/蜂鸣器/继电器
// 同时负责 IWDG：本任务优先级最高，约每 20 ms 喂狗（超时约 4 s）
static void SensorTask(void *argument) {
	TickType_t refreshTick = 0;

	(void)argument;
	printf("> sensor task start\r\n");
	/* 先让出一拍：本任务优先级最高，若立刻开 IWDG/跑业务，Comm 的启动日志会被拖住 */
	vTaskDelay(pdMS_TO_TICKS(1));
	IWDG_AppStart();
	printf("> IWDG start (prescaler/256 reload=624, ~4s)\r\n");
	refreshTick = xTaskGetTickCount();
	for (;;) {
		IWDG_AppRefresh();
		if ((xTaskGetTickCount() - refreshTick) >= pdMS_TO_TICKS(SENSOR_PERIOD_MS)) {
			refreshTick = xTaskGetTickCount();
			Modbus_Refresh();
		}
		Modbus_Parse();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
	}
}

// OTA 任务：下载状态机、写 W25Q、刷进度；收包仍由 CommTask 解析后写入 chunk_buf
static void OtaTask(void *argument) {
	(void)argument;
	printf("> ota task start\r\n");
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		printf("> OtaTask run download\r\n");
		MQTT_OTA_GetFW();   // 成功会重启；失败按重试次数重新置位 gOtaFlag
	}
}
/* USER CODE END Application */

