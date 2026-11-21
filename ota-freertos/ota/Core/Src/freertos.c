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
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// 任务栈大小的单位是「字」（4 字节）
#define COMM_TASK_STACK      512                      // 通信任务：AT 命令、MQTT、cJSON、OTA，局部缓冲区较大
#define COMM_TASK_PRIO       (tskIDLE_PRIORITY + 1)
#define SENSOR_TASK_STACK    256                      // 传感器任务：I2C、ADC、浮点换算
#define SENSOR_TASK_PRIO     (tskIDLE_PRIORITY + 2)   // 高于通信任务：通信任务擦写 W25Q 时会忙等，不能让传感器停更

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
// 任务都用静态内存创建：占用的 RAM 在编译时就能从 ZI-data 看出来，不依赖 FreeRTOS 堆大小
static StaticTask_t commTaskTCB;
static StackType_t commTaskStack[COMM_TASK_STACK];
static TaskHandle_t commTaskHandle = NULL;

static StaticTask_t sensorTaskTCB;
static StackType_t sensorTaskStack[SENSOR_TASK_STACK];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void CommTask(void *argument);
static void SensorTask(void *argument);
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
	commTaskHandle = xTaskCreateStatic(CommTask, "comm", COMM_TASK_STACK, NULL,
		COMM_TASK_PRIO, commTaskStack, &commTaskTCB);
	wifiBindTask(commTaskHandle);   // 串口中断收到数据后通知这个任务
	xTaskCreateStatic(SensorTask, "sensor", SENSOR_TASK_STACK, NULL,
		SENSOR_TASK_PRIO, sensorTaskStack, &sensorTaskTCB);
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
	// 业务都在 CommTask / SensorTask 里，CubeMX 生成的这个任务直接删除自己，内存由空闲任务回收
	vTaskDelete(NULL);
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
// 通信任务：独占 USART3（4G 模组）、W25Q、cJSON 和 printf，这些资源不需要再加锁
static void CommTask(void *argument) {
	TickType_t now = 0;
	TickType_t uploadTick = 0;
	TickType_t retryTick = 0;
	TickType_t otaWaitTick = 0;
	uint8_t otaWaiting = 0;   // OTA 已请求，正在等待 OTA_START_DELAY_MS 后开始

	(void)argument;
	printf("> comm task start\r\n");   // 调试用：确认调度器已经启动
	mqttConnect();            // 连接结果保存在 gWifi4G.connected 中，最长阻塞 180 秒（只阻塞本任务）
	printf("> MQTT %s\r\n", gWifi4G.connected ? "connected" : "not connected");   // 调试用
	retryTick = xTaskGetTickCount();
	uploadTick = retryTick;

	for (;;) {
		wifiWaitRx(COMM_POLL_MS);   // 等串口数据（中断通知），最多等 COMM_POLL_MS
		wifiPoll();
		now = xTaskGetTickCount();

		// 每隔 UPLOAD_PERIOD_MS 上报一次（未连接时不上报）
		if ((now - uploadTick) >= pdMS_TO_TICKS(UPLOAD_PERIOD_MS)) {
			uploadTick = now;
			if (gWifi4G.connected) {
				mqttSendData((uint8_t *)PUB1, 1, 0);
			}
		}
		// 掉线判定：已连接时连续收到多次 ERROR，认为 MQTT 已断开
		if (gWifi4G.connected && gWifi4G.errCount >= MQTT_ERR_LIMIT) {
			gWifi4G.connected = 0;
			gWifi4G.errCount = 0;
			retryTick = now;
			OLED_ShowStr(0, 3, (unsigned char *)"MQTTServer... --", 2);
		}
		// 断开后，每隔 MQTT_RECONNECT_MS 重连一次
		if (!gWifi4G.connected && (now - retryTick) >= pdMS_TO_TICKS(MQTT_RECONNECT_MS)) {
			retryTick = now;
			mqttConnect();
		}
		// 收到 OTA 信息后先等 OTA_START_DELAY_MS，期间照常收数据，再开始下载
		if (gOtaFlag && !otaWaiting) {
			gOtaFlag = 0;
			otaWaiting = 1;
			otaWaitTick = now;
		}
		if (otaWaiting && (now - otaWaitTick) >= pdMS_TO_TICKS(OTA_START_DELAY_MS)) {
			otaWaiting = 0;
			MQTT_OTA_GetFW();   // 成功会重启；失败按重试次数重新置位 gOtaFlag
		}
	}
}

// 传感器任务：读 AHT20、INA226、ADC，按 REG_HOLD_BUF[0] 刷新 LED/蜂鸣器/继电器
static void SensorTask(void *argument) {
	TickType_t refreshTick = 0;

	(void)argument;
	refreshTick = xTaskGetTickCount();
	for (;;) {
		if ((xTaskGetTickCount() - refreshTick) >= pdMS_TO_TICKS(SENSOR_PERIOD_MS)) {
			refreshTick = xTaskGetTickCount();
			Modbus_Refresh();
		}
		Modbus_Parse();
		vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
	}
}
/* USER CODE END Application */

