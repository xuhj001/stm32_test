
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f1xx_it.h"
#include "wifi4g.h"
#include "main.h"
#include "usart.h"
#include "oled.h"
#include "gpio.h"
#include "fifo.h"
#include "mqtt.h"

#pragma diag_suppress 870

#define MQRECV_TAG              "+MQRECV,"  // 模组转发的 MQTT 下行消息的开头
#define MQRECV_TAG_LEN          8
#define WIFI_TEXT_IDLE_MS       10          // 文本类数据（OK/ERROR）空闲满这么久才统计
#define WIFI_FRAME_TIMEOUT_MS   500         // 不完整的数据超过这个时间还没凑齐，就丢弃
#define WIFI_POLL_MS            10          // 没有新数据时，最多等这么久也要解析一次（文本空闲判断靠它）

// 注意：respKeyWord 初始为 NULL，匹配前必须判空
Wifi4G gWifi4G = {0,0,0,0,NULL,0};

// 处理 4G 模组数据的任务；串口中断收到数据后给它发任务通知
static TaskHandle_t wifiTask = NULL;

// 接收累积缓冲区：从队列取到的数据先追加到这里，再按「帧」解析。
// 不再靠「安静多少毫秒」来猜一帧是否结束：模组可能在一个单词、一个数据包中间停顿，
// 所以命令应答按关键字在累积数据里查找，MQTT 下行消息按头部声明的长度收齐后再处理。
uint8_t wifiData[NSize] = {0};
static uint32_t wifiLen = 0;      // wifiData 中的有效字节数
static uint32_t wifiLastRx = 0;   // 最近一次收到新数据的时刻（ms）

// 丢弃队列和累积缓冲区里残留的数据（发送新命令前调用，避免把上一条命令的应答当成本条的结果）
static void wifiFlush(void) {
	uint8_t dummy = 0;

	taskENTER_CRITICAL();
	while (!Queue_IsEmpty(&gQueueUart3)) {
		Dequeue(&gQueueUart3, &dummy);
	}
	gWifi4G.rxPending = 0;
	taskEXIT_CRITICAL();

	wifiLen = 0;
	wifiData[0] = 0;
}

// 丢弃 wifiData 开头的 n 个字节，剩余数据前移
static void wifiShift(uint32_t n) {
	if (n >= wifiLen) {
		wifiLen = 0;
	} else {
		memmove(wifiData, wifiData + n, wifiLen - n);
		wifiLen -= n;
	}
	wifiData[wifiLen] = 0;
}

// 在临界区内把队列中现有的数据全部追加到 wifiData 末尾，返回本次追加的字节数。
// 临界区只做纯内存拷贝；DMA 硬件照常收数，中断请求会挂起，退出临界区后再入队。
// taskENTER_CRITICAL 只屏蔽优先级数值 >= 5 的中断，所以 USART3、DMA1_Channel3 的优先级不能小于 5。
static uint32_t wifiFetch(void) {
	uint32_t n = 0;

	taskENTER_CRITICAL();
	while (!Queue_IsEmpty(&gQueueUart3) && (wifiLen + n) < (NSize - 1)) {
		Dequeue(&gQueueUart3, wifiData + wifiLen + n);
		n++;
	}
	if (Queue_IsEmpty(&gQueueUart3)) {
		gWifi4G.rxPending = 0;
	}
	taskEXIT_CRITICAL();
	return n;
}

// 已连接状态下，统计 wifiData[0, n) 这段文本里的 OK / ERROR（模组对 MQPUB 等命令的应答）
static void wifiCountText(uint32_t n) {
	uint8_t saved = wifiData[n];

	wifiData[n] = 0;
	if (strstr((const char *)wifiData, "ERROR\r\n") != NULL) {
		gWifi4G.errCount++;
	} else if (strstr((const char *)wifiData, "OK\r\n") != NULL) {
		gWifi4G.errCount = 0;
	}
	wifiData[n] = saved;
}

// wifiData 以 "+MQRECV," 开头时，计算整条消息的总长度。
// 格式：+MQRECV,[通道号,]主题,载荷长度,载荷
// 返回：0 = 头部还没收全；-1 = 格式不对；大于 0 = 整条消息的总字节数
static int32_t wifiFrameLen(void) {
	const uint8_t *q = wifiData + MQRECV_TAG_LEN;
	const uint8_t *end = wifiData + wifiLen;
	uint32_t payloadLen = 0;
	uint32_t digits = 0;

	// 可选的通道号：一位数字加逗号，如 "1,"
	if ((q + 1) < end && q[0] >= '0' && q[0] <= '9' && q[1] == ',') {
		q += 2;
	}
	// 主题：直到下一个逗号
	while (q < end && *q != ',') {
		q++;
	}
	if (q >= end) {
		return 0;
	}
	q++;
	// 载荷长度：十进制数字，直到逗号
	while (q < end && *q >= '0' && *q <= '9') {
		payloadLen = payloadLen * 10 + (uint32_t)(*q - '0');
		digits++;
		q++;
		if (digits > 4) {
			return -1;
		}
	}
	if (q >= end) {
		return 0;
	}
	if (digits == 0 || *q != ',') {
		return -1;
	}
	q++;
	if ((uint32_t)(q - wifiData) + payloadLen > (NSize - 1)) {
		return -1;   // 一整条消息比缓冲区还大，无法接收
	}
	return (int32_t)((uint32_t)(q - wifiData) + payloadLen);
}

// 命令阶段（未连接）：在累积的数据里找关键字，数据被拆成几段到达也能找到
static void wifiParseCmd(void) {
	wifiData[wifiLen] = 0;
	// respKeyWord 为 NULL 时不做匹配，避免 strstr 空指针
	if (gWifi4G.respKeyWord != NULL && strstr((const char *)wifiData, gWifi4G.respKeyWord) != NULL) {
		gWifi4G.cmdStatus = WIFI4G_OK;
		wifiLen = 0;
	} else if (strstr((const char *)wifiData, "ERROR\r\n") != NULL) {
		gWifi4G.cmdStatus = WIFI4G_ERROR;
		wifiLen = 0;
	} else if (wifiLen >= (NSize - 1)) {
		// 缓冲区满了还没匹配：只保留末尾 32 字节（关键字可能正好跨在边界上）
		memmove(wifiData, wifiData + wifiLen - 32, 32);
		wifiLen = 32;
		wifiData[wifiLen] = 0;
	}
}

// 已连接阶段：按帧处理。
// "+MQRECV," 开头的消息按头部声明的长度收齐后，整条交给 mqttParse；其余是 OK/ERROR 文本。
static void wifiParseConnected(void) {
	uint32_t now = HAL_GetTick();
	char *p = NULL;
	uint32_t off = 0;
	uint32_t keep = 0;
	uint32_t k = 0;
	int32_t total = 0;
	uint8_t saved = 0;

	while (wifiLen > 0) {
		wifiData[wifiLen] = 0;
		p = strstr((char *)wifiData, MQRECV_TAG);

		if (p == NULL) {
			// 没有下行消息，只是文本：空闲一小段时间后统计 OK/ERROR 并丢弃
			if ((now - wifiLastRx) >= WIFI_TEXT_IDLE_MS || wifiLen >= (NSize - 1)) {
				// 末尾如果是 "+MQRECV," 的前几个字节（消息头还没收全），要留着
				keep = 0;
				for (k = (wifiLen < (MQRECV_TAG_LEN - 1)) ? wifiLen : (MQRECV_TAG_LEN - 1); k > 0; k--) {
					if (memcmp(wifiData + wifiLen - k, MQRECV_TAG, k) == 0) {
						keep = k;
						break;
					}
				}
				if (wifiLen > keep) {
					wifiCountText(wifiLen - keep);
					wifiShift(wifiLen - keep);
				}
				if (wifiLen > 0 && (now - wifiLastRx) >= WIFI_FRAME_TIMEOUT_MS) {
					wifiLen = 0;   // 残留的半截消息头太久没有后续，丢弃
				}
			}
			return;
		}

		off = (uint32_t)((uint8_t *)p - wifiData);
		if (off > 0) {
			// 消息前面的文本（比如 MQPUB 的应答）先统计，再丢弃
			wifiCountText(off);
			wifiShift(off);
			continue;
		}

		// 此时 wifiData 以 "+MQRECV," 开头
		total = wifiFrameLen();
		if (total < 0) {
			wifiShift(MQRECV_TAG_LEN);   // 头部格式不对，跳过这个标记
			continue;
		}
		if (total == 0 || wifiLen < (uint32_t)total) {
			// 消息还没收齐：继续等；等太久就丢弃，避免一直占着缓冲区
			if ((now - wifiLastRx) >= WIFI_FRAME_TIMEOUT_MS) {
				wifiLen = 0;
				wifiData[0] = 0;
			}
			return;
		}

		// 收齐了一整条：只把这一条交给解析（临时在末尾放 '\0'，避免字符串函数越过消息边界）
		saved = wifiData[total];
		wifiData[total] = 0;
		mqttParse(wifiData, (uint32_t)total);
		wifiData[total] = saved;
		wifiShift((uint32_t)total);
	}
}

// 只能在 wifiBindTask 绑定的任务里调用（所有 AT 命令和解析都由这一个任务负责）
uint8_t wifiCheckCmdStatus(const char *atCmd, uint32_t Timeout) {
	TickType_t start = 0;
	uint32_t t0 = HAL_GetTick();
	const char *result = "TIMEOUT";

	wifiFlush();   // 丢弃之前残留的数据
	gWifi4G.cmdStatus = WIFI4G_NOT;
	printf("[AT] >> %s", atCmd);   // atCmd 自带 \r\n
	HAL_UART_Transmit(&huart3, (uint8_t *)atCmd, strlen(atCmd), 1000);
	start = xTaskGetTickCount();
	{
		TickType_t lastBeat = start;
		for (;;) {
			wifiPoll();
			if (gWifi4G.cmdStatus != WIFI4G_NOT) {
				break;
			}
			if ((xTaskGetTickCount() - start) >= pdMS_TO_TICKS(Timeout)) {
				break;
			}
			/* 长等待（如 AT+REST 180s）每 5s 打一拍，避免串口“假死” */
			if ((xTaskGetTickCount() - lastBeat) >= pdMS_TO_TICKS(5000)) {
				lastBeat = xTaskGetTickCount();
				printf("[AT] .. waiting %lu ms\r\n",
					(unsigned long)(HAL_GetTick() - t0));
			}
			wifiWaitRx(WIFI_POLL_MS);
		}
	}
	if (gWifi4G.cmdStatus == WIFI4G_OK) {
		result = "OK";
	} else if (gWifi4G.cmdStatus == WIFI4G_ERROR) {
		result = "ERROR";
	}
	printf("[AT] << %s (%lu ms)\r\n", result, (unsigned long)(HAL_GetTick() - t0));
	return gWifi4G.cmdStatus;
}

// 绑定处理 4G 数据的任务（在调度器启动前调用）
void wifiBindTask(TaskHandle_t task) {
	wifiTask = task;
}

// 阻塞当前任务，直到串口收到新数据或者超时 ms 毫秒，期间不占用 CPU
void wifiWaitRx(uint32_t ms) {
	ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(ms));
}

// 串口/DMA 中断里有数据入队后调用，唤醒处理任务
void wifiRxNotifyFromISR(void) {
	BaseType_t woken = pdFALSE;

	if (wifiTask != NULL) {
		vTaskNotifyGiveFromISR(wifiTask, &woken);
		portYIELD_FROM_ISR(woken);
	}
}

// 通信任务和所有等待循环里调用：
// 取走队列里的数据，追加到累积缓冲区，再按帧解析。
void wifiPoll(void) {
	uint32_t n = wifiFetch();

	if (n > 0) {
		// 连接阶段不再把模组原始数据整段转发到 USART1（会长时间阻塞 Comm）
		wifiLen += n;
		wifiData[wifiLen] = 0;
		wifiLastRx = HAL_GetTick();
	}
	if (wifiLen == 0) {
		return;
	}

	if (gWifi4G.connected) {
		wifiParseConnected();
	} else {
		wifiParseCmd();
	}
}
