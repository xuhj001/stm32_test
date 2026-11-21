#ifndef __WIFI4G_H
#define __WIFI4G_H
#include <stdint.h>
#include "main.h"
#include "fifo.h"
#include "FreeRTOS.h"
#include "task.h"

enum {
  WIFI4G_NOT = 0, // Not found
  WIFI4G_OK,
  WIFI4G_ERROR
};

//extern __IO uint8_t cmdStatus; // OK状态标志位

typedef struct{
  __IO uint8_t cmdStatus;
  __IO uint8_t connected;      // MQTT_Download_Flag
  __IO uint8_t rxPending;      // 有新数据待处理（中断置1，主循环清0）
  __IO uint32_t lastIdleTick;  // 最近一次入队时的 HAL_GetTick() 时间戳（ms）
  const char *respKeyWord;
  __IO uint8_t errCount;       // 已连接时连续收到的 ERROR 次数（达到阈值判定掉线）
} Wifi4G;
extern Wifi4G gWifi4G;
extern __IO uint32_t gRx3FeCnt;     // USART3 帧错误次数（调试）
extern __IO uint32_t gRx3NeCnt;     // USART3 噪声错误次数（调试）
extern __IO uint32_t gRx3OreCnt;    // USART3 溢出错误次数（调试）
extern __IO uint32_t gRx3DropCnt;   // 入队时队列已满而丢弃的字节数（调试）
uint8_t wifiCheckCmdStatus(const char *atCmd, uint32_t Timeout);
void wifiPoll(void);
void wifiBindTask(TaskHandle_t task);
void wifiWaitRx(uint32_t ms);
void wifiRxNotifyFromISR(void);

#endif
