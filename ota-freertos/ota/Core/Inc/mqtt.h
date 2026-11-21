#ifndef __MQTT_H_
#define __MQTT_H_
 
#include "main.h"
#include "fifo.h"

// 下行主题1   负责接收服务器的命令
#define  MqttSubCmd  "v1/devices/me/rpc/request/+"
// 下行主题2   负责接收服务器的OTA信息数据
#define  MqttSubOta  "v1/devices/me/attributes"
// 下行主题3   传输OTA的文件的内容
#define  MqttSubOtaData  "v2/fw/response/+/chunk/#"

// 上行主题1   上传服务器数据的
#define  PUB1  "v1/devices/me/telemetry"
// 上行主题2   回复服务器下发的命令
#define  PUB2  "v1/devices/me/rpc/response/"
// 上行主题3    请求服务器下发数据快
#define  PUB3  "v2/fw/request/+/chunk/#"

// OTA 重试次数
#define OTA_CHUNK_TRY   3   // 单个分包最多请求的次数（超时则重发）
#define OTA_MAX_TRY     3   // 整个固件最多尝试升级的次数（校验失败或分包失败则重下）

// mqttSendData 在 PUB1（遥测主题）下的 id 含义：上报类型
#define MQTT_MSG_TELEMETRY  1   // 传感器遥测
#define MQTT_MSG_OTA_OK     2   // OTA 成功
#define MQTT_MSG_OTA_FAIL   3   // OTA 最终失败

// OTA 专用状态：不含 RPC/属性 JSON。下行 JSON 使用 mqtt.c 内的 mqttJsonBuf。
typedef struct {
	char fw_title[32];
	char fw_version[32];
	uint32_t fw_size;
	char fw_checksum[32];
	uint8_t chunk_buf[NSize];  // 固件分包载荷，以及回读校验临时缓冲
	uint8_t recv_flag;         // 收到匹配的固件分包
	uint32_t request_id;       // 请求的数据包 id
	uint32_t chunk_id;         // 请求的数据块 id
	uint32_t request_bytes;    // 本包请求字节数
	uint32_t received_bytes;   // 已收字节数（进度）
	uint8_t ota_try;           // 本次固件已尝试次数
} OTA_FW_Info_T; 
extern __IO uint8_t gOtaFlag; //  请求服务器发送分包固件标识位
extern __IO uint8_t gOtaBusy; // OTA 下载进行中（Comm 暂停遥测/重连，避免抢 USART3 TX）

// id：PUB1 时为上报类型（MQTT_MSG_*）；PUB2 时为 RPC 请求号
uint8_t mqttSendData(uint8_t *topic, uint32_t id, uint8_t status);
uint8_t mqttSendOtaReq(char *bytes);
uint8_t mqttConnect(void);
uint8_t mqttParse(uint8_t *recvbuf, uint32_t len);
uint8_t mqttParseDevice(uint8_t *json, uint32_t rpcId);
uint8_t mqttParseOtaData(uint8_t *json);
uint8_t MQTT_OTA_GetFW(void);

#endif //  __MQTT_H_
