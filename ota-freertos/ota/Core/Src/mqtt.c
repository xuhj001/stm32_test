#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mqtt.h"
#include "wifi4g.h"
#include "usart.h"
#include "oled.h"
#include "modbus.h"
#include "cJSON.h"
#include "crc32.h"
#include "boot.h"
#include "w25q.h"
#include "FreeRTOS.h"
#include "task.h"

#define OTA_CHUNK_TIMEOUT_MS   10000   // 单个分包请求后最多等待的时间（ms）
#define OTA_POLL_MS            10      // 等待分包时，没有新数据最多等这么久也解析一次

__IO uint8_t gOtaFlag = 0; //  请求服务器发送分包固件标识位
__IO uint8_t gOtaBusy = 0; // OTA 下载进行中
OTA_FW_Info_T OTA_Info ; // OTA 专用状态（不含 RPC/属性 JSON）

// 下行 JSON 临时缓冲：RPC 与 attributes 共用，与 OTA_Info.chunk_buf 隔离
static uint8_t mqttJsonBuf[NSize];
static uint32_t sOtaLastPercent = 0xFFFFFFFFU;

#define WIFI4G_SEND_CHECK(cmd)                                  \
	do {                                                          \
		if (wifiCheckCmdStatus(cmd, 1000) == WIFI4G_ERROR) 					\
			return RESET;                                             \
	} while (0)

// 把 src 拷贝到固定大小的 dst 中，保证以 '\0' 结尾，不会越界
static void mqttCopyStr(char *dst, uint32_t dstSize, const char *src) {
	strncpy(dst, src, dstSize - 1);
	dst[dstSize - 1] = '\0';
}

// 判断字符串是否全部由十六进制字符组成（用于校验 fw_checksum）
static uint8_t mqttIsHex(const char *s) {
	if (s == NULL || *s == '\0') {
		return 0;
	}
	for (; *s != '\0'; s++) {
		if (!((*s >= '0' && *s <= '9') || (*s >= 'a' && *s <= 'f') || (*s >= 'A' && *s <= 'F'))) {
			return 0;
		}
	}
	return 1;
}

// 在 recvbuf 中查找 key：
//   outId 不为空时，取出 key 后面的数字（如 RPC 请求号）；
//   然后取出 key 之后第一个完整的 {...} JSON（按括号配对，支持嵌套），拷贝到 mqttJsonBuf
uint8_t mqttValue(char *recvbuf, char *key, uint32_t *outId) {
	char *startp = NULL;
	char *leftp = NULL;
	char *p = NULL;
	int32_t depth = 0;
	uint32_t jsonLen = 0;

	startp = strstr(recvbuf, key);
	if (startp == NULL) {
		return RESET;
	}
	if (outId != NULL) {
		*outId = strtol(startp + strlen(key), NULL, 10);
	}

	leftp = strchr(startp, '{');
	if (leftp == NULL) {
		return RESET;
	}

	// 按括号深度找到与 leftp 配对的 '}'
	for (p = leftp; *p != '\0'; p++) {
		if (*p == '{') {
			depth++;
		} else if (*p == '}') {
			depth--;
			if (depth == 0) {
				break;
			}
		}
	}
	if (depth != 0) {          // 没有闭合，说明 JSON 不完整
		return RESET;
	}

	jsonLen = (uint32_t)(p - leftp + 1);
	if (jsonLen >= sizeof(mqttJsonBuf)) {
		return RESET;
	}
	memset(mqttJsonBuf, 0, sizeof(mqttJsonBuf));
	memcpy(mqttJsonBuf, leftp, jsonLen);
	return SET;
}

// len 为本次实际收到的字节数（用于校验二进制分包是否完整）
uint8_t mqttParse(uint8_t *recvbuf, uint32_t len) {
	uint32_t rpcId = 0;
	char tmp[64] = {0};
	char *startp = NULL;
	uint32_t used = 0;

	// 解控制命令
	if (mqttValue((char *)recvbuf, "v1/devices/me/rpc/request/", &rpcId) == SET) {
		printf("[DOWN] rpc id=%d %s\r\n", (int)rpcId, (char *)mqttJsonBuf);
		mqttParseDevice(mqttJsonBuf, rpcId);
		return SET;
	}

	// 解 OTA 信息（属性）
	if (mqttValue((char *)recvbuf, "v1/devices/me/attributes", NULL) == SET) {
		printf("[DOWN] attributes %s\r\n", (char *)mqttJsonBuf);
		mqttParseOtaData(mqttJsonBuf);
		return SET;
	}

	// 解 OTA 分包数据（写入 OTA_Info.chunk_buf，与 mqttJsonBuf 隔离）
	sprintf(tmp, "v2/fw/response/%d/chunk/%d,%d,",
		OTA_Info.request_id, OTA_Info.chunk_id, OTA_Info.request_bytes);
	startp = strstr((char *)recvbuf, tmp);
	if (startp != NULL) {
		// 头部 + 请求的字节数 必须都在本次收到的 len 字节之内，否则是半包，丢弃（等超时重发）
		used = (uint32_t)(startp - (char *)recvbuf) + strlen(tmp);
		if (used + OTA_Info.request_bytes > len) {
			return RESET;
		}
		if (OTA_Info.request_bytes > sizeof(OTA_Info.chunk_buf)) {
			return RESET;
		}
		// OtaTask 尚未取走上一包时不覆盖，避免 Comm 抢占写入时破坏正在校验/写 Flash 的数据
		if (OTA_Info.recv_flag != 0) {
			return SET;
		}
		memcpy(OTA_Info.chunk_buf, startp + strlen(tmp), OTA_Info.request_bytes);
		OTA_Info.recv_flag = 1; // OTA 收到固件内容（OtaTask 等待此标志）
		return SET;
	}

	return RESET;
}

void mqttContrl(uint8_t value, uint32_t rpcId, uint16_t cmd) {
	if (value == 1) {
		REG_HOLD_BUF[0] |= cmd;
		mqttSendData((uint8_t *)PUB2, rpcId, value);
	} else if (value == 0) {
		REG_HOLD_BUF[0] = REG_HOLD_BUF[0] & (~cmd);
		mqttSendData((uint8_t *)PUB2, rpcId, value);
	}
}

uint8_t mqttParseDevice(uint8_t *json, uint32_t rpcId) {
	cJSON *cjson_device = NULL;
	cJSON *cjson_method = NULL;
	cJSON *cjson_params = NULL;
	char *method = "";
	uint8_t value = 0;

	cjson_device = cJSON_Parse((char *)json);
	if (cjson_device == NULL) {
		return RESET;
	}

	cjson_method = cJSON_GetObjectItem(cjson_device, "method");
	if (cjson_method && cjson_method->valuestring) {
		method = cjson_method->valuestring;
	}

	cjson_params = cJSON_GetObjectItem(cjson_device, "params");
	if (cjson_params) {
		value = cjson_params->valueint;
	}

	// 获取状态
	if (strcmp(method, "led1Status") == 0) {
		mqttSendData((uint8_t *)PUB2, rpcId, HAL_GPIO_ReadPin(LED1_GPIO_Port, LED1_Pin) == GPIO_PIN_RESET);
	} else if (strcmp(method, "beepStatus") == 0) {
		mqttSendData((uint8_t *)PUB2, rpcId, HAL_GPIO_ReadPin(BEEP_GPIO_Port, BEEP_Pin) == GPIO_PIN_SET);
	} else if (strcmp(method, "relayStatus") == 0) {
		mqttSendData((uint8_t *)PUB2, rpcId, HAL_GPIO_ReadPin(RELAY_GPIO_Port, RELAY_Pin) == GPIO_PIN_SET);
	}

	// 控制
	if (strcmp(method, "led1Set") == 0) {
		mqttContrl(value, rpcId, LED1_CMD);
	} else if (strcmp(method, "beepSet") == 0) {
		mqttContrl(value, rpcId, BEEP_CMD);
	} else if (strcmp(method, "relaySet") == 0) {
		mqttContrl(value, rpcId, RELAY_CMD);
	}

	// 一定要释放内存
	cJSON_Delete(cjson_device);
	return SET;
}

// id：PUB1 时为上报类型（MQTT_MSG_*）；PUB2 时为 RPC 请求号
uint8_t mqttSendData(uint8_t *topic, uint32_t id, uint8_t status) {
	uint8_t sendbuf[128] = {0};
	uint8_t buf[256] = {0};
	char *statusStr = status > 0 ? "true" : "false";

	if (strncmp((char *)topic, PUB1, strlen(PUB1) - 1) == 0) {
		if (id == MQTT_MSG_TELEMETRY) {
			sprintf((char *)sendbuf, "{TP:%d,RH:%d,VO:%d,CU:%d,PW:%d,VR:%d,CPU:%d}",
				REG_HOLD_BUF[1], REG_HOLD_BUF[2], REG_HOLD_BUF[3], REG_HOLD_BUF[4],
				REG_HOLD_BUF[5], REG_HOLD_BUF[6], REG_HOLD_BUF[7]);
			sprintf((char *)buf, "MQPUB,1,%s,%s", PUB1, sendbuf);
		} else if (id == MQTT_MSG_OTA_OK) {
			sprintf((char *)sendbuf, "{\"fw_state\":\"UPDATED\"}");
			sprintf((char *)buf, "MQPUB,1,%s,%s", PUB1, sendbuf);
		} else if (id == MQTT_MSG_OTA_FAIL) {
			sprintf((char *)sendbuf, "{\"fw_state\":\"FAILED\"}");
			sprintf((char *)buf, "MQPUB,1,%s,%s", PUB1, sendbuf);
		}
	} else if (strncmp((char *)topic, PUB2, strlen(PUB2) - 1) == 0) {
		// 回复服务器下发的命令 PUB2  "v1/devices/me/rpc/response/"
		sprintf((char *)buf, "MQPUB,1,%s%d,%s", PUB2, id, statusStr);
	}

	if (buf[0] != '\0') {
		printf("[UP] %s\r\n", (char *)buf);   // 上报：遥测数据、OTA 状态、RPC 回复
	}
	HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen((char *)buf), 1000);
	return SET;
}

uint8_t mqttSendOtaReq(char *bytes) {
	uint8_t buf[256] = {0};
	// 上行主题2   发送下载固件命令 "v2/fw/request/+/chunk/#"
	sprintf((char *)buf, "MQPUB,1,v2/fw/request/%d/chunk/%d,%s",
		OTA_Info.request_id, OTA_Info.chunk_id, bytes);
	HAL_UART_Transmit(&huart3, (uint8_t *)buf, strlen((char *)buf), 1000);
	return SET;
}

uint8_t mqttParseOtaData(uint8_t *json) {
	cJSON *cjson_device = NULL;
	cJSON *cjson_title = NULL;
	cJSON *cjson_version = NULL;
	cJSON *cjson_size = NULL;
	cJSON *cjson_checksum = NULL;

	cjson_device = cJSON_Parse((char *)json);
	if (cjson_device == NULL) {
		return RESET;
	}

	cjson_title = cJSON_GetObjectItem(cjson_device, "fw_title");
	cjson_version = cJSON_GetObjectItem(cjson_device, "fw_version");
	cjson_size = cJSON_GetObjectItem(cjson_device, "fw_size");
	cjson_checksum = cJSON_GetObjectItem(cjson_device, "fw_checksum");

	// 字段必须齐全，且类型正确（统一在这里释放 cjson_device，避免漏释放）
	if (cjson_title == NULL || cjson_version == NULL || cjson_size == NULL || cjson_checksum == NULL ||
		cjson_title->valuestring == NULL || cjson_version->valuestring == NULL ||
		cjson_checksum->valuestring == NULL) {
		cJSON_Delete(cjson_device);
		return RESET;
	}

	// 固件大小：不能为 0，且不能超过应用程序区大小（Bootloader 会把它搬运到应用程序区）
	if (cjson_size->valueint <= 0 || (uint32_t)cjson_size->valueint > Application_Size) {
		cJSON_Delete(cjson_device);
		return RESET;
	}

	// 校验值必须是不超过 8 位的十六进制字符串
	if (strlen(cjson_checksum->valuestring) > 8 || !mqttIsHex(cjson_checksum->valuestring)) {
		cJSON_Delete(cjson_device);
		return RESET;
	}

	mqttCopyStr(OTA_Info.fw_title, sizeof(OTA_Info.fw_title), cjson_title->valuestring);
	mqttCopyStr(OTA_Info.fw_version, sizeof(OTA_Info.fw_version), cjson_version->valuestring);
	mqttCopyStr(OTA_Info.fw_checksum, sizeof(OTA_Info.fw_checksum), cjson_checksum->valuestring);
	OTA_Info.fw_size = (uint32_t)cjson_size->valueint;
	OTA_Info.ota_try = 0; // 新的 OTA 任务，重试次数清零

	gOtaFlag = 1; // 可以处理OTA数据标志位 ， 可以从服务器下载数据包
	cJSON_Delete(cjson_device); // 一定要释放内存
	return SET;
}

// 每收到数据包时调用，len 为本包实际字节数；百分比未变则不刷 OLED，少占 I2C
void Update_Progress(uint32_t len) {
	uint32_t percent = 0;
	char progress[24] = {0};

	OTA_Info.received_bytes += len;
	if (OTA_Info.fw_size == 0) {
		return;
	}
	percent = (OTA_Info.received_bytes * 100) / OTA_Info.fw_size;
	if (percent > 100) {
		percent = 100;
	}
	if (percent == sOtaLastPercent) {
		return;
	}
	sOtaLastPercent = percent;
	if (percent == 100) {
		sprintf(progress, "OTA%.8s        OK", OTA_Info.fw_version);
	} else {
		sprintf(progress, "OTA%.8s      %3d%%", OTA_Info.fw_version, (int)percent);
	}
	printf("[OTA] progress %u%% (%u/%u)\r\n",
		(unsigned int)percent,
		(unsigned int)OTA_Info.received_bytes,
		(unsigned int)OTA_Info.fw_size);
	OLED_ShowStr(0, 6, (uint8_t *)progress, OLED_FONT_8X16);
}

// 本次升级失败：还有重试次数就重新下载，否则放弃并上报 FAILED
static void mqttOtaFail(void) {
	gOtaBusy = 0;   // 允许 CommTask 恢复遥测/重连
	OTA_Info.ota_try++;
	if (OTA_Info.ota_try < OTA_MAX_TRY) {
		printf("[OTA] try %u/%u failed, retry\r\n", (unsigned int)OTA_Info.ota_try, (unsigned int)OTA_MAX_TRY);
		gOtaFlag = 1;   // CommTask 延时后再次通知 OtaTask
	} else {
		printf("[OTA] try %u/%u failed, give up, report FAILED\r\n", (unsigned int)OTA_Info.ota_try, (unsigned int)OTA_MAX_TRY);
		mqttSendData((uint8_t *)PUB1, MQTT_MSG_OTA_FAIL, 0);
		OLED_ShowStr(64 + 16, 6, (uint8_t *)"  Fail", OLED_FONT_8X16);
	}
}

// 下载固件并保存到外部 W25Q32，校验通过后重启，由 Bootloader 搬运到应用程序区
uint8_t MQTT_OTA_GetFW(void) {
	uint32_t size = OTA_Info.fw_size / 256; // 一共要传输多少次
	uint32_t crc32 = 0;     // 服务器格式的 CRC32（字节交换后），用于和 fw_checksum 比较
	uint32_t crcStd = 0;    // 标准 CRC32，写入 W25Q 头部，供 Bootloader 校验
	uint32_t serverCrc = 0;
	uint32_t addr = 0;
	uint32_t n = 0;
	uint32_t i = 0;
	ota_hdr_t hdr;
	uint32_t fe0 = gRx3FeCnt, ne0 = gRx3NeCnt, ore0 = gRx3OreCnt, drop0 = gRx3DropCnt; // 本次下载开始时的错误计数

	if (OTA_Info.fw_size % 256 != 0) { // 说明有余数， 需要再次传输1次
		size++;
	}

	// 没有检测到 W25Q32：升级无法进行，重试也没有意义，直接按"最终失败"处理
	if (!W25Q_IsReady()) {
		OTA_Info.ota_try = OTA_MAX_TRY - 1;
		mqttOtaFail();
		return RESET;
	}

	gOtaBusy = 1;   // CommTask 继续收包，但暂停遥测/重连，避免与分包请求抢 TX
	printf("[OTA] start try %u/%u, version=%s size=%u checksum=%s\r\n",
		(unsigned int)(OTA_Info.ota_try + 1), (unsigned int)OTA_MAX_TRY,
		OTA_Info.fw_version, (unsigned int)OTA_Info.fw_size, OTA_Info.fw_checksum);
	// 擦除 W25Q：先擦头部扇区（使旧头部失效），再擦镜像区
	printf("Erase W25Q ...|size=%d\r\n", (int)size);
	if (W25Q_EraseSector(OTA_HDR_ADDR) != W25Q_OK) {
		mqttOtaFail();
		return RESET;
	}
	for (addr = 0; addr < OTA_Info.fw_size; addr += W25Q_SECTOR_SIZE) {
		if (W25Q_EraseSector(OTA_DATA_ADDR + addr) != W25Q_OK) {
			mqttOtaFail();
			return RESET;
		}
	}

	CRC32_Init(&crc_ctx); // 重置CRC
	OTA_Info.received_bytes = 0;
	sOtaLastPercent = 0xFFFFFFFFU;
	OTA_Info.request_id = HAL_GetTick() + (rand() & 0xFFFF);
	OTA_Info.chunk_id = 0;
	OTA_Info.request_bytes = 256; // 默认请求数据为256字节

	for (i = 0; i < size; i++, OTA_Info.chunk_id++) {
		char strNumber[10] = {0};
		uint8_t got = 0;
		uint8_t tryNo = 0;

		if ((i == size - 1) && (OTA_Info.fw_size % 256 != 0)) { // 最后1包数据
			OTA_Info.request_bytes = OTA_Info.fw_size % 256;
		}
		sprintf(strNumber, "%d", 256);

		// 单包请求：超时未收到就重发，最多 OTA_CHUNK_TRY 次
		for (tryNo = 0; tryNo < OTA_CHUNK_TRY && !got; tryNo++) {
			TickType_t start = 0;

			OTA_Info.recv_flag = 0;
			mqttSendOtaReq(strNumber);
			start = xTaskGetTickCount();
			while (OTA_Info.recv_flag == 0 &&
				(xTaskGetTickCount() - start) < pdMS_TO_TICKS(OTA_CHUNK_TIMEOUT_MS)) {
				vTaskDelay(pdMS_TO_TICKS(OTA_POLL_MS));
			}
			got = OTA_Info.recv_flag;
			if (!got) {
				printf("[OTA] chunk %u no response, try %u/%u\r\n",
					(unsigned int)i, (unsigned int)(tryNo + 1), (unsigned int)OTA_CHUNK_TRY);
			}
		}
		if (!got) {
			mqttOtaFail();
			return RESET;
		}
		/* 逐包 CRC 仅首包打摘要；进度百分比在 Update_Progress */
		if (i == 0) {
			CRC32_Context chunkCtx;
			CRC32_Init(&chunkCtx);
			CRC32_Update(&chunkCtx, OTA_Info.chunk_buf, OTA_Info.request_bytes);
			printf("[OTA] chunk 1/%u ok, len=%u crc=%08lx head: %02x %02x %02x %02x\r\n",
				(unsigned int)size,
				(unsigned int)OTA_Info.request_bytes,
				(unsigned long)CRC32_Final(&chunkCtx, 0),
				OTA_Info.chunk_buf[0], OTA_Info.chunk_buf[1],
				OTA_Info.chunk_buf[2], OTA_Info.chunk_buf[3]);
		}

		Update_Progress(OTA_Info.request_bytes);

		CRC32_Update(&crc_ctx, OTA_Info.chunk_buf, OTA_Info.request_bytes);
		if (W25Q_Write(OTA_DATA_ADDR + i * 256, OTA_Info.chunk_buf, OTA_Info.request_bytes) != W25Q_OK) {
			printf("[W25Q] write FAIL chunk=%d addr=0x%06x\r\n", (int)i, (unsigned int)(OTA_DATA_ADDR + i * 256));
			OTA_Info.recv_flag = 0;
			mqttOtaFail();
			return RESET;
		}
		OTA_Info.recv_flag = 0;
	}

	printf("[OTA] download done, %u chunks, rx since start: FE=%u NE=%u ORE=%u drop=%u\r\n",
		(unsigned int)size,
		(unsigned int)(gRx3FeCnt - fe0), (unsigned int)(gRx3NeCnt - ne0),
		(unsigned int)(gRx3OreCnt - ore0), (unsigned int)(gRx3DropCnt - drop0));
	// 进行crc32 校验：按数值比较，不再依赖字符串格式（大小写、前导零）
	crc32 = CRC32_Final(&crc_ctx, 1);
	crcStd = CRC32_Final(&crc_ctx, 0);
	serverCrc = strtoul(OTA_Info.fw_checksum, NULL, 16);
	if (serverCrc != crc32) {
		printf("[OTA] CRC mismatch: server=%08lx calc=%08lx\r\n",
			(unsigned long)serverCrc, (unsigned long)crc32);
		mqttOtaFail(); // 校验失败：按重试次数决定重下还是放弃
		return RESET;
	}
	printf("[OTA] CRC OK: %08lx, verify W25Q readback\r\n", (unsigned long)crc32);

	// 回读 W25Q 重新计算 CRC，确认写进去的数据和收到的一致（用 chunk_buf，不碰 mqttJsonBuf）
	CRC32_Init(&crc_ctx);
	for (addr = 0; addr < OTA_Info.fw_size; addr += 256) {
		n = OTA_Info.fw_size - addr;
		if (n > 256) {
			n = 256;
		}
		if (W25Q_Read(OTA_DATA_ADDR + addr, OTA_Info.chunk_buf, n) != W25Q_OK) {
			mqttOtaFail();
			return RESET;
		}
		CRC32_Update(&crc_ctx, OTA_Info.chunk_buf, n);
	}
	if (CRC32_Final(&crc_ctx, 0) != crcStd) {
		printf("[OTA] W25Q readback CRC FAIL (data in flash differs)\r\n");
		mqttOtaFail();
		return RESET;
	}
	// 写入 OTA 头部：先写 size/crc/state，最后写 magic（magic 有效就代表镜像完整）
	// state 明确写为 OTA_STATE_PENDING(0xAAAAAAAA)，与擦除态 0xFFFFFFFF（无升级）区分，等待 Bootloader 升级
	hdr.magic = OTA_MAGIC;
	hdr.size = OTA_Info.fw_size;
	hdr.crc = crcStd;
	hdr.state = OTA_STATE_PENDING;
	if (W25Q_Write(OTA_HDR_ADDR + 4, (const uint8_t *)&hdr.size, 12) != W25Q_OK ||
		W25Q_Write(OTA_HDR_ADDR, (const uint8_t *)&hdr.magic, 4) != W25Q_OK) {
		mqttOtaFail();
		return RESET;
	}

	printf("[OTA] SUCCESS, reset to Bootloader\r\n");
	mqttSendData((uint8_t *)PUB1, MQTT_MSG_OTA_OK, 0);
	HAL_NVIC_SystemReset(); // 重启系统，由 Bootloader 把 W25Q 中的固件搬运到应用程序区
	return SET;
}

uint8_t mqttConnect(void) {
	uint8_t buf[128] = {0};
	gWifi4G.respKeyWord = "OK\r\n";

	// 使能多通道订阅与发布主题
	WIFI4G_SEND_CHECK("AT+MQMULTEN=1\r\n");
	// 不过滤任何主题
	WIFI4G_SEND_CHECK("AT+MQTTFILTER=0\r\n");
	// 订阅服务器 下行主题1
	sprintf((char *)buf, "AT+MQSUBM=0,1,0,4,\"%s\"\r\n", MqttSubCmd);
	WIFI4G_SEND_CHECK((char *)buf);
	// 订阅服务器 下行主题2
	sprintf((char *)buf, "AT+MQSUBM=1,1,0,4,\"%s\"\r\n", MqttSubOta);
	WIFI4G_SEND_CHECK((char *)buf);
	// 订阅服务器 下行主题3
	sprintf((char *)buf, "AT+MQSUBM=2,1,0,4,\"%s\"\r\n", MqttSubOtaData);
	WIFI4G_SEND_CHECK((char *)buf);
	// 服务器连接成功的返回值
	gWifi4G.respKeyWord = "MQTT_CONNECT:";
	// 复位4G模组, 复位后自动重连
	sprintf((char *)buf, "AT+REST\r\n");
	uint8_t ret = wifiCheckCmdStatus((char *)buf, 1000 * 180); // 延时180秒
	if (ret == WIFI4G_OK) {
		gWifi4G.connected = 1;
		gWifi4G.errCount = 0;   // 新连接，清零错误计数
		printf("[MQTT] connect OK\r\n");
		OLED_ShowStr(0, 3, (unsigned char *)"MQTTServer... OK", OLED_FONT_8X16);
		return SET;
	} else {
		printf("[MQTT] connect FAIL, AT+REST result=%d (0=timeout, 2=ERROR)\r\n", (int)ret);
		OLED_ShowStr(0, 3, (unsigned char *)"MQTTServer... --", OLED_FONT_8X16);
		return RESET;
	}
}
