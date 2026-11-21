#include "w25q.h"
#include "FreeRTOS.h"
#include "task.h"

/*
 * 寄存器级 SPI1 主机，模式0（CPOL=0, CPHA=0），8位，MSB 在前。
 * APB2 = 72MHz，BR=/8 => 9MHz，W25Q 的 READ(0x03) 指令最高支持 50MHz。
 * waitReady 用 vTaskDelay 让出 CPU（须在调度器启动后调用；Boot 侧副本仍为忙等）。
 */

#define CMD_WRITE_ENABLE    0x06
#define CMD_READ_STATUS1    0x05
#define CMD_READ_DATA       0x03
#define CMD_PAGE_PROGRAM    0x02
#define CMD_SECTOR_ERASE    0x20
#define CMD_RESET_ENABLE    0x66
#define CMD_RESET           0x99
#define CMD_READ_ID         0x90

#define STATUS_BUSY         0x01

#define SPI_WAIT_LOOPS      100000U     // 等待 SPI 标志位的循环次数上限，防止硬件异常时死等
#define PROG_TIMEOUT_MS     100U        // 页编程最大约 3ms，留足余量
#define ERASE_TIMEOUT_MS    1000U       // 扇区擦除最大约 400ms，留足余量

#define CS_LOW()            (W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN << 16U)
#define CS_HIGH()           (W25Q_CS_PORT->BSRR = (uint32_t)W25Q_CS_PIN)

static uint8_t sReady = 0;

// 发送并同时接收 1 字节
static uint8_t spiXfer(uint8_t tx) {
	uint32_t n = SPI_WAIT_LOOPS;

	while (!(SPI1->SR & SPI_SR_TXE) && n) {
		n--;
	}
	SPI1->DR = tx;
	n = SPI_WAIT_LOOPS;
	while (!(SPI1->SR & SPI_SR_RXNE) && n) {
		n--;
	}
	return (uint8_t)SPI1->DR;
}

// 结束一次片选：等总线空闲后再拉高 CS
static void csHigh(void) {
	uint32_t n = SPI_WAIT_LOOPS;

	while ((SPI1->SR & SPI_SR_BSY) && n) {
		n--;
	}
	CS_HIGH();
}

static void sendAddr(uint32_t addr) {
	spiXfer((uint8_t)(addr >> 16));
	spiXfer((uint8_t)(addr >> 8));
	spiXfer((uint8_t)addr);
}

static void writeEnable(void) {
	CS_LOW();
	spiXfer(CMD_WRITE_ENABLE);
	csHigh();
}

// 等待芯片空闲（状态寄存器 BUSY 位清零）；忙时 vTaskDelay(1) 让出 CPU
static uint8_t waitReady(uint32_t timeoutMs) {
	TickType_t start = xTaskGetTickCount();
	uint8_t status;

	for (;;) {
		CS_LOW();
		spiXfer(CMD_READ_STATUS1);
		status = spiXfer(0xFF);
		csHigh();
		if (!(status & STATUS_BUSY)) {
			return W25Q_OK;
		}
		if ((xTaskGetTickCount() - start) > pdMS_TO_TICKS(timeoutMs)) {
			return W25Q_ERR;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

uint8_t W25Q_Init(void) {
	GPIO_InitTypeDef gpio = {0};
	uint8_t mid = 0;
	uint8_t did = 0;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_SPI1_CLK_ENABLE();

	// CS：推挽输出，初始为高（不选中）
	HAL_GPIO_WritePin(W25Q_CS_PORT, W25Q_CS_PIN, GPIO_PIN_SET);
	gpio.Pin = W25Q_CS_PIN;
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(W25Q_CS_PORT, &gpio);

	// SCK(PA5) / MOSI(PA7)：复用推挽
	gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(GPIOA, &gpio);

	// MISO(PA6)：浮空输入
	gpio.Pin = GPIO_PIN_6;
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &gpio);

	// SPI1：主机，软件 NSS，BR=010（/8）
	SPI1->CR1 = 0;
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_1;
	SPI1->CR1 |= SPI_CR1_SPE;

	// 软复位，让芯片回到确定状态（复位恢复时间约 30us）
	CS_LOW();
	spiXfer(CMD_RESET_ENABLE);
	csHigh();
	CS_LOW();
	spiXfer(CMD_RESET);
	csHigh();
	HAL_Delay(1);

	// 读厂商/设备 ID（0x90）：W25Q32 为 0xEF, 0x15
	CS_LOW();
	spiXfer(CMD_READ_ID);
	sendAddr(0);
	mid = spiXfer(0xFF);
	did = spiXfer(0xFF);
	csHigh();

	sReady = (mid == 0xEF && did == 0x15) ? 1 : 0;
	return sReady;
}

uint8_t W25Q_IsReady(void) {
	return sReady;
}

void W25Q_DeInit(void) {
	uint32_t n = SPI_WAIT_LOOPS;

	while ((SPI1->SR & SPI_SR_BSY) && n) {
		n--;
	}
	SPI1->CR1 &= ~SPI_CR1_SPE;
	__HAL_RCC_SPI1_CLK_DISABLE();   // CS 引脚保持输出高电平，芯片不会被误选中
}

uint8_t W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len) {
	uint32_t i;

	if (waitReady(PROG_TIMEOUT_MS) != W25Q_OK) {
		return W25Q_ERR;
	}
	CS_LOW();
	spiXfer(CMD_READ_DATA);
	sendAddr(addr);
	for (i = 0; i < len; i++) {
		buf[i] = spiXfer(0xFF);
	}
	csHigh();
	return W25Q_OK;
}

uint8_t W25Q_EraseSector(uint32_t addr) {
	if (waitReady(PROG_TIMEOUT_MS) != W25Q_OK) {
		return W25Q_ERR;
	}
	writeEnable();
	CS_LOW();
	spiXfer(CMD_SECTOR_ERASE);
	sendAddr(addr);
	csHigh();
	return waitReady(ERASE_TIMEOUT_MS);
}

// 写入任意长度数据，内部按 256 字节页边界拆分
uint8_t W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len) {
	uint32_t n;
	uint32_t i;

	while (len > 0) {
		n = W25Q_PAGE_SIZE - (addr % W25Q_PAGE_SIZE);   // 本页还剩多少字节
		if (n > len) {
			n = len;
		}
		if (waitReady(PROG_TIMEOUT_MS) != W25Q_OK) {
			return W25Q_ERR;
		}
		writeEnable();
		CS_LOW();
		spiXfer(CMD_PAGE_PROGRAM);
		sendAddr(addr);
		for (i = 0; i < n; i++) {
			spiXfer(buf[i]);
		}
		csHigh();
		if (waitReady(PROG_TIMEOUT_MS) != W25Q_OK) {
			return W25Q_ERR;
		}
		addr += n;
		buf += n;
		len -= n;
	}
	return W25Q_OK;
}
