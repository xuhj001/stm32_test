#ifndef __W25Q_H_
#define __W25Q_H_

/*
 * W25Q32 精简驱动（寄存器级 SPI1，不依赖 HAL_SPI）。
 * APP 工程（ota-test）和 Bootloader 工程（boot）共用同一份源码，修改时请两边同步。
 *
 * 硬件连接（与 sample/32-spi-w25q32 一致）:
 *   PA4 = CS（软件控制）  PA5 = SCK  PA6 = MISO  PA7 = MOSI
 */

#include "main.h"

#define W25Q_CS_PORT        GPIOA
#define W25Q_CS_PIN         GPIO_PIN_4

#define W25Q_OK             0
#define W25Q_ERR            1

#define W25Q_SECTOR_SIZE    0x1000U     // 4KB 扇区（擦除单位）
#define W25Q_PAGE_SIZE      0x100U      // 256B 页（编程单位）

/*
 * OTA 暂存区布局（W25Q32 内部）:
 *   0x000000  扇区0  OTA 头部 ota_hdr_t
 *   0x001000  固件镜像，最大 Application_Size 字节
 *
 * 头部状态字利用 NOR Flash 只能 1->0 的特性：
 *   擦除后 state = 0xFFFFFFFF (NONE，无升级)
 *   APP 下载完成后编程 state = 0xAAAAAAAA (PENDING，等待 Bootloader 升级)
 *   Bootloader 搬运成功后直接把 state 编程为 0（DONE），无需擦除
 *   Bootloader 判断镜像损坏时把 state 编程为 BAD
 * APP 下载时先擦除头部扇区，镜像写完并校验通过后才写入 magic，
 * 所以只要 magic 有效，就说明镜像是完整的。
 */
#define OTA_HDR_ADDR        0x000000U
#define OTA_DATA_ADDR       0x001000U
#define OTA_MAGIC           0x3141544FU     // "OTA1"
#define OTA_STATE_NONE      0xFFFFFFFFU     // 擦除态：无升级（同旧代码 Startup_Normol）
#define OTA_STATE_PENDING   0xAAAAAAAAU     // 待升级（同旧代码 Startup_Update）
#define OTA_STATE_DONE      0x00000000U
#define OTA_STATE_BAD       0x88888888U     // 镜像损坏；必须是 PENDING 的位子集（NOR 只能 1->0）

typedef struct {
	uint32_t magic;     // OTA_MAGIC，最后写入
	uint32_t size;      // 固件字节数
	uint32_t crc;       // 标准 CRC32（zlib，不做字节交换）
	uint32_t state;     // OTA_STATE_xxx
} ota_hdr_t;

uint8_t W25Q_Init(void);                // 初始化 SPI1 并检测芯片，返回 1 表示检测到 W25Q32
uint8_t W25Q_IsReady(void);             // Init 时是否检测到芯片
void W25Q_DeInit(void);                 // 关闭 SPI1（Bootloader 跳转 APP 前调用）
uint8_t W25Q_Read(uint32_t addr, uint8_t *buf, uint32_t len);
uint8_t W25Q_EraseSector(uint32_t addr);
uint8_t W25Q_Write(uint32_t addr, const uint8_t *buf, uint32_t len);

#endif
