#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "boot.h"
#include "w25q.h"

/**
 * @bieaf 擦除页
 *
 * @param pageaddr  起始地址	
 * @param num       擦除的页数
 * @return 1
 */
static int Erase_page(uint32_t pageaddr, uint32_t num)
{
	HAL_FLASH_Unlock();
	
	/* 擦除FLASH*/
	FLASH_EraseInitTypeDef FlashSet;
	FlashSet.TypeErase = FLASH_TYPEERASE_PAGES;
	FlashSet.PageAddress = pageaddr;
	FlashSet.NbPages = num;
	
	/*设置PageError，调用擦除函数*/
	uint32_t PageError = 0;
	HAL_FLASHEx_Erase(&FlashSet, &PageError);
	
	HAL_FLASH_Lock();
	return 1;
}


/**
 * @bieaf 写若干个数据
 *
 * @param addr       写入的地址
 * @param buff       写入数据的起始地址
 * @param word_size  长度
 * @return 
 */
static void WriteFlash(uint32_t addr, uint32_t * buff, int word_size)
{	
	/* 1/4解锁FLASH*/
	HAL_FLASH_Unlock();
	
	for(int i = 0; i < word_size; i++)	
	{
		/* 3/4对FLASH烧写*/
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, addr + 4 * i, buff[i]);	
	}

	/* 4/4锁住FLASH*/
	HAL_FLASH_Lock();
}


/**
 * @bieaf CRC32 更新（zlib 多项式 0xEDB88320，与 APP 端 crc32.c 一致）
 *
 * @param crc   中间值，初始为 0xFFFFFFFF
 * @param data  数据
 * @param len   长度
 * @return 新的中间值，最终结果需要再异或 0xFFFFFFFF
 */
static uint32_t Crc32_Update(uint32_t crc, const uint8_t *data, uint32_t len) {
	uint32_t i;
	uint32_t j;

	for (i = 0; i < len; i++) {
		crc ^= data[i];
		for (j = 0; j < 8; j++) {
			crc = (crc >> 1) ^ (0xEDB88320U & (0U - (crc & 1U)));
		}
	}
	return crc;
}


/**
 * @bieaf 计算 W25Q 中固件镜像的 CRC32
 *
 * @param size  镜像字节数
 * @param pcrc  输出：标准 CRC32
 * @return 0 成功，-1 读 W25Q 失败
 */
static int Ext_Crc(uint32_t size, uint32_t *pcrc) {
	uint8_t buf[256];
	uint32_t crc = 0xFFFFFFFFU;
	uint32_t off = 0;
	uint32_t n;

	while (off < size) {
		n = size - off;
		if (n > sizeof(buf)) {
			n = sizeof(buf);
		}
		if (W25Q_Read(OTA_DATA_ADDR + off, buf, n) != W25Q_OK) {
			return -1;
		}
		crc = Crc32_Update(crc, buf, n);
		off += n;
	}
	*pcrc = crc ^ 0xFFFFFFFFU;
	return 0;
}


/**
 * @bieaf 修改 OTA 头部状态字（NOR Flash 只能 1->0，所以不需要擦除）
 */
static void Ota_SetState(uint32_t state) {
	W25Q_Write(OTA_HDR_ADDR + offsetof(ota_hdr_t, state), (const uint8_t *)&state, sizeof(state));
}


/**
 * @bieaf 把 W25Q 中的固件镜像搬运到应用程序区
 * @detail 1.擦除应用程序区
 *         2.每次读 256 字节，写入内部 Flash（最后不足 4 字节的部分用 0xFF 补齐）
 *
 * @param size  镜像字节数
 * @return 0 成功，-1 读 W25Q 失败（此时应用程序区已被擦除，不能再跳转）
 */
static int CopyFromExt(uint32_t size) {
	uint32_t buf[64];   // 256 字节，放在栈上，不占用 RAM 静态空间
	uint32_t off = 0;
	uint32_t n;

	/*1.擦除目的地址*/
	printf("> Start erase app flash......\r\n");
	Erase_page(Application_1_Addr, (size + PageSize - 1) / PageSize);
	printf("> Erase app flash down......\r\n");

	/*2.开始拷贝*/
	printf("> Start copy from W25Q(ext flash) 0x%06x -> internal flash 0x%08x, size=%d\r\n",
		(unsigned int)OTA_DATA_ADDR, (unsigned int)Application_1_Addr, (int)size);
	while (off < size) {
		n = size - off;
		if (n > sizeof(buf)) {
			n = sizeof(buf);
		}
		memset(buf, 0xFF, sizeof(buf));
		if (W25Q_Read(OTA_DATA_ADDR + off, (uint8_t *)buf, n) != W25Q_OK) {
			return -1;
		}
		WriteFlash(Application_1_Addr + off, buf, (n + 3) / 4);
		off += n;
	}
	printf("> Copy down......\r\n");
	return 0;
}


/**
 * @bieaf 检查 W25Q 中是否有待升级的固件，有则校验并搬运
 * @detail 1.读 OTA 头部，magic 有效且状态为 PENDING 才升级
 *         2.先校验 W25Q 中镜像的 CRC32，通过后才开始擦内部 Flash
 *         3.搬运完成后再校验内部 Flash，通过后把状态改为 DONE
 *
 * @return 0 可以继续跳转 APP；-1 搬运失败，应用程序区已不可信，不能跳转
 */
static int Ota_Update(void) {
	ota_hdr_t hdr;
	uint32_t crc = 0;

	if (!W25Q_Init()) {
		printf("> W25Q not found, skip update\r\n");
		return 0;
	}
	if (W25Q_Read(OTA_HDR_ADDR, (uint8_t *)&hdr, sizeof(hdr)) != W25Q_OK) {
		printf("> W25Q read error, skip update\r\n");
		return 0;
	}
	if (hdr.magic != OTA_MAGIC || hdr.state != OTA_STATE_PENDING) {
		printf("> Normal start...\r\n");
		return 0;
	}

	printf("> Start update, size=%d\r\n", (int)hdr.size);
	if (hdr.size == 0 || hdr.size > Application_Size) {
		printf("> Error: bad image size\r\n");
		Ota_SetState(OTA_STATE_BAD);
		return 0;
	}

	/* 先校验 W25Q 中的镜像，通过后才动内部 Flash，镜像损坏时旧程序仍然可用 */
	if (Ext_Crc(hdr.size, &crc) != 0) {
		printf("> W25Q read error, skip update\r\n");
		return 0;
	}
	if (crc != hdr.crc) {
		printf("> Error: image CRC mismatch\r\n");
		Ota_SetState(OTA_STATE_BAD);
		return 0;
	}

	if (CopyFromExt(hdr.size) != 0) {
		printf("> Error: copy failed\r\n");
		return -1;
	}

	/* 搬运后再校验内部 Flash，失败则保持 PENDING，下次循环重新搬运 */
	crc = Crc32_Update(0xFFFFFFFFU, (const uint8_t *)Application_1_Addr, hdr.size) ^ 0xFFFFFFFFU;
	if (crc != hdr.crc) {
		printf("> Error: verify failed\r\n");
		return -1;
	}

	Ota_SetState(OTA_STATE_DONE);
	printf("> Update down......\r\n");
	return 0;
}



/* 采用汇编设置栈的值 */
__asm void MSR_MSP (uint32_t ulAddr) 
{
    MSR MSP, r0 			                   //set Main Stack value
    BX r14
}



/* 程序跳转函数 */
typedef void (*Jump_Fun)(void);
void IAP_ExecuteApp (uint32_t App_Addr)
{
	Jump_Fun JumpToApp; 
    
	if ( ( ( * ( __IO uint32_t * ) App_Addr ) & 0x2FFE0000 ) == 0x20000000 )	//检查栈顶地址是否合法.
	{ 
		JumpToApp = (Jump_Fun) * ( __IO uint32_t *)(App_Addr + 4);	//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP( * ( __IO uint32_t * ) App_Addr );	//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		JumpToApp();		//跳转到APP.
	}
}



/**
 * @bieaf 进行BootLoader的启动
 *
 * @param none
 * @return none
 */
void Start_BootLoader(void)
{
	int ret;

	/*==========打印消息==========*/  
	 printf("\r\n");
	 printf("***********************************\r\n");
	 printf("*        BootLoader               *\r\n");
	 printf("***********************************\r\n");

	ret = Ota_Update();		// 检查 W25Q 中是否有待升级的固件
	W25Q_DeInit();			// 关闭 SPI1，CS 保持高电平
	if (ret != 0) {
		printf("> Update failed, retry later...\r\n");
		return;				// 应用程序区不可信，不跳转，由 main 循环稍后重试
	}

	/* 跳转到应用程序 */
	printf("> Start APP...\r\n");	
	IAP_ExecuteApp(Application_1_Addr);
}
