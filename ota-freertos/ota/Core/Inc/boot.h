#ifndef __BOOT_H_
#define __BOOT_H_

#include "main.h"

#define PageSize FLASH_PAGE_SIZE // 1K

/*=====用户配置(根据自己的分区进行配置)=====*/
// 0x400    1K 
// 0x800    2K 
// 0x1000   4K  
// 0x2000   8K  
// 0xE000   56K
//
// Flash 分区（64K）:
//   0x08000000 ~ 0x08001FFF   BootLoader     8K
//   0x08002000 ~ 0x0800FFFF   应用程序       56K
// 新固件先存放在外部 W25Q32，由 BootLoader 搬运到应用程序区，
// 所以不再需要内部的 APP2 区和升级标志位。
// 修改分区时，Keil 的 Target 的 IROM1（Start=0x08002000, Size=Application_Size）也要同步修改。
#define BootLoader_Size         0x2000U         // BootLoader的大小 8K
#define Application_Size        0xE000U         // APP 应用程序的最大大小 56K

#define Application_1_Addr      0x08002000U     // 应用程序的首地址 0x08002000 + 0xE000
/*==========================================*/

#endif
