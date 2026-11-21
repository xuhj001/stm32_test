#include <stdio.h>
#include "aht20.h"
#include "i2c.h"
#include "oled.h"
#include "modbus.h"

float RH;		// 湿度，转换单位后的实际值，标准单位%
float Temp;		// 温度，转换单位后的实际值，标准单位°C

// 测量采用非阻塞状态机：
//   AHT20_Read() 触发一次测量并记录时间
//   AHT20_Poll() 主循环每轮调用，测量满 80ms 后读取结果并写入 REG_HOLD_BUF
#define AHT20_MEASURE_MS     80   // 触发测量后需要等待的时间（ms）
#define AHT20_MAX_BUSY_POLL  10   // 读到忙状态时最多重试的次数

static uint8_t  aht20Busy = 0;       // 1：已触发测量，等待结果
static uint32_t aht20StartTick = 0;  // 触发测量时的时间戳（ms）
static uint8_t  aht20BusyCnt = 0;    // 读到忙状态的次数

//读取AHT20的状态寄存器
uint8_t AHT20_Read_Status(void)
{	
	uint8_t tmp[1];
	HAL_I2C_Master_Receive(&hi2c1, ATH20_SLAVE_ADDRESS, tmp, 1, 1000);
	return tmp[0];
}

// AHT20 触发测量命令
static void AHT20_TrigMeasureCmd(void)
{
	uint8_t tmp[3];
	tmp[0] = 0xac; // AHT20_TrigMeasure_REG	0xAC	//触发测量 寄存器地址
	tmp[1] = 0x33;
	tmp[2] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, ATH20_SLAVE_ADDRESS, tmp, 3, 1000);
}


//重新初始化AHT20
void AHT20_DeInit(void)
{	
	uint8_t tmp[3];
	tmp[0] = 0xbe; //AHT20_INIT_REG				0xBE	//初始化 寄存器地址
	tmp[1] = 0x08;
	tmp[2] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, ATH20_SLAVE_ADDRESS, tmp, 3, 1000);
}

//初始化AHT20（只在上电时调用一次，允许使用阻塞延时）
void AHT20_Init(void)
{	
	HAL_Delay(40);	//刚上电，延时40ms才可以读取状态
	
	uint8_t tmp[3];
	tmp[0] = 0xbe; //AHT20_INIT_REG				0xBE	//初始化 寄存器地址
	tmp[1] = 0x08;
	tmp[2] = 0x00;
	HAL_I2C_Master_Transmit(&hi2c1, ATH20_SLAVE_ADDRESS, tmp, 3, 1000);
	
	if(!((AHT20_Read_Status()&0x08)==0x08))
	{
		AHT20_DeInit(); //重新初始化AHT20 
	}
}

// 第 1 步：触发测量（不等待）。上一次测量还没读完时直接跳过。
void AHT20_Read(void)
{
	if (aht20Busy) {
		return;
	}
	AHT20_TrigMeasureCmd();          // 向AHT20发送AC命令
	aht20StartTick = HAL_GetTick();
	aht20BusyCnt = 0;
	aht20Busy = 1;
}

// 第 2 步：主循环每轮调用。测量时间到了再读取结果，不阻塞。
void AHT20_Poll(void)
{
	uint8_t tmp[6];
	uint32_t raw;

	if (!aht20Busy) {
		return;
	}
	if ((HAL_GetTick() - aht20StartTick) < AHT20_MEASURE_MS) {
		return;                      // 还没到 80ms，下一轮再看
	}

	if (AHT20_Read_Status() & 0x80) {   // bit7=1 表示仍在测量中
		if (++aht20BusyCnt >= AHT20_MAX_BUSY_POLL) {
			aht20Busy = 0;           // 放弃本次，保留上一次的数据
		}
		return;
	}

	HAL_I2C_Master_Receive(&hi2c1, ATH20_SLAVE_ADDRESS, tmp, 6, 1000);
	aht20Busy = 0;

	// 相对湿度 RH：原始值 20 位，等同于 (tmp[1]<<16 | tmp[2]<<8 | tmp[3]) >> 4
	raw = ((uint32_t)tmp[1] << 12) | ((uint32_t)tmp[2] << 4) | ((uint32_t)tmp[3] >> 4);
	RH = raw * 100.0 / 1024 / 1024;

	// 温度 T：原始值 20 位，取 tmp[3] 的低 4 位作为高位
	raw = (((uint32_t)tmp[3] & 0x0F) << 16) | ((uint32_t)tmp[4] << 8) | (uint32_t)tmp[5];
	Temp = raw * 200.0 / 1024 / 1024 - 50;

	REG_HOLD_BUF[1] = Temp * 100;
	REG_HOLD_BUF[2] = RH * 100;
}
