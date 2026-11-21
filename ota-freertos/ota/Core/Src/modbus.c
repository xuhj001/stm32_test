#include "modbus.h"
#include <stdint.h>
#include <stdio.h>
#include "gpio.h"
#include "aht20.h"
#include "ina226.h"
#include "adc.h"
#include "i2c.h"

// 十路保持寄存器  可读可写的2字节寄存器
#define REG_HOLD_SIZE   10
uint16_t REG_HOLD_BUF[REG_HOLD_SIZE];

// 传感器任务每 500ms 调用一次：触发温湿度测量，刷新 INA226、电位器和 CPU 温度
void Modbus_Refresh(void) {
	I2C1_Lock();
	AHT20_Read();       // 只触发测量，结果由 Modbus_Parse 里的 AHT20_Poll 读取
	INA226_Read();
	I2C1_Unlock();
	ADC_VR_CPU_Read();
	// 状态指示灯 
	HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);	
}

// 传感器任务每轮调用：读取已完成的温湿度测量，并按 REG_HOLD_BUF[0] 刷新输出
void Modbus_Parse(void) {
	I2C1_Lock();
	AHT20_Poll();
	I2C1_Unlock();
	
	if(REG_HOLD_BUF[0] & LED1_CMD) {
		LED_Control(LED1,ON);
	} else {
		LED_Control(LED1,OFF);
	}
	if(REG_HOLD_BUF[0] & BEEP_CMD) {
		BEEP_Control(ON);
	} else {
		BEEP_Control(OFF);
	}
	if(REG_HOLD_BUF[0] & RELAY_CMD) {
		RELAY_Control(ON);
	} else {
		RELAY_Control(OFF);
	}
}
