#include "test.h"
#include "app.h"
#include "i2c_hal.h"

#define    AT24C02_ADDR_WRITE  0xA0
#define    AT24C02_ADDR_READ   0xA1

/**
 * @brief        AT24C02任意地址写一个字节数据
 * @param        addr —— 写数据的地址（0-255）
 * @param        dat  —— 存放写入数据的地址
 * @retval        成功 —— HAL_OK
*/
void At24c02_Write_Byte(uint16_t addr, uint8_t* data)
{
    I2CStart();
    I2CSendByte(AT24C02_ADDR_WRITE);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();
    I2CSendByte(*data);
    I2CWaitAck();
    I2CStop();
    HAL_Delay(5);
}

/**
 * @brief        AT24C02任意地址读一个字节数据
 * @param        addr —— 读数据的地址（0-255）
 * @param        read_buf —— 存放读取数据的地址
 * @retval        成功 —— HAL_OK
*/
uint8_t At24c02_Read_Byte(uint16_t addr, uint8_t* read_buf)
{
    I2CStart();
    I2CSendByte(0xa0);
    I2CWaitAck();
    I2CSendByte(addr);
    I2CWaitAck();

    I2CStart();
    I2CSendByte(0xa1);
    I2CWaitAck();
    *read_buf = I2CReceiveByte();
    I2CWaitAck();
    I2CStop();
}

void test_atc24c02(void){
    uint8_t source_a = 10;
    uint8_t a = 0;
    At24c02_Write_Byte(0x0000,&source_a);

    At24c02_Read_Byte(0x0000,&a);
    test_info("wirte addr:0x0000 -> 10\nread addr:0x0000 %d",a);
}

void testTask(void *arg){
    // ATC24C02 读写测试
    test_info("test start!");
    test_atc24c02();
    test_info("test end!");

    osThreadExit();
}
