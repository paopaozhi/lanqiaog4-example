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
uint8_t At24c02_Read_Byte(uint8_t addr, uint8_t* read_buf)
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

    return HAL_OK;
}

uint8_t x24c02_read(uint8_t address)
{
    unsigned char val;

    I2CStart();
    I2CSendByte(0xa0);
    if(I2CWaitAck() == ERROR){ goto ERROR;}

    I2CSendByte(address);
    if(I2CWaitAck() == ERROR){ goto ERROR;}

    I2CStart();
    I2CSendByte(0xa1);
    if(I2CWaitAck() == ERROR){ goto ERROR;}
    val = I2CReceiveByte();
    if(I2CWaitAck() == ERROR){ goto ERROR;}
    I2CStop();

    return(val);

    ERROR:
    test_info("wait error!");
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();


    /*Configure GPIO pins : PB6 PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);

}

void test_atc24c02(void){
    MX_GPIO_Init();

    a = x24c02_read(0);
    test_info("write addr:0x0000 -> %d",source_a);
    test_info("re\n"
              "    I2CInit();\n"
              "\n"
              "    osDelay(500);\n"
              "\n"
              "    uint8_t source_a = 10;\n"
              "    uint8_t a = 0;\n"
              "//    At24c02_Write_Byte(0x0000,&source_a);\n"
              "\n"
              "//    At24c02_Read_Byte(0x00,&a);ad  addr:0x0000 -> %d",a);
}

void testTask(void *arg){
    // ATC24C02 读写测试
    test_info("test start!");
    test_atc24c02();
    test_info("test end!");

    osThreadExit();
}
