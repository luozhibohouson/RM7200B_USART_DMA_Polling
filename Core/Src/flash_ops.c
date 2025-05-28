#include "main.h"
#include "flash_ops.h"
#include "platform.h"


/**
 * @brief 擦除Flash页
 */
uint8_t flash_erase_page(uint16_t flash_index)
{
    uint8_t status;

    uint32_t OffsetAddress = FLASH_START_ADDRESS + flash_index * FLASH_PAGE_SIZE;

    /* 解锁Flash */
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

    status = FLASH_ErasePage(OffsetAddress);

    FLASH_ClearFlag(FLASH_FLAG_EOP);

    /* 锁定Flash */
    FLASH_Lock();

    return status;
}

/**
 * @brief 写入半字到Flash
 */
uint8_t flash_write_halfword(uint32_t addr, uint16_t* data, uint16_t len)
{
    uint8_t status;

    /* 解锁Flash */
    FLASH_Unlock();

    /* 写入数据 */
    for(uint16_t i = 0; i < len; i+=2) {
        status = FLASH_ProgramHalfWord(addr+i, *data++);
        FLASH_ClearFlag(FLASH_FLAG_EOP);

        if(status != FLASH_COMPLETE) {
            break;
        }
    }

    /* 锁定Flash */
    FLASH_Lock();

    return status;
}

/**
 * @brief 写入一个字到Flash
 */
uint8_t flash_write_word(uint32_t addr, uint32_t* data, uint16_t len)
{
    uint8_t status;

    /* 解锁Flash */
    FLASH_Unlock();

    /* 写入数据 */
    for(uint16_t i = 0; i < len; i+=4) {
        status = FLASH_ProgramWord(addr+i, *data++);
        FLASH_ClearFlag(FLASH_FLAG_EOP);

        if(status != FLASH_COMPLETE) {
            break;
        }
    }

    /* 锁定Flash */
    FLASH_Lock();

    return status;
}

/**
 * @brief 从Flash读取多个字节
 */
uint8_t flash_read_bytes(uint32_t addr, uint8_t *data, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        data[i] = *(__IO uint8_t*)(addr + i);
    }

    return 0;
}

/**
 * @brief 验证Flash数据
 */
uint8_t flash_verify_bytes(uint32_t addr, uint8_t *data, uint16_t len)
{
    uint8_t read_data;

    for (uint16_t i = 0; i < len; i++) {
        read_data = *(__IO uint8_t*)(addr + i);
        if (read_data != data[i]) {
            return 1;
        }
    }

    return 0;
}

/**
 * @brief 验证Flash数据
 */
uint8_t flash_verify_halfword(uint32_t addr, uint16_t *data, uint16_t len)
{
    uint16_t read_data;

    for (uint16_t i = 0; i < len; i+=2) {
        read_data = *(__IO uint16_t*)(addr+i);
        if (read_data != data[i/2]) {
            return 1;
        }
    }

    return 0;
}


//void flash_test(void)
//{
//    uint8_t status;
//    uint16_t data = 0x1234;
//    uint8_t data2[2] = {0x34, 0x12};

//    status = flash_erase_page(APP_FLASH_PAGE_START);
//    if( status == FLASH_COMPLETE ) {
//        status = flash_write_halfword(APP_START_ADDR, &data, 2);
//        if( status == FLASH_COMPLETE ) {
//            uint8_t verify_status;
//            // verify_status = flash_verify_halfword(APP_START_ADDR, &data, 2);
//            verify_status = flash_verify_bytes(APP_START_ADDR, data2, 2);
//            __nop();
//        }
//    }

//    while(1);
//}
