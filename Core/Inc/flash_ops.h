#ifndef __FLASH_OPS_H
#define __FLASH_OPS_H

#include "platform.h"

/* Flash起始地址 */
#define FLASH_START_ADDRESS         ((uint32_t)0x08000000)
/* Flash页大小 */
#define FLASH_PAGE_SIZE             (1024)
/* Flash大小 */
#define FLASH_SIZE                  (32*1024)
/* Flash页数 */
#define FLASH_PAGE_NUM_MAX          ((FLASH_SIZE / FLASH_PAGE_SIZE) - 1)

/* Flash操作函数声明 */
uint8_t flash_erase_page(uint16_t flash_index);
uint8_t flash_write_halfword(uint32_t addr, uint16_t* data, uint16_t len);
uint8_t flash_write_word(uint32_t addr, uint32_t* data, uint16_t len);
uint8_t flash_read_bytes(uint32_t addr, uint8_t *data, uint16_t len);
uint8_t flash_verify_bytes(uint32_t addr, uint8_t *data, uint16_t len);
uint8_t flash_verify_halfword(uint32_t addr, uint16_t *data, uint16_t len);

void flash_test(void);

#endif /* __FLASH_OPS_H */

