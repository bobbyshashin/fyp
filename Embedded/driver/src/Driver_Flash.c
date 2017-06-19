#define FLASH_FILE

#include "Driver_Flash.h"

#define FLASH_ADDR                              0x080E0000UL
#define FLASH_SECTOR                            FLASH_Sector_11

static const uint32_t DATA_SIZE = sizeof(FLASH_DataTypeDef) / 4;
static uint32_t * const pData = (uint32_t*)&FLASH_Data;

void FLASH_Load(void) {
    for (uint32_t i = 0, addr = FLASH_ADDR; i < DATA_SIZE; ++i, addr += 4)
        pData[i] = *(volatile uint32_t *)addr;
}

void FLASH_Save(void) {
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);

    FLASH_EraseSector(FLASH_SECTOR, VoltageRange_3);

    for (uint32_t i = 0, addr = FLASH_ADDR; i < DATA_SIZE; ++i, addr += 4)
        FLASH_ProgramWord(addr, pData[i]);

    FLASH_Lock();
}
