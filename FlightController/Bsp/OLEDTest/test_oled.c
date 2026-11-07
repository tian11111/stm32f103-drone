/* Build the unchanged font data separately with its existing initializer warnings:
 * gcc -std=c11 -Wall -IBsp -c Bsp/OLED_Data.c -o build/Debug/oled_font_test.o
 * gcc -std=c11 -Wall -Wextra -Werror -ffunction-sections -fdata-sections
 * -IBsp/OLEDTest -IBsp Bsp/OLEDTest/test_oled.c Bsp/OLED.c build/Debug/oled_font_test.o
 * -Wl,--gc-sections -o build/Debug/test_oled.exe */
#include "i2c.h"
#include "OLED.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

extern uint8_t OLED_DisplayBuf[8][128];
I2C_HandleTypeDef hi2c1, hi2c2;
uint32_t test_busy;
static uint8_t *sent;
static uint16_t sent_length;
static unsigned starts;
static unsigned reinits;
static HAL_StatusTypeDef start_result = HAL_OK;
HAL_StatusTypeDef HAL_I2C_Init(I2C_HandleTypeDef *h)
{
    assert(h == &hi2c1);
    reinits++;
    h->State = HAL_I2C_STATE_READY;
    h->ErrorCode = 0;
    return HAL_OK;
}
HAL_StatusTypeDef HAL_I2C_Master_Transmit_IT(I2C_HandleTypeDef *h, uint16_t addr, uint8_t *p, uint16_t n)
{
    assert(h == &hi2c1 && addr == 0x78 && !test_busy);
    sent = p; sent_length = n; starts++;
    if (start_result == HAL_OK) h->State = 1;
    return start_result;
}
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *h, uint16_t a, uint16_t m, uint16_t z, uint8_t *p, uint16_t n, uint32_t t)
{ (void)h; (void)a; (void)m; (void)z; (void)p; (void)n; (void)t; return HAL_OK; }
void HAL_Delay(uint32_t t) { (void)t; }
static void complete(void) { hi2c1.State = HAL_I2C_STATE_READY; HAL_I2C_MasterTxCpltCallback(&hi2c1); }
int main(void)
{
    for (unsigned page = 0; page < 8; page++) memset(OLED_DisplayBuf[page], page + 1, 128);
    OLED_UpdateAsync();
    memset(OLED_DisplayBuf, 0xee, sizeof OLED_DisplayBuf);
    OLED_UpdateAsync(); /* Cannot replace the active snapshot. */
    test_busy = 1; OLED_Task(); assert(starts == 0);
    test_busy = 0; hi2c1.State = 1; OLED_Task(); assert(starts == 0);
    hi2c1.State = HAL_I2C_STATE_READY;
    for (unsigned page = 0; page < 8; page++) {
        OLED_Task(); assert(starts == page * 2 + 1 && sent_length == 4);
        assert(sent[0] == 0 && sent[1] == 0xb0 + page && sent[2] == 0 && sent[3] == 0x10);
        OLED_Task(); assert(starts == page * 2 + 1);
        HAL_I2C_MasterTxCpltCallback(&hi2c2); OLED_Task(); assert(starts == page * 2 + 1);
        complete(); test_busy = 1; OLED_Task(); assert(starts == page * 2 + 1); test_busy = 0;
        OLED_Task(); assert(starts == page * 2 + 2 && sent_length == 129 && sent[0] == 0x40);
        for (unsigned col = 1; col <= 128; col++) assert(sent[col] == page + 1);
        complete();
    }
    OLED_Task(); assert(starts == 16);
    OLED_UpdateAsync(); OLED_Task(); assert(starts == 17);
    HAL_I2C_ErrorCallback(&hi2c2); complete(); OLED_Task(); assert(starts == 18);
    hi2c1.State = HAL_I2C_STATE_READY; HAL_I2C_ErrorCallback(&hi2c1);
    OLED_Task(); assert(starts == 18);
    OLED_UpdateAsync(); OLED_Task(); assert(starts == 19 && sent[1] == 0xb0);
    hi2c1.State = HAL_I2C_STATE_READY; HAL_I2C_ErrorCallback(&hi2c1);
    start_result = HAL_ERROR; OLED_UpdateAsync(); OLED_Task(); assert(starts == 20);
    OLED_Task(); assert(starts == 20);
    start_result = HAL_OK; OLED_UpdateAsync(); OLED_Task(); assert(starts == 21);
    hi2c1.State = HAL_I2C_STATE_READY;
    hi2c1.ErrorCode = HAL_I2C_ERROR_BERR;
    HAL_I2C_ErrorCallback(&hi2c1);
    OLED_UpdateAsync(); OLED_Task();
    assert(reinits == 1 && starts == 21); /* Recovery is deferred to the main loop. */
    OLED_UpdateAsync(); OLED_Task(); assert(starts == 22 && sent[1] == 0xb0);
    puts("PASS: OLED async snapshot, pages, busy and error recovery");
    return 0;
}
