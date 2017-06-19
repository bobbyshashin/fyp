#include "BSP_DWT.h"
#include "Driver_Pinout.h"
#include "Driver_ST7735.h"
#include "font.h"

#include <stdio.h>
#include <stdarg.h>

static uint8_t CURR_XS = 0;
static uint8_t CURR_XE = MAX_WIDTH-1;
static uint8_t CURR_YS = 0;
static uint8_t CURR_YE = MAX_HEIGHT-1;
static LCD_OrientationTypeDef CURR_ORIENTATION = kNormal;

// internal functions
static void ST7735_SetPixelPos(uint8_t x, uint8_t y);
static void ST7735_UpdateActiveRegion(void);

static void ST7735_WriteCmd(uint8_t cmd);
static void ST7735_WriteData(uint8_t data);
static void ST7735_Reset(void);
static void ST7735_Startup(void);

void ST7735_Init(void) {
    // reset st7735r
    ST7735_Reset();

    // send startup command
    ST7735_Startup();
}

// fill the screen
void ST7735_FillColor(uint16_t color) {
    ST7735_WriteCmd(0x2A);     // Column addr set
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);    // X START
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x7F);    // X END

    ST7735_WriteCmd(0x2B);     // Row addr set
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);    // Y START
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x9F);    // Y END

    ST7735_WriteCmd(0x2C);     // write to RAM
    
    for (uint16_t i = 0; i < MAX_WIDTH*MAX_HEIGHT; ++i) {
        ST7735_WriteData(color >> 8);
        ST7735_WriteData(color);
    }
}

// (x, y) is global coordinate, not relative to current active region
void ST7735_PutPixel(uint8_t x, uint8_t y, uint16_t color) {
    ST7735_SetPixelPos(x, y);
    ST7735_WriteData(color >> 8);
    ST7735_WriteData(color);
}

void ST7735_SetPixelPos(uint8_t x, uint8_t y) {
    uint8_t realX = 0, realY = 0;
    switch (CURR_ORIENTATION) {
        case kNormal: realX = x, realY = y; break;
        case kRevert: realX = MAX_WIDTH-1-x, realY = MAX_HEIGHT-1-y; break;
    }
    ST7735_WriteCmd(0x2A);     // Column addr set
    ST7735_WriteData(0x00);
    ST7735_WriteData(realX);       // X START
    ST7735_WriteData(0x00);
    ST7735_WriteData(realX);       // X END

    ST7735_WriteCmd(0x2B);     // Row addr set
    ST7735_WriteData(0x00);
    ST7735_WriteData(realY);       // Y START
    ST7735_WriteData(0x00);
    ST7735_WriteData(realY);       // Y END

    ST7735_WriteCmd(0x2C);     // write to RAM
}

void ST7735_PlotImg(uint16_t color_f, uint16_t color_t, uint8_t* data, uint32_t len) {
    ST7735_WriteCmd(0x2C);     // write to RAM

    int32_t is, ie;
    int8_t js, je, di, dj;

    switch (CURR_ORIENTATION) {
        case kNormal:
        is = js = 0;
        di = dj = 1;
        ie = len;
        je = 8;
        break;
        case kRevert:
        ie = je = -1;
        di = dj = -1;
        is = len-1;
        js = 7;
        break;
    }

    for (int32_t i = is; i != ie; i += di) {
        for (int8_t j = js; j != je; j += dj) {
            if (!((data[i]>>(7-j))&1u)) {
                ST7735_WriteData(color_f >> 8);
                ST7735_WriteData(color_f);
            }
            else {
                ST7735_WriteData(color_t >> 8);
                ST7735_WriteData(color_t);
            }
        }
    }
}

// fill the current active region
void ST7735_FillRegion(uint16_t color) {
    ST7735_WriteCmd(0x2C);     // write to RAM

    uint8_t w = CURR_XE-CURR_XS+1;
    uint8_t h = CURR_YE-CURR_YS+1;
    
    for (uint16_t i = 0; i < w*h; i++) {
        ST7735_WriteData(color >> 8);
        ST7735_WriteData(color);
    }
}

void ST7735_SetActiveRegion(uint8_t xs, uint8_t xe, uint8_t ys, uint8_t ye) {
    if (xe >= MAX_WIDTH) xe = MAX_WIDTH-1;
    if (ye >= MAX_HEIGHT) ye = MAX_HEIGHT-1;
    if (xs > xe) xs = xe;
    if (ys > ye) ys = ye;

    CURR_XS = xs;
    CURR_XE = xe;
    CURR_YS = ys;
    CURR_YE = ye;

    ST7735_UpdateActiveRegion();
}

void ST7735_SetOrientation(LCD_OrientationTypeDef orientation) {
    CURR_ORIENTATION = orientation;
}

// (x, y) is global
void ST7735_PutChar(uint8_t x, uint8_t y, uint8_t ch, uint16_t textColor, uint16_t bgColor) {
    // record last active region
    uint8_t last_xs = CURR_XS;
    uint8_t last_xe = CURR_XE;
    uint8_t last_ys = CURR_YS;
    uint8_t last_ye = CURR_YE;

    // set new active region
    if (x >= CHAR_MAX_X) x = CHAR_MAX_X-1;
    if (y >= CHAR_MAX_Y) y = CHAR_MAX_Y-1;
    uint16_t xs = x*CHAR_WIDTH;
    uint16_t ys = y*CHAR_HEIGHT;
    ST7735_SetActiveRegion(xs, xs+CHAR_WIDTH-1, ys, ys+CHAR_HEIGHT-1);
    uint32_t index = (uint32_t)ch;
    index <<= 4;
    ST7735_PlotImg(bgColor, textColor, (uint8_t*)&(ascii_8x16[index]), CHAR_HEIGHT);

    // set active region to original
    ST7735_SetActiveRegion(last_xs, last_xe, last_ys, last_ye);
}

// print a const str on a line starting from a point, and clear the rest of the line
void ST7735_PutLine(uint8_t x, uint8_t y, uint8_t* str, uint16_t textColor, uint16_t bgColor) {
    uint8_t i = x, j = 0;
    while (i < CHAR_MAX_X) {
        if (str[j]) {
            ST7735_PutChar(i, y, str[j], textColor, bgColor);
            j++;
        }
        else {
            ST7735_PutChar(i, y, ' ', textColor, bgColor);
        }
        i++;
    }
}

// printf like function, only one line, the rest are truncated
void ST7735_Print(uint8_t x, uint8_t y, uint16_t textColor, uint16_t bgColor, const uint8_t* str, ...){
    uint8_t buffer[32];

    va_list arglist;
    va_start(arglist, str);
    vsprintf((char*)buffer, (const char*)str, arglist);
    va_end(arglist);

    ST7735_PutLine(x, y, buffer, textColor, bgColor);
}

void ST7735_UpdateActiveRegion(void) {
    uint8_t xs = 0, xe = 0, ys = 0, ye = 0;
    switch (CURR_ORIENTATION) {
        case kNormal:
        xs = CURR_XS;
        xe = CURR_XE;
        ys = CURR_YS;
        ye = CURR_YE;
        break;
        case kRevert:
        xe = MAX_WIDTH-1-CURR_XS;
        xs = MAX_WIDTH-1-CURR_XE;
        ye = MAX_HEIGHT-1-CURR_YS;
        ys = MAX_HEIGHT-1-CURR_YE;
        break;
    }
    ST7735_WriteCmd(0x2A);         // Column addr set
    ST7735_WriteData(0x00);
    ST7735_WriteData(xs);     // X START
    ST7735_WriteData(0x00);
    ST7735_WriteData(xe);     // X END

    ST7735_WriteCmd(0x2B);         // Row addr set
    ST7735_WriteData(0x00);
    ST7735_WriteData(ys);     // Y START
    ST7735_WriteData(0x00);
    ST7735_WriteData(ye);     // Y END
}

static void ST7735_WriteCmd(uint8_t cmd) {
    // set DC to cmd
    GPIO_ResetBits(ST7735_CS_PORT, ST7735_CS_PIN);
    GPIO_ResetBits(ST7735_DC_PORT, ST7735_DC_PIN);

    // send cmd
    while (SPI_I2S_GetFlagStatus(ST7735_SPI, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(ST7735_SPI, cmd);
    while (SPI_I2S_GetFlagStatus(ST7735_SPI, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    SPI_I2S_ReceiveData(ST7735_SPI);

    GPIO_SetBits(ST7735_CS_PORT, ST7735_CS_PIN);
}

static void ST7735_WriteData(uint8_t data) {
    // set DC to data
    GPIO_ResetBits(ST7735_CS_PORT, ST7735_CS_PIN);
    GPIO_SetBits(ST7735_DC_PORT, ST7735_DC_PIN);

    // send cmd
    while (SPI_I2S_GetFlagStatus(ST7735_SPI, SPI_I2S_FLAG_TXE) == RESET)
        ;
    SPI_I2S_SendData(ST7735_SPI, data);
    while (SPI_I2S_GetFlagStatus(ST7735_SPI, SPI_I2S_FLAG_RXNE) == RESET)
        ;
    SPI_I2S_ReceiveData(ST7735_SPI);

    GPIO_SetBits(ST7735_CS_PORT, ST7735_CS_PIN);
}

static void ST7735_Reset(void) {
    GPIO_ResetBits(ST7735_RST_PORT, ST7735_RST_PIN);
    BSP_DWT_DelayMs(1);
    GPIO_SetBits(ST7735_RST_PORT, ST7735_RST_PIN);
    BSP_DWT_DelayMs(1);
}

static void ST7735_Startup(void) {
    ST7735_WriteCmd(0x01); // Software setting
    ST7735_WriteCmd(0x11); // Sleep out
    
    // ST7735R Frame Rate
    ST7735_WriteCmd(0xB1);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    ST7735_WriteCmd(0xB2);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    ST7735_WriteCmd(0xB3);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x2D);
    //------------------------------------End ST7735R Frame Rate-----------------------------------------//
    ST7735_WriteCmd(0xB4); // Column inversion
    ST7735_WriteData(0x07);
    //------------------------------------ST7735R Power Sequence-----------------------------------------//
    ST7735_WriteCmd(0xC0);
    ST7735_WriteData(0xA2);
    ST7735_WriteData(0x02);
    ST7735_WriteData(0x84);
    ST7735_WriteCmd(0xC1);
    ST7735_WriteData(0xC5);
    ST7735_WriteCmd(0xC2);
    ST7735_WriteData(0x0A);
    ST7735_WriteData(0x00);
    ST7735_WriteCmd(0xC3);
    ST7735_WriteData(0x8A);
    ST7735_WriteData(0x2A);
    ST7735_WriteCmd(0xC4);
    ST7735_WriteData(0x8A);
    ST7735_WriteData(0xEE);
    //---------------------------------End ST7735R Power Sequence-------------------------------------//
    ST7735_WriteCmd(0xC5); // VCOM
    ST7735_WriteData(0x0E);
    ST7735_WriteCmd(0x36); // MX, MY, RGB mode
    ST7735_WriteData(0xC8);
    //------------------------------------ST7735R Gamma Sequence-----------------------------------------//
    ST7735_WriteCmd(0xE0);
    ST7735_WriteData(0x02);
    ST7735_WriteData(0x1C);
    ST7735_WriteData(0x07);
    ST7735_WriteData(0x12);
    ST7735_WriteData(0x37);
    ST7735_WriteData(0x32);
    ST7735_WriteData(0x29);
    ST7735_WriteData(0x2D);
    ST7735_WriteData(0x29);
    ST7735_WriteData(0x25);
    ST7735_WriteData(0x2B);
    ST7735_WriteData(0x39);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x01);
    ST7735_WriteData(0x03);
    ST7735_WriteData(0x10);
    ST7735_WriteCmd(0xE1);
    ST7735_WriteData(0x03);
    ST7735_WriteData(0x1D);
    ST7735_WriteData(0x07);
    ST7735_WriteData(0x06);
    ST7735_WriteData(0x2E);
    ST7735_WriteData(0x2C);
    ST7735_WriteData(0x29);
    ST7735_WriteData(0x2D);
    ST7735_WriteData(0x2E);
    ST7735_WriteData(0x2E);
    ST7735_WriteData(0x37);
    ST7735_WriteData(0x3F);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x02);
    ST7735_WriteData(0x10);
    ST7735_WriteCmd(0x2A);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x7F);

    ST7735_WriteCmd(0x2B);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x00);
    ST7735_WriteData(0x9F);
    //------------------------------------End ST7735R Gamma Sequence-----------------------------------------//

    ST7735_WriteCmd(0x3A);
    ST7735_WriteData(0x05);  
    ST7735_WriteCmd(0x29); // Display on
}
