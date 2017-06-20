#include "Driver_Common.h"
#include "Driver_Menu.h"
#include "Driver_Pinout.h"
#include "Driver_ST7735.h"
#include "external.h"
#include <cstdio>
#include <cstring>
#include <cstdlib>

#define DEF_FG_COLOR GREEN
#define DEF_BG_COLOR BLACK
#define FG_COLOR(i) (color[i]?DEF_BG_COLOR:DEF_FG_COLOR)
#define BG_COLOR(i) (color[i]?DEF_FG_COLOR:DEF_BG_COLOR)

#define JU 0
#define JD 1
#define JL 2
#define JR 3
#define JC 4
#define PRESS_THRES 3

static volatile int8_t color[CHAR_MAX_Y]; // 0: +ve, 1: -ve
static uint8_t ch[CHAR_MAX_Y][CHAR_MAX_X+1];

static MenuList_t HomeList, ParamList, StatusList;

static MenuItem_t HomeItems[] = {
  {2, "Param", {0, 0, &ParamList}, {0, 0}},
  {2, "Status", {0, 0, &StatusList}, {0, 0}},
};
static MenuItem_t ParamItems[] = {
  {1, "vxscl", {0, 1.0f, 0}, {0, 0.01f}, &chassis_vx_scale},
  {1, "vyscl", {0, 1.0f, 0}, {0, 0.01f}, &chassis_vy_scale},
};
static MenuItem_t StatusItems[] = {
  {4, "vxscl", {0, 0, 0}, {0, 0}, &chassis_vx_scale},
  {4, "vyscl", {0, 0, 0}, {0, 0}, &chassis_vy_scale},
  {3, "JCNT", {0, 0, 0}, {0, 0}, &ex_JudgeFrameCounter},
  {4, "tx_vx", {0, 0, 0}, {0, 0}, &tx_vx},
  {4, "tx_vy", {0, 0, 0}, {0, 0}, &tx_vy},
  {4, "rx_vx", {0, 0, 0}, {0, 0}, &rx_vx},
  {4, "rx_vy", {0, 0, 0}, {0, 0}, &rx_vy},
  {4, "sp0", {0, 0, 0}, {0, 0}, ex_Speed+0},
  {4, "sp1", {0, 0, 0}, {0, 0}, ex_Speed+1},
  {4, "sp2", {0, 0, 0}, {0, 0}, ex_Speed+2},
  {4, "sp3", {0, 0, 0}, {0, 0}, ex_Speed+3},
};
static MenuList_t HomeList = {"Bobby", COUNT_OF(HomeItems), HomeItems, 0, 0};
static MenuList_t ParamList = {"Parameters", COUNT_OF(ParamItems), ParamItems, 0, &HomeList};
static MenuList_t StatusList = {"Status", COUNT_OF(StatusItems), StatusItems, 0, &HomeList};
static MenuList_t *currList;
static MenuItem_t *currItem;

static int16_t pressCount[5], pressState[5];

void MENU_UpdateData(void) {
}

void MENU_Init(void) {
  memset((char*)color, 0, sizeof(color));
  memset(ch, 0, sizeof(ch));
  memset(pressCount, 0, sizeof(pressCount));
  memset(pressState, 0, sizeof(pressState));

  currList = &HomeList;
  currItem = 0;
  MENU_UpdateList();
  MENU_UpdateScreen();
}

void MENU_Update(void) {
  if (GPIO_ReadInputDataBit(JS_PORT, JS_U) == RESET) ++pressCount[JU];
  else pressCount[JU] = 0;
  if (GPIO_ReadInputDataBit(JS_PORT, JS_D) == RESET) ++pressCount[JD];
  else pressCount[JD] = 0;
  if (GPIO_ReadInputDataBit(JS_PORT, JS_L) == RESET) ++pressCount[JL];
  else pressCount[JL] = 0;
  if (GPIO_ReadInputDataBit(JS_PORT, JS_R) == RESET) ++pressCount[JR];
  else pressCount[JR] = 0;
  if (GPIO_ReadInputDataBit(JS_PORT, JS_C) == RESET) ++pressCount[JC];
  else pressCount[JC] = 0;

  for (int8_t i = 0; i < 5; ++i)
    if (pressCount[i] > PRESS_THRES) pressState[i] = 1, pressCount[i] = 0;
    else if (pressCount[i] == 0) pressState[i] = 0;

  if (pressState[JU]) {
    pressState[JU] = 0;
    if (currItem) {
      switch (currItem->type) {
        case 0:
          currItem->data.i += currItem->delta.i;
          if (currItem->ptr)
            *(int32_t *)(currItem->ptr) = currItem->data.i;
          break;
        case 1:
          currItem->data.f += currItem->delta.f;
          if (currItem->ptr)
            *(float *)(currItem->ptr) = currItem->data.f;
          break;
      }
    }
    else {
      currList->curr -= 1;
      if (currList->curr < 0)
        currList->curr = currList->len - 1;
    }
  }
  else if (pressState[JD]) {
    pressState[JD] = 0;
    if (currItem) {
      switch (currItem->type) {
        case 0:
          currItem->data.i -= currItem->delta.i;
          if (currItem->ptr)
            *(int32_t *)(currItem->ptr) = currItem->data.i;
          break;
        case 1:
          currItem->data.f -= currItem->delta.f;
          if (currItem->ptr)
            *(float *)(currItem->ptr) = currItem->data.f;
          break;
      }
    }
    else {
      currList->curr += 1;
      if (currList->curr >= currList->len)
        currList->curr = 0;
    }
  }
  else if (pressState[JL]) {
    pressState[JL] = 0;
    if (currItem)
      currItem = 0;
    else if (currList->parent)
      currList = currList->parent;
  }
  else if (pressState[JR]) {
    pressState[JR] = 0;
    if (currList->items[currList->curr].type == 2)
      currList = currList->items[currList->curr].data.child;
    else
      currItem = &(currList->items[currList->curr]);
  }
  else if (pressState[JC]) {
    pressState[JC] = 0;
    JC_Handler();
    /*if (currItem) {*/
      /*switch (currItem->type) {*/
        /*case 0:*/
          /**(int32_t *)(currItem->ptr) = currItem->data.i;*/
          /*break;*/
        /*case 1:*/
          /**(float *)(currItem->ptr) = currItem->data.f;*/
          /*break;*/
      /*}*/
    /*}*/
  }
  MENU_UpdateList();
  MENU_UpdateScreen();
}

/*
 * Special function goes here ..
 */
void JC_Handler(void) {
}

void MENU_UpdateScreen(void) {
  int yy = -1;
  if (currItem)
    yy = 1 + currList->curr % (CHAR_MAX_Y - 1);
  for (int16_t y = 0; y < CHAR_MAX_Y; ++y)
    if (y == yy) MENU_UpdateLine(yy);
    else ST7735_Print(0, y, FG_COLOR(y), BG_COLOR(y), ch[y]);
}

void MENU_UpdateLine(int16_t y) {
  int8_t end = 0;
  for (int16_t x = 0; x < CHAR_MAX_X; ++x) {
    if (end)
      ST7735_PutChar(x, y, ' ', FG_COLOR(y), BG_COLOR(y));
    else if (x < 6) 
      ST7735_PutChar(x, y, ch[y][x], BG_COLOR(y), FG_COLOR(y));
    else
      ST7735_PutChar(x, y, ch[y][x], FG_COLOR(y), BG_COLOR(y));
  }
}

void MENU_UpdateList(void) {
  sprintf((char*)ch[0], currList->name);
  int16_t startItem = (currList->curr / (CHAR_MAX_Y-1)) * (CHAR_MAX_Y-1);
  for (int16_t y = 0; y < CHAR_MAX_Y-1; ++y) {
    int16_t yy = startItem + y;
    if (yy < currList->len) {
      MenuItem_t *item = &(currList->items[yy]);
      switch (item->type) {
        case 0:
          sprintf((char*)ch[y+1], "%-6s %9d", item->name, item->data.i = *(int32_t*)(item->ptr));
          break;
        case 1:
          sprintf((char*)ch[y+1], "%-6s %9.4f", item->name, item->data.f = *(float*)(item->ptr));
          break;
        case 2:
          sprintf((char*)ch[y+1], "%s", item->name);
          break;
        case 3:
          sprintf((char*)ch[y+1], "%-6s %9d", item->name, *(const int32_t *)item->ptr);
          break;
        case 4:
          sprintf((char*)ch[y+1], "%-6s %9.4f", item->name, *(const float *)item->ptr);
          break;
      }
    }
    else
      sprintf((char*)ch[y+1], "");
  }
  memset((char*)color, 0, sizeof(color));
  int16_t line = 1+(currList->curr % (CHAR_MAX_Y-1));
  color[line] = 1;
}

