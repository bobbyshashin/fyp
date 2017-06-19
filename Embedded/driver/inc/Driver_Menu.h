#ifndef DRIVER_MENU_H
#define DRIVER_MENU_H

#include "stm32f4xx.h"

struct _MenuList_t;

typedef struct {
  /*
   * 0 - integer
   * 1 - float
   * 2 - sublist
   * 3 - external 32bit interger
   * 4 - external float
   */
  uint8_t type;
  char name[7];
  struct {
    int32_t i;
    float f;
    struct _MenuList_t *child;
  } data;
  struct {
    int32_t i;
    float f;
  } delta;
  void *ptr;
} MenuItem_t;

typedef struct _MenuList_t {
  char name[20];
  int8_t len;
  MenuItem_t *items;
  int8_t curr;
  struct _MenuList_t *parent;
} MenuList_t;

void MENU_Init(void);
void MENU_Update(void);

// helper functions, internal use only
void MENU_UpdateScreen(void);
void MENU_UpdateLine(int16_t y);
void MENU_UpdateList(void);
void MENU_UpdateItem(void);

void JC_Handler(void);

#endif
