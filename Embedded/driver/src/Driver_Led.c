#include "stm32f4xx.h"
#include "Driver_Pinout.h"
#include "Driver_Led.h"

void LED_On(void) {
    GPIO_SetBits(LED_PORT, LED_PIN);
}

void LED_Off(void) {
    GPIO_ResetBits(LED_PORT, LED_PIN);
}

void LED_Toggle(void) {
    GPIO_ToggleBits(LED_PORT, LED_PIN);
}
