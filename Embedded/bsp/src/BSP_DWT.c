#include "BSP_DWT.h"
#include "stm32f4xx.h"

void BSP_DWT_InitConfig(void) {
    /* enable DEM */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    
    /* enable counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/*
    Limitation: at most 25 sec delay under 168MHz core clock.
*/
void BSP_DWT_DelayUs(uint32_t us) {
    uint32_t cnt = us * (SystemCoreClock/1000000); 
    uint32_t startCnt = DWT->CYCCNT;
    uint32_t endCnt = startCnt + cnt;      
    if(endCnt > startCnt)  {
        while(DWT->CYCCNT < endCnt)
            ;       
    }
    else {
        while(DWT->CYCCNT > endCnt)
            ;
        while(DWT->CYCCNT < endCnt)
            ;
    }
}

void BSP_DWT_DelayMs(uint32_t ms) {
    static uint32_t cnt;
    for (cnt = 0; cnt < ms; ++cnt)
        BSP_DWT_DelayUs(1000);
}
