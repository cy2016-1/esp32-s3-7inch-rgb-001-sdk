
#ifndef _BSP_BOARD_H_
#define _BSP_BOARD_H_

#include "driver/gpio.h"

// #define LCD_4r3_480x272    (1)   // 4.3 inch
// #define LCD_5r0_800x480    (1)   // 5.0 inch
#define LCD_7r0_800x480    (1)   // 7.0 inch

#if LCD_4r3_480x272
#define LCD_WIDTH       (480)
#define LCD_HEIGHT      (272)
#elif LCD_5r0_800x480
#define LCD_WIDTH       (800)
#define LCD_HEIGHT      (480)
#elif LCD_7r0_800x480
#define LCD_WIDTH       (800)
#define LCD_HEIGHT      (480)
#endif
#define GPIO_LCD_BL     (GPIO_NUM_1)
void sys_int(void);

#endif 

