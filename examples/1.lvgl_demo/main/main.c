#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "lvgl.h"
#include "lv_port_disp.h"
#include "lv_port_indev.h"
#include "lv_port_fs.h"
#include "lv_demos.h"
#include "bsp_board.h" //1.方案板参数配置初始化必须包含的头文件
#include "lvgl_init.h" //2.方案板LVGL接口初始化必须包含的头文件
static const char *TAG = "main";

void app_main(void)
{
    // 1.方案板参数配置初始化
    sys_int();
    // 2.方案板LVGL接口初始化
    lvgl_init();

    lv_port_sem_take();
    // 3.LVGL应用层,用户程序
    //  lv_demo_widgets();
    // lv_demo_benchmark();
    lv_demo_music();
    // lv_demo_stress(); //LVGL应用层,用户程序
    lv_port_sem_give();
}
