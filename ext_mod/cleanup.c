#include "lv_init.h"
#include "esp_camera.h"
#include <stdio.h>
  #ifndef ESP_PLATFORM
  #error "ESP_PLATFORM not defined!"
  #endif
// 如有全局 LCD 对象指针，可在此处声明 extern
// extern mp_obj_t global_lcd_obj;
// #include "ext_mod/lcd_bus/modlcd_bus.h"
// mp_lcd_err_t lcd_panel_io_del(mp_obj_t obj);

void cleanup_all_resources(void) {
    // 1. 清理 LVGL
    //lv_deinit();

    // 2. 清理摄像头
    //esp_camera_deinit();

    // 3. 清理 LCD（如有全局对象可用，取消注释并实现）
    // if (global_lcd_obj) {
    //     lcd_panel_io_del(global_lcd_obj);
    // }

    // 4. 其他外设、定时器、任务等清理
    // TODO: 添加更多清理代码

    // 打印日志方便调试
    printf("[cleanup_all_resources] All resources cleaned up.\n");
} 