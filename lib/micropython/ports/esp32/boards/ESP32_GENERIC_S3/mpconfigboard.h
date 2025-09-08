#ifndef MICROPY_HW_BOARD_NAME
// Can be set by mpconfigboard.cmake.
#define MICROPY_HW_BOARD_NAME               "Generic ESP32S3 module"
#endif
#define MICROPY_HW_MCU_NAME                 "ESP32S3"

// Enable UART REPL for modules that have an external USB-UART and don't use native USB.
#define MICROPY_HW_ENABLE_UART_REPL         (0)
#define CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG   (1)
#define MICROPY_HW_ENABLE_SDCARD            (1)

#define MICROPY_HW_I2C0_SCL                 (9)
#define MICROPY_HW_I2C0_SDA                 (8)
