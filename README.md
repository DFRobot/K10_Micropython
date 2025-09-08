# K10 Micropython固件源码

```shell

#编译脚本
python3 make.py esp32 BOARD=ESP32_GENERIC_S3 BOARD_VARIANT=SPIRAM_OCT DISPLAY=ili9341 --usb-jtag --flash-size=16
#烧录命令根据实际编译完成后，会有具体提示进行更改
/home/fary/.espressif/python_env/idf5.2_py3.8_env/bin/python -m esptool --chip esp32s3 -p /dev/ttyACM0 -b 460800 --before default_reset --after hard_reset write_flash --flash_mode dio --flash_size 16MB --flash_freq 80m --erase-all 0x0 /home/fary/micropython/lvgl_micropython/build/lvgl_micropy_ESP32_GENERIC_S3-SPIRAM_OCT-16.bin
```

需求及测试记录：https://h7dvigefi0.feishu.cn/docx/Xc8CdpMpQo79x9x29f1coorxnnh?from=from_copylink
