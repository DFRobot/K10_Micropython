# 全局标志解决方案使用说明

## 问题描述
在 ESP32S3 上运行 MicroPython 时，按 Ctrl+C 无法正常退出程序，KeyboardInterrupt 异常被 `micropython.schedule()` 吞掉，导致程序无法终止。这个问题不仅出现在 TaskHandler 中，也出现在加速度计等使用定时器中断的模块中。

## 解决方案
使用全局标志 + 主循环检查的方式实现优雅退出。

## 修改内容

### 1. 修改 TaskHandler (api_drivers/common_api_drivers/frozen/other/task_handler.py)

添加了全局退出标志和检查方法：

```python
# 全局退出标志
_global_exit_flag = False

@classmethod
def should_exit(cls):
    """检查是否应该退出"""
    global _global_exit_flag
    return _global_exit_flag
```

修改了所有 KeyboardInterrupt 处理，将异常处理改为设置全局标志：

```python
except KeyboardInterrupt:
    print("TaskHandler KeyboardInterrupt - shutting down")
    self.deinit()
    self._timer.deinit()
    # 设置全局退出标志
    global _global_exit_flag
    _global_exit_flag = True
    print("设置全局退出标志 _global_exit_flag = True")
    # 不再触发 KeyboardInterrupt，让主循环自己检查标志
```

### 2. 修改加速度计 (lib/micropython/ports/esp32/modules/k10_base/_k10_base.py)

添加了退出标志设置方法：

```python
def _set_exit_flag(self):
    """设置全局退出标志"""
    try:
        # 尝试设置 TaskHandler 的全局标志
        from api_drivers.common_api_drivers.frozen.other.task_handler import _global_exit_flag
        _global_exit_flag = True
        print("加速度计设置全局退出标志 _global_exit_flag = True")
    except:
        try:
            # 尝试设置本地全局标志
            import sys
            if hasattr(sys.modules['__main__'], 'exit_flag'):
                sys.modules['__main__'].exit_flag = True
                print("加速度计设置本地退出标志 exit_flag = True")
        except:
            print("无法设置退出标志")
```

修改了所有 KeyboardInterrupt 处理，添加退出标志设置：

```python
except KeyboardInterrupt:
    print("educore_callback KeyboardInterrupt - shutting down")
    if self.tim:
        self.tim.deinit()
    # 设置全局退出标志
    self._set_exit_flag()
```

### 3. 修改 main.py

将原来的无限循环改为检查退出标志：

```python
def check_exit_flag():
    """检查是否应该退出"""
    try:
        from api_drivers.common_api_drivers.frozen.other.task_handler import TaskHandler
        if TaskHandler.should_exit():
            return True
    except:
        pass
    return False

# 主循环
while True:
    # 检查退出标志
    if check_exit_flag():
        break
    time.sleep(0.1)  # 使用 sleep 而不是 pass
```

## 使用方法

1. 将修改后的文件替换原文件
2. 重启设备或重新运行程序
3. 按 Ctrl+C 时，程序会：
   - 在后台任务中捕获 KeyboardInterrupt
   - 设置全局退出标志 `_global_exit_flag = True`
   - 主循环检测到标志后优雅退出

## 优点

1. **优雅退出**：不重启设备，程序正常结束
2. **兼容性好**：不需要修改 MicroPython 固件
3. **可靠性高**：不依赖异常传递机制
4. **调试友好**：有明确的日志输出
5. **全面覆盖**：同时解决 TaskHandler 和加速度计等模块的退出问题

## 测试

运行以下测试文件来验证解决方案：

```bash
python test_global_flag_solution.py      # 测试 TaskHandler
python test_accelerometer_exit.py        # 测试加速度计
```

## 注意事项

1. 主循环必须定期检查退出标志（建议间隔 0.1 秒）
2. 使用 `time.sleep()` 而不是 `pass` 来避免 CPU 占用过高
3. 确保 TaskHandler 正确导入和初始化
4. 如果遇到模块导入问题，可以使用本地全局标志

## 文件清单

- `main_with_exit_check.py` - 修改后的 main.py
- `api_drivers/common_api_drivers/frozen/other/task_handler.py` - 修改后的 TaskHandler
- `lib/micropython/ports/esp32/modules/k10_base/_k10_base.py` - 修改后的加速度计
- `test_global_flag_solution.py` - TaskHandler 测试文件
- `test_accelerometer_exit.py` - 加速度计测试文件
- `test_single_ctrl_c.py` - 单次 Ctrl+C 测试文件
- `README_global_flag_solution.md` - 本说明文档 