import lvgl as lv  # NOQA
import micropython  # NOQA
import sys
import time

from machine import Timer  # NOQA

# 全局退出标志
_global_exit_flag = False

TASK_HANDLER_STARTED = 0x00
TASK_HANDLER_FINISHED = 0x01

_default_timer_id = 0

if sys.platform in ('pyboard', 'rp2'):
    _default_timer_id = -1


class _DefaultUserData(object):
    pass


def _default_exception_hook(e):
    sys.print_exception(e)
    TaskHandler._current_instance.deinit()  # NOQA


class TaskHandler(object):
    _current_instance = None

    def __init__(
        self,
        duration=33,
        timer_id=_default_timer_id,
        max_scheduled=2,
        exception_hook=_default_exception_hook
    ):
        if TaskHandler._current_instance is not None:
            self.__dict__.update(TaskHandler._current_instance.__dict__)
        else:
            if not lv.is_initialized():
                lv.init()

            TaskHandler._current_instance = self

            self._callbacks = []

            self.duration = duration
            self.exception_hook = exception_hook

            self._timer = Timer(timer_id)

            # Allocation occurs here
            self._task_handler_ref = self._task_handler
            self.max_scheduled = max_scheduled

            self._start_time = time.ticks_ms()  # NOQA
            self._timer.init(
                mode=Timer.PERIODIC,
                period=self.duration,
                callback=self._timer_cb
            )
            self._scheduled = 0
            self._running = False
            print("TaskHandler init")

    def add_event_cb(self, callback, event, user_data=_DefaultUserData):
        for i, (cb, evt, data) in enumerate(self._callbacks):
            if cb == callback:
                evt = event
                if user_data != _DefaultUserData:
                    data = user_data

                self._callbacks[i] = (cb, evt, data)
                break
        else:
            if user_data == _DefaultUserData:
                user_data = None

            self._callbacks.append((callback, event, user_data))

    def remove_event_cb(self, callback):
        for i, obj in self._callbacks:
            if obj[0] == callback:
                self._callbacks.remove(obj)
                break

    def deinit(self):
        print("TaskHandler deinit")
        self._timer.deinit()
        TaskHandler._current_instance = None

    def disable(self):
        self._scheduled += self.max_scheduled

    def enable(self):
        self._scheduled -= self.max_scheduled

    @classmethod
    def is_running(cls):
        return cls._current_instance is not None
    
    @classmethod
    def should_exit(cls):
        """检查是否应该退出"""
        global _global_exit_flag
        return _global_exit_flag

    def _task_handler(self, _):
        try:
            self._scheduled -= 1

            if lv._nesting.value == 0:
                self._running = True

                run_update = True
                for cb, evt, data in self._callbacks:
                    if not evt ^ TASK_HANDLER_STARTED:
                        continue

                    try:
                        if cb(TASK_HANDLER_STARTED, data) is False:
                            run_update = False

                    except Exception as err:  # NOQA
                        if (
                            self.exception_hook and
                            self.exception_hook != _default_exception_hook
                        ):
                            self.exception_hook(err)
                        else:
                            sys.print_exception(err)

                stop_time = time.ticks_ms()

                ticks_diff = time.ticks_diff(stop_time, self._start_time)
                self._start_time = stop_time
                lv.tick_inc(ticks_diff)

                if run_update:
                    try:
                        lv.task_handler()
                    except KeyboardInterrupt:
                        print("TaskHandler lv.task_handler KeyboardInterrupt - shutting down")
                        self.deinit()
                        # 停止定时器，防止新的任务被调度
                        self._timer.deinit()
                        # 设置全局退出标志
                        global _global_exit_flag
                        _global_exit_flag = True
                        print("设置全局退出标志 _global_exit_flag = True")
                        # 不再触发 KeyboardInterrupt，让主循环自己检查标志
                    start_time = time.ticks_ms()

                    for cb, evt, data in self._callbacks:
                        if not evt ^ TASK_HANDLER_FINISHED:
                            continue

                        try:
                            cb(TASK_HANDLER_FINISHED, data)
                        except Exception as err:  # NOQA
                            if (
                                self.exception_hook and
                                self.exception_hook != _default_exception_hook
                            ):
                                self.exception_hook(err)
                            else:
                                sys.print_exception(err)

                    stop_time = time.ticks_ms()
                    ticks_diff = time.ticks_diff(stop_time, start_time)
                    lv.tick_inc(ticks_diff)
                
                self._running = False
        except KeyboardInterrupt:
            print("TaskHandler KeyboardInterrupt - shutting down")
            self.deinit()
            # 停止定时器，防止新的任务被调度
            self._timer.deinit()
            self.disable()
            # 设置全局退出标志
            global _global_exit_flag
            _global_exit_flag = True
            print("设置全局退出标志 _global_exit_flag = True")
            # 不再触发 KeyboardInterrupt，让主循环自己检查标志
        except Exception as e:
            self._running = False
            
            if self.exception_hook:
                self.exception_hook(e)
            

    def _timer_cb(self, _):
        try:
            lv.tick_inc(self.duration)
            if self._running:
                return

            if self._scheduled < self.max_scheduled:
                try:
                    micropython.schedule(self._task_handler_ref, 0)
                    self._scheduled += 1
                except KeyboardInterrupt:
                    print("TaskHandler micropython.schedule KeyboardInterrupt - shutting down")
                    self.deinit()
                    # 停止定时器，防止新的任务被调度
                    self._timer.deinit()
                    # 设置全局退出标志
                    global _global_exit_flag
                    _global_exit_flag = True
                    print("设置全局退出标志 _global_exit_flag = True")
                    # 不再触发 KeyboardInterrupt，让主循环自己检查标志
                except:  # NOQA
                    pass
        except KeyboardInterrupt:
            print("TaskHandler _timer_cb KeyboardInterrupt - shutting down")
            self.deinit()
            # 停止定时器，防止新的任务被调度
            self._timer.deinit()
            # 设置全局退出标志
            global _global_exit_flag
            _global_exit_flag = True
            print("设置全局退出标志 _global_exit_flag = True")
            # 不再触发 KeyboardInterrupt，让主循环自己检查标志
        except Exception as e:
            print(f"TaskHandler _timer_cb Exception: {e}")
            if self.exception_hook:
                self.exception_hook(e)
