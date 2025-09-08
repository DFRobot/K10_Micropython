import gc
import sys
import lvgl as lv
import lcd_bus
import ili9341
from machine import I2S,SPI,Timer,SDCard,I2C
import machine
from mpython import MPythonPin,PinMode,Pin, AHT20, Accelerometer,Light,Scan_Rfid_Edu,wifi,Button,button_a,button_b,i2c
from umqtt.robust import MQTTClient as MQTT
import ubinascii
import fs_driver, math

import time
import vfs

import camera
gc.collect()

'''
50以上的使用的是扩展IO芯片
P0  = 1
P1  = 2
P2  = P07 = 57
P3  = P16 = 66
P4  = P15 = 65
P5  = P14 = 64
P6  = P13 = 63
P7  = NC = -1
P8  = P10 = 60
P9  = P11 = 61
P10 = P12 = 62
P11 = P02 = 52
P12 = P03 = 53
P13 = P04 = 54
P14 = P05 = 55
P15 = P06 = 56
P16 = NC = -1
P17 = NC = -1
P18 = NC = -1
P19 = 48
P20 = 47
'''
pins_k10 = (1, 2, 57, 66, 65, 64, 63, -1, 60, 61, 62, 52, 53, 54, 55, 56, -1, -1, -1, 48, 47)
pins_state = [None] * len(pins_k10)
k10_i2c = I2C(0, scl=48, sda=47, freq=100000) 
'''
输入输出引脚控制 pin类
MPythonPin,PinMode,Pin
'''
class pin():
    def __init__(self, pin):
        self.pin_num = pin
        self.mode = PinMode.IN
        self.pull = None
        self._event_change = None
        self._event_rising = None
        self._event_falling = None
        self._pin = MPythonPin(self.pin_num, PinMode.IN)
        self._iqr_func = None

    def read_digital(self):
        # if(pins_state[self.pin_num]!=PinMode.IN):
        pins_state[self.pin_num]=PinMode.IN
        self._pin = MPythonPin(self.pin_num, PinMode.IN)
        return self._pin.read_digital()
        # else:
        #     return self._pin.read_digital()

    def write_digital(self, value):
        # if(pins_state[self.pin_num]!=PinMode.OUT):
        pins_state[self.pin_num]=PinMode.OUT
        self._pin = MPythonPin(self.pin_num, PinMode.OUT, Pin.PULL_UP)
        return self._pin.write_digital(value)
        # else:
        #     return self._pin.write_digital(value)

    def read_analog(self):
        if self.pin_num not in [0, 1, 2, 3, 4, 10]:
            tmp = self.read_digital()
            if(tmp==0):
                return 0
            elif(tmp==1):
                return 4095
            else:
                return None
        pins_state[self.pin_num]=PinMode.ANALOG
        self._pin = MPythonPin(self.pin_num, PinMode.ANALOG)
        return self._pin.read_analog()
        
    def write_analog(self, value=0, freq=5000):
        # if(pins_state[self.pin_num]!=PinMode.PWM):
        pins_state[self.pin_num]=PinMode.PWM
        self._pin = MPythonPin(self.pin_num, PinMode.PWM)
        return self._pin.write_analog(duty=value, freq=freq)
        # else:
        #     return self._pin.write_analog(duty=value, freq=freq)

    def irq(self, handler=None, trigger=Pin.IRQ_RISING):
        # if(pins_state[self.pin_num]!=PinMode.IN):
        pins_state[self.pin_num]=PinMode.IN
        self._pin = MPythonPin(self.pin_num, PinMode.IN)
        self._pin.irq(trigger=trigger, handler=handler)
        # else:
        #     self._pin.irq(trigger=trigger, handler=handler)
    
    @property
    def event_change(self):
        return self._event_change

    @event_change.setter
    def event_change(self, new_event_change):
        if new_event_change != self._event_change:
            self._event_change = new_event_change
            self._iqr_func = self._event_change
            self.irq(handler=self.func, trigger=Pin.IRQ_RISING|Pin.IRQ_FALLING)

    @property
    def event_rising(self):
        return self._event_rising

    @event_rising.setter
    def event_rising(self, new_event_rising):
        if new_event_rising != self._event_rising:
            self._event_rising = new_event_rising
            self._iqr_func = self._event_rising
            self.irq(handler=self.func, trigger=Pin.IRQ_RISING)

    @property
    def event_falling(self):
        return self._event_falling

    @event_falling.setter
    def event_falling(self, new_event_falling):
        if new_event_falling != self._event_falling:
            self._event_falling = new_event_falling
            self._iqr_func = self._event_falling
            self.irq(handler=self.func, trigger=Pin.IRQ_FALLING)
    
    def func(self,_):
        self._iqr_func()

class button:
    a = 'a'
    b = 'b'
    def __init__(self,_type='a'): 
        self.button_a = button_a
        self.button_b = button_b
        self.type = _type
        self.func_event_change = None
        self.func_event_released = None
        if(self.type not in ['a','b']):
            self.pin = pins_k10[self.type]
            self.button = Button(self.pin)

    def func(self,_):
        self.func_event_change()

    def func_released(self,_):
        self.func_event_released()

    @property
    def event_pressed(self):
        return self.func_event_change

    @event_pressed.setter
    def event_pressed(self, new_event_change):
        if new_event_change != self.func_event_change:
            self.func_event_change = new_event_change
            if(self.type=='a'):
                self.button_a.event_pressed = self.func
            elif(self.type=='b'):
                self.button_b.event_pressed = self.func
            else:
                # print('Not supported')
                self.button.event_pressed = self.func
    @property
    def event_released(self):
        return self.func_event_released

    @event_released.setter
    def event_released(self, new_event_released):
        if new_event_released != self.func_event_released:
            self.func_event_released = new_event_released
            if(self.type=='a'):
                self.button_a.event_released = self.func_released
            elif(self.type=='b'):
                self.button_b.event_released = self.func_released
            else:
                # print('Not supported')
                self.button.event_released = self.func_released

    def status(self):
        if(self.type=='a'):
            return self.button_a.status()
        elif(self.type=='b'):
            return self.button_b.status()
        else:
            return self.button.status()    

class aht20(object):
    def __init__(self):
        self.aht20 = AHT20()
        #self.tim = Timer(16)
        #self.tim.init(period=1000, mode=Timer.PERIODIC, callback=self.timer_tick)
        time.sleep(1.5)
        self.aht20.measure()
        pass

    def timer_tick(self,_):
        try: 
            self.aht20.measure()
        except: 
            pass
    def measure(self):
        try: 
            self.aht20.measure()
        except: 
            pass

    def read(self):
        return self.aht20.temperature(), self.aht20.humidity()
    
    def read_temp(self):
        return self.aht20.temperature()
    
    def read_temp_f(self):
        celsius = self.read_temp()
        fahrenheit = celsius * 9 / 5 + 32
        return round(fahrenheit,2)
    
    def read_humi(self):
        return self.aht20.humidity()

class Screen(object):
    def __init__(self,dir=2):
        self.spi_bus = SPI.Bus(host=2,mosi=21,miso=-1,sck=12)
        self.display_bus = lcd_bus.SPIBus(spi_bus = self.spi_bus, dc = 13, cs = 14, freq = 40000000)
        '''
        self.display = ili9341.ILI9341(data_bus = self.display_bus, display_width = 240, display_height = 320,
                                       reset_state = ili9341.STATE_LOW, color_byte_order = ili9341.BYTE_ORDER_BGR,
                                       color_space = lv.COLOR_FORMAT.RGB565, rgb565_byte_swap=True)
        '''
        self.display = ili9341.ILI9341(data_bus = self.display_bus, display_width = 240, display_height = 320,
                                       reset_state = ili9341.STATE_LOW, color_byte_order = ili9341.BYTE_ORDER_BGR,
                                       color_space = lv.COLOR_FORMAT.RGB565, rgb565_byte_swap=True)
        self.linewidth = 1
        self.fs_drv = lv.fs_drv_t()
        fs_driver.fs_register(self.fs_drv, 'S')
        self.myfont_cn = lv.binfont_create("S:./font_big.bin")
        
    #初始化屏幕，设置方向为(0~3)
    def init(self,dir=2):
        #用来打开屏幕背光
        myi2c = I2C(0, scl=Pin(48), sda=Pin(47), freq=100000)
        temp = myi2c.readfrom_mem(0x20, 0x02, 1)
        myi2c.writeto(0x20,bytearray([0x02, (temp[0] | 0x01)]))
        temp = myi2c.readfrom_mem(0x20, 0x06, 1)
        myi2c.writeto(0x20,bytearray([0x06, (temp[0] & 0xFE)]))

        self.display.set_power(True)
        self.display.init(1)
        if dir == 0:
            self.display.set_rotation(lv.DISPLAY_ROTATION._0)
        elif dir == 1:
            self.display.set_rotation(lv.DISPLAY_ROTATION._90)
        elif dir == 2:
            self.display.set_rotation(lv.DISPLAY_ROTATION._180)
        elif dir == 3:
            self.display.set_rotation(lv.DISPLAY_ROTATION._270)
        else:
            self.display.set_rotation(lv.DISPLAY_ROTATION._180)

        #self.screen = lv.obj()
        self.screen = lv.screen_active()
        self.img = lv.image(self.screen)
        self.canvas = lv.canvas(self.screen)
        self.screen.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
        self.img.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
        self.canvas.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)


        self.canvas.set_size(240,320)
        self.canvas.align(lv.ALIGN.CENTER, 0, 0)
        self.canvas_buf = bytearray(240*320*4)
        self.canvas.set_buffer(self.canvas_buf, 240, 320, lv.COLOR_FORMAT.ARGB8888)
        self.canvas.fill_bg(lv.color_white(), lv.OPA.TRANSP)
        self.layer = lv.layer_t()
        self.canvas.init_layer(self.layer)
        self.area = lv.area_t()
        self.clear_rect = lv.draw_rect_dsc_t()
        
        self.img_dsc = lv.image_dsc_t(
            dict(
                header = dict(cf =lv.COLOR_FORMAT.RGB565, w=480, h=320),
                data_size = 480*320*2,
                data = None
            )
        )

    #显示指定颜色背景
    def show_bg(self,color=0xFFFFFF):
        self.screen.set_style_bg_color(lv.color_hex(color),0)

    #将缓存内容显示
    def show_draw(self):
        self.canvas.finish_layer(self.layer)
        self.canvas.invalidate()
        #lv.screen_load(self.screen)

    #清除全屏，颜色为指定颜色
    def clear(self,line=0,font=None,color=None):
        if font == None:
            #清除全屏，颜色为指定颜色
            if color == None:
                self.canvas.fill_bg(lv.color_white(), lv.OPA.TRANSP)
            else:
                self.canvas.fill_bg(lv.color_hex(color), lv.OPA.COVER)
        else:
            #清除第几行文字
            pass
        pass

    #显示文字xxx在第line行,字号font_size,颜色color
    def draw_text(self,text="",line=None, x=0,y=0,font_size=16,color=0x0000FF):
        #self.canvas.finish_layer(self.layer)
        #self.canvas.get_draw_buf().clear(self.area)
        self.desc = lv.draw_label_dsc_t()
        self.desc.init()
        self.desc.color = lv.color_hex(color)
        self.desc.text = text
        self.desc.font = self.myfont_cn
        '''
        if font_size == 16:
            self.desc.font = lv.font_montserrat_16
        elif font_size == 14:
            self.desc.font = lv.font_montserrat_14
        elif font_size == 12:
            self.desc.font = lv.font_montserrat_12
        else:
            font_size = 16'''

        #按坐标显示
        if line == None:
            self.area.x1 = x
            self.area.y1 = y
        #按行显示 
        else:
            self.area.x1 = 0
            self.area.y1 = line * (font_size + 2)
        self.area.set_width(240-self.area.x1)
        self.area.set_height(font_size + 2)

        #self.layer.draw_buf.clear(self.area)
        #self.canvas.fill_bg(lv.color_white(), lv.OPA.TRANSP)
        #self.canvas.get_draw_buf().clear(self.area)  # 强制清除画布缓冲区
        #bytearray(self.canvas.get_buf())[:] = b'\x00' * len(self.canvas_buf)
        self.layer.draw_buf.clear(self.area)  # 清除图层缓冲区
        #self.canvas_buf[:] = b'\x00' * len(self.canvas_buf)
        lv.draw_label(self.layer, self.desc, self.area)
        #self.layer.draw_buf.clear(self.area)

    #画点
    def draw_point(self,x=0,y=0,color=0x0000FF):
        #self.canvas.set_px(x=x, y=y, color = lv.color_hex(color))
        self.draw_line(x0=x,y0=y,x1=(x+self.linewidth),y1=y,color = color)

    #设置线宽/边框宽
    def set_width(self,width=1):
        self.linewidth = width

    #画线
    def draw_line(self,x0=0,y0=0,x1=0,y1=0,color=0x000000):
        self.desc = lv.draw_line_dsc_t()
        self.desc.init()
        self.desc.p1.x = x0
        self.desc.p1.y = y0
        self.desc.p2.x = x1
        self.desc.p2.y = y1
        self.desc.color = lv.color_hex(color)
        self.desc.width = self.linewidth
        self.desc.opa = 255
        self.desc.blend_mode = lv.BLEND_MODE.NORMAL
        self.desc.round_start = 0
        self.desc.round_end = 0
        lv.draw_line(self.layer, self.desc)

    #画圆,圆心(x,y),r半径,边框颜色bcolor,填充颜色fcolor
    def draw_circle(self, x=0,y=0,r=0,bcolor=0x000000,fcolor=None):
        self.desc = lv.draw_rect_dsc_t()
        self.desc.init()
        self.desc.radius = lv.RADIUS_CIRCLE
        if fcolor == None:
            #设置矩形背景透明度为透明色
            self.desc.bg_opa = lv.OPA.TRANSP
        else:
            #设置矩形背景透明度为不透明
            self.desc.bg_opa = lv.OPA.COVER
            #设置矩形背景色
            self.desc.bg_color = lv.color_hex(fcolor)
        #设置矩形边框宽度
        self.desc.border_width = self.linewidth
        #设置矩形边框颜色
        self.desc.border_color = lv.color_hex(bcolor)
        area = lv.area_t()
        area.x1 = x-r
        area.y1 = y-r
        area.set_width(2*r)
        area.set_height(2*r)
        lv.draw_rect(self.layer, self.desc, area)
        pass
    
    #显示矩形顶点(x,y),宽w、高h,边框颜色bcolor,填充颜色fcolor
    def draw_rect(self, x = 0, y = 0, w = 0, h = 0, bcolor = 0x000000, fcolor = None):
        self.desc = lv.draw_rect_dsc_t()
        self.desc.init()
        
        if fcolor == None:
            #设置矩形背景透明度为透明色
            self.desc.bg_opa = lv.OPA.TRANSP
        else:
            #设置矩形背景透明度为不透明
            self.desc.bg_opa = lv.OPA.COVER
            #设置矩形背景色
            self.desc.bg_color = lv.color_hex(fcolor)
        #设置矩形边框宽度
        self.desc.border_width = self.linewidth
        #设置矩形边框颜色
        self.desc.border_color = lv.color_hex(bcolor)
        area = lv.area_t()
        area.x1 = x
        area.y1 = y
        area.set_width(w)
        area.set_height(h)
        lv.draw_rect(self.layer, self.desc, area)
        pass
    def show_camera_feed(self, buf):
        # 将摄像头数据填充到canvas缓冲区
        #self.canvas_buf[:] = buf[:len(self.canvas_buf)]  # 假设buf与canvas分辨率匹配
        #self.show_draw()
        img_dsc = lv.draw_image_dsc_t()
        img_dsc.init()
        img_dsc.src = buf
        area = lv.area_t()
        area.x1 = 0
        area.y1 = 0
        area.set_width(240)
        area.set_height(320)
        lv.draw_image(self.layer,self.desc, area)
        lv.screen_load(self.screen)

    def show_camera_img(self,buf):
        lv.draw_sw_rgb565_swap(buf,240*320*2)
        self.img_dsc.data = buf
        self.img.set_src(self.img_dsc)
        lv.refr_now(None)

    def show_camera(self,camera):
        self.timer =lv.timer_create(lambda t: self.show_camera_img(camera.capture()), 50, None)
        
    def deinit(self):
        if hasattr(self, 'timer') and self.timer:
            try:
                self.timer.pause()
                self.timer.delete()
            except:
                pass

        

class Camera(object):
    def __init__(self):
        self.cam = camera
    def init(self):
        #复位摄像头
        myi2c = I2C(0, scl=Pin(48), sda=Pin(47), freq=100000)
        temp = myi2c.readfrom_mem(0x20, 0x06, 1)
        myi2c.writeto(0x20,bytearray([0x06, (temp[0] & 0xFD)]))
        temp = myi2c.readfrom_mem(0x20, 0x02, 1)
        myi2c.writeto(0x20,bytearray([0x02, (temp[0] | 0x00)]))
        time.sleep(0.1)
        temp = myi2c.readfrom_mem(0x20, 0x02, 1)
        myi2c.writeto(0x20,bytearray([0x02, (temp[0] | 0x02)]))
        self.cam.init(0)
    def capture(self):
        return self.cam.capture()
    
    def deinit(self):
        self.can.deinit()
      

'''
六轴 educore定时器
'''
class accelerometer():
    def __init__(self):
        self.tim_count = 0
        self._is_shaked = False
        self._last_x = 0
        self._last_y = 0
        self._last_z = 0
        self._count_shaked = 0
        myi2c = I2C(0, scl=Pin(48), sda=Pin(47), freq=100000)
        devices = myi2c.scan()
        if 0x19 in devices:
            #K10
            self.tim = Timer(17)
            self.tim.init(period=100, mode=Timer.PERIODIC, callback=self.educore_callback)
            self.accel_sensor = Accelerometer()
        else:
            #K10box
            self.tim = None
            self.accel_sensor = None

    def educore_callback(self, _):
        self.tim_count += 1 
        try:
            self.accelerometer_callback()
            try:
                gc.collect()
            except KeyboardInterrupt:
                print("educore_callback gc.collect KeyboardInterrupt - shutting down")
                if self.tim:
                    self.tim.deinit()
        except KeyboardInterrupt:
            print("KeyboardInterrupt out - shutting down")
            if self.tim:
                self.tim.deinit()
        except Exception as e:
            print("Exception out")
            print(str(e))
        if self.tim_count == 200:
            self.tim_count = 0

    def accelerometer_callback(self):
        '''加速度计'''
        try:
            if self._is_shaked:
                self._count_shaked += 1
                if self._count_shaked == 5:
                    self._count_shaked = 0
                    self.accel_sensor.shake_status = False
            self.accel_sensor._measure()
            x = self.accel_sensor.x()
            y = self.accel_sensor.y()
            z = self.accel_sensor.z()
            if self._last_x == 0 and self._last_y == 0 and self._last_z == 0:
                self._last_x = x
                self._last_y = y
                self._last_z = z
                self.accel_sensor.shake_status = False
                return
            diff_x = x - self._last_x
            diff_y = y - self._last_y
            diff_z = z - self._last_z
            self._last_x = x
            self.last_y = y
            self._last_z = z
            if self._count_shaked > 0:
                return
            self._is_shaked = (diff_x * diff_x + diff_y * diff_y + diff_z * diff_z > 1)
            if self._is_shaked:
                self.accel_sensor.shake_status = True
        except KeyboardInterrupt:
            print("accelerometer_callback KeyboardInterrupt - shutting down")
            if self.tim:
                self.tim.deinit()
            raise
        except Exception as e:
            print(f"accelerometer_callback Exception: {e}")
        
    def X(self):
        return self.accel_sensor.x()
    
    def Y(self):
        return self.accel_sensor.y()
    
    def Z(self):
        return self.accel_sensor.z()
    
    def read_x(self):
        return self.accel_sensor.x()
    
    def read_y(self):
        return self.accel_sensor.y()
    
    def read_z(self):
        return self.accel_sensor.z()
    
    def shake(self):
        return self.accel_sensor.shake()
    def gesture(self):
        return self.accel_sensor._gesture
    def status(self,status=""):
        if status is "forward":
            if self.gesture() == self.accel_sensor.TILT_FORWARD:
                return True
            else:
                return False
        elif status is "back":
            if self.gesture() == self.accel_sensor.TILT_BACK:
                return True
            else:
                return False
        elif status is "left":
            if self.gesture() == self.accel_sensor.TILT_LEFT:
                return True
            else:
                return False
        elif status is "right":
            if self.gesture() == self.accel_sensor.TILT_RIGHT:
                return True
            else:
                return False
        elif status is "up":
            if self.gesture() == self.accel_sensor.SCREEN_UP:
                return True
            else:
                return False
        elif status is "down":
            if self.gesture() == self.accel_sensor.SCREEN_DOWN:
                return True
            else:
                return False
        else:
            return False

    def deinit(self):
        self.tim.deinit()

'''继承wifi'''
class WiFi(wifi):
    def __init__(self):
        super().__init__()

    def connect(self, ssid, psd, timeout=10000):
        self.connectWiFi(ssid, psd, int(timeout/1000))
    
    def status(self):
        return self.sta.isconnected()

    def info(self):
        return str(self.sta.ifconfig())

'''MQTT'''
class MqttClient():
    def __init__(self):
        self.client = None
        self.server = None
        self.port = None
        self.client_id = None
        self.user = None
        self.passsword = None
        self.topic_msg_dict = {}
        self.topic_callback = {}
        self.tim_count = 0
        self._connected = False
        self.lock = False

    def connect(self, **kwargs):
        server = kwargs.get('server',"iot.mpython.cn" )
        port = kwargs.get('port',1883 )
        client_id = kwargs.get('client_id',"" )
        user = kwargs.get('user',"" )
        psd = kwargs.get('psd',None)
        password = kwargs.get('password',None)
        if(psd==None and password==None):
            psd = ""
        elif(password!=None):
            psd = password
        try:
            self.client = MQTT(client_id, server, port, user, psd, 60)
            self.client.connect()
            self.server = server
            self.port = port
            self.client_id = client_id
            self.user = user
            self.passsword = psd
            print('Connected to MQTT Broker "{}"'.format(self.server))
            self._connected = True
            self.client.set_callback(self.on_message)
            time.sleep(0.5)
            self.tim = Timer(15)
            self.tim.init(period=100, mode=Timer.PERIODIC, callback=self.mqtt_heartbeat)
            gc.collect()
        except Exception as e:
            print('Connected to MQTT Broker error:{}'.format(e))

    def connected(self):
        return self._connected

    def publish(self, topic, content, _qos = 1):
        try:
            self.lock = True
            self.client.publish(str(topic),str(content).encode("utf-8"),qos=_qos)
            self.lock = False
        except Exception as e:
            print('publish error:{}'.format(e))

    def message(self, topic):
        topic = str(topic)
        if(not topic in self.topic_msg_dict):
            # self.topic_msg_dict[topic] = None
            self.topic_callback[topic] = False 
            self.subscribe(topic, self.default_callbak)
            return self.topic_msg_dict[topic]
        else:
            return self.topic_msg_dict[topic]
        
    def received(self, topic, callback):
        self.subscribe(topic, callback)

    def subscribe(self, topic, callback):
        self.lock = True
        try:
            topic = str(topic)
            if(not topic in self.topic_msg_dict):
                global _callback
                _callback = callback
                self.topic_msg_dict[topic] = None
                self.topic_callback[topic] = True
                exec('global mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)),globals())
                exec('mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)) + ' = _callback',globals())
                self.client.subscribe(topic)
                time.sleep(0.1)
            elif(topic in self.topic_msg_dict and self.topic_callback[topic] == False):
                global _callback
                _callback = callback
                self.topic_callback[topic] = True
                exec('global mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)),globals())
                exec('mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)) + ' = _callback',globals())
                time.sleep(0.1)
            else:
                print('Already subscribed to the topic:{}'.format(topic))
            self.lock = False
        except Exception as e:
            print('MQTT subscribe error:'+str(e))

    def on_message(self, topic, msg):
        try:
            gc.collect()
            topic = topic.decode('utf-8', 'ignore')
            msg = msg.decode('utf-8', 'ignore')
            #print("Received '{payload}' from topic '{topic}'\n".format(payload = msg, topic = topic))
            if(topic in self.topic_msg_dict):
                self.topic_msg_dict[topic] = msg
                if(self.topic_callback[topic]):
                    exec('global mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic)),globals())
                    eval('mqtt_topic_' + bytes.decode(ubinascii.hexlify(topic))+'()',globals())
        except Exception as e:
            print('MQTT on_message error:'+str(e))
    
    def default_callbak(self):
        pass
    
    def mqtt_check_msg(self):
        try:
            self.client.check_msg()
        except Exception as e:
            print('MQTT check msg error:'+str(e))

    def mqtt_heartbeat(self,_):
        self.tim_count += 1 
        if(not self.lock):
            self.mqtt_check_msg()
        if(self.tim_count==200):
            self.tim_count = 0
            try:
                self.client.ping() # 心跳消息
                self._connected = True
            except Exception as e:
                print('MQTT keepalive ping error:'+str(e))
                self._connected = False

'''
K10扬声器类
'''
class Speaker(object):
    def __init__(self):
        print("init speaker\n")
        self.i2s = I2S(1,sck = 0,ws=38, sd= 45, mode=I2S.TX, bits=32, format=I2S.MONO, rate=16000, ibuf=20000)
        self.i2s.deinit()
        self._i2c = k10_i2c
        print("init done\n")
        self.buzzMelody = 2
        self.playTone = 2
        self.freqTable = [ 31, 33, 35, 37, 39, 41, 44, 46, 49, 52, 55, 58, 62, 65, 69, 73, 78, 82, 87, 92, 98, 104, 110, 
                          117, 123, 131, 139, 147, 156, 165, 175, 185, 196, 208, 220, 233, 247, 262, 277, 294, 311, 330, 349, 370, 392, 
                          415, 440, 466, 494, 523, 554, 587, 622, 659, 698, 740, 784, 831, 880, 932, 988, 1047, 1109, 1175, 1245, 1319, 
                          1397, 1480, 1568, 1661, 1760, 1865, 1976, 2093, 2217, 2349, 2489, 2637, 2794, 2960, 3136, 3322, 3520, 3729, 3951, 4186]
        self.currentDuration = 4  # Default duration (Crotchet)
        self.currentOctave = 4    # Middle octave
        self.beatsPerMinute = 15  # Default BPM
        self.TWO_PI = 6.283185307179586476925286766559
        self.music_notes={"DADADADUM":"r4:2|g|g|g|eb:8|r:2|f|f|f|d:8|",
                          "ENTERTAINER":"d4:1|d#|e|c5:2|e4:1|c5:2|e4:1|c5:3|c:1|d|d#|e|c|d|e:2|b4:1|d5:2|c:4|",
                          "PRELUDE":"c4:1|e|g|c5|e|g4|c5|e|c4|e|g|c5|e|g4|c5|e|c4|d|g|d5|f|g4|d5|f|c4|d|g|d5|f|g4|d5|f|b3|d4|g|d5|f|g4|d5|f|b3|d4|g|d5|f|g4|d5|f|c4|e|g|c5|e|g4|c5|e|c4|e|g|c5|e|g4|c5|e|",
                          "ODE":"e4|e|f|g|g|f|e|d|c|c|d|e|e:6|d:2|d:8|e:4|e|f|g|g|f|e|d|c|c|d|e|d:6|c:2|c:8|",
                          "NYAN":"f#5:2|g#|c#:1|d#:2|b4:1|d5:1|c#|b4:2|b|c#5|d|d:1|c#|b4:1|c#5:1|d#|f#|g#|d#|f#|c#|d|b4|c#5|b4|d#5:2|f#|g#:1|d#|f#|c#|d#|b4|d5|d#|d|c#|b4|c#5|d:2|b4:1|c#5|d#|f#|c#|d|c#|b4|c#5:2|b4|c#5|b4|f#:1|g#|b:2|f#:1|g#|b|c#5|d#|b4|e5|d#|e|f#|b4:2|b|f#:1|g#|b|f#|e5|d#|c#|b4|f#|d#|e|f#|b:2|f#:1|g#|b:2|f#:1|g#|b|b|c#5|d#|b4|f#|g#|f#|b:2|b:1|a#|b|f#|g#|b|e5|d#|e|f#|b4:2|c#5|",
                          "RINGTONE":"c4:1|d|e:2|g|d:1|e|f:2|a|e:1|f|g:2|b|c5:4|",
                          "FUNK":"c2:2|c|d#|c:1|f:2|c:1|f:2|f#|g|c|c|g|c:1|f#:2|c:1|f#:2|f|d#|",
                          "BIRTHDAY":"c4:3|c:1|d:4|c:4|f|e:8|c:3|c:1|d:4|c:4|g|f:8|c:3|c:1|c5:4|a4|f|e|d|a#:3|a#:1|a:4|f|g|f:8|",
                          "WEDDING":"c4:4|f:3|f:1|f:8|c:4|g:3|e:1|f:8|c:4|f:3|a:1|c5:4|a4:3|f:1|f:4|e:3|f:1|g:8|",
                          "FUNERAL":"c3:4|c:3|c:1|c:4|d#:3|d:1|d:3|c:1|c:3|b2:1|c3:4|",
                          "PUNCHLINE":"c4:3|g3:1|f#|g|g#:3|g|r|b|c4|",
                          "BADDY":"c3:3|r|d:2|d#|r|c|r|f#:8|",
                          "CHASE":"a4:1|b|c5|b4|a:2|r|a:1|b|c5|b4|a:2|r|a:2|e5|d#|e|f|e|d#|e|b4:1|c5|d|c|b4:2|r|b:1|c5|d|c|b4:2|r|b:2|e5|d#|e|f|e|d#|e|",
                          "BA_DING":"b5:1|e6:3|",
                          "JUMP_UP":"c5:1|d|e|f|g|",
                          "JUMP_DOWN":"g5:1|f|e|d|c|",
                          "POWER_UP":"g4:1|c5|e|g:2|e:1|g:3|",
                          "POWER_DOWN":"g5:1|d#|c|g4:2|b:1|c5:3|"}
        

    def __del__(self):
        print("Speaker deleted\n")
        if self.i2s:
            self.i2s.deinit()
            self.i2s = None
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass

    def deinit(self):
        print("Speaker deinit\n")
        if self.i2s:
            self.i2s.deinit()
            self.i2s = None
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass
        
    def reinit(self,bits=16,sample_rate=16000,channels=1):
        self.i2s = I2S(1,sck = 0, ws = 38, sd = 45,mode=I2S.TX, bits=bits, 
                       format=I2S.MONO if channels == 1 else I2S.STEREO, rate=sample_rate, ibuf=20000)

    def parse_wav_header(self, wav_file):
        # 解析 WAV 文件头部
        wav_file.seek(0)
        riff = wav_file.read(4)
        if riff != b'RIFF':
            raise ValueError("Not a valid WAV file")

        wav_file.seek(22)
        num_channels = int.from_bytes(wav_file.read(2), 'little')
        sample_rate = int.from_bytes(wav_file.read(4), 'little')

        wav_file.seek(34)
        bits_per_sample = int.from_bytes(wav_file.read(2), 'little')

        return sample_rate, bits_per_sample, num_channels
    
    def play_tone(self,freq, beat):
        buf = bytearray(4)
        for i in range(beat):
            sample = int(32767.0 * math.sin(i * 2 * math.pi  * freq / 8000))
            struct.pack_into('<hh', buf, 0, sample, sample)  # 打包左右声道
            self.i2s.write(buf)

    def play_tone_music(self,tone_music: str):
        name = tone_music.upper()  # 确保输入大写
        if name in self.music_notes:
            music_sequence = self.music_notes[name]
            for note in music_sequence.split("|"):
                if note:  # 跳过空字符串
                    self.play_next_note(note)  # 调用实际播放方法
                    
        else:
            print(f"Error: {name} not found in music_data")
            pass

    def play_next_note(self, tone: str):
        curr_note = tone
        current_duration = self.currentDuration
        current_octave = self.currentOctave
        is_rest = False
        parsing_octave = True
        note = 0
        beat_pos = 0

        for pos, note_char in enumerate(curr_note):
            if note_char in ['c', 'C']:
                note = 1
            elif note_char in ['d', 'D']:
                note = 3
            elif note_char in ['e', 'E']:
                note = 5
            elif note_char in ['f', 'F']:
                note = 6
            elif note_char in ['g', 'G']:
                note = 8
            elif note_char in ['a', 'A']:
                note = 10
            elif note_char in ['b', 'B']:
                note = 12
            elif note_char in ['r', 'R']:
                is_rest = True
            elif note_char == '#':
                note += 1
            elif note_char == ':':
                parsing_octave = False
                beat_pos = pos
            elif parsing_octave and note_char.isdigit():
                current_octave = int(note_char)
        
        if not parsing_octave and beat_pos + 1 < len(curr_note) and curr_note[beat_pos + 1].isdigit():
            current_duration = int(curr_note[beat_pos + 1])

        beat = (60000 / self.beatsPerMinute) / 4

        if not is_rest:
            key_number = note + (12 * (current_octave - 1))
            frequency = self.freqTable[key_number] if 0 <= key_number < len(self.freqTable) else 0
            self.play_tone(frequency, current_duration * beat) #发送声音效果

        self.currentDuration = current_duration
        self.currentOctave = current_octave

    def play_sys_music(self,path):
        full_path = "/" + path
        self.play_music(full_path)

    def play_tf_music(self, path):
        full_path = "/sd/" + path
        self.play_music(full_path)
        
    def play_music(self,path):
        #使能功放(k10 box才有的功能)
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x01]))
        except:
            pass
        #打开WAV文件
        with open(path,"rb") as wav_file:
            sample_rate, bits_per_sample, num_channels = self.parse_wav_header(wav_file)
            self.reinit(bits=bits_per_sample,sample_rate=sample_rate,channels=num_channels)
            while True:
                audio_buf = wav_file.read(1024)
                if not audio_buf:
                    break
                self.i2s.write(audio_buf)
        self.i2s.deinit()
        #失能功放
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass

    def stop_music(self):
        self.i2s.deinit()
        #失能功放
        try:
            self._i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
        except:
            pass


class Es7243e(object):
    ES7243E_ADDR1 = 0X15
    ES7243E_ADDR2 = 0X11
    def __init__(self, i2c):
        self.i2c = i2c
        self.devices = self.i2c.scan()
        if self.devices:
            if self.ES7243E_ADDR1 in self.devices:
                self.ctrl_state(self.ES7243E_ADDR1,False)
                time.sleep(0.1)
                self.config(self.ES7243E_ADDR1)
                time.sleep(0.1)
                self.ctrl_state(self.ES7243E_ADDR1,True)
            elif self.ES7243E_ADDR2 in self.devices:
                self.ctrl_state(self.ES7243E_ADDR2,False)
                time.sleep(0.1)
                self.config(self.ES7243E_ADDR2)
                time.sleep(0.1)
                self.ctrl_state(self.ES7243E_ADDR2,True)
            else:
                print("mic init error")
                pass
    def write_cmd(self,addr, reg, cmd):
        send_buf = bytearray(2)
        send_buf[0] = reg
        send_buf[1] = cmd
        self.i2c.writeto(addr, send_buf)
    
    def ctrl_state(self, addr, state):
        if state:
            self.write_cmd(addr, 0xF9, 0x00)
            self.write_cmd(addr, 0xF9, 0x00)
            self.write_cmd(addr, 0x04, 0x01)
            self.write_cmd(addr, 0x17, 0x01)
            self.write_cmd(addr, 0x20, 0x10)
            self.write_cmd(addr, 0x21, 0x10)
            self.write_cmd(addr, 0x00, 0x80)
            self.write_cmd(addr, 0x01, 0x3A)
            self.write_cmd(addr, 0x16, 0x3F)
            self.write_cmd(addr, 0x16, 0x00)
        else:
            self.write_cmd(addr, 0x04, 0x02)
            self.write_cmd(addr, 0x04, 0x01)
            self.write_cmd(addr, 0xF7, 0x30)
            self.write_cmd(addr, 0xF9, 0x01)
            self.write_cmd(addr, 0x16, 0xFF)
            self.write_cmd(addr, 0x17, 0x00)
            self.write_cmd(addr, 0x01, 0x38)
            self.write_cmd(addr, 0x20, 0x00)
            self.write_cmd(addr, 0x21, 0x00)
            self.write_cmd(addr, 0x00, 0x00)
            self.write_cmd(addr, 0x00, 0x1E)
            self.write_cmd(addr, 0x01, 0x30)
            self.write_cmd(addr, 0x01, 0x00)
    
    def config(self,addr):
        self.write_cmd(addr, 0x01, 0x3A)
        self.write_cmd(addr, 0x00, 0x80)
        self.write_cmd(addr, 0xF9, 0x00)
        self.write_cmd(addr, 0x04, 0x02)
        self.write_cmd(addr, 0x04, 0x01)
        self.write_cmd(addr, 0xF9, 0x01)
        self.write_cmd(addr, 0x00, 0x1E)
        self.write_cmd(addr, 0x01, 0x00)
        
        self.write_cmd(addr, 0x02, 0x00)
        self.write_cmd(addr, 0x03, 0x20)
        self.write_cmd(addr, 0x04, 0x03)
        self.write_cmd(addr, 0x0D, 0x00)
        self.write_cmd(addr, 0x05, 0x00)
        self.write_cmd(addr, 0x06, 0x03)
        self.write_cmd(addr, 0x07, 0x00)
        self.write_cmd(addr, 0x08, 0xFF)

        self.write_cmd(addr, 0x09, 0xCA)
        self.write_cmd(addr, 0x0A, 0x85)
        self.write_cmd(addr, 0x0B, 0x2C)
        self.write_cmd(addr, 0x0E, 0xff)
        self.write_cmd(addr, 0x0F, 0x80)
        self.write_cmd(addr, 0x14, 0x0C)
        self.write_cmd(addr, 0x15, 0x0C)
        self.write_cmd(addr, 0x17, 0x02)
        self.write_cmd(addr, 0x18, 0x26)
        self.write_cmd(addr, 0x19, 0x77)
        self.write_cmd(addr, 0x1A, 0xF4)
        self.write_cmd(addr, 0x1B, 0x66)
        self.write_cmd(addr, 0x1C, 0x44)
        self.write_cmd(addr, 0x1E, 0x00)
        self.write_cmd(addr, 0x1F, 0x0C)
        self.write_cmd(addr, 0x20, 0x1A)
        self.write_cmd(addr, 0x21, 0x1A)
        
        self.write_cmd(addr, 0x00, 0x80)
        self.write_cmd(addr, 0x01, 0x3A)
        self.write_cmd(addr, 0x16, 0x3F)
        self.write_cmd(addr, 0x16, 0x00)
        

class Mic(object):
    def __init__(self,bits=16,sample_rate=16000,channels=1):
        self.mic = Es7243e(i2c)
        self.i2s = I2S(0,sck = 0, ws = 38, sd = 39,mode=I2S.RX, bits=bits, 
                       format=I2S.MONO if channels == 1 else I2S.STEREO, rate=sample_rate, ibuf=20000)
        self.i2s.deinit()
        self.bits = bits
        self.sample_rate = sample_rate
        self.channels = channels
        self.time = time
    def reinit(self,bits=16,sample_rate=16000,channels=1):
        self.i2s = I2S(0,sck = 0, ws = 38, sd = 39,mode=I2S.RX, bits=bits, 
                       format=I2S.MONO if channels == 1 else I2S.STEREO, rate=sample_rate, ibuf=20000)


    def write_wav_header(self, file, num_samples):
        # 计算文件大小
        byte_rate = self.sample_rate * self.channels * self.bits // 8
        block_align = self.channels * self.bits // 8
        data_size = num_samples * block_align
        file_size = data_size + 36
        
        # 写入 WAV 文件头
        file.write(b'RIFF')
        file.write(file_size.to_bytes(4, 'little'))
        file.write(b'WAVE')
        file.write(b'fmt ')  # 子块ID
        file.write((16).to_bytes(4, 'little'))  # 子块大小
        file.write((1).to_bytes(2, 'little'))   # 音频格式（1是PCM）
        file.write(self.channels.to_bytes(2, 'little'))
        file.write(self.sample_rate.to_bytes(4, 'little'))
        file.write(byte_rate.to_bytes(4, 'little'))
        file.write(block_align.to_bytes(2, 'little'))
        file.write(self.bits.to_bytes(2, 'little'))
        file.write(b'data')
        file.write(data_size.to_bytes(4, 'little'))

    def recode_to_wav(self,path,time):
        self.reinit(bits = self.bits, sample_rate = self.sample_rate, channels=self.channels)
        #创建录音缓存区
        buffer_size = 1024
        audio_buf = bytearray(buffer_size)

        #打开WAV文件
        with open(path, 'wb') as wav_file:
            #暂时写入WAV文件头，后续更新数据大小
            self.write_wav_header(wav_file,num_samples=0)

            #开始录音
            num_samples = 0
            start_time = self.time.time()
            while self.time.time() - start_time < time:
                #从I2S中读取数据
                self.i2s.readinto(audio_buf)
                wav_file.write(audio_buf)
                num_samples += len(audio_buf) // (self.bits // 8)
            #更新文件头中的实际数据大小
            wav_file.seek(0)
            self.write_wav_header(wav_file, num_samples)
        self.i2s.deinit()
        print("Recording saved to:", path)
    def recode_sys(self, name="",time=10):
        full_path = "/" + name
        self.recode_to_wav(path=full_path, time=time)

    def recode_tf(self, name="",time=10):
        full_path = "/sd/" + name
        self.recode_to_wav(path=full_path, time=time)

class TF_card(object):
    def __init__(self):
        self.spi_bus = SPI.Bus(host = 1, mosi = 42, miso = 41, sck = 44)
        self.sd = SDCard(spi_bus = self.spi_bus, cs = 40, freq = 1000000)
        try:
            vfs.mount(self.sd, "/sd")
        except:
            print("SD card not detected")




