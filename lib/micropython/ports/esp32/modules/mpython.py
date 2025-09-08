
from machine import I2C, PWM, Pin, ADC, TouchPad,Timer
import esp, math, time, network
import ustruct, array
from neopixel import NeoPixel
# from esp import dht_readinto
from time import sleep_ms, sleep_us, sleep
import framebuf 
from micropython import schedule,const


i2c = I2C(0, scl=Pin(48), sda=Pin(47), freq=100000)

class PinMode(object):
    IN = 1
    OUT = 2
    PWM = 3
    ANALOG = 4
    OUT_DRAIN = 5

pins_remap_k10 = (1, 2, 57, 66, 65, 64, 63, -1, 60, 61, 62, 52, 53, 54, 55, 56, -1, -1, -1, 48, 47)

'''
定时读取IO扩展的所有IO口，这样可以避免频繁操作不同IO口时，频繁的去读取
也是为了同步解决所有IO都设置为了输入时，因为是浮空状态会频繁的产生中断去读，就会导致I2C总线上拥挤
'''
__extio_dfrobot_value = bytearray([0x00, 0x00])  # 确保变量在全局范围内初始化
def dfrobot_global_extio_timer_handler(_):
    global __extio_dfrobot_value
    i2c.writeto(0x20, bytearray([0x00]))
    __extio_dfrobot_value = i2c.readfrom(0x20, 2)
    dfrobot_global_irq_handler_timer()

class extIO(object):
    """
    引脚拓展模块控制类

    :param i2c: I2C实例对象,默认i2c=i2c.
    输入端口寄存器
    地址：0x00和0x01
    功能：读取16个IO引脚的输入电平
    0x00对应P00-P07，0x01对应P10-P17

    输出端口寄存器
    地址：0x02和0x03
    功能：控制16个IO引脚的输出电平
    0x02对应P00-P07，0x03对应P10-P17

    极性反转寄存器
    地址：0x04，0x05
    功能：反转输入信号的逻辑电平
    0x04对应P00-P7，0x05对应P10-P17
    暂不使用

    配置寄存器
    地址：0x06和0x07
    功能：配置引脚为输入或输出
    0x06对应P00-P07，0x07对应P10-P17
    设置为1时为输入，为0时为输出
    """

    OUTPUT = const(0)
    """引脚输出类型常量"""

    INPUT = const(1)
    """引脚输入类型常量"""
    def __init__(self, pin, mode):
        self._i2c = i2c
        self._addr = 0x20
        self.pin = pin
        """
        引脚初始化

        :param pin: 拓展引脚,0~7
        :param mode: 引脚模式输入或输出;OUTPUT,INPUT
        """
        real_pin = 0
        if not hasattr(extIO, '__tim'):
            # 确保只注册一次中断
            pass
            #extIO.__tim = Timer(18)
            #extIO.__tim.init(period=50, mode=Timer.PERIODIC, callback=dfrobot_global_extio_timer_handler)
        if pin >= 60:
            self._i2c.writeto(self._addr, bytearray([0x07]))
            real_pin = pin - 60
        else:
            self._i2c.writeto(self._addr, bytearray([0x06]))
            real_pin = pin - 50
        mode_old = self._i2c.readfrom(0x20, 1)
        mode_new = 0
        if mode == PinMode.IN:
            mode_new = mode_old[0] | (1 << (real_pin))
        elif mode == PinMode.OUT:
            mode_new = mode_old[0] & (~(1 << real_pin))
        
        if pin >= 60:
            cfg = bytearray([0x07, mode_new])
        else:
            cfg = bytearray([0x06, mode_new])
        self._i2c.writeto(0x20, cfg)


    def value(self, value=None):
        if value == None:
            return self.readIO()
        else:
            return self.writeIO(value)
    
    def readIO(self):
        """
        读引脚

        :param pin: 拓展引脚,0~7
        """
        '''
        real_pin = 0
        if self.pin >= 60:
            real_pin = self.pin - 60
            reg = bytearray([0x01])
        else:
            real_pin = self.pin - 50
            reg = bytearray([0x00])
        self._i2c.writeto(0x20, reg)
        dat = self._i2c.readfrom(0x20, 1)
        return (dat[0] >> real_pin) & 0x01
        '''
        real_pin = 0
        dat = 0
        if self.pin >= 60:
            real_pin = self.pin - 60
            dat = __extio_dfrobot_value[1]
        else:
            real_pin = self.pin - 50
            dat = __extio_dfrobot_value[0]
        return (dat >> real_pin) & 0x01

    def writeIO(self, output):
        """
        写引脚

        :param pin: 拓展引脚,0~7
        :param output: 电平值;1 or 0
        """
        real_pin = 0
        if self.pin >= 60:
            real_pin = self.pin - 60
            reg = bytearray([0x03])
        else:
            real_pin = self.pin - 50
            reg = bytearray([0x02])
        
        self._i2c.writeto(0x20, reg)
        stat_old = self._i2c.readfrom(0x20, 1)
        stat_new = 0
        if output == 1:
            stat_new = stat_old[0] | (1 << real_pin)
        elif output == 0:
            stat_new = stat_old[0] & (~(1 << real_pin))
        cfg = bytearray([reg[0], stat_new])
        self._i2c.writeto(0x20, cfg)    

class MPythonPin():
    def __init__(self, pin, mode=PinMode.IN, pull=None):
        if mode not in [PinMode.IN, PinMode.OUT, PinMode.PWM, PinMode.ANALOG, PinMode.OUT_DRAIN]:
            raise TypeError("mode must be 'IN, OUT, PWM, ANALOG,OUT_DRAIN'")
        '''
        if pin == 4:
            raise TypeError("P4 is used for light sensor")
        if pin == 10:
            raise TypeError("P10 is used for sound sensor")
        '''
        try:
            self.id = pins_remap_k10[pin]
            if self.id == -1:
                raise IndexError('P%d is not defined' % pin)
        except IndexError:
            raise IndexError("Out of Pin range")
        if self.id >= 50:
            #使用扩展IO
            self.Pin = extIO(self.id, mode)
            self.mode = mode
        else:
            if mode == PinMode.IN:
                # if pin in [3]:
                #     raise TypeError('IN not supported on P%d' % pin)
                self.Pin = Pin(self.id, Pin.IN, pull)
            if mode == PinMode.OUT:
                if pin in [2, 3]:
                    raise TypeError('OUT not supported on P%d' % pin)
                self.Pin = Pin(self.id, Pin.OUT, pull)
            if mode == PinMode.OUT_DRAIN:
                if pin in [2, 3]:
                    raise TypeError('OUT_DRAIN not supported on P%d' % pin)
                self.Pin = Pin(self.id, Pin.OPEN_DRAIN, pull)
            if mode == PinMode.PWM:
                if pin not in [0,1]:
                    raise TypeError('PWM not supported on P%d' % pin)
                self.pwm = PWM(Pin(self.id), duty=0)
            if mode == PinMode.ANALOG:
                if pin not in [0, 1]:
                    raise TypeError('ANALOG not supported on P%d' % pin)
                self.adc = ADC(Pin(self.id))
                self.adc.atten(ADC.ATTN_11DB)
            self.mode = mode

    def irq(self, handler=None, trigger=Pin.IRQ_RISING):
        if self.id >= 50:
            raise TypeError('this pin does not support interrupts')
        if not self.mode == PinMode.IN:
            raise TypeError('the pin is not in IN mode')
        return self.Pin.irq(handler, trigger)

    def read_digital(self):
        if not self.mode == PinMode.IN:
            raise TypeError('the pin is not in IN mode')
        return self.Pin.value()

    def write_digital(self, value):
        if self.mode not in [PinMode.OUT, PinMode.OUT_DRAIN]:
            raise TypeError('the pin is not in OUT or OUT_DRAIN mode')
        self.Pin.value(value)

    def read_analog(self):
        if self.id >= 50:
            raise TypeError('the pin is not in ANALOG mode')
        if not self.mode == PinMode.ANALOG:
            raise TypeError('the pin is not in ANALOG mode')
        return self.adc.read()
        

    def write_analog(self, duty, freq=1000):
        if self.id >= 50:
            raise TypeError('the pin is not in PWM mode')
        if not self.mode == PinMode.PWM:
            raise TypeError('the pin is not in PWM mode')
        self.pwm.freq(freq)
        self.pwm.duty(duty)

class AHT20():
    ## Default I2C address of AHT20 sensor 
    AHT20_DEF_I2C_ADDR           = 0x38
    ## Init command
    CMD_INIT                     = 0xBE  
    ## The first parameter of init command: 0x08
    CMD_INIT_PARAMS_1ST          = 0x08  
    ## The second parameter of init command: 0x00
    CMD_INIT_PARAMS_2ND          = 0x00  
    ## Waiting time for init completion: 0.01s
    CMD_INIT_TIME                = 0.01    
    ## Trigger measurement command
    CMD_MEASUREMENT              = 0xAC  
    ## The first parameter of trigger measurement command: 0x33
    CMD_MEASUREMENT_PARAMS_1ST   = 0x33  
    ## The second parameter of trigger measurement command: 0x00
    CMD_MEASUREMENT_PARAMS_2ND   = 0x00  
    ## Measurement command completion time：0.08s
    CMD_MEASUREMENT_TIME         = 0.08   
    ## Return data length when the measurement command is without CRC check.
    CMD_MEASUREMENT_DATA_LEN     = 6     
    ## Return data length when the measurement command is with CRC check.
    CMD_MEASUREMENT_DATA_CRC_LEN = 7     
    ## Soft reset command
    CMD_SOFT_RESET               = 0xBA  
    ## Soft reset time: 0.02s
    CMD_SOFT_RESET_TIME          = 0.02   
    ## Get status word command
    CMD_STATUS                   = 0x71

    _humidity = 0.0
    _temperature = 0.0

    def __init__(self):
        self._addr = self.AHT20_DEF_I2C_ADDR
        self._value_buffer = bytearray(1)
        self._i2c = i2c
        self.reset()
        #self.init()
    def init(self):
      #解决新版硬件异常问题
        if self._ready():
            print("true 0")
        else:
            print("false 0")
        self._write_command_args(self.CMD_INIT, self.CMD_INIT_PARAMS_1ST, self.CMD_INIT_PARAMS_2ND)
        time.sleep(self.CMD_INIT_TIME)
        if self._ready():
            print("true 1")
        else:
            print("false 1")

    def reset(self):
      '''!
        @brief   Sensor soft reset, restore the sensor to the initial status
        @return  NONE
      '''
      self._write_command(self.CMD_SOFT_RESET)
      time.sleep(self.CMD_SOFT_RESET_TIME)

    def _write_command(self, cmd):
      self._value_buffer[0] = cmd
      self._i2c.writeto(self._addr, self._value_buffer)
      time.sleep(self.CMD_SOFT_RESET_TIME)

    def _write_command_args(self, cmd, args1, args2):
      #
      l = bytearray(2)
      l[0] = args1
      l[1] = args2
      self._write_bytes(cmd, l)

    def _read_bytes(self, reg, size):
        rslt = self._i2c.readfrom_mem(self._addr, reg, size)
        return rslt

    
    def _write_bytes(self, reg, buf):
        self._i2c.writeto_mem(self._addr, reg, buf)
        return len(buf)
    def _check_crc8(self, crc8, data):
      # CRC initial value: 0xFF
      # CRC8 check polynomial: CRC[7: 0] = X8 + X5 + X4 + 1  -  0x1 0011 0001 - 0x131
      crc = 0xFF
      pos = 0
      size = len(data)
      #print(data)
      while pos < size:
        i = 8
        #crc &= 0xFF
        crc ^= data[pos]
        while i > 0:
          if crc & 0x80:
            crc <<= 1
            crc ^= 0x31
          else:
            crc <<= 1
          i -= 1
        pos += 1
      crc &= 0xFF
      #print(crc)
      if crc8 == crc:
        return True
      return False
    
    def _ready(self):
      status = self._get_status_data()
      if status & 0x08:
        return True
      return False
    
    def _get_status_data(self):
      status = self._read_data(self.CMD_STATUS, 1)
      if len(status):
        return status[0]
      else:
        return 0
      
    def _read_data(self, cmd, len):
      return self._read_bytes(cmd, len)
    
    def _start_measurement_ready(self, crc_en = False):
      '''!
        @brief   Start measurement and determine if it's completed.
        @param crc_en Whether to enable check during measurement
        @n     True  If the measurement is completed, call a related function such as get* to obtain the measured data.
        @n     False If the measurement failed, the obtained data is the data of last measurement or the initial value 0 if the related function such as get* is called at this time.
        @return  Whether the measurement is done
        @retval True  If the measurement is completed, call a related function such as get* to obtain the measured data.
        @retval False If the measurement failed, the obtained data is the data of last measurement or the initial value 0 if the related function such as get* is called at this time.
      '''
      recv_len = self.CMD_MEASUREMENT_DATA_LEN
      if self._ready() == False:
        print("Not cailibration.")
        return False
      if crc_en:
        recv_len = self.CMD_MEASUREMENT_DATA_CRC_LEN
      self._write_command_args(self.CMD_MEASUREMENT, self.CMD_MEASUREMENT_PARAMS_1ST, self.CMD_MEASUREMENT_PARAMS_2ND)
      time.sleep(self.CMD_MEASUREMENT_TIME)
      #l_data = self._read_data(0x00, recv_len)
      l_data = self._i2c.readfrom(self._addr, recv_len)
      #print(l_data)
      if l_data[0] & 0x80:
        print("AHT20 is busy!")
        return False
      if crc_en and self._check_crc8(l_data[6], l_data[:6]) == False:
        print("crc8 check failed.")
        return False
      temp = l_data[1]
      temp <<= 8
      temp |= l_data[2]
      temp <<= 4
      temp = temp | (l_data[3] >> 4)
      temp = (temp & 0xFFFFF) * 100.0
      self._humidity = temp / 0x100000
  
      temp = l_data[3] & 0x0F
      temp <<= 8
      temp |= l_data[4]
      temp <<= 8
      temp |= l_data[5]
      temp = (temp & 0xFFFFF) * 200.0
      self._temperature = temp / 0x100000 - 50
      return True
    
    def temperature(self):
        return round(self._temperature, 2)
    
    def humidity(self):
        return round(self._humidity, 2)
    
    def measure(self):
        self._start_measurement_ready(crc_en = True)
        pass 

'''
加速度计
'''
class Accelerometer():
    SHANK = 0          # Shake gesture
    SCREEN_UP = 1       # Screen facing up
    SCREEN_DOWN = 2     # Screen facing down
    TILT_LEFT = 3       # Tilt left
    TILT_RIGHT = 4      # Tilt right
    TILT_FORWARD = 5    # Tilt forward
    TILT_BACK = 6       # Tilt backward
    GESTURE_NONE = 7     # No gesture detected
    def __init__(self) :
        self.shake_status = False
        self._i2c = i2c
        self._addr = 0x19
        self._gesture = self.GESTURE_NONE
        self.X = 0.0
        self.Y = 0.0
        self.Z = 0.0
        self._begin()
        self._measure()

    def _begin(self):
        buf = self._read_bytes(0x24, 1)
        data = buf[0]
        data |= 0x08
        self._writeReg(0x24, data)

        data = 0x00
        data |= 0x40
        data |= 0x03
        data |= 0x0C
        data |= 0x38
        self._writeReg(0x30, data)

        buf = self._read_bytes(0x21, 1)
        data = buf[0]
        data |= 0x81
        self._writeReg(0x21, data)
        self._writeReg(0x32, 0x60)
        self._writeReg(0x33, 0x02)

        buf = self._read_bytes(0x22, 1)
        data = buf[0]
        data |= 0x40
        self._writeReg(0x21, data)

        buf = self._read_bytes(0x24, 1)
        data = buf[0]
        data |= 0x02
        self._writeReg(0x24, data)

        buf = self._read_bytes(0x25, 1)
        data = buf[0]
        data |= 0x02
        self._writeReg(0x25, data)

        data = 0x00
        data |= 0xc0
        data |= 0x3f
        self._writeReg(0x34, data)

        buf = self._read_bytes(0x21, 1)
        data = buf[0]
        data |= 0xfd
        self._writeReg(0x21, data)

        self._writeReg(0x36, 0x18)
        self._writeReg(0x37, 0x02)

        buf = self._read_bytes(0x25, 1)
        data = buf[0]
        data |= 0x20
        self._writeReg(0x21, data)

        self._read_bytes(0x23, 1)
        self._writeReg(0x23, 0x88)
        self._writeReg(0x20, 0x27)


    def _read_bytes(self, reg, size):
        rslt = self._i2c.readfrom_mem(self._addr, reg, size)
        return rslt
    def _writeReg(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, value.to_bytes(1, 'little'))
    def _measure(self):
        tempbuf = self._read_bytes(0x27,1)
        if (tempbuf[0] & 0x0F) == 0x0F:
            accbuf = self._read_bytes(0xA8, 6)
            self.X = (accbuf[1]<<8 | accbuf[0]) >> 4
            self.Y = (accbuf[3]<<8 | accbuf[2]) >> 4
            self.Z = (accbuf[5]<<8 | accbuf[4]) >> 4
            if (self.X & 0x800) == 0x800:
                self.X -= 4096
            if (self.Y & 0x800) == 0x800:
                self.Y -= 4096
            if (self.Z & 0x800) == 0x800:
                self.Z -= 4096
            self.X = self.X / 1024.0
            self.Y = self.Y / 1024.0
            self.Z = self.Z / 1024.0
        tempbuf = self._read_bytes(0x35,1)

        if(tempbuf[0] & 0x60) == 0x60:
            self._gesture = self.SCREEN_DOWN
        elif (tempbuf[0] & 0x50) == 0x50:
            self._gesture = self.SCREEN_UP
        elif (tempbuf[0] & 0x41) == 0x41:
            self._gesture = self.TILT_LEFT
        elif (tempbuf[0] & 0x42) == 0x42:
            self._gesture = self.TILT_RIGHT
        elif (tempbuf[0] & 0x44) == 0x44:
            self._gesture = self.TILT_FORWARD
        elif (tempbuf[0] & 0x48) == 0x48:
            self._gesture = self.TILT_BACK
        elif (tempbuf[0] != 0):
            self._gesture = self.SHANK
       

    def x(self):
        #self.X = _accelerometer.get_x()
        return self.X

    def y(self):
        #self.Y = _accelerometer.get_y()
        return self.Y

    def z(self):
        #self.Z = _accelerometer.get_z()
        return self.Z

    def shake(self):
        return self.shake_status
    
class Light():
    LTR303_DATA_CH1_0    = 0x88
    LTR303ALS_CTRL       = 0x80
    LTR303ALS_GAIN_MODE  = 0x01
    LTR303ALS_MEAS_RATE  = 0x85
    LTR303ALS_INTEG_RATE = 0x03
    def __init__(self):
        self._i2c = i2c
        self._addr = 0x29
        self._begin()
    def _begin(self):
        self._writeReg(self.LTR303ALS_CTRL, self.LTR303ALS_GAIN_MODE)
        self._writeReg(self.LTR303ALS_MEAS_RATE, self.LTR303ALS_INTEG_RATE)
        pass
    def _writeReg(self, reg, value):
        self._i2c.writeto_mem(self._addr, reg, value.to_bytes(1, 'little'))

    def _read_bytes(self, reg, size):
        rslt = self._i2c.readfrom_mem(self._addr, reg, size)
        return rslt
    def read(self):
        _als = 0
        buf = self._read_bytes(self.LTR303_DATA_CH1_0,4)
        _als_ch1 = (buf[1] << 8) | buf[0]
        _als_ch0 = (buf[3] << 8) | buf[2]
        if(_als_ch1 + _als_ch0) != 0:
            _ratio = _als_ch1/(_als_ch1 + _als_ch0)
        else:
            _ratio = 0
        if _ratio < 0.45:
            _als = (1.7743 * _als_ch0 + 1.1059 * _als_ch1)
        elif _ratio < 0.64 and _ratio >= 0.45:
            _als = (4.2785*_als_ch0 - 1.9548*_als_ch1)
        elif _ratio<0.85 and _ratio>=0.64:
            _als = (0.5926*_als_ch0 + 0.1185*_als_ch1)
        else:
            _als = 0
        return round(_als, 2)

# -*- coding: utf-8 -*
"""
    I2C PN7150近场通讯NFC模块
"""
class Rfid(object):
    """!
    @brief Define rfid basic class
    """
    # MT=1 GID=0 OID=0 PL=1 ResetType=1 (Reset Configuration)
    NCI_CORE_RESET_CMD = b"\x20\x00\x01\x01"
    # MT=1 GID=0 OID=1 PL=0
    NCI_CORE_INIT_CMD = b"\x20\x01\x00"
    # MT=1 GID=f OID=2 PL=0
    NCI_PROP_ACT_CMD = b"\x2f\x02\x00"
    # MT=1 GID=1 OID=0
    NCI_RF_DISCOVER_MAP_RW = (
        b"\x21\x00\x10\x05\x01\x01\x01\x02\x01\x01\x03\x01\x01\x04\x01\x02\x80\x01\x80"
    )
    # MT=1 GID=1 OID=3
    NCI_RF_DISCOVER_CMD_RW = b"\x21\x03\x09\x04\x00\x01\x02\x01\x01\x01\x06\x01"
    # MODE_POLL | TECH_PASSIVE_NFCA,
    # MODE_POLL | TECH_PASSIVE_NFCF,
    # MODE_POLL | TECH_PASSIVE_NFCB,
    # MODE_POLL | TECH_PASSIVE_15693,
    NCI_RF_DEACTIVATE_CMD = b"\x21\x06\x01\x00"

    PROT_UNDETERMINED = 0x0
    PROT_T1T = 0x1
    PROT_T2T = 0x2
    PROT_T3T = 0x3
    PROT_ISODEP = 0x4
    PROT_NFCDEP = 0x5
    PROT_T5T = 0x6
    PROT_MIFARE = 0x80
    PN7150_I2C_ADDR = 0x28

    _STATUS_OK = 0x00
    def __init__(self, i2c,serial_number=0):
        """!
        @brief Module I2C communication init
        @param i2c_addr - I2C communication address
        @param bus_num - I2C bus
        """
        self._addr = self.PN7150_I2C_ADDR
        self._i2c = i2c
        self._debug = False
        self._buf = bytearray(3 + 255)
        self.fw_version = self._buf[64]
        self.nfc_uid = [0]
        self.nfc_serial_number = serial_number
        self.nfc_protocol = 0
        self.block_data = [0 for i in range(16)]

    def connect(self):
        """!
        @brief Function connect.
        @return Boolean type, the result of operation
        """
        try:
            self._connect()
            ret = self._mode_rw()
        finally:
            # print("finally connect")
            pass
        return ret

    def read_block(self, block, index=None):
        """!
        @brief Read a byte from a specified block of a MIFARE Classic NFC smart card/tag.
        @param block - The number of the block to read from.
        @param index - The offset of the block.
        @return Read from the card.
        """
        data = self._read_data(block)
        if (
            data == "no card!"
            or data == "read error!"
            or data == "read timeout!"
            or data == "wake up error!"
            or data == "false"
        ):
            return None
        if index is None:
            return data
        else:
            return self.block_data[index - 1]

    def write_block(self, block,data,index=0,):
        """!
        @brief Write a byte to a MIFARE Classic NFC smart card/tag.
        @param block - The number of pages you want to writes the data.
        @param index - The offset of the data.
        @param data - The byte to be written.
        @return Boolean type, the result of operation
        """
        if isinstance(data, str):
            real_val = []
            for i in data:
                real_val.append(int(ord(i)))
            if len(real_val) < 16:
                for i in range(16 - len(real_val)):
                    real_val.append(0)
            elif len(real_val) > 16:
                return False
        if isinstance(data, list):
            real_val = []
            if len(data) < 16:
                for i in range(16 - len(data)):
                    data.append(0)
            elif len(data) > 16:
                return False
            real_val = data
        index = max(min(index, 16), 1)
        self.read_block(block)
        if isinstance(data, int):
            self.block_data[index - 1] = data
            self.write_data(block, self.block_data)
        else:
            block_data = [0 for i in range(index - 1)]
            block_data[index:] = real_val
            self.write_data(block, block_data)
        return True

    def write_data(self, block, data):
        """!
        @brief Write a block to a MIFARE Classic NFC smart card/tag.
        @param block - The number of the block to write to.
        @param data - The buffer of the data to be written.
        @return Boolean type, the result of operation
        """
        if isinstance(data, tuple):
            data = list(data)
        if isinstance(data, str):
            real_val = []
            for i in data:
                real_val.append(int(ord(i)))
            if len(real_val) < 16:
                for i in range(16 - len(real_val)):
                    real_val.append(0)
            elif len(real_val) > 16:
                return False
        if isinstance(data, list):
            real_val = []
            if len(data) < 16:
                for i in range(16 - len(data)):
                    data.append(0)
            elif len(data) > 16:
                return False
            real_val = data
        if block < 128 and ((block + 1) % 4 == 0 or block == 0):
            return False
        if block > 127 and block < 256 and (block + 1) % 16 == 0:
            return False
        if block > 255:
            return False
        if not self._scan() or self.scan_serial_num!=self.nfc_serial_number:
            return False
        cmd_auth = [0x40, int(block / 4), 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        resp = self._tag_cmd(cmd_auth)
        if 0 == len(resp) or 0 != resp[-1]:
            return False
        cmd_write_1 = [0x10, 0xA0, block]
        resp = self._tag_cmd(cmd_write_1)
        if 0 == len(resp) or 0 != resp[-1]:
            return False
        cmd_write_2 = [0x10]
        cmd_write_2[1:] = real_val
        resp = self._tag_cmd(cmd_write_2)
        if 0 == len(resp) or 0 != resp[-1]:
            return False
        return True

    def read_protocol(self):
        if not self._scan():
            return "no card!"
        if self.nfc_protocol == self.PROT_T2T:
            return "T2T"
        elif self.nfc_protocol == self.PROT_UNDETERMINED:
            return "undetermined"
        elif self.nfc_protocol == self.PROT_T1T:
            return "T1T"
        elif self.nfc_protocol == self.PROT_T3T:
            return "T3T"
        elif self.nfc_protocol == self.PROT_ISODEP:
            return "isodep"
        elif self.nfc_protocol == self.PROT_NFCDEP:
            return "nfcdep"
        elif self.nfc_protocol == self.PROT_T5T:
            return "T5T"
        elif self.nfc_protocol == self.PROT_MIFARE:
            return "mifare"
        else:
            return "Unknow"
                
    def _mode_rw(self):
        """!
        @brief Function mode Read/Write.
        @return Boolean type, the result of operation
        """
        self._write_block(self.NCI_RF_DISCOVER_MAP_RW)
        end = self._read_wait(10)
        return (
            end >= 4
            and self._buf[0] == 0x41
            and self._buf[1] == 0x00
            and self._buf[3] == self._STATUS_OK
        )

    def _start_discovery_rw(self):
        """!
        @brief Function Start Discovery Read/Write.
        @return Boolean type, the result of operation
        """
        self._write_block(self.NCI_RF_DISCOVER_CMD_RW)
        end = self._read_wait()
        return (
            end >= 4
            and self._buf[0] == 0x41
            and self._buf[1] == 0x03
            and self._buf[3] == self._STATUS_OK
        )

    def _stop_discovery(self):
        """!
        @brief Function stop Discovery.
        @return Boolean type, the result of operation
        """
        self._write_block(self.NCI_RF_DEACTIVATE_CMD)
        end = self._read_wait()
        return (
            end >= 4
            and self._buf[0] == 0x41
            and self._buf[1] == 0x06
            and self._buf[3] == self._STATUS_OK
        )

    def _tag_cmd(self, cmd, conn_id=0):
        """!
        @brief Function tag cmd.
        @param cmd - tag cmd
        @param conn_id - conn_id
        @return Data of the nfc module
        """
        self._buf[0] = conn_id
        self._buf[1] = 0x00
        self._buf[2] = len(cmd)
        end = 3 + len(cmd)
        self._buf[3:end] = bytearray(cmd[0:len(cmd)])
        self._write_block(self._buf, end=end)
        base = time.time() * 100
        timeout = 10
        while (time.time() * 100 - base) < timeout:
            end = self._read_wait(30)
            if self._buf[0] & 0xE0 == 0x00:
                break
            time.sleep(0.001)

        return self._buf[3:end]
    
    def read_uid(self):
        """!
        @brief Obtain the UID of the card .
        @return UID of the card.
        """
        if not self._scan():
            return None
        if self.nfc_uid is None:
            return None
        else:
            return "".join([str(hex(u))[2:] for u in self.nfc_uid])
        
    def _scan(self):
        ret= False
        self._stop_discovery()
        if self._start_discovery_rw():
            end = self._read_wait(30)
            if end == 0 or self._buf[0] != 0x61 or self._buf[1] != 0x05:
                return False
            self.nfc_uid = self._buf[13:17]
            self.scan_serial_num = int.from_bytes(bytes(self.nfc_uid))
            self.nfc_protocol = self._buf[5]
            ret = True
        
        return ret

    def _read_data(self, page):
        if page > 255:
            return "false"
        if not self._scan() or self.scan_serial_num!=self.nfc_serial_number:
            return "no card!"
        cmd_auth = [0x40, int(page / 4), 0x10, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF]
        resp = self._tag_cmd(cmd_auth)
        #print(resp)
        if 0 == len(resp) or 0 != resp[-1]:
            return "read error!"
        cmd_read = [0x10, 0x30, page]
        resp = self._tag_cmd(cmd_read)
        if 0 == len(resp) or 0 != resp[-1]:
            return "read timeout!"
        dataStr = ""
        for i in range(len(resp) - 2):
            self.block_data[i] = resp[i + 1]
            if resp[i + 1] <= 0x0F:
                # dataStr += "0"
                # dataStr += str(hex(resp[i + 1]))
                dataStr += hex(resp[i + 1])[2:].upper()
            else:
                # dataStr += str(hex(resp[i + 1]))
                dataStr += hex(resp[i + 1])[2:].upper()
            if i < len(resp) - 3:
                dataStr += "."
        return dataStr

    def _connect(self):
        """!
        @brief Function connect.
        """
        self._write_block(self.NCI_CORE_RESET_CMD)
        end = self._read_wait(30)
        if (
            end < 6
            or self._buf[0] != 0x40
            or self._buf[1] != 0x00
            or self._buf[3] != self._STATUS_OK
            or self._buf[5] != 0x01
        ):
            return False
        self._write_block(self.NCI_CORE_INIT_CMD)
        end = self._read_wait()
        if (
            end < 20
            or self._buf[0] != 0x40
            or self._buf[1] != 0x01
            or self._buf[3] != self._STATUS_OK
        ):
            return False

        nrf_int = self._buf[8]
        self.fw_version = self._buf[17 + nrf_int : 20 + nrf_int]
        self._write_block(self.NCI_PROP_ACT_CMD)
        end = self._read_wait()
        if (
            end < 4
            or self._buf[0] != 0x4F
            or self._buf[1] != 0x02
            or self._buf[3] != self._STATUS_OK
        ):
            return False
        return True

    def _read_wait(self, timeout=10):
        """!
        @brief read the data from the register
        @param timeout - timeout
        @return read data
        """
        base = time.time() * 100
        while (time.time() * 100 - base) < timeout:
            count = self._read_block()
            #print(count)
            if 3 < count:
                return count
            time.sleep(0.001)
        return 0

    def _read_block(self):
        """!
        @brief read the data from the register
        @return read data
        """
        end = 0 
        try:
            read_msg = self._i2c.readfrom(self._addr, 3)
            #print(f"read_msg1 {read_msg}")
            if None != read_msg:
                self._buf[0:2] = bytearray(read_msg)
                end = 3
                if self._buf[2] > 0:
                    read_msg = self._i2c.readfrom(self._addr, self._buf[2])
                    #print(f"read_msg2 {read_msg}")
                    if len(bytearray(read_msg)) == self._buf[2]:
                        self._buf[3:] = bytearray(read_msg)
                        end = 3 + self._buf[2] 
        except OSError as e:
            #print(f"I/O error: {e}")
            pass
        except Exception as e:
            print("An unexpected error occurred: {}".format(e))
        return end

    def _write_block(self, cmd, end=0):
        """!
        @brief writes data to a register
        @param cmd - written data
        @param end - data len
        """
        cycle_count = 5
        self._read_block()
        if not end:
            end = len(cmd)
        while cycle_count:
            try:
                self._i2c.writeto(self._addr, bytearray(cmd[0:end]))
                break
            except OSError as e:
                #print(f"_write_block I/O error: {e}")
                cycle_count -= 1
                self._read_block()
                time.sleep(0.01)
            except Exception as e:
                print("An unexpected error occurred: {}".format(e))

class Rfid_Edu(Rfid):
    def __init__(self, i2c, serial_number):
        self._serial_number = serial_number
        super().__init__(i2c,serial_number)

    def _get_serNum(self, serial_number):
        serNumCheck = 0
        buf = serial_number.to_bytes(4, 'little')
        for i in range(4):
            serNumCheck ^= buf[i]
        serNum_list = [int(i) for i in buf]
        serNum_list.append(serNumCheck)
        return (tuple(serNum_list))

    def serial_number(self):
        """
        获取序列号
        """
        return str(self._serial_number)


class Scan_Rfid_Edu():
    """扫描Rfid卡类.
    """
    @classmethod
    def scanning(cls, i2c):
        """
        扫描RFID卡,返回Rfid对象

        :param obj i2c: I2C实例对象
        :return: 返回Rfid对象
        """
        rfid_instance = Rfid(i2c)
        if not rfid_instance.connect():
            return None
        if rfid_instance.read_uid():
            serial_tuple = rfid_instance.scan_serial_num
            if serial_tuple:
                return Rfid_Edu(i2c, serial_tuple)
        return None

# 3 rgb leds
#rgb = NeoPixel(Pin(46, Pin.OUT), 3, 3, 1)
#rgb.write()

class wifi:
    def __init__(self):
        self.sta = network.WLAN(network.STA_IF)
        self.ap = network.WLAN(network.AP_IF)

    def connectWiFi(self, ssid, passwd, timeout=10):
        if self.sta.isconnected():
            self.sta.disconnect()
        self.sta.active(True)
        list = self.sta.scan()
        for i, wifi_info in enumerate(list):
            try:
                if wifi_info[0].decode() == ssid:
                    self.sta.connect(ssid, passwd)
                    wifi_dbm = wifi_info[3]
                    break
            except UnicodeError:
                self.sta.connect(ssid, passwd)
                wifi_dbm = '?'
                break
            if i == len(list) - 1:
                raise OSError("SSID invalid / failed to scan this wifi")
        start = time.time()
        print("Connection WiFi", end="")
        while (self.sta.ifconfig()[0] == '0.0.0.0'):
            if time.ticks_diff(time.time(), start) > timeout:
                print("")
                raise OSError("Timeout!,check your wifi password and keep your network unblocked")
            print(".", end="")
            time.sleep_ms(500)
        print("")
        print('WiFi(%s,%sdBm) Connection Successful, Config:%s' % (ssid, str(wifi_dbm), str(self.sta.ifconfig())))

    def disconnectWiFi(self):
        if self.sta.isconnected():
            self.sta.disconnect()
        self.sta.active(False)
        print('disconnect WiFi...')

    def enable_APWiFi(self, essid, password=b'',channel=10):
        self.ap.active(True)
        if password:
            authmode=4
        else:
            authmode=0
        self.ap.config(essid=essid,password=password,authmode=authmode, channel=channel)

    def disable_APWiFi(self):
        self.ap.active(False)
        print('disable AP WiFi...')


def dfrobot_global_irq_handler(pin):
    # 遍历所有按钮，根据引脚状态处理
    try:
        #time.sleep_ms(10)  # 防抖处理
        for pin_id, btn in __dfrobot_buttons.items():
            btn.check_state()
    except KeyboardInterrupt:
        # 清理所有按钮的中断
        for btn in __dfrobot_buttons.values():
            if hasattr(btn, '_Button__interrupt_pin'):
                btn._Button__interrupt_pin.irq(handler=None)
        # 重新抛出KeyboardInterrupt，让REPL处理
        raise
def dfrobot_global_irq_handler_timer():
    # 遍历所有按钮，根据引脚状态处理
    try:
        #time.sleep_ms(10)  # 防抖处理
        if not __dfrobot_buttons:
            pass
        else:
            for btn in __dfrobot_buttons.values():
                btn.check_state()
    except KeyboardInterrupt:
        # 清理所有按钮的中断
        for btn in __dfrobot_buttons.values():
            if hasattr(btn, '_Button__interrupt_pin'):
                btn._Button__interrupt_pin.irq(handler=None)
        # 重新抛出KeyboardInterrupt，让REPL处理
        raise

__dfrobot_buttons = {}
class Button:
    def __init__(self, pin_num, reverse=False):
        """
        :param ext_io: extIO 实例，用于控制 I2C 扩展引脚
        :param pin_num: 按钮连接的扩展引脚编号
        :param interrupt_pin: ESP32 的 GPIO 引脚，用于检测中断信号（如 IO43）
        :param reverse: 是否反转逻辑，默认按下为低电平，释放为高电平
        """
        self.__reverse = reverse
        (self.__press_level, self.__release_level) = (0, 1) if not self.__reverse else (1, 0)
        self.id = pins_remap_k10[pin_num]
        self.__pin = None
        self.__last_value = None
        # 检查是否已经存在绑定到同一引脚的按钮
        if self.id in __dfrobot_buttons:
            print("Warning: Button already exists on pin", self.id)
            # 覆盖已有的按钮回调
            __dfrobot_buttons[self.id] = self
        else:
            # 添加到按钮字典中
            __dfrobot_buttons[self.id] = self

        if self.id >= 50:
            self.__pin = extIO(self.id, PinMode.IN)
        else:
            self.__pin = Pin(self.id, Pin.IN, pull=Pin.PULL_UP)
        self.event_pressed = None
        self.event_released = None
        self.__pressed_count = 0
        self.__was_pressed = False

        # 初始化按钮状态
        self.__last_value = self.__pin.value()
        # 配置中断引脚
        if self.id >= 50:
            #IO扩展芯片
            if not hasattr(Button, '__interrupt_pin'):
                # 确保只注册一次中断
                #Button.__tim = Timer(18)
                #Button.__tim.init(period=50, mode=Timer.PERIODIC, callback=dfrobot_global_extio_timer_handler)
                #Button.__interrupt_pin = Pin(43, Pin.IN)
                #Button.__interrupt_pin.irq(trigger=Pin.IRQ_FALLING, handler=dfrobot_global_irq_handler)
                pass
        else:
            #原生IO口
            self.__interrupt_pin = Pin(self.id, Pin.IN)
            self.__interrupt_pin.irq(trigger=Pin.IRQ_FALLING, handler=self.__irq_handler)

    def __irq_handler(self, pin):
        irq_falling = True if pin.value() == self.__press_level else False
        # debounce
        #time.sleep_ms(10)
        if self.__pin.value() == (self.__press_level if irq_falling else self.__release_level):
            # new event handler
            # pressed event
            if irq_falling:
                if self.event_pressed is not None:
                    schedule(self.event_pressed, self.__pin)
                # key status
                self.__was_pressed = True
                if (self.__pressed_count < 100):
                    self.__pressed_count = self.__pressed_count + 1
            # release event
            else:
                if self.event_released is not None:
                    schedule(self.event_released, self.__pin)

    def __irq_handler_ext(self, pin):
        """ 当检测到中断信号时，读取按钮状态 """
        time.sleep_ms(10)  # 防抖处理
        current_value = self.__pin.value()
        print("__irq_handler_ext=%d\n",current_value)
        # 按下事件
        if current_value == self.__press_level and self.__last_value != self.__press_level:
            if self.event_pressed is not None:
                schedule(self.event_pressed, None)
            self.__was_pressed = True
            if self.__pressed_count < 100:
                self.__pressed_count += 1

        # 释放事件
        elif current_value == self.__release_level and self.__last_value != self.__release_level:
            if self.event_released is not None:
                schedule(self.event_released, None)

        # 更新按钮状态
        self.__last_value = current_value

    def check_state(self):
        """检查按钮状态，并触发相应事件"""
        current_value = self.__pin.value()
        # 按下事件
        if current_value == self.__press_level and self.__last_value != self.__press_level:
            if self.event_pressed is not None:
                schedule(self.event_pressed, None)
            self.__was_pressed = True
            if self.__pressed_count < 100:
                self.__pressed_count += 1

        # 释放事件
        elif current_value == self.__release_level and self.__last_value != self.__release_level:
            if self.event_released is not None:
                schedule(self.event_released, None)

        # 更新按钮状态
        self.__last_value = current_value

    def is_pressed(self):
        if self.__pin.value() == self.__press_level:
            return True
        else:
            return False

    def was_pressed(self):
        """ 检查按钮是否曾被按下，并清除状态 """
        r = self.__was_pressed
        self.__was_pressed = False
        return r

    def get_presses(self):
        """ 获取按钮被按下的次数，并清零计数 """
        r = self.__pressed_count
        self.__pressed_count = 0
        return r

    def value(self):
        """ 获取当前引脚的状态 """
        return self.__pin.value()

    def status(self):
        """ 返回按钮的状态，按下返回 1，释放返回 0 """
        val = self.__pin.value()
        if(val==0):
            return 1
        elif(val==1):
            return 0
    def irq(self, *args, **kws):
        if self.id >= 50:
            pass
        else:
            self.__pin.irq(*args, **kws)

#P5和P11
button_a = Button(5)
button_b = Button(11)