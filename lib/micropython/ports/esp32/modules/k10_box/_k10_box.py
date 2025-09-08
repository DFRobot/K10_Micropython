from k10_base import i2c
import time
import math
import struct

'''
int.from_bytes(bytearray([0x01,0x02]),'big') = 0x0102
int.from_bytes(bytearray([0x01,0x02]),'little') = 0x0201
'''
#获取声音大小值
class Voice(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.reg = 0x0D
    def read(self):
        data = self.i2c.readfrom_mem(self.addr, self.reg, 2)
        result = int.from_bytes(data,'big')
        return result

#获取旋钮大小值
class Knob(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.reg = 0x0F
    def read(self):
        data = self.i2c.readfrom_mem(self.addr, self.reg, 2)
        result = int.from_bytes(data,'big')
        return result

#获取红外接收码
class Ir(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.reg = 0x08
    def data(self):
        ir = 0
        data = self.i2c.readfrom_mem(self.addr, self.reg, 5)
        if data[0] == 1:
            ir = int.from_bytes(data[1:5], 'big')
        return ir
    
#获取SR04的距离值    
class Sr04(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.reg = 0x11
    def distance(self):
        distance = 0xFFFF
        self.i2c.writeto(self.addr, bytearray([self.reg, 0x01]))
        time.sleep(0.1)
        data = self.i2c.readfrom_mem(self.addr, self.reg, 3)
        if data[0] == 0x02:
            distance = int.from_bytes(data[1:3], 'big')
        return distance

class Buzzer(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.reg = 0x26
        self.duty_reg = 0x28
    def play(self,freq):
        if freq == 0:
            self.stop()
        realfreq = 1000000 // freq
        realduty = realfreq // 2
        tempbuf = realfreq.to_bytes(2,'big') + realduty.to_bytes(2,'big')
        self.i2c.writeto_mem(self.addr, self.reg, tempbuf)
    def stop(self):
        self.i2c.writeto_mem(self.addr, self.duty_reg, bytearray([0x00,0x00]))
        pass

class Led(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.freq_reg = 0x1E
        self.r_reg = 0x24
        self.y_reg = 0x22
        self.g_reg = 0x20
    def red_digital(self, state):
        if state == 0:
            self.red_analog(0)
        else:
            self.red_analog(255)

    def yellow_digital(self, state):
        if state == 0:
            self.yellow_analog(0)
        else:
            self.yellow_analog(255)

    def green_digital(self, state):
        if state == 0:
            self.green_analog(0)
        else:
            self.green_analog(255)

    def red_analog(self,value):
        self.i2c.writeto_mem(self.addr, self.freq_reg, bytearray([0x00, 0xFF]))
        self.i2c.writeto_mem(self.addr, self.r_reg, bytearray([0x00, value & 0xFF]))

    def yellow_analog(self, value):
        self.i2c.writeto_mem(self.addr, self.freq_reg, bytearray([0x00, 0xFF]))
        self.i2c.writeto_mem(self.addr, self.y_reg, bytearray([0x00, value & 0xFF]))

    def green_analog(self, value):
        self.i2c.writeto_mem(self.addr, self.freq_reg, bytearray([0x00, 0xFF]))
        self.i2c.writeto_mem(self.addr, self.g_reg, bytearray([0x00, value & 0xFF]))

class Motor(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x20
        self.freq_reg = 0x14
        self.m1_reg = 0x16
        self.m2_reg = 0x1A

    def set_m1_speed(self,m1a, m1b):
        realfreq = 1000
        m1a_val = (m1a * 1000) // 255
        m1b_val = (m1b * 1000) // 255
        self.i2c.writeto_mem(self.addr, self.freq_reg,  realfreq.to_bytes(2,'big'))
        tempbuf = m1a_val.to_bytes(2,'big') + m1b_val.to_bytes(2,'big')
        self.i2c.writeto_mem(self.addr, self.m1_reg,  tempbuf)
        
    def set_m2_speed(self, m2a, m2b):
        realfreq = 1000
        m2a_val = (m2a * 1000) // 255
        m2b_val = (m2b * 1000) // 255
        self.i2c.writeto_mem(self.addr, self.freq_reg,  realfreq.to_bytes(2,'big'))
        tempbuf = m2a_val.to_bytes(2,'big') + m2b_val.to_bytes(2,'big')
        self.i2c.writeto_mem(self.addr, self.m2_reg,  tempbuf)

    def M1_run(self,dir,speed):
        #正转(前进)
        if dir == 1:
            m1a = 0
            m1b = speed & 0xFF
        else:
            m1a = speed & 0xFF
            m1b = 0
        self.set_m1_speed(m1a, m1b)
    def M2_run(self,dir,speed):
        #正转(前进)
        if dir == 1:
            m2a = speed & 0xFF
            m2b = 0
        else:
            m2a = 0
            m2b = speed & 0xFF
        self.set_m2_speed(m2a, m2b)

    def M1_stop(self):
        self.set_m1_speed(0, 0)
        
    def M2_stop(self):
        self.set_m2_speed(0, 0)

class Line(object):
    def __init__(self):
        self.i2c = i2c
        self.addr = 0x30
        self.threshod_reg = 0x0E
        self.adc_reg = 0x09
        self.status_reg = 0x08
    def begin(self):
        pass
    def set_threshod(self,num,value):
        if num < 5:
            self.i2c.writeto_mem(self.addr, self.threshod_reg + num,  bytearray([value&0xFF]))
            time.sleep(0.1)
    #获取所有探头的阈值
    def get_threshod(self):
        threshod = self.i2c.readfrom_mem(self.addr, self.threshod_reg, 5)
        return threshod
    #获取所有探头的模拟值
    def get_adc_all(self):
        adc = self.i2c.readfrom_mem(self.addr, self.adc_reg, 5)
        return adc
    #获取所有探头的状态，输出二进制的字符串，保留5位
    def get_status_all(self):
        status = self.i2c.readfrom_mem(self.addr, self.status_reg, 1)
        status_int = int.from_bytes(status, 'big')
        status_bin = bin(status_int)[2:]
        status_bin = '0' * (8 - len(status_bin)) + status_bin
        last_5_bits = status_bin[-5:]
        reversed_bits = ''.join(reversed(last_5_bits))
        return reversed_bits

    #获取指定探头的状态，输出0或1
    def get_status(self,num):
        status = self.i2c.readfrom_mem(self.addr, self.status_reg, 1)
        if ((status[0] >> num) & 0x01) == 0x01:
            return 1
        else:
            return 0

class qmi8658(object):
    QMI8658_ACCRANGE_8G = 0x02 << 4
    QMI8658_ACCODR_250HZ = 0x05
    QMI8658_GRYORANG_1024DPS = 0x06 << 4
    QMI8658_GRYOODR_250HZ = 0x05

    def __init__(self):
        self.i2c = i2c
        self.addr = 0x6B
        self.ssvtA = 1<<12
        self.ssvtG = 32
        self._read_qmi8658c_id()
        self._set_qmi8658c_mode(0x00)
        self._set_qmi8658c_acc_config(self.QMI8658_ACCRANGE_8G, self.QMI8658_ACCODR_250HZ)
        self._set_qmi8658c_gyro_config(self.QMI8658_GRYORANG_1024DPS, self.QMI8658_GRYOODR_250HZ)
        self._set_qmi8658c_mode(0x03)
        self.accX = 0
        self.accY = 0
        self.accZ = 0
        self.gyroX = 0
        self.gyroY = 0
        self.gyroZ = 0
        

    def _set_qmi8658c_mode(self,mode):
        self._mode_reg = 0x08
        self.i2c.writeto_mem(self.addr, self._mode_reg, bytearray([mode]))

    def _set_qmi8658c_acc_config(self, rang, odr):
        if rang == self.QMI8658_ACCRANGE_8G:
            self.ssvtA = 1<<12
        reg_value = rang | odr
        self.i2c.writeto_mem(self.addr, 0x03, bytearray([reg_value]))
        temp = self.i2c.readfrom_mem(self.addr, 0x06, 1)
        reg_value = temp[0] & 0xF0
        reg_value &= ~0x01
        self.i2c.writeto_mem(self.addr, 0x06, bytearray([reg_value]))

    def _set_qmi8658c_gyro_config(self, rang, odr):
        if rang == self.QMI8658_GRYORANG_1024DPS:
            self.ssvtG = 32
        reg_value = rang | odr
        self.i2c.writeto_mem(self.addr, 0x04, bytearray([reg_value]))
        temp = self.i2c.readfrom_mem(self.addr, 0x06, 1)
        reg_value = temp[0] & 0x0F
        reg_value &= ~0x10
        self.i2c.writeto_mem(self.addr, 0x06, bytearray([reg_value]))

    def _read_qmi8658c_id(self):
        temp = self.i2c.readfrom_mem(self.addr, 0x00, 1)
        if temp[0] == 0x05:
            reg_value = (0x60 | 0x10 | 0x08)
            self.i2c.writeto_mem(self.addr, 0x02, bytearray([reg_value]))
            reg_value = 0x00
            self.i2c.writeto_mem(self.addr, 0x08, bytearray([reg_value]))
            reg_value = 0xC0
            self.i2c.writeto_mem(self.addr, 0x09, bytearray([reg_value]))
    def _read_qmi8658c_xyz(self):
        status = self.i2c.readfrom_mem(self.addr, 0x2E, 1)
        if (status[0] & 0x03):
            temp = self.i2c.readfrom_mem(self.addr, 0x35, 12)
            self.accX = struct.unpack('>h', bytes([temp[1], temp[0]]))[0]
            self.accY = struct.unpack('>h', bytes([temp[3], temp[2]]))[0]
            self.accZ = struct.unpack('>h', bytes([temp[5], temp[4]]))[0]
            self.gyroX = struct.unpack('>h', bytes([temp[7], temp[6]]))[0]
            self.gyroY = struct.unpack('>h', bytes([temp[9], temp[8]]))[0]
            self.gyroZ = struct.unpack('>h', bytes([temp[11], temp[10]]))[0]
    #单位mg
    def read_acc_x(self):
        self._read_qmi8658c_xyz()
        return (self.accX * 1000.0)/self.ssvtA

    def read_acc_y(self):
        self._read_qmi8658c_xyz()
        return (self.accY * 1000.0)/self.ssvtA
    
    def read_acc_z(self):
        self._read_qmi8658c_xyz()
        return (self.accZ * 1000.0)/self.ssvtA
    #单位dps
    def read_gyro_x(self):
        self._read_qmi8658c_xyz()
        return (self.gyroX * 1.0)/self.ssvtG
    
    def read_gyro_y(self):
        self._read_qmi8658c_xyz()
        return (self.gyroY * 1.0)/self.ssvtG

    def read_gyro_z(self):
        self._read_qmi8658c_xyz()
        return (self.gyroZ * 1.0)/self.ssvtG
    def read_acc_streng(self):
        self._read_qmi8658c_xyz()
        x = (self.accX * 1000.0)/self.ssvtA
        y = (self.accY * 1000.0)/self.ssvtA
        z = (self.accZ * 1000.0)/self.ssvtA
        return math.sqrt(x*x + y*y + z*z)


class Acc(qmi8658):
    def __init__(self):
        super().__init__()
    def read_x(self):
        return self.read_acc_x()
    def read_y(self):
        return self.read_acc_y()
    def read_z(self):
        return self.read_acc_z()
    def read_strength(self):
        return self.read_acc_streng()

class Gyro(qmi8658):
    def __init__(self):
        super().__init__()
    def read_x(self):
        return self.read_gyro_x()
    def read_y(self):
        return self.read_gyro_y()
    def read_z(self):
        return self.read_gyro_z()

voice = Voice()
knob = Knob()
ir = Ir()
sr04 = Sr04()
acc = Acc()
gyro = Gyro()
buzzer = Buzzer()
led = Led()
motor = Motor()
line = Line()