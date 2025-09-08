import gc
import vfs
from flashbdev import bdev
from mpython import i2c
#失能功放
try:
    #k10原生会报异常
    i2c.writeto_mem(0x20,0x2A,bytearray([0x00]))
except:
    pass

try:
    if bdev:
        vfs.mount(bdev, "/")
except OSError:
    import inisetup

    inisetup.setup()

gc.collect()
