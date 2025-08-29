import time
import os
import sys

from media.sensor import *
from media.display import *
from media.media import *
from time import ticks_ms
from machine import FPIOA
from machine import Pin
from machine import PWM
from machine import Timer
from machine import UART
import time
from machine import TOUCH

sensor = None

try:
    offset_x = (800 - 512) // 2
    offset_y = (480 - 384) // 2
    fpioa = FPIOA()
    fpioa.help()
    fpioa.set_function(53, FPIOA.GPIO53)
    fpioa.set_function(11, FPIOA.UART2_TXD)
    fpioa.set_function(12, FPIOA.UART2_RXD)
    detection_start_time = None  # 用于记录检测到目标的时间
    should_stop = False         # 停止标志
    key = Pin(53, Pin.IN, Pin.PULL_DOWN)
    uart2 = UART(UART.UART2, 115200)
    print("camera_test")

#    sensor = Sensor(width=1920, height=1080)
    sensor = Sensor(width=320, height=320)

    sensor.reset()

    # 鼠标悬停在函数上可以查看允许接收的参数
    #    sensor = Sensor(width=1920, height=1080)

    sensor.set_framesize(width=320, height=320)
    sensor.set_pixformat(Sensor.RGB565)

    Display.init(Display.ST7701,width=800, height=480,to_ide=True)
    # 初始化媒体管理器
    MediaManager.init()
    # 启动 sensor
    sensor.run()
    clock = time.clock()
    flag1 = 0
    flag0 = 0
    flag2 = 0
    flag4 = 0
    flag3 = 0
    flag5 = 0
    touch_counter = 0
    tp = TOUCH(0)
    scale = 1
    cut_roi = (720, 460, 480, 480)
    zuobiao = [(70,126),(81,379),(459,339),(414,71)]
    def show_img_2_screen():
        global img
        global scale
        if img.height() > 480 or img.width() > 800:
            scale = max(img.height() // 480, img.width() // 800) + 1
            img.midpoint_pool(scale, scale)
        img.compress_for_ide()
        Display.show_image(img, x=(800-img.width())//2, y=(480-img.height())//2)
    while True:
        clock.tick()
        os.exitpoint()
        info = uart2.read()
        img = sensor.snapshot(chn = CAM_CHN_ID_0)
#        img = img.copy(roi = cut_roi)
        print(info)
        if info == b'nextclb':
            flag2 = 1
            flag0 = 0
            flag1 = 1
            flag4 = flag4 + 1
        elif info == b'start':
            flag0 = 1
            flag1 = 0
            flag2 = 0
        elif info == b'trackOK':
            flag0 = 1
            flag1 = 0
            flag2 = 0
        elif info == b'setOK':
            flag0 = 1
            flag1 = 0
            flag2 = 0
        elif info == b'r':
            flag0 = 0
            flag1 = 0
            flag2 = 0
            flag4 = 0
            flag5 = 0
        elif info == b'tgstart':
            flag3 = 1
            flag0 = 0
        elif info == b'tgOK':
            flag5 = flag5 + 1
        elif info == b'stop':
            flag0 = 0
        elif info == b'tgstop':
            flag3 = 0
        if flag0 == 1:
            blobs = img.find_blobs([(96, 100, -6, 30, -23, 27)], False,\
                                        x_stride=5, y_stride=5, \
                                       pixels_threshold=50, margin=True)
            for blob in blobs:
                img.draw_rectangle(blob.x() ,blob.y(), blob.w(), blob.h(), color=(0, 255, 0), thickness=2, fill=False)
                v_x = blob.x() + blob.w() / 2
                v_y = blob.y() + blob.h() / 2
                # 固定总长度 6（3位整数 + 1小数点 + 2位小数）
                s1 = f"{v_x:06.2f}"
                s2 = f"{v_y:06.2f}"
                uart2.write(f"X{s1},Y{s2}")  # 发送格式如 "X123.46,Y045.00"
                print(s1+' '+s2)
    #           print("center_point: {}".format([round(v_x), round(v_y)]))
        if flag1 == 1 and flag4 == 1:
            x = zuobiao[1][0]
            y = zuobiao[1][1]
            s1 = f"{x:06.2f}"
            s2 = f"{y:06.2f}"
            uart2.write(f"#{s1},!{s2}")  # 发送格式如 "#123.46,!045.00"
            print(s1+' '+s2)
        if flag2 == 1 and flag4 == 2:
            x = zuobiao[3][0]
            y = zuobiao[3][1]
            s1 = f"{x:06.2f}"
            s2 = f"{y:06.2f}"
            uart2.write(f"#{s1},!{s2}")  # 发送格式如 "#123.46,!045.00"
            print(s1+' '+s2)
        if flag3 == 1:
            x = zuobiao[flag5][0]
            y = zuobiao[flag5][1]
            s1 = f"{x:06.2f}"
            s2 = f"{y:06.2f}"
            uart2.write(f"#{s1},!{s2}")
            print(s1+" "+s2)
        show_img_2_screen()
except KeyboardInterrupt as e:
    print("用户停止: ", e)
except BaseException as e:
    print(f"异常: {e}")
finally:
    if isinstance(sensor, Sensor):
        sensor.stop()
    Display.deinit()
    os.exitpoint(os.EXITPOINT_ENABLE_SLEEP)
    time.sleep_ms(100)
    MediaManager.deinit()

