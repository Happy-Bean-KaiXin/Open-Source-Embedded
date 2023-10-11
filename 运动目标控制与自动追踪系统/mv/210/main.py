from Maix import GPIO
from fpioa_manager import fm
from machine import UART, Timer
import sensor, image, time, lcd, utime

# 定义全局变量
global uart, blobs, tmp, blob1
red_threshold = (30, 100, 15, 127, -40, 127)  # 红色激光笔的颜色阈值

# lcd屏幕设置
lcd.init(type = 1, freq = 15000000, color = lcd.BLACK)
lcd.rotation(0)                     # 设置屏幕方向
lcd.mirror(0)
lcd.clear(lcd.WHITE)                # 清屏白色
lcd.draw_string(110, 120, "Happy Bean!!!", lcd.BLACK, lcd.WHITE)  # 显示字符

# 摄像头参数设置
sensor.reset()                      # 初始化摄像头
sensor.set_vflip(True)              #垂直翻转
sensor.set_hmirror(True) # 打开水平镜像 如果是 01Studio 的 K210 不开启会导致画面方向与运动方向相反
sensor.set_pixformat(sensor.RGB565) # 设置图像的像素格式
sensor.set_framesize(sensor.QVGA)   # 设置图像每帧的大小
sensor.skip_frames(time = 2000)     # Wait for settings take effect.
sensor.set_brightness(0)           # 设置亮度-3~3
sensor.set_contrast(0)              # 对比度-3~3
#sensor.set_gainceiling(16)           # 增益上限
#sensor.set_auto_gain(False,gain_db = -1) #增益

sensor.set_auto_gain(False)
sensor.set_auto_exposure(False,exposure_us = 500)  #曝光速度
sensor.set_auto_whitebal(False)

# 建立clock
clock = time.clock()                # Create a clock object to track the FPS.

# 函数定义
def UART_Init():
    # 注册串口引脚
    fm.register(6, fm.fpioa.UART1_RX, force = True)
    fm.register(7, fm.fpioa.UART1_TX, force = True)
    uart = UART(UART.UART1, 115200, read_buf_len = 4096)

def find_max(blobs):#取最大色块
    max_size=0
    for blob in blobs:
        if blob[2]*blob[3] > max_size:
            max_blob=blob
            max_size = blob[2]*blob[3]
    return max_blob

#识别红点
def find_redPoint(img,thre):
    blobs = img.find_blobs(thre)
    if blobs:
        max_blob = find_max(blobs)
        img.draw_rectangle(max_blob.rect(),color=(0,255,0))
        img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0, 255, 0),size=10)
        print(max_blob.cx(), max_blob.cy(), type(max_blob.cx()))

        #uart.write("X="+str(blob1.cx())+"Y="+str(blob1.cy())+'\n')
    else:
        #uart.write("None\n")
        print("None")

#识别矩形
def find_rect(img):
    for r in img.find_rects(threshold = 10000):
        img.draw_rectangle(r.rect(), color = (255, 0, 0))
        for p in r.corners():
            img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))

        out = clockwise_sort(r.corners())

        p1 = out[0]
        p2 = out[1]
        p3 = out[2]
        p4 = out[3]
        print(out)


if __name__ == "__main__":
    UART_Init()
    while(True):
        clock.tick()                    # 更新FPS时钟
        img = sensor.snapshot()         # 拍摄一个图像并保存.
        #img.draw_string(0, 0, "Fps: "+str(clock.fps()), color=(0, 0, 0), scale=1)
        find_redPoint(img, [red_threshold])


        lcd.display(img)                # 显示在LCD上

