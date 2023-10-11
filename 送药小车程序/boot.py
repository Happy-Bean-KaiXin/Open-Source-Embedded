import sensor, image, lcd, time
import KPU as kpu
import gc, sys
from Maix import GPIO
from machine import UART
from fpioa_manager import fm

# 注册引脚，定义Tx与RX
fm.register(6, fm.fpioa.UART1_RX, force=True)
fm.register(7, fm.fpioa.UART1_TX, force=True)
fm.register(16, fm.fpioa.GPIO1)


KEY = GPIO(GPIO.GPIO1, GPIO.IN)

uart = UART(UART.UART1, 115200, read_buf_len=4096)

# 定义全局变量
target = 0   # 目标数 如 1 2 3
temp = 0
position = 0  # 转弯位置


def lcd_show_except(e):
    import uio
    err_str = uio.StringIO()
    sys.print_exception(e, err_str)
    err_str = err_str.getvalue()
    img = image.Image(size=(224,224))
    img.draw_string(0, 10, err_str, scale=1, color=(0xff,0x00,0x00))
    lcd.display(img)
def main(anchors, labels = None, model_addr="/sd/m.kmodel", sensor_window=(224, 224), lcd_rotation=0, sensor_hmirror=False, sensor_vflip=False):
    global target
    global temp
    global position
    sensor.reset()
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.set_windowing(sensor_window)
    sensor.set_hmirror(sensor_hmirror)
    sensor.set_vflip(sensor_vflip)
    sensor.run(1)
    lcd.init(type=1)
    lcd.rotation(lcd_rotation)
    lcd.clear(lcd.WHITE)
    if not labels:
        with open('labels.txt','r') as f:
            exec(f.read())
    if not labels:
        print("no labels.txt")
        img = image.Image(size=(320, 240))
        img.draw_string(90, 110, "no labels.txt", color=(255, 0, 0), scale=2)
        lcd.display(img)
        return 1
    try:
        img = image.Image("first.jpg")
        lcd.display(img)
    except Exception:
        img = image.Image(size=(320, 240))
        img.draw_string(90, 110, "loading model...", color=(255, 255, 255), scale=2)
        lcd.display(img)
    task = kpu.load(model_addr)
    kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
    try:
        while 1:
            img = sensor.snapshot()
            t = time.ticks_ms()
            objects = kpu.run_yolo2(task, img)
            t = time.ticks_ms() - t
            if objects:
                for obj in objects:
                    pos = obj.rect()
                    img.draw_rectangle(pos)
                    img.draw_string(pos[0], pos[1], "%s : %.2f" %(labels[obj.classid()], obj.value()), scale=2, color=(255, 0, 0))
                    temp = labels[obj.classid()]
                    # 如果按键被按下
                    if KEY.value() == 0:
                        target = labels[obj.classid()]  # 将识别的数字存入target

                    if(labels[obj.classid()] == target):
                        if labels[obj.classid()] == '3':
                            position = "Succ3"
                            uart.write('%sThree\r\n'%(labels[obj.classid()]))
                            print("target on the Three!\r\n")
                        elif labels[obj.classid()] == '4':
                            position = "Succ4"
                            uart.write('%sFour\r\n'%(labels[obj.classid()]))
                            print("target on the Four!\r\n")
                        elif labels[obj.classid()] == '5':
                            position = "Succ5"
                            uart.write('%sFive\r\n'%(labels[obj.classid()]))
                            print("target on the Five!\r\n")
                        elif labels[obj.classid()] == '6':
                            position = "Succ6"
                            uart.write('%sSix\r\n'%(labels[obj.classid()]))
                            print("target on the Six!\r\n")
                        elif labels[obj.classid()] == '7':
                            position = "Succ7"
                            uart.write('%sSeven\r\n'%(labels[obj.classid()]))
                            print("target on the Seven!\r\n")
                        elif labels[obj.classid()] == '8':
                            position = "Succ8"
                            uart.write('%sEight\r\n'%(labels[obj.classid()]))
                            print("target on the Eight!\r\n")

                        elif labels[obj.classid()] == '1':
                            position = "<---"
                            uart.write('%sLEFT\r\n'%(labels[obj.classid()]))
                            print("target on the Left!\r\n")
                        elif labels[obj.classid()] == '2':
                            position = "--->"
                            uart.write('%sRIGHT\r\n'%(labels[obj.classid()]))
                            print("target on the Right!\r\n")

                    #if (target == '3' or target == '4' or target == '5' or target == '6' or
                        #target == '7' or target == '8'):
                        #position = "MID_FAR"
                        #uart.write('%sMID_FAR\r\n'%(labels[obj.classid()]))
                        #print("target on the MID_FAR!\r\n")

                    # if labels[obj.classid()] == '1':
                    #     position = "<---"
                    #     uart.write('%sLEFT\r\n'%(labels[obj.classid()]))
                    #     print("target on the Left!\r\n")
                    # elif labels[obj.classid()] == '2':
                    #     position = "--->"
                    #     uart.write('%sRIGHT\r\n'%(labels[obj.classid()]))
                    #     print("target on the Right!\r\n")


            else:
                position = 0
            img.draw_string(150,0,"P%s" %(position), scale=2, color=(255, 0, 0))
            img.draw_string(0,0,"target:%s" %(target), scale=2, color=(255, 0, 0))
            img.draw_string(0, 200, "t:%dms" %(t), scale=2, color=(255, 0, 0))
            lcd.display(img)
    except Exception as e:
        raise e
    finally:
        kpu.deinit(task)
if __name__ == "__main__":
    try:
        labels = ['1', '2', '3', '4', '5', '6', '7', '8']
        anchors = [1.40625, 1.8125000000000002, 5.09375, 5.28125, 3.46875, 3.8124999999999996, 2.0, 2.3125, 2.71875, 2.90625]
        main(anchors = anchors, labels=labels, model_addr="/sd/m.kmodel", lcd_rotation=2, sensor_window=(224, 224))
    except Exception as e:
        sys.print_exception(e)
        lcd_show_except(e)
    finally:
        gc.collect()
