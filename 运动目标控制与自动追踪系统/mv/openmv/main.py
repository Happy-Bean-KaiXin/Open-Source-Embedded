import sensor, image, time, math
from pid import PID
from pyb import UART,Pin
from time import sleep

# led声明
led = Pin('P4', Pin.OUT_PP)
led.low()

# 全局变量定义
Red_threshold  = [(15, 100, 0, 72, -26, 55)] # 红色阈值
Is_Recognize_Flag = 0   # 识别次数标志位
Flag_Mode_Set = 0       # 识别模式标志位
pencil_points = [[59, 16], [59+176, 16], [59, 16+176], [59+176, 16+176], [0, 0]]

# ============ A4 靶纸检测滤波（避免误检屏幕边框）============
# A4 纸 ISO 216 标准长宽比：297mm / 210mm ≈ 1.414
A4_ASPECT        = 297.0 / 210.0
# 长宽比容差：胶带贴歪、透视/镜头畸变带来的误差，给 ±0.18（≈ ±12.7%）
A4_ASPECT_TOL    = 0.18
# 面积占比上限：固定机位下 A4 仅占画面一部分；屏幕边框几乎占满整幅 QVGA(320x240)。
# 占比超过此值视为屏幕边框/整屏，直接排除。0.55 = 面积超过整幅 55% 即判为屏幕。
A4_MAX_AREA_FRAC = 0.55
# 是否启用 A4 滤波（置 False 可回到“整幅搜索、不过滤”的调试状态）
A4_FILTER_ENABLE = True

uart = UART(1, 115200)  # 初始化串口3
#data = str()
## 进行对帧头帧尾数据判断
#def data_test(data):

    #if(data[0] != 0xa3):
        #return 0  # 帧头
    #if(data[1] != 0xb3):
        #return 0  # 帧头
    #if(data[5] != 0xc3):
        #return 0  # 帧尾
    #return 1

#Flag_Is_Rec = 0
#def uart_callback(line):
    #global Flag_Is_Rec, Rec_string
    #if uart.any():
        #lenth = len(data)
        #data[Flag_Is_Rec] = uart.read(1)  # 读取一个字节数据
        #Flag_Is_Rec += 1                  # 将接收标志位加1
        #if data[0] != 0xa3:
            #Flag_Is_Rec = 0
        #if (i==2) and (data[1] != 0xb3):
            #Flag_Is_Rec = 0
        #if Flag_Is_Rec == lenth:
            #if data_test(data):
                #Rec_string = data[2:lenth-1]
            #Flag_Is_Rec = 0
        #print(data)

#uart.irq(handler = uart_callback)  # 初始化串口中断函数

def find_max(blobs):
    global max_blob
    max_size=0
    for blob in blobs:
        if blob.pixels() > max_size:
            max_blob=blob
            max_size = blob.pixels()
    return max_blob

#识别红点
def find_redPoint(img,thre):
    sensor.set_auto_exposure(False,500)  #曝光速度  3000
    blobs = img.find_blobs(thre)
    if blobs:
        max_blob = find_max(blobs)
        img.draw_rectangle(max_blob.rect(),color=(0,255,0))
        img.draw_cross(max_blob.cx(), max_blob.cy(), color=(0, 255, 0),size=10)
        print(max_blob.cx(), max_blob.cy(), type(max_blob.cx()))  # 打印x与y的坐标
        uart.write("X="+str(max_blob.cx())+"Y="+str(max_blob.cy())+'\n')
    else:
        uart.write("None\n")
        print("None")

#坐标排序（顺）
def clockwise_sort(points):
    # 计算矩形的中心点
    center_x = (points[0][0] + points[2][0]) / 2.0
    center_y = (points[0][1] + points[2][1]) / 2.0
    # 计算每个点相对于中心点的极角
    polar_angles = []
    for point in points:
        dx = point[0] - center_x
        dy = point[1] - center_y
        angle = math.atan2(dy, dx)
        polar_angles.append(angle)
    # 将点按照极角进行排序
    # [BUGFIX] corner sorting must happen AFTER the whole for-loop has collected all polar_angles.
    # The original 'return' sat inside the for-body, so only the first point was processed:
    # 'out' had 1 element and out[1..3] raised IndexError, crashing find_rect (mode 2 rectangle recognition).
    sorted_points = [point for _, point in sorted(zip(polar_angles, points))]
    return sorted_points

# ============ A4 靶纸检测滤波（避免误检屏幕边框）============
# 由 4 个角点计算矩形长宽比（对旋转、透视鲁棒：两条长边均值 / 两条短边均值）
def rect_aspect(corners):
    def d(a, b):
        return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    edges = [d(corners[i], corners[(i+1) % 4]) for i in range(4)]
    edges.sort()
    short = (edges[0] + edges[1]) / 2.0   # 两条短边
    long  = (edges[2] + edges[3]) / 2.0   # 两条长边
    if short <= 0:
        return None
    return long / short

# 判定是否为 A4 靶纸：长宽比接近 1.414 且 面积不占满整幅（排除屏幕边框误检）
def is_a4_target(r, img):
    if not A4_FILTER_ENABLE:
        return True
    aspect = rect_aspect(r.corners())
    if aspect is None:
        return False
    if abs(aspect - A4_ASPECT) > A4_ASPECT_TOL:
        return False
    area_frac = (r.w() * r.h()) / float(img.width() * img.height())
    if area_frac > A4_MAX_AREA_FRAC:
        return False
    return True

#识别矩形
def find_rect(img):

    global Is_Recognize_Flag
    print(2)
    # BUG1 修复：整幅搜索（无硬编码 roi）。
    # A4 滤波：用长宽比(≈1.414) + 面积占比排除“屏幕边框”等伪矩形，避免误检。
    best_r = None
    best_err = 1e9
    for r in img.find_rects(threshold=18000):#矩形roi设置
        if not is_a4_target(r, img):
            continue  # 非 A4 靶纸（如屏幕边框、噪声），跳过
        # 在通过初筛的候选里，选长宽比最接近 A4 的一个
        err = abs(rect_aspect(r.corners()) - A4_ASPECT)
        if err < best_err:
            best_err = err
            best_r = r

    if best_r is None:
        return  # 未找到符合 A4 特征的矩形（屏幕边框/噪声已被过滤）
    r = best_r

    img.draw_rectangle(r.rect(), color = (255, 0, 0))
    for p in r.corners():
        img.draw_circle(p[0], p[1], 5, color = (0, 255, 0))

    out = clockwise_sort(r.corners())  # 获取矩形的坐标

    p1 = out[0]
    img.draw_string(p1[0], p1[1], '1', color=130, scale=3)  # 在图像中写字 8x10的像素
    p2 = out[1]
    img.draw_string(p2[0], p2[1], '2', color=130, scale=3)  # 在图像中写字 8x10的像素
    p3 = out[2]
    img.draw_string(p3[0], p3[1], '3', color=130, scale=3)  # 在图像中写字 8x10的像素
    p4 = out[3]
    img.draw_string(p4[0], p4[1], '4', color=130, scale=3)  # 在图像中写字 8x10的像素
    print(out)
    uart.write('p1'+str(p1[0])+'p2'+str(p1[1])+
               'p3'+str(p2[0])+'p4'+str(p2[1])+
               'p5'+str(p3[0])+'p6'+str(p3[1])+
               'p7'+str(p4[0])+'p8'+str(p4[1])+'\n')
    Is_Recognize_Flag += 1  # 识别着了，将识别标志位加1

'''
现场校准程序，手动将激光点，从左上角开始，顺时针依次落在铅笔正方形的角上
然后按下触屏（或其他输入装置）
最后落在中心点上，一共5个点。
'''
Number_Of_Corrections = 0
Identification_Complete = 0
def cam_calibration():
    global pencil_points, Red_threshold, Rec_string
    global Location_Actual, Flag_Mode_Set, Number_Of_Corrections
    global Identification_Complete
    #pencil_points = []  #先清空原有数据

    sensor.set_brightness(-3)   #设置亮度
    sensor.set_contrast(3) #对比度
    sensor.set_auto_exposure(False,500)  # 曝光速度
    img_correct = sensor.snapshot()      # 重新拍一张照片，找点
    Captured_Blocks = img_correct.find_blobs(Red_threshold)   # 找红点的最大色块
    if Captured_Blocks:
        img_correct.draw_cross(Captured_Blocks[0].cx(),Captured_Blocks[0].cy(),color=(0,0,0))
        #真实的物体 Real_Object
        Real_Object = max(Captured_Blocks, key = lambda b: b.pixels())          #按结果的像素值，找最大值的数据。也就是找最大的色块。
        if (Real_Object.w()>=3 and Real_Object.h()>=3):   #过滤掉长宽小于10的结果
            img_correct.draw_rectangle(Real_Object.rect(),color=(0,255,0))                #按寻找色块结果的前四个值，绘制方形，框选识别结果。
            img_correct.draw_cross(Real_Object.cx(),Real_Object.cy(),color=(0,255,0))   #用结果的中心值坐标，绘制十字
            Location_Actual = [Real_Object.cx(),Real_Object.cy()]   #识别到物体的位置

        # 顺序采集 5 个点：0 左上, 1 右上, 2 右下, 3 左下, 4 中心
        # 只有尚未采满(状态<5)时才读取指令；采满后在下方发送态统一回传一次
        # （修复原代码：每采 1 个点就跳到发送态，导致其余 4 个点停留在默认值）
        if Number_Of_Corrections < 5:
            if uart.any():  # 串口接收数据
                Rec_string = uart.readline()
                print(Rec_string)
                # 矫正顶点顺序：左上角 - 右上角 - 右下角 - 左下角 - 中心点
            if (Rec_string == b"upleft"):         # 如果接收到 "upleft"   左上角
                pencil_points[0][0] = Location_Actual[0]    #左上角X
                pencil_points[0][1] = Location_Actual[1]    #左上角Y
                Number_Of_Corrections = 1
                print("upleft ->", pencil_points[0])

            elif (Rec_string == b"upright"):          # 如果接收到 "upright" 右上角
                pencil_points[1][0] = Location_Actual[0]    #右上角X
                pencil_points[1][1] = Location_Actual[1]    #右上角Y
                Number_Of_Corrections = 2
                print("upright ->", pencil_points[1])

            elif (Rec_string == b"lowright"):          # 如果接收到 "lowright"右下角
                pencil_points[2][0] = Location_Actual[0]    #右下角X
                pencil_points[2][1] = Location_Actual[1]    #右下角Y
                Number_Of_Corrections = 3
                print("lowright ->", pencil_points[2])

            elif (Rec_string == b"lowleft"):          # 如果接收到 "lowleft" 左下角
                pencil_points[3][0] = Location_Actual[0]   #左下角X
                pencil_points[3][1] = Location_Actual[1]   #左下角Y
                Number_Of_Corrections = 4
                print("lowleft ->", pencil_points[3])

            elif (Rec_string == b"midpoint"):          # 如果接收到 "midpoint" 中心点
                pencil_points[4][0] = Location_Actual[0]   #中心点X
                pencil_points[4][1] = Location_Actual[1]   #中心点Y
                Number_Of_Corrections = 5
                Identification_Complete = 1    # 识别完成，将此标志位置为1
                print("midpoint ->", pencil_points[4])

    # 五点已采齐（状态到 5），统一回传一次；放在 if Captured_Blocks 之外，不依赖当前是否看到红点
    if Number_Of_Corrections == 5:
        uart.write('r1'+str(pencil_points[0][0])+'r2'+str(pencil_points[0][1])+
                    'r3'+str(pencil_points[1][0])+'r4'+str(pencil_points[1][1])+
                    'r5'+str(pencil_points[2][0])+'r6'+str(pencil_points[2][1])+
                    'r7'+str(pencil_points[3][0])+'r8'+str(pencil_points[3][1])+
                    'r9'+str(pencil_points[4][0])+'r0'+str(pencil_points[4][1])+
                    '\n')
        print("calibration sent:", pencil_points)
        Number_Of_Corrections = 0

        if Identification_Complete:
            Flag_Mode_Set = 0            # 识别完成进入模式0


if  __name__ == "__main__":
    sensor.reset() # 摄像头初始化
    sensor.set_pixformat(sensor.RGB565)
    sensor.set_framesize(sensor.QVGA)
    sensor.skip_frames(time = 2000)
    sensor.set_vflip(False)
    sensor.set_auto_whitebal(False)   # 自动白平衡
    sensor.set_auto_gain(False)
    sensor.set_auto_exposure(False,exposure_us = 10000)
    sensor.set_contrast(0)
    sensor.set_brightness(0)
    # BUG1 修复：注释掉固定窗口(44,35,240,191)，改为整幅 QVGA(320x240) 捕获，
    # 配合 find_rects 全幅搜索，使 A4 靶纸可贴在屏幕任意位置（满足赛题(3)(4)）
    sensor.set_brightness(-3)   #设置亮度
    sensor.set_contrast(3) #对比度
    #sensor.set_gainceiling(2)   #增益上限
    #sensor.set_auto_gain(False,gain_db=-1) #增益

    clock = time.clock()                  #建立clock

    while(True):
        clock.tick()
        img = sensor.snapshot()

        # 串口接收，处理模式标志位--------------
        # 校准模式(Flag_Mode_Set==3)下，子指令(upleft/upright/...)由 cam_calibration 自行读取，
        # 主循环不抢占，避免“命令被主循环读走、校准函数读不到”的竞态
        if Flag_Mode_Set != 3 and uart.any():  # 串口接收数据
            Rec_string = uart.readline()
            print(Rec_string)

            if (Rec_string == b"RecognizeRed"):        # 如果接收到 "Recognize_Red"
                Flag_Mode_Set = 1                     # 模式1，进入识别红色光斑程序
            elif (Rec_string == b"RecognizeRetangle"):     # 如果接收到  "Recognize_Retangle"
                Flag_Mode_Set = 2                     # 模式2, 进入识别矩形程序
            elif (Rec_string == b"CorrectRedSpots"):
                Flag_Mode_Set = 3


        if Flag_Mode_Set == 1:
            find_redPoint(img, Red_threshold)  # 识别红点

        elif Flag_Mode_Set == 2:

            if Is_Recognize_Flag <= 2:
                sensor.set_brightness(-1)   #设置亮度
                sensor.set_contrast(3) #对比度
                #sensor.set_gainceiling(2)   #增益上限
                find_rect(img)
            elif Is_Recognize_Flag == 3:   # 如果识别到了三次，将模式变成识别红点模式
                Is_Recognize_Flag = 0
                Flag_Mode_Set = 1
        elif Flag_Mode_Set == 3:
            cam_calibration()              # 矫正红点程序


