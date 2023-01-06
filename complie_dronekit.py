# -*- coding: utf-8 -*-
"""
Created on Wed Sep 28 17:49:49 2022

@author: 林子倫
"""


from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # 命令消息定義需要
import time
import math
import argparse
import tkinter as tk
from getkey import getkey, keys
import numpy as np
import cv2
import time
import RPi.GPIO as GPIO


def sign(x):
    if x > 0:
        return 1.0
    else:
        return -1.0


CONTROL_PIN = 17
PWM_FREQ = 50
STEP=15


GPIO.setmode(GPIO.BCM)
GPIO.setup(CONTROL_PIN, GPIO.OUT)

pwm = GPIO.PWM(CONTROL_PIN, PWM_FREQ)
pwm.start(0)

def angle_to_duty_cycle(angle=0):
    duty_cycle = (0.05 * PWM_FREQ) + (0.19 * PWM_FREQ * angle / 180)
    return duty_cycle

def switch2deg(deg):
    dc = angle_to_duty_cycle(deg)
    pwm.ChangeDutyCycle(dc)

degrees = [100 ,180]


# center定义
center_now = 320

# 打开摄像头，图像尺寸640*480（长*高），opencv存储值为480*640（行*列）
cap = cv2.VideoCapture(0)

# PID 定义和初始化三个error和adjust数据
error = [0.0] * 3
adjust = [0.0] * 3

# PID 参数配置、目标值、左右轮基准占空比和占空比偏差范围（根据实际情况调整）
kp = 1.3
ki = 0.5
kd = 0.2
target = 320
lspeed = 60
rspeed = 60
control = 35
ret, frame = cap.read()

video_capture = cv2.VideoCapture(0)
video_capture.set(3, 300)
video_capture.set(4, 300)

#- 導入Tkinter : sudo apt-get install python-tk
#-- 連接到無人機
print('無人機連線...')
connection_string = "/dev/serial0"
baud_rate = 912600
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    在沒有 GPS 數據的情況下武裝車輛並飛往目標高度。
    """

    ##### 常數 #####
    DEFAULT_TAKEOFF_THRUST = 0.6
    SMOOTH_TAKEOFF_THRUST = 0.55

    print("基本的預裝檢查")
    # 在自動駕駛準備好之前不要讓用戶嘗試布防
    # 如果需要禁用武裝狀態檢查，
    # 用你自己的責任評論它。

    ###### 禁用檢查 #####
    #while not vehicle.is_armable:
    #print(" 等待車輛初始化...")
    #time.sleep(1)


    print("武裝馬達")
    # 無人機應該在 GUIDED_NOGPS 模式下布防
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True

    while not vehicle.armed:
        print(" 等待武裝...")
        vehicle.armed = True
        time.sleep(1)

    print("起飛!")

    thrust = DEFAULT_TAKEOFF_THRUST
    while True:
        current_altitude = vehicle.rangefinder.distance
        print(" 高度: %f  Desired: %f" %
              (current_altitude, aTargetAltitude))
        if current_altitude >= aTargetAltitude*0.95: # 低於目標值持續觸發alt.
            print("達到目標高度")
            break
        elif current_altitude >= aTargetAltitude*0.6:
            thrust = SMOOTH_TAKEOFF_THRUST
        set_attitude(thrust = thrust)
        time.sleep(0.2)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: 可以使用 yaw_angle 或 yaw_rate 控制偏航。
                  當一個被使用時，另一個被 Ardupilot 忽略。
    thrust: 0 <= thrust <= 1, 作為最大垂直推力的一小部分。
             請注意，從 Copter版本 3.5 開始，推力 = 0.5 會觸發特殊情況
             保持當前高度的代碼。
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # 啟動時間毫秒
        1, # 目標系統
        1, # 目標組件
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # 四軸數值
        0, # 以弧度表示的車身側傾率
        0, # 以弧度表示的車身俯仰率
        math.radians(yaw_rate), # 以弧度/秒為單位的車身偏航率
        thrust  # 推力
    )
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
     請注意，從 AC3.3 開始，消息的重新發送頻率應高於每次
     其次，因為 ATTITUDE_TARGET 訂單的超時時間為 1 秒。
     在 AC3.2.1 和更早的版本中，指定的姿態會持續到它被取消。
     下面的代碼應該適用於任一版本。
     多次發送消息是推薦的方式。
     roll_angle   側傾角
     pitch_angle  俯仰角
     yaw_angle    偏航角
     yaw_rate     偏航率
     use_yaw_rate 使用偏航率
     thrust       推力
     duration     期間
     
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # 重置姿態，否則會因為超時而多持續 1秒
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    將度數轉換為四軸數值
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]


light_red_lower = np.array([50,50,200])  # 設定紅色最低值範圍
light_red_upper = np.array([150,150,255])  # 設定紅色最高值範圍


light_green_lower = np.array([0,180,0])  # 設定紅色最低值範圍
light_green_upper = np.array([100,255,255])  # 設定紅色最高值範圍


lower = np.array([20,20,70])  # 設定紅色最低值範圍
upper = np.array([50,50,200])  # 設定紅色最高值範圍

green_lower = np.array([20,60,20])     # 設定綠色最低值範圍
green_upper = np.array([50,150,50])  # 設定綠色最高值範圍

blue_lower = np.array([70,20,20])     # 設定藍色最低值範圍
blue_upper = np.array([200,50,50])  # 設定藍色最高值範圍


key = getkey()

if key == 's':
    try:
        

        # 在 GUIDED_NOGPS 模式下起飛至指定高度(米)。
        arm_and_takeoff_nogps(1)
        # 保持姿勢 3 秒鐘。
        print("保持姿勢 3 秒鐘。")
        set_attitude(duration = 3)
        
        
        while True:
        
            ret, img = cap.read()
            ret, frame = cap.read()
            # 转化为灰度图
            
            
            
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # 大津法二值化
            ret, dst = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
            # 膨胀，白区域变大
            dst = cv2.dilate(dst, None, iterations=2)
            cv2.imshow("镜头画面", dst)

            # 单看第400行的像素值s
            color = dst[400]
            # 找到black色的像素点个数
            black_count = np.sum(color == 0)
            
            
            img = cv2.resize(img,(640,360))
            
            output = cv2.inRange(img, light_red_lower, light_red_upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
            output = cv2.dilate(output, kernel)
            output = cv2.erode(output, kernel)
            contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

            for contour in contours:
                area = cv2.contourArea(contour)
                color = (0,0,255)
                if(area > 300):
                    print("視頻中有淺紅色像素")
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 3)
                    print("懸停")
                    
                    
                    
            output = cv2.inRange(img, light_green_lower, light_green_upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
            output = cv2.dilate(output, kernel)
            output = cv2.erode(output, kernel)
            contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

            for contour in contours:
                area = cv2.contourArea(contour)
                color = (0,0,255)
                if(area > 300):
                    print("視頻中有淺綠色像素")
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 3)  
                    print("繼續前進")
            
            
                                    
            output = cv2.inRange(img, lower, upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
            output = cv2.dilate(output, kernel)
            output = cv2.erode(output, kernel)
            contours, hierarchy = cv2.findContours(output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) 

            for contour in contours:
                area = cv2.contourArea(contour)
                color = (0,0,255)
                if(area > 300):
                    print("視頻中有紅色像素")
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 3)
                    
                    
                    print("強迫下降")
                    vehicle.mode = VehicleMode("LAND")
                    time.sleep(1)
                    
                    print("關閉通訊控制對象")
                    vehicle.close()
                    

            # 設定選取綠色的程式
            green_output = cv2.inRange(img, green_lower, green_upper)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
            green_output = cv2.dilate(green_output, kernel)
            green_output = cv2.erode(green_output, kernel)
            contours, hierarchy = cv2.findContours(green_output, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                color = (0,255,0)
                if(area > 300):
                    print("視頻中有綠色像素")
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 3)
                    
                    
                    print("強迫下降")
                    vehicle.mode = VehicleMode("LAND")
                    time.sleep(1)
                    
                    print("關閉通訊控制對象")
                    vehicle.close()
                    
            
            
            for contour in contours:
                area = cv2.contourArea(contour)
                color = (255,0,0)
                if(area > 300):
                    print("視頻中有藍色像素")
                    x, y, w, h = cv2.boundingRect(contour)
                    img = cv2.rectangle(img, (x, y), (x + w, y + h), color, 3)
                    
                    print("懸停並投擲物品")
                    for i in range(1):
                        for deg in degrees:
                            switch2deg(deg)
            
            cv2.imshow('oxxostudio', img)
            
            # 防止black_count=0的报错
            if black_count == 0:
                continue
            else:
                black_index = np.where(color == 0)
            # 找到黑色像素的中心点位置
            center_now = (black_index[0][black_count - 1] + black_index[0][0]) / 2

            # 计算出center_now与标准中心点的偏移量
            direction = center_now - 320

            print("偏差值：", direction)

            # 更新PID误差
            error[0] = error[1]
            error[1] = error[2]
            error[2] = center_now - target

            # 更新PID输出（增量式PID表达式）
            adjust[0] = adjust[1]
            adjust[1] = adjust[2]
            adjust[2] = adjust[1] \
                        + kp * (error[2] - error[1]) \
                        + ki * error[2] \
                        + kd * (error[2] - 2 * error[1] + error[0]);
            print(adjust[2])

            # 饱和输出限制在control绝对值之内
            if abs(adjust[2]) > control:
                adjust[2] = sign(adjust[2]) * control
            # print(adjust[2])

            # 执行PID


            if adjust[2] > 20  and adjust[2] < 20 :
                print("前進")
                set_attitude(pitch_angle = -5, thrust = 0.5)
                
                
            # 右转
            if adjust[2] > 20:
                print("右轉")
                set_attitude(roll_angle = 1, thrust = 0.5, duration = 3)
                set_attitude(yaw_rate = 30, thrust = 0.5, duration = 3)
                
                
            # 左转
            elif adjust[2] < -20:
                print("左轉")
                
                

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
        
                                   
    except KeyboardInterrupt:
        
        print("強迫下降")
        vehicle.mode = VehicleMode("LAND")
        time.sleep(1)
        
        print("關閉通訊控制對象")
        vehicle.close()

