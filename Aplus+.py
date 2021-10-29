import RPi.GPIO as GPIO
import time
import signal
import atexit
import numpy as np
import matplotlib.pyplot as plt                                     #我们在这里引用以上六个模板

plt.figure(figsize=(8.0, 6.0))                                      #设置matplotlib的窗口尺寸为8*6
theta1=[]                                                           #将theta1声明为空数组，下同
r1=[]

atexit.register(GPIO.cleanup)                                       #当程序正常退出时将gpio还原

GPIO.setwarnings(False)                                             #禁用警告
GPIO.setmode(GPIO.BCM)                                              #设置gpio输出模式为bcm
servopin=21                                                         #因为舵机的信号线接gpio21所以我们将servopin赋上21
trig=17                                                             #雷达输出为gpio17
echo=27                                                             #雷达输入为27
GPIO.setup(trig,GPIO.OUT,initial=GPIO.LOW)                          #初始化gpio17
GPIO.setup(echo,GPIO.IN)                                            #初始化27
GPIO.setup(servopin, GPIO.OUT, initial=False)                       #初始化21
p = GPIO.PWM(servopin,50)                                           #将p设置为向gpio21输出50HZ的信号
p.start(0)                                                          #设置一个初始PWM信号
time.sleep(2)                                                       #持续2秒

GPIO.setup(13, GPIO.OUT)                                            #初始化gpio13
GPIO.output(13, GPIO.LOW)                                           #设置gpio13为低电平
GPIO.output(13, GPIO.HIGH)                                          #设置其为高电平
time.sleep(0.3)                                                     #持续0.3秒
GPIO.output(13, GPIO.LOW)
time.sleep(0.3)
GPIO.output(13, GPIO.HIGH)
time.sleep(0.3)
GPIO.output(13, GPIO.LOW)
time.sleep(0.3)
GPIO.output(13, GPIO.HIGH)
time.sleep(0.3)
GPIO.output(13, GPIO.LOW)                                           #以上总共3个高电平，3个低电平，中间持续0.3秒

def Measure():                                                      #定义measure函数
 
    GPIO.output(trig,True)
    time.sleep(0.00001)
    GPIO.output(trig,False)                                         #向trig口输出一个信号，持续0.00001秒
 
    while GPIO.input(echo)==0:
        pass
    start=time.time()                                               #随即开始计时
 
    while GPIO.input(echo)==1:
        pass
    end=time.time()                                                 #当echo为真即收到信号时结束计时
     
    global distance                                                 #声明ditance为全局变量
    distance=round((end-start)*343/2*100,2)                         #计算距离，四舍五入到后2位
    if distance <= 50:
        GPIO.output(13, GPIO.HIGH)
    else:
        GPIO.output(13, GPIO.LOW)                                   #如果距离小于50cm将gpio13设置为高电平，否则为低电平
    print("distance:{0}cm,{1}m".format(distance,distance/100))      #打印距离
 
while True:                                                         #以下为循环
    a=input("go on? y/n")
    while a == 'y':
        c=input("If you want to set angle, type 'y' please. Type 'n' to automatic rotation.")
        if c == 'y':
            angle = input('Please type an angle:')
            angle = int(angle)
            for i in range(0,angle,10):                            #以10为步长遍历一遍0到angle
                p.ChangeDutyCycle((18 + angle) / 18)               #经测试发现角度和占空比的关系如左式于是我们这样设置转动
                time.sleep(0.02)                                   #等该20ms周期结束
                p.ChangeDutyCycle(0)                               #归零信号
                time.sleep(0.2)
            Measure()                                              #遍历结束后执行measure函数
            Rangle = (angle/180)*np.pi                             #将角度转换为弧度
            long = distance
            ttheta1 = Rangle
            rr1 = long
            theta1.append(ttheta1)
            r1.append(rr1)                                         #将弧度和距离添加到绘制函数中
            plt.polar(theta1, r1, marker='.', ms=16,color='r')     #绘制极坐标图像
            plt.pause(4)                                           #持续4秒
            plt.close()                                            #关闭窗口
            print("angle:{0}".format(angle))                       #打印当前角度
        else:
            while(True):
                for i in range(0,180,10):
                    p.ChangeDutyCycle((18 + i)/ 18)               #设置转动角度
                    time.sleep(0.02)                              #等该20ms周期结束
                    p.ChangeDutyCycle(0)                          #归零信号
                    Measure()
                    print("angle:{0}".format(i))
                    Rangle = (i/180)*np.pi
                    long = distance
                    ttheta1 = Rangle
                    rr1 = long
                    theta1.append(ttheta1)
                    r1.append(rr1) 
                    plt.polar(theta1, r1, marker='.', ms=16,color='r')
                    plt.pause(0.2)
                    time.sleep(0.2)
 
                for i in range(180,0,-10):                        #反向遍历
                    p.ChangeDutyCycle((18 + i)/ 18)
                    time.sleep(0.02)
                    p.ChangeDutyCycle(0)
                    Measure()
                    print("angle:{0}".format(i))
                    Rangle = (i/180)*np.pi
                    long = distance
                    ttheta1 = Rangle
                    rr1 = long
                    theta1.append(ttheta1)
                    r1.append(rr1) 
                    plt.polar(theta1, r1, marker='.', ms=16,color='r')
                    plt.pause(0.2)
                    time.sleep(0.2)
    exit()
GPIO.cleanup();

'''
舵机测试
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
GPIO.setup(21,GPIO.OUT)
p = GPIO.PWM(21,50)       #产生频率为50HZ的PWM信号
p.start(0)                #设置一个初始PWM信号
p.ChangeDutyCycle(10)     #输入不同的占空比来观察舵机的位置变化
time.sleep(1)
p.stop()                  #停止PWM。
GPIO.cleanup()            #清理GPIO
'''