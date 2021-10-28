import RPi.GPIO as GPIO
import time
import signal
import atexit
import numpy as np
import matplotlib.pyplot as plt

plt.figure(figsize=(8.0, 6.0))
theta1=[]
r1=[]

atexit.register(GPIO.cleanup) 

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
servopin = 21
trig=17
echo=27
GPIO.setup(trig,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(echo,GPIO.IN)
GPIO.setup(servopin, GPIO.OUT, initial=False)
p = GPIO.PWM(servopin,50) #50HZ
p.start(0)
time.sleep(2)

GPIO.setup(13, GPIO.OUT)      
GPIO.output(13, GPIO.LOW)
GPIO.output(13, GPIO.HIGH)
time.sleep(0.3)
GPIO.output(13, GPIO.LOW)
time.sleep(0.3)
GPIO.output(13, GPIO.HIGH)
time.sleep(0.3)
GPIO.output(13, GPIO.LOW)
time.sleep(0.3)
GPIO.output(13, GPIO.HIGH)
time.sleep(0.3)
GPIO.output(13, GPIO.LOW)

def Measure():
 
    GPIO.output(trig,True)
    time.sleep(0.00001)
    GPIO.output(trig,False)
 
    while GPIO.input(echo)==0:
        pass
    start=time.time()
 
    while GPIO.input(echo)==1:
        pass
    end=time.time()
     
    global distance
    distance=round((end-start)*343/2*100,2)
    if distance <= 50:
        GPIO.output(13, GPIO.HIGH)
    else:
        GPIO.output(13, GPIO.LOW)
    print("distance:{0}cm,{1}m".format(distance,distance/100))
 
while True:    
    a=input("go on? y/n")
    while a == 'y':
        c=input("If you want to set angle, type 'y' please. Type 'n' to automatic rotation.")
        if c == 'y':
            angle = input('Please type an angle:')
            angle = int(angle)
            for i in range(0,angle,10):
                p.ChangeDutyCycle((18 + angle) / 18)#设置转动角度
                time.sleep(0.02)                      #等该20ms周期结束
                p.ChangeDutyCycle(0)                  #归零信号
                time.sleep(0.2)
            Measure()
            Rangle = (angle/180)*np.pi
            long = distance
            ttheta1 = Rangle
            rr1 = long
            theta1.append(ttheta1)
            r1.append(rr1) 
            plt.polar(theta1, r1, marker='.', ms=16,color='r')
            plt.pause(4)
            plt.close()
            print("angle:{0}".format(angle))
        else:
            while(True):
                for i in range(0,180,10):
                    p.ChangeDutyCycle((18 + i)/ 18) #设置转动角度
                    time.sleep(0.02)                      #等该20ms周期结束
                    p.ChangeDutyCycle(0)                  #归零信号
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
 
                for i in range(180,0,-10):
                    p.ChangeDutyCycle((18 + i)/ 18) #设置转动角度
                    time.sleep(0.02)                      #等该20ms周期结束
                    p.ChangeDutyCycle(0)                  #归零信号
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
#!/usr/bin/env python3
import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BCM) # GPIO引脚编号的两种方式之一，另一种是BCM。所用编号方式不同，pin也不同。
GPIO.setwarnings(False) # 禁用警告。
GPIO.setup(21,GPIO.OUT) # pin为你所接的数据输出GPIO引脚。
p = GPIO.PWM(21,50) # 在pin脚上产生频率为50HZ的PWM信号。
p.start(0) # 在引脚上设置一个初始PWM信号。
p.ChangeDutyCycle(10) # 通过输入不同的“占空比值”来观察舵机的位置变化，这里的“占空比”是3%。
time.sleep(1)
p.stop() # 停止PWM。
GPIO.cleanup() # 清理GPIO。
舵机测试代码
'''
