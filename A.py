import RPi.GPIO as GPIO
import time
import signal
import atexit
 
atexit.register(GPIO.cleanup) 

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
servopin = 21
trig=17
echo=27
GPIO.setup(trig,GPIO.OUT,initial=GPIO.LOW)
GPIO.setup(echo,GPIO.IN)
GPIO.setup(servopin, GPIO.OUT, initial=False)
p = GPIO.PWM(servopin,50)
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
     
    distance=round((end-start)*343/2*100,2)
    if distance <= 50:
        GPIO.output(13, GPIO.HIGH)
    else:
        GPIO.output(13, GPIO.LOW)
    print("distance:{0}cm,{1}m".format(distance,distance/100))
    
while True:    
    a=input("go on? y/n")
    while a == 'y':
        c=input("If you want to set angle, type 'y' please. Type 'n' to automatic rotation(cannot exit).")
        if c == 'y':
            angle = input('Please type an angle:')
            angle = int(angle)
            for i in range(0,angle,10):
                p.ChangeDutyCycle(2.5 + 10 * angle / 180)#设置转动角度
                time.sleep(0.02)                      #等该20ms周期结束
                p.ChangeDutyCycle(0)                  #归零信号
                time.sleep(0.2)
            print("angle:{0}".format(angle))
            Measure()
        else:
            while(True):
                for i in range(0,181,10):
                    p.ChangeDutyCycle(2.5 + 10 * i / 180)
                    time.sleep(0.02)
                    p.ChangeDutyCycle(0)
                    Measure()
                    print("angle:{0}".format(i))
                    time.sleep(0.2)
 
                for i in range(181,0,-10):
                    p.ChangeDutyCycle(2.5 + 10 * i / 180)
                    time.sleep(0.02)
                    p.ChangeDutyCycle(0)
                    Measure()
                    print("angle:{0}".format(i))
                    time.sleep(0.2)
    exit()
GPIO.cleanup();