import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12,GPIO.OUT)
pwm = GPIO.PWM(12, 100)

dc = 0
pwm.start(dc)

try:
    while True:
        for dc in range (0, 101, 5):
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.05)
            print(dc)
        for dc in range (95, 0, -5):
            pwm.ChangeDutyCycle(dc)
            time.sleep(0.05)
            print(dc)
except KeyBoardInterrupt:
    print "exitting"

pwm.stop()
GPIO.cleanup()
