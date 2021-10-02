import RPi.GPIO as GPIO
import time
GPIO.setmode(GPIO.BOARD)

GPIO.setup(12,GPIO.OUT)

pwm = GPIO.PWM(12,100)



print "50% brightness"
pwm.start(50)


time.sleep(10)

print "1% brightness"
pwm.ChangeDutyCycle(1)

time.sleep(10)

print "100% brightness"
pwm.ChangeDutyCycle(100)

time.sleep(10)

print "change frequency"
pwm.ChangeFrequency(1000)

time.sleep(10)

print "turning off..."
pwm.stop()

GPIO.cleanup()
