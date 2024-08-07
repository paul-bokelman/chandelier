import RPi.GPIO as GPIO

try: 
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(8, GPIO.OUT, initial=GPIO.LOW)

finally:
    GPIO.cleanup()