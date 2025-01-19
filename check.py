import RPi.GPIO as GPIO
from time import sleep
f = open("output.txt", "w")
g = open("output_index.txt", "w")
try:
    for i in range(20):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(16, GPIO.IN)
        if GPIO.input(16) == GPIO.HIGH:
            f.write("Огонь обнаружен!")
            f.write("\n")
            g.write("Огонь обнаружен!")
            g.write("\n")
            GPIO.cleanup()
            

        else:
            f.write("Огонь не обнаружен")
            f.write("\n")
            g.write("Огонь не обнаружен")
            g.write("\n")
            GPIO.cleanup()
            
        sleep(1)
except KeyboardInterrupt:
    GPIO.cleanup()

f.close()
g.close()
