import serial
import time
ser = serial.Serial('/dev/ttyACM0', 9600)


print ("writing serial values")
i = 20

try:
    while True:
        if i < 255:
            left = i
            right = i - 20
            left = str(100)
            right = str(100)
            ser.write("L")
            ser.write(left)
            ser.write("R")
	    ser.write(right)
            ser.write("D")
            time.sleep(1)
            i = i + 10
        else:
            i = 20
except KeyboardInterrupt:
        left = str(0)
        right = str(0)
        ser.write("L")
        ser.write(left)
        ser.write("R")
        ser.write(right)
        ser.write("D")

    
