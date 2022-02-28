from cgitb import text
from turtle import color, title, width
import serial

import vpython

arduino = serial.Serial("/dev/ttyACM0", 9600)

import io
# sio = io.TextIOWrapper(io.BufferedRWPair(arduino, arduino))


side = 4.0
thk = 0.3
s2 = 2*side - thk
s3 = 2*side + thk

wallL = vpython.box (pos=vpython.vector(-.8, 0, 0), size=vpython.vector(thk, s2, s3),  color = vpython.color.yellow)


box = vpython.box(title="Obstacle", length=2, height=3, width=.5, color=vpython.color.red, pos=vpython.vector(0,0,0), make_trail=True, retain=200)
label = vpython.label(pos = vpython.vector(0,7,0), text="Current distance is: ", box=False, height=10 )



try:

    while(True):
        # vpython.rate(20)
        if(arduino.inWaiting() > 0):
            data = arduino.readline()
            try:
                if data:
                    # print(data.decode())
                    distance = float(data.decode())
                    label.text = "Current distance is: " + data.decode()
                    box.pos = vpython.vector(distance/100, 0, 0)

            except Exception as ex:
                print(ex)
            

except Exception as e:
    print("Closing:", e)
    arduino.close()
