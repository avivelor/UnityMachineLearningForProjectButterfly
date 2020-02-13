from mjremote import mjremote
import time
import math
import numpy


from tkinter import *
master = Tk() 
sFinX = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sFinY = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sFinZ = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sFinW = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sFinX.pack() 
sFinY.pack() 
sFinZ.pack() 
sFinW.pack() 

sArmX = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sArmY = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sArmZ = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sArmX.pack() 
sArmY.pack() 
sArmZ.pack() 

sTest1 = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sTest2 = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sTest3 = Scale(master, from_=-1000, to=1000, orient=HORIZONTAL) 
sTest1.pack() 
sTest2.pack() 
sTest3.pack() 

m = mjremote()
print('Connect: ', m.connect())
b = bytearray(3*m.width*m.height)
t0 = time.time()
t1 = time.time()
m.getimage(b)
bb = numpy.frombuffer(b)
print(bb.shape)


print(m.getnqpose())
m.setqpos(numpy.array([
                0.1,   # base yaw
                -50 / 180 * math.pi, # base roll
                61.03 / 180 * math.pi, # arm pitch
               0, 0, 0, # wrist near fingers
               0, 0, 0, 0, #right fingers
               0, 0, 0, 0, #right fingers
               -0.62, 0.32, 0, math.cos(math.pi * 0.16), 0, 0, math.sin(math.pi * 0.16), #block one
               -0.72, 0.38, 0, math.cos(math.pi * 0.18), 0, 0, math.sin(math.pi * 0.18), #block two
               -0.835, 0.425, 0, math.cos(math.pi * 0.195), 0, 0, math.sin(math.pi * 0.195), #block three
               -0.935, 0.46, 0, math.cos(math.pi * 0.23), 0, 0, math.sin(math.pi * 0.23)] + [0] * 77)) #block four
#print('FPS: ', 100/(t1-t0))
while True:
    master.update_idletasks()
    master.update()
    m.setqpos(numpy.array([
               sTest1.get()/1000, # base yaw
               sTest2.get()/1000, # base roll
               sTest3.get()/1000, # arm pitch
               sArmX.get()/1000, sArmY.get()/1000, sArmZ.get()/1000, # arm yaw
               sFinX.get()/1000, sFinY.get()/1000, sFinZ.get()/1000, sFinW.get()/1000, # right two fingers
               sFinX.get()/1000, sFinY.get()/1000, sFinZ.get()/1000, sFinW.get()/1000, # left two fingers
               -0.62, 0.32, 0, math.cos(math.pi * 0.16), 0, 0, math.sin(math.pi * 0.16),
               -0.72, 0.38, 0, math.cos(math.pi * 0.18), 0, 0, math.sin(math.pi * 0.18),
               -0.835, 0.425, 0, math.cos(math.pi * 0.195), 0, 0, math.sin(math.pi * 0.195),
               -0.935, 0.46, 0, math.cos(math.pi * 0.23), 0, 0, math.sin(math.pi * 0.23)] + [0] * 77))
    time.sleep(0.01)

print('Exiting mujoco remote')
m.close()
