import serial
ser = serial.Serial("/dev/ttyS0", 9600)
r_power = 90
l_power = 70
f = [r_power, l_power]
s = [0,0]
b = [127+r_power, 127+l_power]
r = [127+r_power, l_power]
l = [r_power, 127+l_power]
