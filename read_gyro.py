import serial

ser = serial.Serial('/dev/cu.usbserial-10', 9600)
while True:
    line = ser.readline().decode('utf-8').rstrip()
    print(line)
 