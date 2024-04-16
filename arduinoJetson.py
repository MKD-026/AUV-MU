import serial

arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

direction = "R"
count = 1
while True:
    dir = input()
    direction = dir
    if(direction == "L"):
        arduino.write(bytes('L\n','utf-8'))
    elif(direction == "R"):
        arduino.write(bytes('R\n','utf-8'))

    count = count + 1
    if count == 25:
        break
        
