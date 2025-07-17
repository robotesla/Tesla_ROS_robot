import serial

ser = serial.Serial(
    "/dev/serial/by-path/platform-xhci-hcd.1-usb-0:1:1.0-port0",
    baudrate=115200,
    timeout=1
)

#ser.write(b"PING\n")
# print(ser.readline().decode())
#print(ser.readline())
#ser.close()

while True:
    bs = ser.readline()
    print(bs)

#ser.write(b"SET_ROBOT_VELOCITY 200 1\n")
