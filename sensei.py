import serial   # pip install PySerial

ser = serial.Serial(port="COM4", baudrate=115200, bytesize=8, timeout=2, stopbits=serial.STOPBITS_ONE)
# ser.flushInput()
# ser.flushOutput()
while True:
    data_raw = ser.readline()
    print(data_raw)    # b'CPS[436]\r\n'
    a = data_raw.decode()   # a is str
    print(a)

