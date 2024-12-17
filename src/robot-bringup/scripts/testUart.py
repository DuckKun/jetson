# Importing Libraries
import serial
import time
arduino = serial.Serial(port = "/dev/ttyACM0", 
                        baudrate = 115200, 
                        bytesize = serial.EIGHTBITS,
                        parity = serial.PARITY_NONE,
                        stopbits = serial.STOPBITS_ONE,
                        timeout=0.1,
                        rtscts=False)
# num = input("Enter a number: ") # Taking input from user
# # def write_read(x):
# arduino.write(bytes(num, 'utf-8'))
# time.sleep(0.05)
def read_data():
    data = arduino.readline()
    return data
while True:
    # write_read(num)
    data = read_data()
    print(data) # printing the value