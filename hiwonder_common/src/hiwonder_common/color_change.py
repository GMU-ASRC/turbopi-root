import serial
import time

SERIAL_PORT = '/dev/ttyACM1'
BAUD = 115200

#{ Color Index }# 
red = b"255,0,0\n"
green = b"0,255,0\n"
blue = b"0,0,255\n"
off = b"0,0,0\n"

def change_color(color=off):
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD)
        time.sleep(0.1)

        if color == "red":
            ser.write(red)
            print("red info sent")
            ser.close()

        elif color == "green":
            ser.write(green)
            print("green info sent")
            ser.close()

        elif color == "blue":
            ser.write(blue)
            print("blue info sent")
            ser.close()
            
        else: print("Invalid Color")

    except serial.SerialException as e:
        print(f"Error opening or communicating with the serial port: {e}")

if __name__ == '__main__':
    while True:
        color = input("Color, State of Flash: ").strip().lower()
        if color != "off":
            change_color(color)
        else:
            ser = serial.Serial(SERIAL_PORT, BAUD)
            ser.write(off)
            ser.close()
            break