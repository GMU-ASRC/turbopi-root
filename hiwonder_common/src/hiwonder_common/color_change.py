import serial
import time

SERIAL_PORT = '/dev/ttyACM1'
BAUD = 115200

class ColorChange:
    def __init__(self):
        
        #{ Color Index }# 
        self.red = b"255,0,0\n"
        self.green = b"0,255,0\n"
        self.blue = b"0,0,255\n"
        self.off = b"0,0,0\n"
    
    def change_color(self, color='off'):
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD)
            time.sleep(0.1)

            if color == 'red':
                ser.write(self.red)
                print("red info sent")
                ser.close()

            elif color == 'green':
                ser.write(self.green)
                print("green info sent")
                ser.close()

            elif color == 'blue':
                ser.write(self.blue)
                print("blue info sent")
                ser.close()
            
            elif color == 'off':
                ser.write(self.off)
                print("off info sent")
                ser.close()
                
            else: print("Invalid Color")

        except serial.SerialException as e:
            print(f"Error opening or communicating with the serial port: {e}")

# if __name__ == '__main__':
#     while True:
#         color = input("Color, State of Flash: ").strip().lower()
#         if color != "off":
#             change_color(color)
#         else:
#             ser = serial.Serial(SERIAL_PORT, BAUD)
#             ser.write(off)
#             ser.close()
#             break