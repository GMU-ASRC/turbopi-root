import serial
import time

SERIAL_PORT = '/dev/pixeltrinkey'
BAUD = 115200

class ColorChange:
    def __init__(self):
        
        #{ Color Index }# 
        self.red = b"255,0,0\n"
        self.green = b"0,255,0\n"
        self.blue = b"0,0,255\n"
        self.off = b"0,0,0\n"
    
    def change_color(self, color='off'):
        colors = {'red': self.red, 'green': self.green, 'blue': self.blue, 'off': self.off}
        if color not in colors:
            print("Invalid Color")
            return

        ser = None
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD)
            time.sleep(0.1)
            ser.write(colors[color])
            print(f"{color} info sent")

        except serial.SerialException as e:
            print(f"Error opening or communicating with the serial port: {e}")
        finally:
            if ser is not None and ser.is_open:
                ser.close()

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