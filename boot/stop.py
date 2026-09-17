#!/usr/bin/python3
import sys
try:
    sys.path.append('/home/pi/boot/')
    import buttonman
    buttonman.TaskManager().close_all_registered()
except Exception:
    pass
sys.path.append('/home/pi/TurboPi/')
import HiwonderSDK.Board as Board
import HiwonderSDK.Sonar as Sonar
for i in range(1, 5):
    Board.setMotor(i, 1)
    Board.setMotor(i, 0)
for i in range(3):
    for i in range(1, 5):
        Board.setMotor(i, 1)
    for i in range(1, 5):
        Board.setMotor(i, 0)
for i in range(1, 5):
    Board.setMotor(i, 0)
Board.setBuzzer(0)
s = Sonar.Sonar()
s.setRGBMode(0)
r, g, b = 0, 0, 0
for i in range(2):
    Board.RGB.setPixelColor(i, Board.PixelColor(r, g, b))
    s.setPixelColor(i, Board.PixelColor(r, g, b))
Board.RGB.show()
s.show()
