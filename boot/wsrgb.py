import sys
import struct

sys.path.append('/home/pi/boot/')  # gross path hacking
import rgbd
from rgbd import path, BYTEORDER, BOC, HEADER_FORMAT, DATA_FORMATS, AWRGB_FORMAT, WRGB_FORMAT


def write(cmd: bytes, pin: int, data=b''):
    header = struct.pack(HEADER_FORMAT, cmd, pin, len(data))
    packet = header + data
    with open(path, 'wb') as pipe:
        return pipe.write(packet)


def rgbw_to_int(r, g, b, w=0):
    return int.from_bytes(struct.pack('>BBBB', w, r, g, b), byteorder='big')


def set_pixel(pin, i, r=0, g=None, b=None, w=None):
    CMD = b'S'
    if (g, b, w) == (None, None, None):
        rgb = r
        if isinstance(rgb, bytes):
            rgb = int.from_bytes(rgb, byteorder='big')
    elif all(isinstance(x, int) for x in (r, g, b)):
        if w is not None:
            raise NotImplementedError("RGBW white channel not implemented yet.")
        w = 0 if w is None else w
        rgb = rgbw_to_int(r, g, b, w)
    else:
        raise ValueError
    write(CMD, pin, struct.pack(AWRGB_FORMAT, i, rgb))


def set_pixels(pin, array):
    # four array formats are supported:
    # 1. [(i, rgbw_int), ...]
    # 2. [(i, (r, g, b, [w])), ...]
    # 3. [(r, g, b, [w]), ...]
    # 4. [rgbw_int, ...]

    cmd = b'a'  # for addressed
    if all(isinstance(obj, int) for obj in array):  # case 4
        cmd = b'u'  # for unaddressed
    elif all(3 <= len(pix) <= 4  for pix in array):  # case 3
        cmd = b'u'  # for unaddressed
        array = [rgbw_to_int(*pix) for pix in array]
    elif all(isinstance(obj[1], (tuple, list)) for obj in array):
        array = [(i, rgbw_to_int(*rgbw)) for i, rgbw in array]
    if cmd == b'a':
        data = b''.join(struct.pack(AWRGB_FORMAT, i, rgb_int) for i, rgb_int in array)
    else:
        data = b''.join(struct.pack(WRGB_FORMAT, rgb_int) for rgb_int in array)
    write(cmd, pin, data)


def setup(
    pin: int,
    num_leds: int,
    freq_hz: int = 800000,
    dma: int = 10,
    invert: bool = False,
    brightness: int = 120,
    channel: int = 0,
    strip_type: int = rgbd.WS2811_STRIP_GRB,
):
    cmd = b'N'
    fmts = DATA_FORMATS[cmd]
    data = (num_leds, freq_hz, dma, invert, brightness, channel, strip_type)
    strip = struct.pack(fmts, *data)
    write(cmd, pin, strip)


def setup_default_pixels(
    n=2,
    pin=12,
    f=800000,
    dma_channel=10,
    brightness=120,
    channel=0,
    invert=False,
):
    setup(pin, n, f, dma_channel, invert, brightness, channel)
