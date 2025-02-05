import os
import stat
import struct
from tempfile import gettempdir

ISNT = os.name == 'nt'
ISROOT = not ISNT and os.geteuid() == 0
if ISROOT:
    try:
        import rpi_ws281x as ws
    except ImportError:
        ws = None
        print("rpi_ws281x not found.")
else:
    ws = None


# DEFINE constants from swigged ws2811.h (in case rpi_ws281x hasn't loaded yet)

# 4 color R, G, B and W ordering
SK6812_STRIP_RGBW = 0x18100800
SK6812_STRIP_RBGW = 0x18100008
SK6812_STRIP_GRBW = 0x18081000
SK6812_STRIP_GBRW = 0x18080010
SK6812_STRIP_BRGW = 0x18001008
SK6812_STRIP_BGRW = 0x18000810
SK6812_SHIFT_WMASK = 0xf0000000

# 3 color R, G and B ordering
WS2811_STRIP_RGB = 0x00100800
WS2811_STRIP_RBG = 0x00100008
WS2811_STRIP_GRB = 0x00081000
WS2811_STRIP_GBR = 0x00080010
WS2811_STRIP_BRG = 0x00001008
WS2811_STRIP_BGR = 0x00000810

# predefined fixed LED types
WS2812_STRIP = WS2811_STRIP_GRB
SK6812_STRIP = WS2811_STRIP_GRB
SK6812W_STRIP = SK6812_STRIP_GRBW


directory = gettempdir()
name = "rgbd.pipe"
path = os.path.join(directory, name)
print(path)

if not ISNT:
    os.makedirs(os.path.dirname(path), exist_ok=True)  # ensure that /tmp/ exists


if os.path.exists(path):
    is_fifo = os.stat(path).st_mode & stat.S_IFIFO
    # if path is not pipe, remove it
    if ISROOT:
        if not is_fifo:
            os.unlink(path)
            os.mkfifo(path, mode=0o666)
        os.chmod(path, 0o666)
else:
    os.mkfifo(path, mode=0o666)
    os.chmod(path, 0o666)


strips = {}


def setup_default_pixels(
    n=2,
    pin=12,
    f=800000,
    dma_channel=10,
    brightness=120,
    channel=0,
    invert=False,
    strip_type=WS2811_STRIP_GRB,
):
    setup_pixels()


def setup_pixels(
    num_leds: int,
    pin: int,
    freq_hz: int,
    dma: int,
    invert: bool,
    brightness: int,
    channel: int,
    strip_type: int,
):
    global strips
    try:
        strip = ws.PixelStrip(num_leds, pin, freq_hz, dma, invert, brightness, channel, strip_type)
        strips[pin] = strip
        strip.begin()
    except RuntimeError as err:
        strips.pop(pin, None)
        print(f"Error: {err}")
    # for i in range(strip.numPixels()):
    #     # strip.setPixelColor(i, ws.Color(0, 0, 0))
    #     strip.setPixelColor(i, 0)
    # strip.show()


def set_pixel(pin: int, i: int, color: int):
    # if not ws:
    #     return
    # strip = strips[pin]
    # color should be a rgb 24-bit or wrgb 32-bit integer (8-bit colors)
    if not isinstance(i, int):
        raise TypeError('Bad type received for pixel number.')
    if not isinstance(color, int):
        raise TypeError('Bad type received for rgb value.')
    if not 0 < i < 0xFF:
        raise ValueError('Pixel number out of range.')

    print(f"Setting pixel {i} to 0x{color:08x}")
    # strip.setPixelColor(i, color)
    # strip.show()


def set_strip_int(pin, array):
    if not ws:
        return
    strip = strips[pin]
    array = list(array)
    if not array:
        return
    if not isinstance(array, (tuple, list)):
        raise TypeError('Did not receive a useful iterable')

    # if all ints, convert to enumerated list
    if all(isinstance(x, int) for x in array):
        array = list(enumerate(array))

    # check types
    for x in array:
        if not isinstance(x, tuple) or not len(x) == 2:
            raise TypeError('Bad type received for RGB array.')
        i, color = x
        if not isinstance(i, int) or not isinstance(color, int):
            raise TypeError('Bad type received for RGB array.')

    for i, color in array:
        # color should be a rgb 24-bit or wrgb 32-bit integer (8-bit colors)
        # print(f"Setting pixel {i} to 0x{color:08x}")
        if not 0 <= i <= 0xFF:
            raise ValueError('Pixel number out of range.')
        strip.setPixelColor(i, color)
    strip.show()


BYTEORDER = 'little'
BOC = '<' if BYTEORDER == 'little' else '>'  # byte order character for struct
WRGB_FORMAT = BOC + 'L'
WRGB_SIZE = struct.calcsize(WRGB_FORMAT)
AWRGB_FORMAT = BOC + 'BL'
AWRGB_SIZE = struct.calcsize(AWRGB_FORMAT)
HEADER_FORMAT = BOC + f'shH'
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)


DATA_FORMATS = {
    b'N': BOC + 'BLb?BBL',
}


def unpack_new(pin, data):
    fmt = DATA_FORMATS[b'N']
    args = list(struct.unpack(fmt, data))
    args.insert(1, pin)
    return args


def unpack_addressed(data):
    return struct.iter_unpack(AWRGB_FORMAT, data)


def unpack_unaddressed(data):
    return [x[0] for x in struct.iter_unpack(WRGB_FORMAT, data) if x and len(x) == 1]


def unpack_packet_inner(actions, remainder):
    header = remainder[:HEADER_SIZE]
    remainder = remainder[HEADER_SIZE:]
    if len(header) != HEADER_SIZE:
        # not enough data to continue. bail out.
        return actions
    action, pin, size = struct.unpack(HEADER_FORMAT, header)
    packed_data = remainder[:size]
    remainder = remainder[size:]
    if len(packed_data) < size or size > 2000:
        # packet is lying/incomplete. Possible corruption.
        return actions
    if packed_data:
        actions.append((action, pin, packed_data))
    if remainder:
        return unpack_packet_inner(actions, remainder)
    else:
        return actions


def unpack_packets(commands):
    actions = unpack_packet_inner([], commands)
    for action, pin, data in actions:
        a = str(action)[1:].strip('\'')
        print(f"{a:>3s} pin {pin: >3}: {data}")

        if action == b'N':
            args = unpack_new(pin, data)
            print(args)
            setup_pixels(*args)
        elif action == b'a':
            if pin not in strips:
                continue
            set_strip_int(pin, unpack_addressed(data))
        elif action == b'u':
            if pin not in strips:
                continue
            set_strip_int(pin, unpack_unaddressed(data))
        else:
            raise ValueError(f"Unknown action: {action}")


if __name__ == '__main__':
    # Read from and write to the named pipe
    i = 0
    while True:
        with open(path, "rb") as pipe:
            data = pipe.read()
            if data:
                # print(f"{i: >3}: {data}")
                try:
                    unpack_packets(data)
                except TypeError as err:
                    print(f"Error: {err}")
                except struct.error as err:
                    print(f"Error: {err}")
        i += 1
