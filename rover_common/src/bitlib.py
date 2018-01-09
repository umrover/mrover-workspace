import struct


def float_to_bits(x, signed=False):
    s = struct.pack('>f', x)
    int_format = '>L'
    if signed:
        int_format = '>l'
    return struct.unpack(int_format, s)[0]


def bits_to_float(x, signed=False):
    int_format = '>L'
    if signed:
        int_format = '>l'
    s = struct.pack(int_format, x)
    return struct.unpack('>f', s)[0]
