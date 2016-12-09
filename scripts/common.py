import struct
import sys


def pack(fmt, *args):
    return struct.pack('<' + fmt, *args)


def unpack(fmt, *args):
    return struct.unpack('<' + fmt, *args)


def multichr(ords):
    if sys.version_info[0] >= 3:
        return bytes(ords)
    else:
        return ''.join(map(chr, ords))


def multiord(b):
    if sys.version_info[0] >= 3:
        return list(b)
    else:
        return map(ord, b)


class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = multichr(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % b for b in multiord(self.payload)))
