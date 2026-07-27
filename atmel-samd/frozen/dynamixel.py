# -----------------------------------------------------------------------------
#
# dynamixel.py
#
# -----------------------------------------------------------------------------
#
#
# The MIT License (MIT)
#
# Copyright (c) 2026 Nick Donaldson <nick@gotrobots.com>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.
#
#
# -----------------------------------------------------------------------------
#
#
# dynamixel.py
#
# CircuitPython library implementing minimal feature set needed to build
# simple robots using the Robotis OpenRB-150 board flashed with a custom 
# CircuitPython firmware and programmed in Pyhthon.
#
# dynamixel.py is supplied as a frozen library built into the UF2 firmware
# used to reflash the OpenRB-150 board as a CircuitPython board
#
# To enable CircuitPython usage on the ATMEL SAMD21 processor with only 32k
# of RAM, only a minimal feature set is provided:
# 
#   - torque enable/disable (write)
#   - goal position (write)
#   - position P gain (write)
#   - profile velocity  (write)
#   - sync write (write)
#   - present position (read)
#   - present temperature (read)
#
# This is sufficient to move motors to encoder targets, check if the target
# was achieved, monitor temperature, tune position gain, and de-energize
# motors after motion. Write commands are sent as fire-and-forget with no
# status checking to keep code implementation minimal; only reads (present
# position, present temperature) wait for and validate a reply.
# 
# Target positions can be sent on a per-motor basis or multiple motors can be 
# moved in a single command with sync_write.
# 
# It is recommended to compile the main program to an mpy file before copying
# to the CIRCUITPYTHON folder to reduce RAM overhead.  Additionally, part or 
# all of the program can be added to the ./frozen/ directory and the firmware 
# can be recompiled to further reduce RAM usage if the program does not fit 
# within the 32K available.
#
# Motor-specific parameters (control table values, encoder ticks etc.) are 
# supplied in the frozen file config.py
#
# An example program demo.py is supplied that demonstrates how all available 
# methods can be used.
#
# -----------------------------------------------------------------------------
#

from time import sleep
import digitalio
import array


class DynamixelPort:
    """Wraps a busio.UART, handling the OpenRB-150 DXL power-enable pin."""

    __slots__ = ("_uart", "_pwr_pin", "_open_state")

    def __init__(self, uart, pwr_pin=None):
        self._uart = uart
        self._pwr_pin = pwr_pin
        self._open_state = False

    def begin(self):
        if self._pwr_pin is not None and not self._open_state:
            self._pwr_pin.direction = digitalio.Direction.OUTPUT
            self._pwr_pin.value = True
            sleep(0.5)  # same 500ms settle time as port_handler.cpp
        self._open_state = True

    def write(self, data):
        return self._uart.write(data)

    def read(self, nbytes):
        # Blocks up to the UART's configured timeout (see demo.py's
        # `busio.UART(..., timeout=0.01)`), returning fewer bytes than
        # requested (or None) if nothing/not enough arrived in time.
        return self._uart.read(nbytes)

    def flush(self):
        # Drains and discards whatever's already sitting in the receive
        # buffer. Every write elsewhere in this file is fire-and-forget,
        # but the servo still sends a status reply for each one (as long
        # as its Status Return Level is at the factory default of
        # "respond to everything") -- nothing has ever read those
        # replies before now, so they pile up. A genuine read needs to
        # start from a clean buffer, or it'll parse stale leftovers from
        # an unrelated earlier write instead of the real reply.
        n = self._uart.in_waiting
        while n:
            self._uart.read(n)
            n = self._uart.in_waiting

    @property
    def baudrate(self):
        return self._uart.baudrate

    @baudrate.setter
    def baudrate(self, value):
        self._uart.baudrate = value


# ---------------------------------------------------------------------------
# CRC-16 (DYNAMIXEL Protocol 2.0) -- required on every outgoing packet so
# the servo's firmware will actually accept it. See the note up top.
# ---------------------------------------------------------------------------

def _make_crc_table():
    table = array.array("H", bytes(512))  # 256 entries x 2 bytes each
    poly = 0x8005
    for i in range(256):
        crc = i << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ poly) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
        table[i] = crc
    return table


_CRC_TABLE = _make_crc_table()


def _update_crc(crc_accum, data):
    for byte in data:
        i = ((crc_accum >> 8) ^ byte) & 0xFF
        crc_accum = ((crc_accum << 8) ^ _CRC_TABLE[i]) & 0xFFFF
    return crc_accum


INST_WRITE = 0x03
INST_READ = 0x02
INST_SYNC_WRITE = 0x83
BROADCAST_ID = 0xFE
_HEADER = b"\xFF\xFF\xFD\x00"


def _finish_packet(dxl_id, inst, params):
    inst_and_params = bytearray([inst]) + params
    orig_len = len(inst_and_params) + 2  # + crc16

    body = bytearray()
    body.append(dxl_id)
    body.append(orig_len & 0xFF)
    body.append((orig_len >> 8) & 0xFF)
    body.extend(inst_and_params)

    # Byte-stuff FF FF FD sequences so the servo can't mistake payload data
    # for a new packet header. Practically never triggers for TE/GP values,
    # but it's cheap and it's the difference between "correct" and "usually
    # correct," so it stays.
    data_b = bytes(body)
    out = bytearray()
    added = 0
    for i, b in enumerate(data_b):
        out.append(b)
        if i >= 2 and data_b[i - 2] == 0xFF and data_b[i - 1] == 0xFF and b == 0xFD:
            out.append(0xFD)
            added += 1
    stuffed_body = bytes(out)

    if added:
        new_len = orig_len + added
        stuffed_body = bytearray(stuffed_body)
        stuffed_body[1] = new_len & 0xFF
        stuffed_body[2] = (new_len >> 8) & 0xFF
        stuffed_body = bytes(stuffed_body)

    crc = _update_crc(0, _HEADER + stuffed_body)
    return _HEADER + stuffed_body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def _build_write_packet(dxl_id, address, data):
    params = bytearray(2 + len(data))
    params[0] = address & 0xFF
    params[1] = (address >> 8) & 0xFF
    params[2:] = data
    return _finish_packet(dxl_id, INST_WRITE, params)


def _build_read_packet(dxl_id, address, length):
    params = bytearray(4)
    params[0] = address & 0xFF
    params[1] = (address >> 8) & 0xFF
    params[2] = length & 0xFF
    params[3] = (length >> 8) & 0xFF
    return _finish_packet(dxl_id, INST_READ, params)


def _build_sync_write_packet(address, size, id_value_pairs):
    # Sync Write params: address(2) + data_length(2), then for each
    # servo: id(1) + data(size). Always sent to BROADCAST_ID -- per
    # protocol, the broadcast ID never gets a status reply regardless
    # of Status Return Level, so this is inherently fire-and-forget,
    # same as every other write in this file.
    params = bytearray(4 + len(id_value_pairs) * (1 + size))
    params[0] = address & 0xFF
    params[1] = (address >> 8) & 0xFF
    params[2] = size & 0xFF
    params[3] = (size >> 8) & 0xFF
    offset = 4
    for dxl_id, value in id_value_pairs:
        params[offset] = dxl_id
        if isinstance(value, int):
            value = (value & ((1 << (8 * size)) - 1)).to_bytes(size, "little")
        params[offset + 1:offset + 1 + size] = value
        offset += 1 + size
    return _finish_packet(BROADCAST_ID, INST_SYNC_WRITE, params)


class DynamixelBus:
    __slots__ = ("port",)

    def __init__(self, port):
        self.port = port

    def begin(self, baudrate=57600):
        self.port.baudrate = baudrate
        self.port.begin()

    def write(self, dxl_id, address, data, size):
        if isinstance(data, int):
            data = (data & ((1 << (8 * size)) - 1)).to_bytes(size, "little")
        self.port.write(_build_write_packet(dxl_id, address, data))

    def sync_write(self, address, size, id_value_pairs):
        # Writes the same register address to multiple servo IDs in ONE
        # packet, each with its own value -- e.g. moving several servos
        # to different positions with a single instruction instead of
        # one packet per servo. id_value_pairs is an iterable of
        # (dxl_id, value) pairs; value may be an int or raw bytes.
        self.port.write(_build_sync_write_packet(address, size, id_value_pairs))

    def read(self, dxl_id, address, size):
        # Sends a Read instruction and returns `size` bytes of raw
        # response data, or None if the read failed for any reason:
        # timeout, too few bytes back, bad header, bad CRC, or the servo
        # reporting an error. This is the ONE place in this file that
        # waits for and parses anything back from a servo -- see the
        # note at the top about why everything else is write-only.
        #
        # NOTE: doesn't un-stuff the response payload (the byte-stuffing
        # a servo applies if its raw data happens to contain FF FF FD) --
        # for a position value in a servo's normal range this essentially
        # never triggers, so it's skipped to keep this small.
        self.port.flush()
        packet = _build_read_packet(dxl_id, address, size)
        self.port.write(packet)
        # NOTE: this board does NOT echo outgoing bytes back onto RX --
        # confirmed directly against real hardware captures, where the
        # very first bytes read back were already a complete, genuine,
        # CRC-valid status reply. An earlier version of this function
        # assumed a half-duplex echo (a real issue on some other boards)
        # and discarded len(packet) bytes before reading the reply --
        # which, on THIS board, was silently eating the start of every
        # real reply instead of a nonexistent echo. Read the actual
        # response directly, with nothing discarded first.
        expected_len = 4 + 1 + 2 + 1 + 1 + size + 2  # header+id+len+inst+err+data+crc
        raw = self.port.read(expected_len)
        if raw is None or len(raw) < expected_len:
            return None
        if raw[0:4] != _HEADER:
            return None
        crc_received = raw[expected_len - 2] | (raw[expected_len - 1] << 8)
        crc_computed = _update_crc(0, raw[0:expected_len - 2])
        if crc_received != crc_computed:
            return None
        if raw[8] != 0:  # error byte
            return None
        return raw[9:9 + size]


class Servo:
    # Object-oriented wrapper around a single servo: create one per
    # motor, then call methods on it (arm.move_to(2048), arm.read_enc())
    # instead of passing dxl_id to a module-level function each time.
    # Carries its own bus/config per instance, so different Servo
    # objects can use different buses or motor models in the same
    # program -- this class does NOT depend on set_bus()/
    # set_motor_config() having been called at all, unlike every
    # module-level function in this file.
    #
    # Every method here duplicates the logic of the equivalent
    # module-level function (motor_on(), move_to(), read_enc(), etc.),
    # just sourcing bus/config from self instead of the module's
    # globals -- the two APIs are intentionally parallel, not one
    # built on top of the other, since instance state and global state
    # aren't interchangeable here.

    __slots__ = ("bus", "id", "config")

    def __init__(self, bus, dxl_id, servo_config):
        self.bus = bus
        self.id = dxl_id
        self.config = servo_config

    def torque_enable(self, enable):
        addr = self.config.te
        self.bus.write(self.id, addr.a, 1 if enable else 0, addr.s)

    def on(self):
        self.torque_enable(True)

    def off(self):
        self.torque_enable(False)

    def move_to(self, position):
        addr = self.config.gp
        self.bus.write(self.id, addr.a, position, addr.s)

    def set_p_gain(self, value):
        addr = self.config.pg
        self.bus.write(self.id, addr.a, value, addr.s)

    def set_velocity(self, value):
        addr = self.config.prv
        self.bus.write(self.id, addr.a, value, addr.s)

    def read_enc(self):
        # Returns the current encoder/position value as an int, or
        # None if the read failed -- same caveats as the module-level
        # read_enc(): this is the one call here that can fail and
        # needs the caller to check for it.
        addr = self.config.pp
        data = self.bus.read(self.id, addr.a, addr.s)
        if data is None:
            return None
        return int.from_bytes(data, "little")

    def read_temp(self):
        # Returns the internal temperature in degrees Celsius, or None
        # if the read failed. Same caveats as read_enc() above.
        addr = self.config.pt
        data = self.bus.read(self.id, addr.a, addr.s)
        if data is None:
            return None
        return int.from_bytes(data, "little")


# ---------------------------------------------------------------------------
# Motor control -- this whole file is intentionally standalone: give it a
# bus (set_bus) and control table (set_motor_config) and it needs
# nothing else. Board-specific pin wiring, application-specific concepts
# (which motor IDs exist, what they're attached to), and any input
# handling belong in your own program (see demo.py), not in this file.
#
# WAIT delays in run_seq()/_exec() are a plain sleep() -- no mid-sequence
# interrupt-checking of any kind, since that would require this library
# to depend on some input mechanism (buttons, RF, etc.) that varies by
# project and has no place in a generic motor-control library.
# ---------------------------------------------------------------------------

motor_config = None


def set_motor_config(cfg):
    global motor_config
    motor_config = cfg


_bus = None


def set_bus(b):
    global _bus
    _bus = b


def motor_on(dxl_id):
    _bus.write(dxl_id, motor_config.te.a, 1, motor_config.te.s)


def motor_off(dxl_id):
    _bus.write(dxl_id, motor_config.te.a, 0, motor_config.te.s)


def motors_on(dxl_ids):
    for mid in dxl_ids:
        motor_on(mid)


def motors_off(dxl_ids):
    for mid in dxl_ids:
        motor_off(mid)


def move_to(dxl_id, pos):
    _bus.write(dxl_id, motor_config.gp.a, pos, motor_config.gp.s)
    #sleep(0.01)


def sync_write(address, size, id_value_pairs):
    # Generic Sync Write: writes `address` on every (dxl_id, value) pair
    # in id_value_pairs, all in one packet. For the common case of
    # moving several servos to different positions at once, see
    # sync_move_to() below.
    _bus.sync_write(address, size, id_value_pairs)


def sync_move_to(id_position_pairs):
    # Moves multiple servos to (possibly different) positions in a
    # single packet instead of one move_to() call per servo.
    # id_position_pairs is an iterable of (dxl_id, position) pairs.
    _bus.sync_write(motor_config.gp.a, motor_config.gp.s, id_position_pairs)


def read_enc(dxl_id):
    # Returns the servo's current encoder/position value as an int, or
    # None if the read failed (timeout, corrupted response, servo
    # error). Unlike every other function in this file, this one can
    # fail and needs the caller to check for it -- reading requires
    # waiting for and trusting a reply, which nothing else here does.
    data = _bus.read(dxl_id, motor_config.pp.a, motor_config.pp.s)
    if data is None:
        return None
    return int.from_bytes(data, "little")


def read_temp(dxl_id):
    # Returns the servo's internal temperature in degrees Celsius, or
    # None if the read failed. Same caveats as read_enc() -- this is a
    # read, so it can fail where the write-only functions in this file
    # can't.
    data = _bus.read(dxl_id, motor_config.pt.a, motor_config.pt.s)
    if data is None:
        return None
    return int.from_bytes(data, "little")


def set_p_gain(dxl_id, value):
    # Sets the Position P Gain (proportional gain of the position
    # control loop). Range and byte size differ by servo family --
    # X-series: 0-16383, 2 bytes. XL320: 0-254, 1 byte -- config.py's
    # motor_config.pg.s already reflects the correct size, so this
    # works unmodified across both.
    _bus.write(dxl_id, motor_config.pg.a, value, motor_config.pg.s)



def set_velocity(dxl_id, value):
    # Sets Profile Velocity -- controls how fast Goal Position moves
    # happen, in Position Control Mode (the servo factory default).
    # 0 means "as fast as possible" (no velocity limiting).
    _bus.write(dxl_id, motor_config.prv.a, value, motor_config.prv.s)


# ---------------------------------------------------------------------------
# run_seq() engine -- FLAT tuple format, not nested, and here's why it
# matters: MicroPython allocates real heap for every tuple literal a
# filesystem-loaded module builds at import time (unlike CPython, it does
# NOT constant-fold nested tuples into flash-resident data), so a sequence
# written as many small nested tuples -- (moves, wait) pairs wrapped in a
# Repeat's own tuple, wrapped in the outer sequence tuple -- pays for every
# one of those container objects on the heap, every time the calling
# module is imported. A single flat tuple per sequence, decoded here with
# sentinel markers, has exactly one container to allocate instead of ~10.
#
# Format: a sequence is one flat tuple of:
#   motor_id, pos, motor_id, pos, ..., WAIT, wait_seconds   -- one step
#   REPEAT, n, <flat sub-sequence...>, END             -- repeat n times
#   some_callable                                       -- called with no args
# WAIT/REPEAT/END are unique sentinel objects (identity-checked), so they
# can't collide with any real motor ID or position value.
#
# _exec() itself is deliberately ITERATIVE, not recursive, for REPEAT
# blocks -- CircuitPython on this board pays real C stack per nested
# Python call, and the available stack here is small enough that even one
# level of REPEAT nesting plus a callable that itself calls into
# motor_on()/write() can exhaust it. The old recursive version called
# _exec() again for every REPEAT; this version tracks loop position/reps-
# remaining on ordinary Python lists (heap memory, which this board has
# more of than C stack), so REPEAT nesting costs zero extra call depth no
# matter how deep or how many behaviors nest it.
# ---------------------------------------------------------------------------

WAIT = object()
REPEAT = object()
END = object()


def run_seq(motor_ids, seq, end="off", home_fn=None, name=None, after=None):
    # home_fn: passed in by your own program's home()-equivalent
    # function when end="home", since that function lives in your
    # application code, not in this library.
    # name: optional label, printed here so every behavior's call site
    # stays a single line.
    # after: optional no-arg callable (e.g. home) invoked once the
    # sequence completes.
    if name:
        print(name)
    for m in motor_ids:
        motor_on(m)
    _exec(seq, 0, len(seq))
    if end == "home":
        home_fn()
    elif end == "off":
        for m in motor_ids:
            motor_off(m)
    if after:
        after()
    return True


def _exec(seq, start, stop):
    call_stack = []   # (resume_i, resume_end) to pop to when a REPEAT ends
    loop_stack = []    # [body_start, body_end, reps_left] for active REPEATs
    i = start
    end = stop
    moves = []

    while True:
        if i >= end:
            if not loop_stack:
                if not call_stack:
                    return True
                i, end = call_stack.pop()
                continue
            body_start, body_end, reps_left = loop_stack[-1]
            reps_left -= 1
            if reps_left > 0:
                loop_stack[-1][2] = reps_left
                i = body_start
                end = body_end
            else:
                loop_stack.pop()
                if call_stack:
                    i, end = call_stack.pop()
                else:
                    return True
            continue

        item = seq[i]
        if item is WAIT:
            wait = seq[i + 1]
            for j in range(0, len(moves), 2):
                move_to(moves[j], moves[j + 1])
            if wait:
                sleep(wait)
            moves = []
            i += 2
        elif item is REPEAT:
            n = seq[i + 1]
            depth = 1
            j = i + 2
            while depth:
                if seq[j] is REPEAT:
                    depth += 1
                elif seq[j] is END:
                    depth -= 1
                j += 1
            inner_stop = j - 1
            call_stack.append((j, end))
            if n > 0:
                loop_stack.append([i + 2, inner_stop, n])
                i = i + 2
                end = inner_stop
            else:
                i, end = call_stack.pop()
            continue
        elif callable(item):
            item()
            i += 1
        else:
            moves.append(item)
            i += 1
