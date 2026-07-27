# -----------------------------------------------------------------------------
#
# config.py
#
# -----------------------------------------------------------------------------
#
#
# The MIT License (MIT)
#
# Copyright (c) 2026 Nick Donaldson <nick@gotrobotsd.com>
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
# Config file for the dynamixel.py CircuitPython library needed to build
# simple robots using the Robotis OpenRB-150 board flashed with a custom 
# CircuitPython firmware and programmed in Pyhthon.
#
# Contains motor-specific parameters (control table values, encoder ticks etc.)
# 
# config.py is supplied as a frozen library built into the uf2 firmware
# alongside the main library dynamixel.py 
# 
# An example program demo.py is supplied that demonstrates how all available 
# methods can be used.
#
# Control table entries: Torque Enable, Goal Position, Present Position,
# Present Temperature, Position P Gain, Profile Velocity
#
#
# demo.py picks its motor model by name through get_config():
#
#     import config
#     XL330_CONFIG = config.get_config("XL330")
#
# so swapping motor models is a one-line change in demo.py, no firmware
# rebuild, as long as the new model shares one of the two register maps
# below (X-series, or XL320).
#
# -----------------------------------------------------------------------------
#

class Addr:
    __slots__ = ("a", "s")

    def __init__(self, a, s):
        self.a = a
        self.s = s


class MotorConfig:
    __slots__ = ("nm", "te", "gp", "pp", "pt", "pg", "prv")

    def __init__(self, nm, te, gp, pp, pt, pg, prv):
        self.nm = nm
        self.te = te
        self.gp = gp
        self.pp = pp
        self.pt = pt    # Present Temperature (read)
        self.pg = pg    # Position P Gain (write)
        self.prv = prv  # Profile Velocity -- controls how fast a position
                        # move happens; works in Position Control Mode,
                        # the servo's factory-default operating mode.


# X-series control table -- identical Torque Enable / Goal Position /
# Present Position addresses across XL330, XM, XC, MX(2.0), etc.
_TE_X = Addr(64, 1)
_GP_X = Addr(116, 4)
_PP_X = Addr(132, 4)
_PT_X = Addr(146, 1)
_PG_X = Addr(84, 2)
_PRV_X = Addr(112, 4)

_X_SERIES_MODELS = (
    "XL330", "XM450", "XM540", "XL430", "2XL430", "XC330", "XC430",
    "MX-28(2.0)", "MX-64(2.0)",
)

# XL320 has its own, different control table. Its P Gain is 1 byte (not
# 2, like X-series), and it has no separate Profile Velocity register --
# "Moving Speed" is the closest equivalent (speed of a Joint Mode move
# to Goal Position), so prv points there directly.
_TE_XL320 = Addr(24, 1)
_GP_XL320 = Addr(30, 2)
_PP_XL320 = Addr(37, 2)
_PT_XL320 = Addr(46, 1)
_PG_XL320 = Addr(29, 1)
_PRV_XL320 = Addr(32, 2)

_cache = {}


def get_config(name):
    """Look up (and lazily build + cache) the MotorConfig for `name`."""
    if name in _cache:
        return _cache[name]
    if name == "XL320":
        cfg = MotorConfig(name, _TE_XL320, _GP_XL320, _PP_XL320,
                           _PT_XL320, _PG_XL320, _PRV_XL320)
    elif name in _X_SERIES_MODELS:
        cfg = MotorConfig(name, _TE_X, _GP_X, _PP_X,
                           _PT_X, _PG_X, _PRV_X)
    else:
        raise KeyError("unknown motor model %r" % (name,))
    _cache[name] = cfg
    return cfg
