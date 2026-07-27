# -----------------------------------------------------------------------------
#
# demo.py
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
# Minimal, standalone demo of dynamixel.py for OpenRB-150 + custom
# CircuitPython firmware. Uses the Servo class (motor_a.move_to(...),
# motor_a.read_enc(), etc.) rather than module-level functions -- each
# Servo carries its own bus/config. Exception: sync_move_to() is
# module-level (Sync Write addresses multiple servos at once), so setup
# still calls set_bus()/set_motor_config() for its sake.
#
# Runs two motors (IDs 1/2), sweeping START_ENC<->END_ENC: a simple
# move, then a sin-based motion profile, then a single Sync Write
# packet -- each alone, then both together. Every move sets its own
# Profile Velocity from four named tiers (VELOCITY_MAX/FAST/MEDIUM/
# SLOW) beforehand, and prints target/actual position plus temperature
# after every move.
#
# The multi-motor functions (move_multi/sweep_multi/sync_move) take a
# list of (servo, target, velocity) tuples -- or (servo, start, end,
# velocity) for sweeps -- so they work for any number of motors, not
# just two.
#
# This demo is right at the RAM limit for a plain .py file; anything
# larger needs compiling to .mpy before copying to CIRCUITPY.
#
# -----------------------------------------------------------------------------
#

import gc
gc.collect()
print("Free RAM at start:", gc.mem_free())

from time import sleep
import math
import board
import digitalio
import busio

import dynamixel
import config

gc.collect()
print("Free RAM after import:", gc.mem_free())

# demo uses XL330 motors but you can change is to any X-series motor:
# "XL330", "XM450", "XM540", "XL430", "2XL430", "XC330", "XC430",
# "MX-28(2.0)", "MX-64(2.0)",
MOTOR_TYPE = "XL330"

MOTOR_A = 1
MOTOR_B = 2

BAUD = 115200

# safe sweep range for X series (0-4095)
# change this to 256 / 768 for XL-320
START_ENC = 1024
END_ENC = 3072

TARGET_TOLERANCE = 10 # ticks -- how close to target counts as "arrived"

TARGET_TIMEOUT = 5.0 # max seconds to wait for arrival before giving up

TARGET_POLL_INTERVAL = 0.05  # seconds between polls while waiting

SWEEP_TIME = 2.0  # seconds each sweep_motor() move takes, start to end

# Profile Velocity units are ~0.229 rev/min each (X-series); 100 is a
# moderate, safe-by-default speed (~23 rev/min), comfortably under the
# XL330's default Velocity Limit of 445 for the XL330-M288, or 1620 for
# the XL330-M077.  Motor B runs at double motor A's speed if the motors 
# are the same gear ratio, so the difference is clearly visible. Every 
# move function below sets this before its move, rather than it being set
# once and left alone -- so changing these values takes effect on the
# very next move, at the cost of one extra write per move.
VELOCITY_MAX = 444
VELOCITY_FAST = 333
VELOCITY_MEDIUM = 222
VELOCITY_SLOW = 111

# The return leg of every START<->END pair below runs at half the
# corresponding full-speed velocity.


def _ease(x):
    # x is a fraction in [0, 1] -- returns the eased fraction via the
    # raised-cosine curve (1 - cos(pi*x)) / 2. Its derivative (velocity)
    # is a sine curve: zero at x=0 and x=1, maximum at x=0.5.
    return (1 - math.cos(math.pi * x)) / 2


def _wait_for_target(servo_targets):
    # servo_targets: iterable of (servo, target) pairs. Polls
    # servo.read_enc() for every motor until each is within
    # TARGET_TOLERANCE ticks of its own target, or until TARGET_TIMEOUT
    # seconds have elapsed, whichever comes first -- so a single slow
    # or stuck motor doesn't block forever, and a single failed read
    # (read_enc() can return None) doesn't abort the wait early, since
    # it keeps retrying on the next poll. Returns a dict of
    # {servo: last_known_actual}, which may still be far from target
    # (if it timed out) or None (if every single read failed).
    servo_targets = list(servo_targets)
    actuals = {servo: None for servo, _ in servo_targets}
    elapsed = 0.0
    while elapsed < TARGET_TIMEOUT:
        all_arrived = True
        for servo, target in servo_targets:
            actual = servo.read_enc()
            if actual is not None:
                actuals[servo] = actual
            if actuals[servo] is None or abs(actuals[servo] - target) > TARGET_TOLERANCE:
                all_arrived = False
        if all_arrived:
            break
        sleep(TARGET_POLL_INTERVAL)
        elapsed += TARGET_POLL_INTERVAL
    return actuals

# ---------------------------------------------------------------------------
# Hardware setup -- same DXL bus wiring/baud rate as the rest of this
# project (OpenRB-150, UART on TX1/RX1, DXL_PWR_EN power-enable pin).
# ---------------------------------------------------------------------------

uart = busio.UART(board.TX1, board.RX1, baudrate=BAUD, timeout=0.01)
pwr_pin = digitalio.DigitalInOut(board.DXL_PWR_EN)
port = dynamixel.DynamixelPort(uart, pwr_pin=pwr_pin)
bus = dynamixel.DynamixelBus(port)
bus.begin(baudrate=BAUD)

motor_config = config.get_config(MOTOR_TYPE)

# sync_move_to() is module-level (Sync Write addresses multiple servos
# at once, so it doesn't map onto a single Servo instance) and still
# needs these globals set, even though the Servo instances below don't.
dynamixel.set_bus(bus)
dynamixel.set_motor_config(motor_config)

motor_a = dynamixel.Servo(bus, MOTOR_A, motor_config)
motor_b = dynamixel.Servo(bus, MOTOR_B, motor_config)


def simple_move(servo, target, velocity):
    # moves single servo to target at velocity
    servo.on()
    servo.set_velocity(velocity)
    servo.move_to(target)


def move_multi(moves):
    # moves: list of (servo, target, velocity) tuples -- works for any
    # number of motors, not just two. move_motor() below is a thin
    # wrapper around this for the single-motor case, so there's only
    # one copy of this logic to compile, not two.
    print("\nmove single:" if len(moves) == 1 else "\nmove dual:")
    for servo, target, velocity in moves:
        servo.on()
        servo.set_velocity(velocity)
        servo.move_to(target)
    actuals = _wait_for_target([(servo, target) for servo, target, velocity in moves])
    for servo, target, velocity in moves:
        temp = servo.read_temp()
        print(f"motor {servo.id}: target={target} actual={actuals[servo]} temp={temp}")
        servo.off()


def move_motor(servo, target, velocity):
    move_multi([(servo, target, velocity)])


def sweep_multi(moves, duration, steps_per_sec=20):
    # moves: list of (servo, start, end, velocity) tuples -- each motor
    # can have its own range, sweeping through it in lockstep with the
    # others (same shared time base), rather than being hardcoded to
    # exactly two motors sharing one start/end. sweep_motor() below is
    # a thin wrapper around this for the single-motor case.
    #
    # Position follows the raised-cosine ease curve (see _ease() above).
    # Its derivative (velocity) is a sine curve: zero at t=0 and t=duration,
    # maximum at t=duration/2, exactly matching the requested speed profile.
    # Profile Velocity is set once before the sweep starts, not per step --
    # the sweep's own timing (steps_per_sec/duration) already controls the
    # motion shape; Profile Velocity here caps how fast the servo will
    # chase each individual intermediate target.
    print("\nsweep single:" if len(moves) == 1 else "\nsweep dual:")
    for servo, start, end, velocity in moves:
        servo.on()
        servo.set_velocity(velocity)
    n_steps = max(2, int(duration * steps_per_sec))
    step_time = duration / n_steps
    for i in range(n_steps + 1):
        frac = _ease(i / n_steps)
        for servo, start, end, velocity in moves:
            pos = round(start + (end - start) * frac)
            servo.move_to(pos)
        sleep(step_time)
    actuals = _wait_for_target([(servo, end) for servo, start, end, velocity in moves])
    for servo, start, end, velocity in moves:
        temp = servo.read_temp()
        print(f"motor {servo.id}: target={end} actual={actuals[servo]} temp={temp}")
        servo.off()


def sweep_motor(servo, start, end, duration, velocity, steps_per_sec=20):
    sweep_multi([(servo, start, end, velocity)], duration, steps_per_sec=steps_per_sec)


def sync_move(moves):
    # moves: list of (servo, target, velocity) tuples. Moves every
    # motor to its own target in a SINGLE packet (Sync Write), instead
    # of one move_to() call per motor like move_multi() above. Every
    # motor still gets its own Profile Velocity, torque on/off, and
    # reported values -- only the move command itself is combined into
    # one packet. sync_move_to() is module-level (not a Servo method,
    # since Sync Write addresses multiple IDs at once), so each
    # servo's .id is pulled out to build the underlying call.
    print("\nsync move:")
    for servo, target, velocity in moves:
        servo.on()
        servo.set_velocity(velocity)
    dynamixel.sync_move_to([(servo.id, target) for servo, target, velocity in moves])
    actuals = _wait_for_target([(servo, target) for servo, target, velocity in moves])
    for servo, target, velocity in moves:
        temp = servo.read_temp()
        print(f"motor {servo.id}: target={target} actual={actuals[servo]} temp={temp}")
        servo.off()


def demo():
    # simple move example
    simple_move(motor_a, START_ENC, VELOCITY_MAX)
    simple_move(motor_a, END_ENC, VELOCITY_FAST)

    # complex move examples
    while True:
        # Motor A alone: START -> END (full speed) -> START (half speed)
        move_motor(motor_a, END_ENC, VELOCITY_MAX)
        move_motor(motor_a, START_ENC, VELOCITY_FAST)

        # Motor B alone: START -> END (full speed) -> START (half speed)
        move_motor(motor_b, END_ENC, VELOCITY_MEDIUM)
        move_motor(motor_b, START_ENC, VELOCITY_SLOW)

        # Both together: START -> END (full speed) -> START (half speed)
        move_multi([(motor_a, END_ENC, VELOCITY_MAX), (motor_b, END_ENC, VELOCITY_MEDIUM)])
        move_multi([(motor_a, START_ENC, VELOCITY_FAST), (motor_b, START_ENC, VELOCITY_SLOW)])

        # Motor A alone, smooth sweep: START -> END (full speed) -> START (half speed)
        sweep_motor(motor_a, START_ENC, END_ENC, SWEEP_TIME, VELOCITY_MAX)
        sweep_motor(motor_a, END_ENC, START_ENC, SWEEP_TIME, VELOCITY_FAST)

        # Motor B alone, smooth sweep: START -> END (full speed) -> START (half speed)
        sweep_motor(motor_b, START_ENC, END_ENC, SWEEP_TIME, VELOCITY_MEDIUM)
        sweep_motor(motor_b, END_ENC, START_ENC, SWEEP_TIME, VELOCITY_SLOW)

        # Both together, smooth sweep: START -> END (full speed) -> START (half speed)
        sweep_multi([(motor_a, START_ENC, END_ENC, VELOCITY_MAX), (motor_b, START_ENC, END_ENC, VELOCITY_MEDIUM)], SWEEP_TIME)
        sweep_multi([(motor_a, END_ENC, START_ENC, VELOCITY_FAST), (motor_b, END_ENC, START_ENC, VELOCITY_SLOW)], SWEEP_TIME)

        # Both together, single Sync Write packet: START -> END (full speed) -> START (half speed)
        sync_move([(motor_a, END_ENC, VELOCITY_MAX), (motor_b, END_ENC, VELOCITY_MEDIUM)])
        sync_move([(motor_a, START_ENC, VELOCITY_FAST), (motor_b, START_ENC, VELOCITY_SLOW)])

        gc.collect()
        print("\nFree RAM at end of loop:", gc.mem_free())

gc.collect()
print("Free RAM before loop:", gc.mem_free())

# start demo loop
demo()
