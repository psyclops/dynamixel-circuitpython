-----------------------------------------------------------------------------

   dynamixel-circuitpython / dynamixel.py                          7/27/26

-----------------------------------------------------------------------------

CircuitPython library and associated custom CircuitPython firmware to enable
driving Robotis Dynamixel servo motors attached to an OpenRB-150 board 
running as a CircuitPython device.

-----------------------------------------------------------------------------

The MIT License (MIT)

Copyright (c) 2026 Nick Donaldson <nick@gotrobotsd.com>

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

-----------------------------------------------------------------------------

Inventory:

    circuitpython_robotis_openrb-150_en_US-10.2.1.bin - firmware file

    code.py - CIRCUITPYTHON default launch point file

    demo.py - demonstration program 

    demo.mpy - compiled demo.py

    README.md - this file
    
    atmel-samd/ - CircuitPython ATMEL build config overlay directory

      boards/

        openrb_150/ - OpenRB-150 board config

          board.c - board inclusion file

          mpconfigboard.h - C header file 

          mpconfigboard.mk - Makefile

          pins.c - hardware pin mappings

      frozen/ - files frozen into UF2 firmware

        config.py - motor specific configuration

        dynamixel.py - main dynamixel library

-----------------------------------------------------------------------------

Installation

To flash the firmware to the OpenRB-150, you must first install the Arduino 
IDE:

    https://docs.arduino.cc/software/ide/

Once installed, we can use the Arduino bossac application to flash the board.

Put the board into bootloader mode by double clicking the reset button.  The 
yellow LED should pulse slowly.

Run the following command, updating the pathing for your system (MacOS):

~/Library/Arduino15/packages/arduino/tools/bossac/1.7.0-arduino3/bossac
  -i -d --port=cu.usbmodem1101 -e -w -v -b 
  circuitpython_robotis_openrb-150_en_US-10.2.1.bin -R

Once flashed, the board will boot as a CircuitPython device and a CIRCUITPY
flash drive should appear in Finder.  Copy both demo.py and code.py to 
CIRCUITPY, attach two XL330 motors as ID 1 & 2 at 1,000,000 baud and you 
should see the motors move.

-----------------------------------------------------------------------------

About the dynamixel.py library

dynamixel.py is a CircuitPython library implementing a limited feature set 
needed to build simple robots using the Robotis OpenRB-150 board flashed with 
a custom CircuitPython firmware and programmed in Pyhthon.

dynamixel.py is supplied as a frozen library built into the UF2 firmware used
to reflash the OpenRB-150 board as a CircuitPython board

To enable CircuitPython usage on the ATMEL SAMD21 processor with only 32k
of RAM, only a limited feature set is provided:

  - torque enable/disable (write)
  - goal position (write)
  - position P gain (write)
  - profile velocity  (write)
  - sync write (write)
  - present position (read)
  - present temperature (read)

This is sufficient to move motors to encoder targets, check if the target
was achieved, monitor temperature, tune position gain, and de-energize
motors after motion. Write commands are sent as fire-and-forget with no
status checking to keep code implementation minimal; only reads (present
position, present temperature) wait for and validate a reply.

Target positions can be sent on a per-motor basis or multiple motors can be 
moved in a single command with sync_write.

It is recommended to compile the main program to an mpy file before copying
to the CIRCUITPYTHON folder to reduce RAM overhead.  Additionally, part or 
all of the program can be added to the ./frozen/ directory and the firmware 
can be recompiled to further reduce RAM usage if the program does not fit 
within the 32K available.

Motor-specific parameters (control table values, encoder ticks etc.) are 
supplied in the frozen file config.py

An example program demo.py is supplied that demonstrates how all available 
methods can be used.

-----------------------------------------------------------------------------

Usage:

A uart bus object is created for communication on the serial bus:

    uart = busio.UART(board.TX1, board.RX1, baudrate=BAUD, timeout=0.01)
    pwr_pin = digitalio.DigitalInOut(board.DXL_PWR_EN)
    port = dynamixel.DynamixelPort(uart, pwr_pin=pwr_pin)
    bus = dynamixel.DynamixelBus(port)
    bus.begin(baudrate=BAUD)

A servo instance is then created for each motor.  This instance is then referenced for all read and write actions.

    motor_a = dynamixel.Servo(bus, ID, motor_config)


-----------------------------------------------------------------------------

Recompiling the CircuitPython firmware

If you want to modify either the dynamixel.py or config.py files you will need to recompile the CircuitPython firmware.  This is failry straight forward with provided circuitpython/ config overlay.

1. Install build sofware and dependencies

    brew install git gettext Python3
    
    
2. Download the CircuitPython repo:

    git clone https://github.com/adafruit/circuitpython
    cd circuitpython

3. (Optionally) check out a specific version (this firmware was compiled with 
CircuitPython version 10.2.1

    git checkout 10.2.1

4. Build the MicroPython cross-compiler tool mpy-cross first. This component packages built-in libraries into the core image. 

    make -C mpy-cross

5. Fetch submodules for your port: navigate into the specific hardware directory (port) for the board. For the OpenRB-150 with its ATMEL SAMD21processor the correct directory is ports/atmel-samd. Fetch only the submodules needed for that hardware to save time and storage. 

    cd ports/atmel-samd 
    make fetch-port-submodules

6. Copy the OpenRB-150 build overlay over the current directory

    cp -R /path/to/dynamixel-circuitpython/atmel-samd/* .

7. Build the firmware

    make BOARD=openrb_150

 - Note: if the build failes with the following error:

build-openrb_150/firmware.elf will not fit in region `FLASH_FIRMWARE'    

then you must edit the mpconfigboard.h file to decrease the space reserved
for the flash filesystem.  Edit the file and reduce the value in the
following line by 1 and recompile:

    vi boards/openrb_150/mpconfigboard.h

#define CIRCUITPY_INTERNAL_FLASH_FILESYSTEM_SIZE (37* 1024)

becomes:

#define CIRCUITPY_INTERNAL_FLASH_FILESYSTEM_SIZE (36* 1024)

Delete the build dir, recompile and repeat until the build succeeds:

    rm -rf build-openrb_150
    make BOARD=openrb_150

8. Flash the firmware to the board.  Put the board into bootloader mode by double clicking the reset button.  The yellow LED should pulse slowly.

Run the following command, updating the pathing for your system (MacOS):

~/Library/Arduino15/packages/arduino/tools/bossac/1.7.0-arduino3/bossac
  -i -d --port=cu.usbmodem1101 -e -w -v -b 
  circuitpython_robotis_openrb-150_en_US-10.2.1.bin -R

Once flashed, the board will boot as a CircuitPython device and a CIRCUITPY
flash drive should appear in Finder.  

9. Install program files and test.  Copy both demo.py and code.py to CIRCUITPY and attach two XL330 motors as ID 1 & 2 at 1,000,000. The motors 
should move.

10. Check the repl

With the board connected, look for the /dev/cu.usbmodem1101 device in /dev and connect a serial monitor to it:

    ls /dev/cu.usbmodem*
    /dev/cu.usbmodem1101

    screen /dev/cu.usbmodem1101

You should see debug output including target and actual position and motor 
temperature as the motors move.

-----------------------------------------------------------------------------
