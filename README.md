### HMC5883L/BNO055/BNO085 header-only library for pico c/c++ sdk

- Suported magnetometers HMC5883L(mag), BNO055 (9dof IMU), BNO085 (fused absolute rotation quaternion)
- The original bosch sensortec driver for bno055 is used as is (under imu/bno055 folder)
- The original ceva hillcrest labs driver for bno085 is used as is (under imu/sh2 folder)
- bno055.h and bno085.h headers under the root folder are just wrapers over the original bno055/085 drivers with pico specific i2c read/write HAL callbacks.
- HMC5883L uses NED reference frame (x is the yaw/heading vector) while bno055/085 deafult is ENU (y is the yaw/heading vector)
- Clone this repository at the same level as other examples under the pico-examples directory.
- Edit the root CMakeLists.txt of the pico-examples repository and include this repository in the build using add_subdirectory(hmc5883l).
- Build the examples as usual using 'cmake --build ..'
- Calibrate the magnetometer first using the calibrate function.
- Copy the resulting UF2 file (main.uf2) onto the Pico and use PuTTY to check the heading values.
- Reported heading can be cross-verified using a floating magnetized needle or something equivalent.
- The heading should vary from 0 degrees (true geographic north) all the way to 360 degrees (one full x-y plane rotation), and then wrap back to 0.
- all mag/imu headings are pointing to magnetic north. As such, local declination needs to be accounted for.
- The app directory has a c++ proxy app that reads messages off pico's COM port (usb) connection and publishes them over a websocket server
- App directory also has a three.js visualizer that maintains a websocket connection to the c++ proxy app.
- The axis helper gets rotate around the z axis based on the yaw reported by the IMU over USB/UART.

c++ proxy app
```
vcpkg install websocketpp
cmake -DCMAKE_TOOLCHAIN_FILE=D:/vcpkg/scripts/buildsystems/vcpkg.cmake ..
cmake --build . --config Debug
app.exe \\.\COM4
```

three.js visualizer
```
npm install http-server -g
http-server -p 1234 --cors
launch the visualizer url http://localhost:1234/
```

```
Download and unzip arm-none-eabi toolchain.
Download and unzip nanopb from the below url
https://jpa.kapsi.fi/nanopb/download/
SET PATH=D:\Python39;%PATH%
```
### PICO
```
Download and clone the latest pico sdk
git clone https://github.com/raspberrypi/pico-sdk.git
git submodule update --init
SET PICO_BOARD=PICO_W (if building for pico-w and "pico2" or "pico2_w" for the rp2350 variants)
SET PICO_TOOLCHAIN_PATH=D:\arm-none-eabi-14.2.1
cmake -G "Ninja" -DPLATFORM=PICO -DNANOPB_SRC_ROOT_FOLDER=D:/nanopb-0.4.9.1  ..
cmake --build .
```

### STM32
```
git clone https://github.com/ARM-software/CMSIS_5
cd CMSIS_5/Device && mkdir ST && cd ST
git clone https://github.com/STMicroelectronics/cmsis_device_f4
git clone https://github.com/STMicroelectronics/cmsis_device_f7
git clone https://github.com/STMicroelectronics/cmsis-device-h7.git
```

NOTE
```
Please make sure that the HSE_VALUE defined in system_stm32f4xx.c and
system_stm32f7xx.c matches the actual HSE resonator present on the board. For example, WeAct
v3.1 blackpill has a 25MHz crystal and the HSE_VALUE in system_stm32f4xx.c is also 25000000Hz.
However on Nucleo-F767ZI the 8MHz MCO output of the attached stlink is used as HSE source while
the system_stm32f7xx.c has HSE_VALUE of 25000000Hz. So we need to change the HSE_VALUE for F7.
```

cmake configure - F4 line (F411 and F446 targets)
```
cmake -G "Ninja" ^
-DPLATFORM=STM32 ^
-DCMSIS_LINE=f4 ^
-DCMSIS_TARGET=f411 ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMSIS_ROOT=D:/CMSIS_5 ^
-DTOOLCHAIN_PREFIX="D:/arm-none-eabi-14.2.1" ^
-DNANOPB_SRC_ROOT_FOLDER=D:/nanopb-0.4.9.1 ^
-DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ..

cmake --build .
```

connect st-link and verify that it is detected
```
st-info --probe
~$ st-info  --probe
Found 1 stlink programmers
  version:    V2J39S7
  serial:     B55B5A1A000000001062E701
  flash:      524288 (pagesize: 16384)
  sram:       131072
  chipid:     0x0431
  descr:      stm32f411re
```

openocd
```
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

openocd -f interface/stlink.cfg -f target/stm32h7x_dual_bank.cfg ^
  -c "program build/m7core.elf verify; program build/m4core.elf reset verify exit"
```

Flash (telnet)
```
~/code/mcl/build$ telnet localhost 4444
Trying 127.0.0.1...
Connected to localhost.
Escape character is '^]'.
Open On-Chip Debugger
> reset init
Unable to match requested speed 2000 kHz, using 1800 kHz
Unable to match requested speed 2000 kHz, using 1800 kHz
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08000430 msp: 0x20020000
Unable to match requested speed 8000 kHz, using 4000 kHz
Unable to match requested speed 8000 kHz, using 4000 kHz
> flash write_image erase mcu.elf
auto erase enabled
wrote 16384 bytes from file mcu.elf in 0.486814s (32.867 KiB/s)
> reset halt
Unable to match requested speed 2000 kHz, using 1800 kHz
Unable to match requested speed 2000 kHz, using 1800 kHz
target halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08000430 msp: 0x20020000
> resume
```

instead of resume you can quit this telnet session and attach gdb as shown below

Debug (gdb)
```
~/code/mcl/build$ gdb-multiarch ./mcu.elf
GNU gdb (Ubuntu 12.1-0ubuntu1~22.04.2) 12.1
Reading symbols from ./mcu.elf...
(gdb) target extended-remote localhost:3333
Remote debugging using localhost:3333
Reset_Handler () at /home/nmam/code/mcl/startup_stm32.s:45
45	  movs	r1, #0
(gdb) b main
Breakpoint 1 at 0x80003e4: file /home/nmam/code/mcl/main.c, line 11.
Note: automatically using hardware breakpoints for read-only addresses.
(gdb) c
Continuing.
Breakpoint 1, main () at /home/nmam/code/mcl/main.c:11
11	int main(void) {
(gdb) p SystemCoreClock
$1 = 16000000
(gdb) n
halted: PC: 0x080003e6
13	    rcc_init();
(gdb) n
halted: PC: 0x080002fc
15	    SystemCoreClockUpdate();
(gdb) n
halted: PC: 0x08000490
17	    RCC->AHB1ENR |= (1 << RCC_AHB1ENR_GPIOCEN_Pos);
(gdb) p SystemCoreClock
$2 = 100000000
(gdb) c
Continuing.
```

```
https://github.com/WeActStudio/WeAct_HID_Bootloader_F4x1
https://github.com/rbm78bln/STM32duino-bootloader_stm32f411-blackpill
https://kleinembedded.com/stm32-without-cubeide-part-2-cmsis-make-and-clock-configuration/
WeAct Studio Black Pill V3.1 has an external oscillator that has a frequency of 25 MHz
https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1
full F4 package(not needed)
git clone --recursive https://github.com/STMicroelectronics/STM32CubeF4.git
```

H7
```
cm4@200MHz ---> SMPS
cm7@400Mhz ---> SMPS
cm4@240Mhz ---> LDO (board rework required)
cm7@480MHz ---> LDO (board rework required)
cm4/cm7 flash:
openocd -f interface/stlink.cfg -f target/stm32h7x_dual_bank.cfg
2 seperate firmware, one each for cm4 and cm7 which can be flashed independently
flash write_image erase C:/Users/nmam/Desktop/stm/NUCLEO-H755ZI-Q/CM4/Debug/NUCLEO-H755ZI-Q_CM4.elf
flash write_image erase C:/Users/nmam/Desktop/stm/NUCLEO-H755ZI-Q/CM7/Debug/NUCLEO-H755ZI-Q_CM7.elf
```
```
STM32H7x7 and STM32H7x5 devices contains two cores : CM7 + CM4
The second core creation is only done when
  DUAL_CORE variable is set to true in stm32h7x_dual_bank.cfg
  non HLA interface (DAP) is used
A second check for the second core existence is done in cpu1 examine-end
Once the second core is detected it gets examined.

stm32h7x_dual_bank.cfg
----------
set DUAL_CORE 1
set USE_CTI 1
set DUAL_BANK 1
source [find target/stm32h7x.cfg]
-----------
```

```
openocd -f interface/stlink-dap.cfg -f target/stm32h7x_dual_bank.cfg
Open On-Chip Debugger 0.12.0 (2023-01-14-23:37)
Licensed under GNU GPL v2
For bug reports, read
        http://openocd.org/doc/doxygen/bugs.html
Info : auto-selecting first available session transport dapdirect_swd. To override use transport select <transport>.
stm32h7x_cti_prepare_restart
Info : Listening on port 6666 for tcl connections
Info : Listening on port 4444 for telnet connections
Info : STLINK V3J13M4 (API v3) VID:PID 0483:3754
Info : Target voltage: 3.278671
Info : Unable to match requested speed 1800 kHz, using 1000 kHz
Info : Unable to match requested speed 1800 kHz, using 1000 kHz
Info : clock speed 1000 kHz
Info : stlink_dap_op_connect(connect)
Info : SWD DPIDR 0x6ba02477
Info : [stm32h7x.cpu0] Cortex-M7 r1p1 processor detected
Info : [stm32h7x.cpu0] target has 8 breakpoints, 4 watchpoints
Info : [stm32h7x.cpu1] Cortex-M4 r0p1 processor detected
Info : [stm32h7x.cpu1] target has 6 breakpoints, 4 watchpoints
Info : gdb port disabled
Info : starting gdb server for stm32h7x.cpu0 on 3333
Info : Listening on port 3333 for gdb connections
Info : starting gdb server for stm32h7x.cpu1 on 3334
Info : Listening on port 3334 for gdb connections
Info : accepting 'telnet' connection on tcp/4444
[stm32h7x.cpu0] halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08000b48 msp: 0x24080000
[stm32h7x.cpu1] halted due to debug-request, current mode: Thread
xPSR: 0x01000000 pc: 0x08100c08 msp: 0x10048000
Info : Unable to match requested speed 4000 kHz, using 3300 kHz
Info : Unable to match requested speed 4000 kHz, using 3300 kHz
Info : accepting 'gdb' connection on tcp/3334
Info : Device: STM32H74x/75x
Warn : stm32h7x.cpu1 cannot read the flash size register
Info : assuming 2048k flash
Info : STM32H7 flash has dual banks
Info : Bank (2) size is 1024 kb, base address is 0x08000000
Info : Device: STM32H74x/75x
Warn : stm32h7x.cpu1 cannot read the flash size register
Info : assuming 2048k flash
Info : STM32H7 flash has dual banks
Info : Bank (3) size is 1024 kb, base address is 0x08100000
Info : New GDB Connection: 1, Target stm32h7x.cpu1, state: halted
Info : dropped 'gdb' connection
shutdown command invoked
```
```
cmake -G "Ninja" ^
-DPLATFORM=STM32 ^
-DCMSIS_LINE=h7 ^
-DCMSIS_CORE=CM7 ^
-DCMAKE_BUILD_TYPE=Debug ^
-DCMSIS_ROOT=D:/CMSIS_5 ^
-DTOOLCHAIN_PREFIX="D:/arm-none-eabi-14.2.1" ^
-DNANOPB_SRC_ROOT_FOLDER=D:/nanopb-0.4.9.1 ^
-DCMAKE_TOOLCHAIN_FILE=../toolchain.cmake ..

cmake --build .
```

wifi Setup

```
find the MAC of the MCU's wifi chip like so:

C:\Windows\System32>arp /a

Interface: 192.168.31.35 --- 0xa
  Internet Address      Physical Address      Type
  192.168.31.1          54-3d-60-1f-ab-b8     dynamic
  192.168.31.17         14-85-54-06-59-d8     dynamic
  192.168.31.36         28-cd-c1-0a-16-68     dynamic <----------- MCU's ip entry
  192.168.31.255        ff-ff-ff-ff-ff-ff     static
  224.0.0.22            01-00-5e-00-00-16     static
  224.0.0.251           01-00-5e-00-00-fb     static
  224.0.0.252           01-00-5e-00-00-fc     static
  239.255.255.250       01-00-5e-7f-ff-fa     static
  255.255.255.255       ff-ff-ff-ff-ff-ff     static

Make the MCU's ip entry in arp cache as static as then it won't get purged across reboot.

Delete the existing dynamic arp cache entry:

arp /d 192.168.31.36

Then add it back again as static (requires elevated cmd prompt)

netsh interface ipv4 add neighbors "Wi-Fi" "192.168.31.36" "28-cd-c1-0a-16-68"

- "Wi-Fi" is the name of the wifi adapter as shown in the ncpa.cpl GUI
- MAC address is permananet and wont change for the wifi device
- "192.168.31.36" would also be usually assigned back to the same device by the router based on the MAC

C:\Windows\System32>arp /a

Interface: 192.168.31.35 --- 0xa
  Internet Address      Physical Address      Type
  192.168.31.1          54-3d-60-1f-ab-b8     dynamic
  192.168.31.17         14-85-54-06-59-d8     dynamic
  192.168.31.36         28-cd-c1-0a-16-68     static -------------> arp entry is now static
  192.168.31.255        ff-ff-ff-ff-ff-ff     static
  224.0.0.22            01-00-5e-00-00-16     static
  224.0.0.251           01-00-5e-00-00-fb     static
  224.0.0.252           01-00-5e-00-00-fc     static
  239.255.255.250       01-00-5e-7f-ff-fa     static
  255.255.255.255       ff-ff-ff-ff-ff-ff     static

```