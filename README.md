# stm32f4-template

## Description

A Standard Template for starting STM32F4xx C projects.  This project includes the FreeRTOS Realtime Operating System source code.  This project also includes the ST CMSIS
 and ST Standard Peripheral Drivers.  In addition to the libraries supplied by ST, I have also included libopencm3 as an option for accessing the STM32F4 hardware with a 
 higher-level C library.  Finally, I have included a usbhost library for simplified USB host function development.

## Libraries

* FreeRTOS - https://github.com/FreeRTOS/FreeRTOS.git
* libopencm3 - https://github.com/libopencm3/libopencm3.git
* ST CMSIS - https://github.com/ARM-software/CMSIS_5.git
* libusbhost - https://github.com/libusbhost/libusbhost.git

## Installation

### Cloning GitHub Repository
Clone the project using the following command:

```bash
$ git clone --recurse-submodules https://github.com/tmendoza/stm32f4-template.git
```

### Install GCC ARM Toolchain and STLINK ARM Programmer

```bash
$ cd stm32f4-template
$ make -f makefiles/makefile.tools tools_install
$ curl -L -k -o "/Users/tmendoza/Development/gitrepos/work/stm32f4-template/makefiles/../downloads/gcc-arm-none-eabi-9-2019-q4-major-mac.tar.bz2" -z "/Users/tmendoza/Development/gitrepos/work/stm32f4-template/makefiles/../downloads/gcc-arm-none-eabi-9-2019-q4-major-mac.tar.bz2" "https://developer.arm.com/-/media/Files/downloads/gnu-rm/9-2019q4/RC2.1/gcc-arm-none-eabi-9-2019-q4-major-mac.tar.bz2"
Warning: Illegal date format for -z, --time-cond (and not a file name). 
Warning: Disabling time condition. See curl_getdate(3) for valid date syntax.
  % Total    % Received % Xferd  Average Speed   Time    Time     Time  Current
                                 Dload  Upload   Total   Spent    Left  Speed
100   248  100   248    0     0    317      0 --:--:-- --:--:-- --:--:--   317
 54  111M   54 60.9M    0     0  7428k      0  0:00:15  0:00:08  0:00:07  9.7M
 ...
```

### Compile and Install libopencm3

```bash
$ cd libs/libopencm3
$ make
  GENHDR  include/libopencm3/stm32/f0/irq.json
  GENHDR  include/libopencm3/stm32/f1/irq.json
  GENHDR  include/libopencm3/stm32/f2/irq.json
  GENHDR  include/libopencm3/stm32/f3/irq.json
  GENHDR  include/libopencm3/stm32/f4/irq.json
  GENHDR  include/libopencm3/stm32/f7/irq.json
  GENHDR  include/libopencm3/stm32/l0/irq.json
  GENHDR  include/libopencm3/stm32/l1/irq.json
  ...
```

## Development
The repository has everything needed within it to successfully build the test file.  

### Compile/Build the Template application 

```bash
$ cd ../..
$ make
[CC] stm32f4xx_it.c
[CC] system_stm32f4xx.c
[CC] main.c
[CC] syscalls.c
[CC] port.c
[CC] list.c
[CC] queue.c
[CC] tasks.c
[CC] event_groups.c
[CC] timers.c
[CC] heap_4.c
[CC] misc.c
[CC] stm32f4xx_dcmi.c
[CC] stm32f4xx_rtc.c
[CC] stm32f4xx_adc.c
[CC] stm32f4xx_dma.c
[CC] stm32f4xx_sai.c
[CC] stm32f4xx_can.c
[CC] stm32f4xx_dma2d.c
[CC] stm32f4xx_sdio.c
[CC] stm32f4xx_cec.c
[CC] stm32f4xx_dsi.c
[CC] stm32f4xx_i2c.c
[CC] stm32f4xx_spdifrx.c
[CC] stm32f4xx_crc.c
[CC] stm32f4xx_exti.c
[CC] stm32f4xx_iwdg.c
[CC] stm32f4xx_spi.c
[CC] stm32f4xx_flash.c
[CC] stm32f4xx_lptim.c
[CC] stm32f4xx_syscfg.c
[CC] stm32f4xx_flash_ramfunc.c
[CC] stm32f4xx_ltdc.c
[CC] stm32f4xx_tim.c
[CC] stm32f4xx_pwr.c
[CC] stm32f4xx_usart.c
[CC] stm32f4xx_fmpi2c.c
[CC] stm32f4xx_qspi.c
[CC] stm32f4xx_wwdg.c
[CC] stm32f4xx_dac.c
[CC] stm32f4xx_fsmc.c
[CC] stm32f4xx_rcc.c
[CC] stm32f4xx_dbgmcu.c
[CC] stm32f4xx_gpio.c
[CC] stm32f4xx_rng.c
[AS] startup_stm32f4xx.s
[LD] manufunkture.elf
[HEX] manufunkture.hex
[BIN] manufunkture.bin
```

### Flash the STM32F4xx Discovery Board
```bash
$ make burn
st-flash 1.3.0
2019-12-29T21:25:50 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Loading device parameters....
2019-12-29T21:25:50 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Device connected is: F4 device, id 0x10076413
2019-12-29T21:25:50 INFO /Users/jerry/Downloads/stlink-master/src/common.c: SRAM size: 0x30000 bytes (192 KiB), Flash: 0x100000 bytes (1024 KiB) in pages of 16384 bytes
2019-12-29T21:25:50 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Attempting to write 46924 (0xb74c) bytes to stm32 address: 134217728 (0x8000000)
Flash page at addr: 0x08008000 erasedEraseFlash - Sector:0x2 Size:0x4000 
2019-12-29T21:25:51 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Finished erasing 3 pages of 16384 (0x4000) bytes
2019-12-29T21:25:51 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Starting Flash write for F2/F4/L4
2019-12-29T21:25:51 INFO /Users/jerry/Downloads/stlink-master/src/flash_loader.c: Successfully loaded flash loader in sram
enabling 32-bit flash writes
size: 32768
size: 14156
2019-12-29T21:25:51 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Starting verification of write complete
2019-12-29T21:25:52 INFO /Users/jerry/Downloads/stlink-master/src/common.c: Flash written and verified! jolly good!
```

### TTY Console Access
The test application (main.c) has code using USART3 to communicate back to the host.  You will need a USB-to-Serial adapter
to connect to the USART2 pins on the discovery board. 

* USB to TTL Serial Cable - https://www.adafruit.com/product/954

Within the application (main.c) USART3(PB10, PB11) is used to redirect printf data to the host PC.

You can use the screen command to connect to the Discovery board using the USB-to-Serial cable

```bash
$ screen /dev/tty.usbserial 115200
```

### Disassemble

Run make disass / make disass-all to disassamble.

```bash
$ make disass
...
Disassembly of section .text:

080001c0 <__do_global_dtors_aux>:
 80001c0:       b510            push    {r4, lr}
 80001c2:       4c05            ldr     r4, [pc, #20]   ; (80001d8 <__do_global_dtors_aux+0x18>)
 80001c4:       7823            ldrb    r3, [r4, #0]
 80001c6:       b933            cbnz    r3, 80001d6 <__do_global_dtors_aux+0x16>
 80001c8:       4b04            ldr     r3, [pc, #16]   ; (80001dc <__do_global_dtors_aux+0x1c>)
 80001ca:       b113            cbz     r3, 80001d2 <__do_global_dtors_aux+0x12>
 80001cc:       4804            ldr     r0, [pc, #16]   ; (80001e0 <__do_global_dtors_aux+0x20>)
 80001ce:       f3af 8000       nop.w
 80001d2:       2301            movs    r3, #1
 80001d4:       7023            strb    r3, [r4, #0]
 80001d6:       bd10            pop     {r4, pc}
 80001d8:       200009c8        .word   0x200009c8
 80001dc:       00000000        .word   0x00000000
 80001e0:       08009ee0        .word   0x08009ee0
...
```

### Debugging

In order to debug your code, connect your board to the PC, run st-util (comes with stlink utility) from one terminal, and from another terminal within the project directory run make debug. You can then use general gdb commands to browse through the code.

#### Launch st-util
```bash
$ ./tools/stlink/bin/st-util
st-util 1.3.0
2019-12-29T21:40:46 INFO common.c: Loading device parameters....
2019-12-29T21:40:46 INFO common.c: Device connected is: F4 device, id 0x10076413
2019-12-29T21:40:46 INFO common.c: SRAM size: 0x30000 bytes (192 KiB), Flash: 0x100000 bytes (1024 KiB) in pages of 16384 bytes
2019-12-29T21:40:46 INFO gdb-server.c: Chip ID is 00000413, Core ID is  2ba01477.
2019-12-29T21:40:46 INFO gdb-server.c: Listening at *:4242...
```

#### Run make debug 
```bash
$ make debug
GNU gdb (GNU Tools for Arm Embedded Processors 9-2019-q4-major) 8.3.0.20190709-git
Copyright (C) 2019 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.
Type "show copying" and "show warranty" for details.
This GDB was configured as "--host=x86_64-apple-darwin10 --target=arm-none-eabi".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
    <http://www.gnu.org/software/gdb/documentation/>.

For help, type "help".
Type "apropos word" to search for commands related to "word"...
Reading symbols from /Users/tmendoza/Development/gitrepos/work/stm32f4-template/binary/manufunkture.elf...
Remote debugging using :4242
0x08004fec in Reset_Handler ()
(gdb) 
```
## History
Much of the code in this repository is done in preparation for developing a MIDI/OSC/CV Sequencer.

### Sequencer Features

* MIDI Din and MIDI USB connectivity
* OSC connectivity over Ethernet
* CV connectivity for Eurorack integration
* 16 tracks / 64 steps per track
* SD Card Project storage

### References and Inspiration

* [fcayci](https://github.com/fcayci/stm32f4-c-template) - https://github.com/fcayci/stm32f4-c-template
* [performer](https://westlicht.github.io/performer/) - https://westlicht.github.io/performer/
* [mutable instruments](https://github.com/pichenettes/eurorack) - https://github.com/pichenettes/eurorack

## Bugs
I am sure there are some but... Where are they?

## Author
[Tony Mendoza](mailto:commercial@tonymendoza.us)
