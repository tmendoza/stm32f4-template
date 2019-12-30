# stm32f4-template

A Standard Template for starting STM32F4xx C projects.

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

## Libraries

* FreeRTOS - https://github.com/FreeRTOS/FreeRTOS.git
* libopencm3 - https://github.com/libopencm3/libopencm3.git
* ST CMSIS - https://github.com/ARM-software/CMSIS_5.git

## Development
The repository has everything needed within it to successfully build the test file.  
