#
# Copyright (c) 2011 Atmel Corporation. All rights reserved.
#
# \asf_license_start
#
# \page License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. The name of Atmel may not be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# 4. This software may only be redistributed and used in connection with an
#    Atmel microcontroller product.
#
# THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
# EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
# OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
# STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# \asf_license_stop
#

# Path to top level ASF directory relative to this project directory.
PRJ_PATH = ../../../../../../..

# Target CPU architecture: cortex-m3, cortex-m4
ARCH = cortex-m4

# Target part: none, sam3n4 or sam4l4aa
PART = sam4sd32c

# Application target name. Given with suffix .a for library and .elf for a
# standalone application.
TARGET_FLASH = sd_mmc_example2_flash.elf
TARGET_SRAM = sd_mmc_example2_sram.elf

# List of C source files.
CSRCS = \
       common/components/memory/sd_mmc/example2/example.c \
       common/components/memory/sd_mmc/sd_mmc.c           \
       common/components/memory/sd_mmc/sd_mmc_mem.c       \
       common/services/clock/sam4s/sysclk.c               \
       common/services/delay/sam/cycle_counter.c          \
       common/services/serial/usart_serial.c              \
       common/services/storage/ctrl_access/ctrl_access.c  \
       common/utils/interrupt/interrupt_sam_nvic.c        \
       common/utils/stdio/read.c                          \
       common/utils/stdio/write.c                         \
       sam/boards/sam4s_ek2/init.c                        \
       sam/boards/sam4s_ek2/led.c                         \
       sam/drivers/hsmci/hsmci.c                          \
       sam/drivers/pdc/pdc.c                              \
       sam/drivers/pio/pio.c                              \
       sam/drivers/pio/pio_handler.c                      \
       sam/drivers/pmc/pmc.c                              \
       sam/drivers/pmc/sleep.c                            \
       sam/drivers/rtc/rtc.c                              \
       sam/drivers/uart/uart.c                            \
       sam/drivers/usart/usart.c                          \
       sam/utils/cmsis/sam4s/source/templates/gcc/startup_sam4s.c \
       sam/utils/cmsis/sam4s/source/templates/system_sam4s.c \
       sam/utils/syscalls/gcc/syscalls.c                  \
       thirdparty/fatfs/fatfs-port-r0.09/diskio.c         \
       thirdparty/fatfs/fatfs-port-r0.09/sam/fattime_rtc.c \
       thirdparty/fatfs/fatfs-r0.09/src/ff.c              \
       thirdparty/fatfs/fatfs-r0.09/src/option/ccsbcs.c

# List of assembler source files.
ASSRCS = 

# List of include paths.
INC_PATH = \
       common/boards                                      \
       common/components/memory/sd_mmc                    \
       common/components/memory/sd_mmc/example2           \
       common/components/memory/sd_mmc/example2/sam4sd32c_sam4s_ek2 \
       common/services/clock                              \
       common/services/delay                              \
       common/services/gpio                               \
       common/services/ioport                             \
       common/services/serial                             \
       common/services/serial/sam_uart                    \
       common/services/storage/ctrl_access                \
       common/utils                                       \
       common/utils/stdio/stdio_serial                    \
       sam/boards                                         \
       sam/boards/sam4s_ek2                               \
       sam/drivers/hsmci                                  \
       sam/drivers/pdc                                    \
       sam/drivers/pio                                    \
       sam/drivers/pmc                                    \
       sam/drivers/rtc                                    \
       sam/drivers/uart                                   \
       sam/drivers/usart                                  \
       sam/utils                                          \
       sam/utils/cmsis/sam4s/include                      \
       sam/utils/cmsis/sam4s/source/templates             \
       sam/utils/header_files                             \
       sam/utils/preprocessor                             \
       thirdparty/CMSIS/Include                           \
       thirdparty/CMSIS/Lib/GCC                           \
       thirdparty/fatfs/fatfs-port-r0.09/sam              \
       thirdparty/fatfs/fatfs-r0.09/src \
       common/components/memory/sd_mmc/example2/sam4sd32c_sam4s_ek2/gcc

# Additional search paths for libraries.
LIB_PATH =  \
       thirdparty/CMSIS/Lib/GCC                          

# List of libraries to use during linking.
LIBS =  \
       arm_cortexM4l_math                                 \
       m                                                 

# Path relative to top level directory pointing to a linker script.
LINKER_SCRIPT_FLASH = sam/utils/linker_scripts/sam4s/sam4sd32/gcc/flash.ld
LINKER_SCRIPT_SRAM  = sam/utils/linker_scripts/sam4s/sam4sd32/gcc/sram.ld

# Path relative to top level directory pointing to a linker script.
DEBUG_SCRIPT_FLASH = sam/boards/sam4s_ek2/debug_scripts/gcc/sam4s_ek2_flash.gdb
DEBUG_SCRIPT_SRAM  = sam/boards/sam4s_ek2/debug_scripts/gcc/sam4s_ek2_sram.gdb

# Project type parameter: all, sram or flash
PROJECT_TYPE        = flash

# Additional options for debugging. By default the common Makefile.in will
# add -g3.
DBGFLAGS = 

# Application optimization used during compilation and linking:
# -O0, -O1, -O2, -O3 or -Os
OPTIMIZATION = -O1

# Extra flags to use when archiving.
ARFLAGS = 

# Extra flags to use when assembling.
ASFLAGS = 

# Extra flags to use when compiling.
CFLAGS = 

# Extra flags to use when preprocessing.
#
# Preprocessor symbol definitions
#   To add a definition use the format "-D name[=definition]".
#   To cancel a definition use the format "-U name".
#
# The most relevant symbols to define for the preprocessor are:
#   BOARD      Target board in use, see boards/board.h for a list.
#   EXT_BOARD  Optional extension board in use, see boards/board.h for a list.
CPPFLAGS = \
       -D ARM_MATH_CM4=true                               \
       -D BOARD=SAM4S_EK2                                 \
       -D SD_MMC_ENABLE                                   \
       -D __SAM4SD32C__                                   \
       -D printf=iprintf                                  \
       -D scanf=iscanf

# Extra flags to use when linking
LDFLAGS = \

# Pre- and post-build commands
PREBUILD_CMD = 
POSTBUILD_CMD = 