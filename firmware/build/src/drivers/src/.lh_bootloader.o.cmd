cmd_src/drivers/src/lh_bootloader.o := arm-none-eabi-gcc -Wp,-MD,src/drivers/src/.lh_bootloader.o.d    -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/src -Isrc/drivers/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/platform/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/deck/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/deck/drivers/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/hal/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/cpx   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/controller   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/estimator   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/kve   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/FatFS   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/vl53l1   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/umut/projects/lab/crazyfile/SAFE-GIL-firmware/firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/umut/projects/lab/crazyfile/SAFE-GIL-firmware/firmware/src   -I/Users/umut/projects/lab/crazyfile/SAFE-GIL-firmware/firmware/src/tof_api -Wno-error   -c -o src/drivers/src/lh_bootloader.o /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/src/lh_bootloader.c

source_src/drivers/src/lh_bootloader.o := /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/src/lh_bootloader.c

deps_src/drivers/src/lh_bootloader.o := \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/string.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_locale.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/strings.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/string.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config/trace.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/hal/interface/usec_time.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/cfassert.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface/lh_bootloader.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config/config.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/console.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/eprintf.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config/stm32fxxx.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \

src/drivers/src/lh_bootloader.o: $(deps_src/drivers/src/lh_bootloader.o)

$(deps_src/drivers/src/lh_bootloader.o):
