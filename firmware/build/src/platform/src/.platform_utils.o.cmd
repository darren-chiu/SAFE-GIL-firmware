cmd_src/platform/src/platform_utils.o := arm-none-eabi-gcc -Wp,-MD,src/platform/src/.platform_utils.o.d    -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/src -Isrc/platform/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/cpx   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/controller   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/kve   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/FatFS   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src/tof_api -Wno-error   -c -o src/platform/src/platform_utils.o /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/src/platform_utils.c

source_src/platform/src/platform_utils.o := /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/src/platform_utils.c

deps_src/platform/src/platform_utils.o := \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/interface/platform.h \
    $(wildcard include/config/sensors/bmi088/bmp388.h) \
    $(wildcard include/config/sensors/bmi088/spi.h) \
    $(wildcard include/config/sensors/mpu9250/lps25h.h) \
    $(wildcard include/config/sensors/bosch.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/motors.h \
    $(wildcard include/config/motors/esc/protocol/oneshot125.h) \
    $(wildcard include/config/motors/esc/protocol/oneshot42.h) \
    $(wildcard include/config/motors/esc/protocol/dshot.h) \
    $(wildcard include/config/motors/dshot/pwm/150khz.h) \
    $(wildcard include/config/motors/dshot/pwm/300khz.h) \
    $(wildcard include/config/motors/dshot/pwm/600khz.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/trace.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/usec_time.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/stm32fxxx.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_version.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/mpu_armv7.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/system_stm32f4xx.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/stm32f4xx_conf.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_adc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_crc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dbgmcu.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dma.h \
    $(wildcard include/config/it.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_exti.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_flash.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_gpio.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_i2c.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_iwdg.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_pwr.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rcc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rtc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_sdio.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_spi.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_syscfg.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_tim.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_usart.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_wwdg.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_misc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_cryp.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_hash.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_rng.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_can.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dac.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_dcmi.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc/stm32f4xx_fsmc.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/radiolink.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/syslink.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/console.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \

src/platform/src/platform_utils.o: $(deps_src/platform/src/platform_utils.o)

$(deps_src/platform/src/platform_utils.o):
