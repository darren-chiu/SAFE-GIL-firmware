cmd_src/modules/src/estimator/position_estimator_altitude.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/estimator/.position_estimator_altitude.o.d    -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/src/estimator -Isrc/modules/src/estimator -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/config   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/platform/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/deck/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/deck/drivers/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/hal/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/cpx   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/controller   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/estimator   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/kve   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/FatFS   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/vl53l1   -I/Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/umut/projects/lab/crazyfile/SAFE-GIL-firmware/firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/umut/projects/lab/crazyfile/SAFE-GIL-firmware/firmware/src   -I/Users/umut/projects/lab/crazyfile/SAFE-GIL-firmware/firmware/src/tof_api -Wno-error   -c -o src/modules/src/estimator/position_estimator_altitude.o /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/src/estimator/position_estimator_altitude.c

source_src/modules/src/estimator/position_estimator_altitude.o := /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/src/estimator/position_estimator_altitude.c

deps_src/modules/src/estimator/position_estimator_altitude.o := \
    $(wildcard include/config/controller/pid/improved/baro/z/hold.h) \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include/stm32f4xx.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/core_cm4.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
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
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /opt/homebrew/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
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
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/param.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/param_logic.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/crtp.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/num.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/estimator/position_estimator.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/hal/interface/imu_types.h \
  /Users/umut/projects/lab/crazyfile/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \

src/modules/src/estimator/position_estimator_altitude.o: $(deps_src/modules/src/estimator/position_estimator_altitude.o)

$(deps_src/modules/src/estimator/position_estimator_altitude.o):
