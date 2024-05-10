cmd_src/deck/drivers/src/lpsTdoa2Tag.o := arm-none-eabi-gcc -Wp,-MD,src/deck/drivers/src/.lpsTdoa2Tag.o.d    -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/src -Isrc/deck/drivers/src -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/cpx   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/controller   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/kve   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/FatFS   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src/tof_api -Wno-error   -c -o src/deck/drivers/src/lpsTdoa2Tag.o /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/src/lpsTdoa2Tag.c

source_src/deck/drivers/src/lpsTdoa2Tag.o := /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/src/lpsTdoa2Tag.c

deps_src/deck/drivers/src/lpsTdoa2Tag.o := \
    $(wildcard include/config/deck/loco/2d/position.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/string.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_locale.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/strings.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/string.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/stdlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/alloca.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/trace.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/usec_time.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/cfassert.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/param.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/param_logic.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/crtp.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface/lpsTdoa2Tag.h \
    $(wildcard include/config/deck/loco/nr/of/anchors.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface/locodeck.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc/libdw1000.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc/libdw1000Spi.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc/dw1000.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc/libdw1000Types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/imu_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface/mac.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/physicalConstants.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/tdoaEngineInstance.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa/tdoaEngine.h \
    $(wildcard include/config/deck/loco/longer/range.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa/tdoaStorage.h \
    $(wildcard include/config/deck/loco/tdoa3/hybrid/mode.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/clockCorrectionEngine.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa/tdoaStats.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/inttypes.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/statsCnt.h \

src/deck/drivers/src/lpsTdoa2Tag.o: $(deps_src/deck/drivers/src/lpsTdoa2Tag.o)

$(deps_src/deck/drivers/src/lpsTdoa2Tag.o):
