cmd_src/modules/src/estimator/estimator_kalman.o := arm-none-eabi-gcc -Wp,-MD,src/modules/src/estimator/.estimator_kalman.o.d    -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/src/estimator -Isrc/modules/src/estimator -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/cpx   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/controller   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/kve   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/FatFS   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src/tof_api -Wno-error   -c -o src/modules/src/estimator/estimator_kalman.o /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/src/estimator/estimator_kalman.c

source_src/modules/src/estimator/estimator_kalman.o := /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/src/estimator/estimator_kalman.c

deps_src/modules/src/estimator/estimator_kalman.o := \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/kalman_core.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/cf_math.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include/arm_math.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_compiler.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include/cmsis_gcc.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/string.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_locale.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/strings.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/string.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/math.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/float.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include-fixed/limits.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include-fixed/syslimits.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/limits.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/syslimits.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/cfassert.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/imu_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/lighthouse/lighthouse_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_supervisor.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/config.h \
    $(wildcard include/config/h/.h) \
    $(wildcard include/config/block/address.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/nrf24l01.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/nRF24L01reg.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/trace.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/usec_time.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/cfassert.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/projdefs.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/portable.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/deprecated_definitions.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F/portmacro.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/mpu_wrappers.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface/sensors.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/static_mem.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator/estimator.h \
    $(wildcard include/config/estimator/kalman/enable.h) \
    $(wildcard include/config/estimator/ukf/enable.h) \
    $(wildcard include/config/estimator/oot.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator/estimator_kalman.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/system.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/log.h \
    $(wildcard include/config/debug/log/enable.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/param.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/param_logic.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/crtp.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/physicalConstants.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/supervisor.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/stabilizer_types.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/axis3fSubSampler.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/statsCnt.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/rateSupervisor.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_distance.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_absolute_height.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_position.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_pose.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_tdoa.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/outlierfilter/outlierFilterTdoa.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_flow.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_tof.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_yaw_error.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_sweep_angles.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/outlierfilter/outlierFilterLighthouse.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_tdoa_robust.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core/mm_distance_robust.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/debug.h \
    $(wildcard include/config/debug/print/on/uart1.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/config.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/console.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/eprintf.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \

src/modules/src/estimator/estimator_kalman.o: $(deps_src/modules/src/estimator/estimator_kalman.o)

$(deps_src/modules/src/estimator/estimator_kalman.o):
