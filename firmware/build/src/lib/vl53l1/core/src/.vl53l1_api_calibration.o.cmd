cmd_src/lib/vl53l1/core/src/vl53l1_api_calibration.o := arm-none-eabi-gcc -Wp,-MD,src/lib/vl53l1/core/src/.vl53l1_api_calibration.o.d    -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib -Isrc/lib -D__firmware__ -fno-exceptions -Wall -Wmissing-braces -fno-strict-aliasing -ffunction-sections -fdata-sections -Wdouble-promotion -std=gnu11 -DCRAZYFLIE_FW   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/Core/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/CMSIS/CMSIS/DSP/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/libdw1000/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/portable/GCC/ARM_CM4F   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/platform/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/deck/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/bosch/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/esp32/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/hal/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/kalman_core   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/outlierfilter   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/cpx   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/p2pDTR   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/controller   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/modules/interface/estimator   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/kve   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/lighthouse   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/utils/interface/tdoa   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/FatFS   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/CMSIS/STM32F4xx/Include   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_Device_Library/Core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32_USB_OTG_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/STM32F4xx_StdPeriph_Driver/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1   -I/Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/build/include/generated -fno-delete-null-pointer-checks --param=allow-store-data-races=0 -Wno-unused-but-set-variable -Wno-unused-const-variable -fomit-frame-pointer -fno-var-tracking-assignments -Wno-pointer-sign -fno-strict-overflow -fconserve-stack -Werror=implicit-int -Werror=date-time -DCC_HAVE_ASM_GOTO -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 -g3 -fno-math-errno -DARM_MATH_CM4 -D__FPU_PRESENT=1 -mfp16-format=ieee -Wno-array-bounds -Wno-stringop-overread -Wno-stringop-overflow -DSTM32F4XX -DSTM32F40_41xxx -DHSE_VALUE=8000000 -DUSE_STDPERIPH_DRIVER -Os -Werror   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src   -I/Users/darrenchiu/Documents/Projects/crazyflie/SAFE-GIL-firmware/firmware/src/tof_api -Wno-error   -c -o src/lib/vl53l1/core/src/vl53l1_api_calibration.o /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/src/vl53l1_api_calibration.c

source_src/lib/vl53l1/core/src/vl53l1_api_calibration.o := /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/src/vl53l1_api_calibration.c

deps_src/lib/vl53l1/core/src/vl53l1_api_calibration.o := \
    $(wildcard include/config//spad/enables/ref/0.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_ll_def.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_ll_device.h \
    $(wildcard include/config/vhv.h) \
    $(wildcard include/config/phasecal.h) \
    $(wildcard include/config/reference/phase.h) \
    $(wildcard include/config/dss1.h) \
    $(wildcard include/config/dss2.h) \
    $(wildcard include/config/mm1.h) \
    $(wildcard include/config/mm2.h) \
    $(wildcard include/config/range.h) \
    $(wildcard include/config/timeout/us.h) \
    $(wildcard include/config/target/total/rate/mcps.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/vl53l1_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_default_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/features.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_newlib_version.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_intsup.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_stdint.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stddef.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/string.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/newlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/config.h \
    $(wildcard include/config/h//.h) \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/ieeefp.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/reent.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/_ansi.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/lock.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/cdefs.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_locale.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/strings.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/string.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdio.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdarg.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/endian.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/_endian.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/select.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_sigset.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_timeval.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/timespec.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_timespec.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/_pthreadtypes.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/sched.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/types.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/sys/stdio.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/stdlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/machine/stdlib.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/arm-none-eabi/include/alloca.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/vl53l1_platform_user_config.h \
    $(wildcard include/config/h/.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_error_codes.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_register_structs.h \
    $(wildcard include/config/i2c/index.h) \
    $(wildcard include/config//target/total/rate/mcps.h) \
    $(wildcard include/config//stream/count/update/value.h) \
    $(wildcard include/config//timeout/macrop/a/hi.h) \
    $(wildcard include/config//roi/mode/control.h) \
    $(wildcard include/config/i2c/size/bytes.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_register_map.h \
    $(wildcard include/config//vhv/ref/sel/vddpix.h) \
    $(wildcard include/config//vhv/ref/sel/vquench.h) \
    $(wildcard include/config//reg/avdd1v2/sel.h) \
    $(wildcard include/config//fast/osc//trim.h) \
    $(wildcard include/config//timeout/macrop/loop/bound.h) \
    $(wildcard include/config//count/thresh.h) \
    $(wildcard include/config//offset.h) \
    $(wildcard include/config//init.h) \
    $(wildcard include/config//spad/enables/ref/1.h) \
    $(wildcard include/config//spad/enables/ref/2.h) \
    $(wildcard include/config//spad/enables/ref/3.h) \
    $(wildcard include/config//spad/enables/ref/4.h) \
    $(wildcard include/config//spad/enables/ref/5.h) \
    $(wildcard include/config//ref/en/start/select.h) \
    $(wildcard include/config//inner/offset/mm.h) \
    $(wildcard include/config//inner/offset/mm/hi.h) \
    $(wildcard include/config//inner/offset/mm/lo.h) \
    $(wildcard include/config//outer/offset/mm.h) \
    $(wildcard include/config//outer/offset/mm/hi.h) \
    $(wildcard include/config//outer/offset/mm/lo.h) \
    $(wildcard include/config//target/total/rate/mcps/hi.h) \
    $(wildcard include/config//target/total/rate/mcps/lo.h) \
    $(wildcard include/config//spad/sel/pswidth.h) \
    $(wildcard include/config//vcsel/pulse/width/offset.h) \
    $(wildcard include/config//fast/osc//config/ctrl.h) \
    $(wildcard include/config/ctrl.h) \
    $(wildcard include/config//static/config/spare/0.h) \
    $(wildcard include/config/spare/0.h) \
    $(wildcard include/config//static/config/spare/1.h) \
    $(wildcard include/config/spare/1.h) \
    $(wildcard include/config//static/config/spare/2.h) \
    $(wildcard include/config/spare/2.h) \
    $(wildcard include/config//reset/stages/msb.h) \
    $(wildcard include/config//reset/stages/lsb.h) \
    $(wildcard include/config//stream/divider.h) \
    $(wildcard include/config/gpio.h) \
    $(wildcard include/config//vcsel/start.h) \
    $(wildcard include/config//repeat/rate.h) \
    $(wildcard include/config//repeat/rate/hi.h) \
    $(wildcard include/config//repeat/rate/lo.h) \
    $(wildcard include/config//vcsel/width.h) \
    $(wildcard include/config//timeout/macrop.h) \
    $(wildcard include/config//target.h) \
    $(wildcard include/config//override.h) \
    $(wildcard include/config//manual/effective/spads/select.h) \
    $(wildcard include/config//manual/effective/spads/select/hi.h) \
    $(wildcard include/config//manual/effective/spads/select/lo.h) \
    $(wildcard include/config//manual/block/select.h) \
    $(wildcard include/config//aperture/attenuation.h) \
    $(wildcard include/config//max/spads/limit.h) \
    $(wildcard include/config//min/spads/limit.h) \
    $(wildcard include/config//timeout/macrop/a/lo.h) \
    $(wildcard include/config//timeout/macrop/b/hi.h) \
    $(wildcard include/config//timeout/macrop/b/lo.h) \
    $(wildcard include/config//vcsel/period/a.h) \
    $(wildcard include/config//vcsel/period/b.h) \
    $(wildcard include/config//sigma/thresh.h) \
    $(wildcard include/config//sigma/thresh/hi.h) \
    $(wildcard include/config//sigma/thresh/lo.h) \
    $(wildcard include/config//min/count/rate/rtn/limit/mcps.h) \
    $(wildcard include/config//min/count/rate/rtn/limit/mcps/hi.h) \
    $(wildcard include/config//min/count/rate/rtn/limit/mcps/lo.h) \
    $(wildcard include/config//valid/phase/low.h) \
    $(wildcard include/config//valid/phase/high.h) \
    $(wildcard include/config//woi/sd0.h) \
    $(wildcard include/config//woi/sd1.h) \
    $(wildcard include/config//initial/phase/sd0.h) \
    $(wildcard include/config//initial/phase/sd1.h) \
    $(wildcard include/config//first/order/select.h) \
    $(wildcard include/config//quantifier.h) \
    $(wildcard include/config//user/roi/centre/spad.h) \
    $(wildcard include/config//user/roi/requested/global/xy/size.h) \
    $(wildcard include/config//powerdown/go1.h) \
    $(wildcard include/config//ref/bg/ctrl.h) \
    $(wildcard include/config//regdvdd1v2/ctrl.h) \
    $(wildcard include/config//osc/slow/ctrl.h) \
    $(wildcard include/config//fast/osc//trim/max.h) \
    $(wildcard include/config//fast/osc//freq/set.h) \
    $(wildcard include/config//vcsel/trim.h) \
    $(wildcard include/config//vcsel/selion.h) \
    $(wildcard include/config//vcsel/selion/max.h) \
    $(wildcard include/config//spad/enables/rtn/0.h) \
    $(wildcard include/config//spad/enables/rtn/1.h) \
    $(wildcard include/config//spad/enables/rtn/2.h) \
    $(wildcard include/config//spad/enables/rtn/3.h) \
    $(wildcard include/config//spad/enables/rtn/4.h) \
    $(wildcard include/config//spad/enables/rtn/5.h) \
    $(wildcard include/config//spad/enables/rtn/6.h) \
    $(wildcard include/config//spad/enables/rtn/7.h) \
    $(wildcard include/config//spad/enables/rtn/8.h) \
    $(wildcard include/config//spad/enables/rtn/9.h) \
    $(wildcard include/config//spad/enables/rtn/10.h) \
    $(wildcard include/config//spad/enables/rtn/11.h) \
    $(wildcard include/config//spad/enables/rtn/12.h) \
    $(wildcard include/config//spad/enables/rtn/13.h) \
    $(wildcard include/config//spad/enables/rtn/14.h) \
    $(wildcard include/config//spad/enables/rtn/15.h) \
    $(wildcard include/config//spad/enables/rtn/16.h) \
    $(wildcard include/config//spad/enables/rtn/17.h) \
    $(wildcard include/config//spad/enables/rtn/18.h) \
    $(wildcard include/config//spad/enables/rtn/19.h) \
    $(wildcard include/config//spad/enables/rtn/20.h) \
    $(wildcard include/config//spad/enables/rtn/21.h) \
    $(wildcard include/config//spad/enables/rtn/22.h) \
    $(wildcard include/config//spad/enables/rtn/23.h) \
    $(wildcard include/config//spad/enables/rtn/24.h) \
    $(wildcard include/config//spad/enables/rtn/25.h) \
    $(wildcard include/config//spad/enables/rtn/26.h) \
    $(wildcard include/config//spad/enables/rtn/27.h) \
    $(wildcard include/config//spad/enables/rtn/28.h) \
    $(wildcard include/config//spad/enables/rtn/29.h) \
    $(wildcard include/config//spad/enables/rtn/30.h) \
    $(wildcard include/config//spad/enables/rtn/31.h) \
    $(wildcard include/config//mode/roi/centre/spad.h) \
    $(wildcard include/config//mode/roi/xy/size.h) \
    $(wildcard include/config//a0.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/vl53l1_platform_user_defines.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_error_exceptions.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_ll_device.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/vl53l1_platform.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/vl53l1x.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/vl53l1_platform_user_data.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_def.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_ll_def.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/i2cdev.h \
  /usr/local/Cellar/gcc-arm-none-eabi/20200630/lib/gcc/arm-none-eabi/9.3.1/include/stdbool.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/drivers/interface/i2c_drv.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/FreeRTOS.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/FreeRTOSConfig.h \
    $(wildcard include/config/h.h) \
    $(wildcard include/config/debug/queue/monitor.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/config/config.h \
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
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/semphr.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/task.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/list.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/vendor/FreeRTOS/include/queue.h \
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
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_api.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_api_strings.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_def.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_api_core.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/vl53l1_platform_log.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_register_map.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_register_funcs.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_register_settings.h \
    $(wildcard include/config//manual.h) \
    $(wildcard include/config//standard.h) \
    $(wildcard include/config//even/update/only.h) \
    $(wildcard include/config/level/low.h) \
    $(wildcard include/config/level/high.h) \
    $(wildcard include/config/out/of/window.h) \
    $(wildcard include/config/in/window.h) \
    $(wildcard include/config/new/sample/ready.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_core.h \
    $(wildcard include/config//timeout/macrop/.h) \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_core_support.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_wait.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/../../../../drivers/interface/vl53l1x.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_api_preset_modes.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_silicon_core.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_api_core.h \
  /Users/darrenchiu/Documents/Projects/crazyflie/crazyflie-firmware/src/lib/vl53l1/core/inc/vl53l1_api_calibration.h \

src/lib/vl53l1/core/src/vl53l1_api_calibration.o: $(deps_src/lib/vl53l1/core/src/vl53l1_api_calibration.o)

$(deps_src/lib/vl53l1/core/src/vl53l1_api_calibration.o):
