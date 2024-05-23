#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"
#include "param.h"

PARAM_GROUP_START(SAFE_GIL_PARAMS)
PARAM_ADD(PARAM_UINT16, FLIGHT_ENABLE, &FLIGHT_ENABLE)
PARAM_GROUP_STOP(SAFE_GIL_PARAMS)