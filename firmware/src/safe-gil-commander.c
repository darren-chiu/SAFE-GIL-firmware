#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"
#include "param.h"
// Tracks whether to start the flight process.
uint8_t FLIGHT_ENABLE = 0;
// Bool that tracks when the obstacle embedder needs to be updated.
static bool isToFStale = false;
// Timer that tracks the observation request process. 
static xTimerHandle ObservationTimer;

// The below function will call based on the xTimerCreate interval. 
static void pullObs(xTimerHandle timer) {
	isToFStale = tof_task(&tof_config, tof_addresses, &sensor_status, tof_input, tof_status);
}

void appMain() {	
	#ifdef TOF_ENABLE
    	//Initialize sensor platform
		tof_init(&tof_config, tof_addresses);
		vTaskDelay(M2T(10));
        //Start RTOS task for observations. The task will thus run at M2T(67) ~ 15Hz.
        ObsTimer = xTimerCreate("ObservationTimer", M2T(67), pdTRUE, NULL, pullObs);
        xTimerStart(ObsTimer, 20);
	#endif

	while(1) {
        if (FLIGHT_ENABLE) {
            if (!isToFStale) {
                isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
            }
        }
	}
}


PARAM_GROUP_START(SAFE_GIL_PARAMS)
PARAM_ADD(PARAM_UINT8, FLIGHT_ENABLE, &FLIGHT_ENABLE)
PARAM_GROUP_STOP(SAFE_GIL_PARAMS)