#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"
#include "param.h"
#include "app.h"
#include "FreeRTOS.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app.h"
#include "FreeRTOS.h"



#include "task.h"
#include "timers.h"
#include "math3d.h"

// Tracks whether to start the flight process.
uint8_t FLIGHT_ENABLE = 0;
static uint8_t sensor_status;
static VL53L5CX_Configuration tof_config;
/**
 * @brief Defines the minimum distance of each input column of the ToF sensor. 
 * This is an intermeddiate that is a copy of the sensor matrix.
 * 
 */
static uint16_t tof_input[OBST_DIM*OBST_DIM];

/**
 * @brief Defines the status of each zone where 5 and 9 means that the range status
 * is OK.
 */
static uint8_t tof_status[OBST_DIM*OBST_DIM];

/**
 * @brief The input vector seen for the obstacle encoder. 
 * 
 */
static float obstacle_inputs[OBST_DIM];

// Bool that tracks when the obstacle embedder needs to be updated.
static bool isToFStale = false;
// Timer that tracks the observation request process. 
static xTimerHandle ObservationTimer;

// The below function will call based on the xTimerCreate interval. 
static void pullObs(xTimerHandle timer) {
	isToFStale = tof_task(&tof_config, &sensor_status, tof_input, tof_status);
}

void appMain() {	
	#ifdef TOF_ENABLE
    	//Initialize sensor platform
		tof_init(&tof_config);
		vTaskDelay(M2T(10));
        //Start RTOS task for observations. The task will thus run at M2T(67) ~ 15Hz.
        ObservationTimer = xTimerCreate("ObservationTimer", M2T(67), pdTRUE, NULL, pullObs);
        xTimerStart(ObservationTimer, 20);
	#endif

	while(1) {
        if (FLIGHT_ENABLE) {
            if (!isToFStale) {
                isToFStale = process_obst(obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
            }
            
        }
	}
}


PARAM_GROUP_START(SAFE_GIL_PARAMS)
PARAM_ADD(PARAM_UINT8, FLIGHT_ENABLE, &FLIGHT_ENABLE)
PARAM_GROUP_STOP(SAFE_GIL_PARAMS)