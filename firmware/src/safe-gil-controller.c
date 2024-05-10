#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "safe-gil"

#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"

static int count = 0;

// Observations
static uint8_t sensor_status;
static VL53L5CX_Configuration tof_config;

// Bool that tracks when the obstacle embedder needs to be updated.
static bool isToFStale = false;
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
 * @brief The vector of horizontal values.
 * 
 */
static float obstacle_inputs[OBST_DIM];

void controllerOutOfTreeInit() {

  controllerPidInit();
}

bool controllerOutOfTreeTest() {
  // Always return true
  return true;
}

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  count++;
  if (!isToFStale) {
		#ifdef TOF_ENABLE
			isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
		#endif
	}
  if (count == 250) {
    DEBUG_PRINT("ToF: (%f,%f,%f,%f,%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3], obstacle_inputs[4], obstacle_inputs[5], obstacle_inputs[6], obstacle_inputs[7]);
    count = 0;
  }
  // Call the PID controller 
  controllerPid(control, setpoint, sensors, state, tick);
}


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  
  tof_init(&tof_config);
  vTaskDelay(M2T(10));

  while(1) {
    #ifdef ENABLE_4X4_CONTROLLER
			vTaskDelay(M2T(34)); // 30Hz is roughly 30 ms intervals
		#else
			vTaskDelay(M2T(67)); // 15Hz is roughly 67 ms intervals
		#endif
		#ifdef TOF_ENABLE
			isToFStale = tof_task(&tof_config, &sensor_status, tof_input, tof_status);
		#endif
  }
}

LOG_GROUP_START(gil)
LOG_ADD(LOG_FLOAT, obs1, &obstacle_inputs[0])
LOG_ADD(LOG_FLOAT, obs2, &obstacle_inputs[1])
LOG_ADD(LOG_FLOAT, obs3, &obstacle_inputs[2])
LOG_ADD(LOG_FLOAT, obs4, &obstacle_inputs[3])
LOG_ADD(LOG_FLOAT, obs5, &obstacle_inputs[4])
LOG_ADD(LOG_FLOAT, obs6, &obstacle_inputs[5])
LOG_ADD(LOG_FLOAT, obs7, &obstacle_inputs[6])
LOG_ADD(LOG_FLOAT, obs8, &obstacle_inputs[7])
LOG_GROUP_STOP(gil)