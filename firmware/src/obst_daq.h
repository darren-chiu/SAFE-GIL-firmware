#ifndef OBST_DAQ_H
#define OBST_DAQ_H

// Custom Includes
#include "vl53l5cx_api.h"
#include "stabilizer_types.h"
#include "debug.h"

#define TOF_ENABLE
// #define ENABLE_4X4_CONTROLLER
#define OBST_MAX 2.0


#ifdef ENABLE_4X4_CONTROLLER
	#define OBST_DIM 4
#else
	#define OBST_DIM 8
#endif

/**
 * @brief Initializes all the tof sensors.
 * 
 * @return uint8_t 
 */
uint8_t tof_init(VL53L5CX_Configuration *tof_config);
/**
 * @brief 
 * 
 * @return bool: Indicates that the measurement is now stale. 
 */

bool process_obst(const state_t *state, float *obstacle_inputs, uint16_t *tof_input, uint8_t *tof_status);
/**
 * @brief Collects ToF matrix data.     
 * ORIGINAL: [Front, Back, Left, Right]
 * MODIFIED: [Front, Right, Back, Left]
 * [0-15,16-31,32-47, 48-63]
 * 
 * @return bool: update on measurement to track stale data.
 */
bool tof_task(VL53L5CX_Configuration *tof_config, uint8_t* sensor_status, uint16_t *tof_input, uint8_t *tof_status);
#endif