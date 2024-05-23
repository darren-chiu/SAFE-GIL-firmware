#ifndef NETWORK_EVALUATE_TOF_H
#define NETWORK_EVALUATE_TOF_H

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "debug.h"
// When defined, enables the ToF module.
#define TOF_ENABLE 
// Enable print statements
#define DEBUG_LOCALIZATION

// Enable print statements for communication
// #define DEBUG_COMMUNICATION

//Enable Multi Drone
// #define MULTI_DRONE_ENABLE
// When defined, uses 4x4 as ToF input with corresponding controller
// #define ENABLE_4X4_CONTROLLER
// Defines the dynamics dimmensions
#define STATE_DIM 18
// The number of ToF sensors to be used
#define NUM_SENSORS 4
// Number of TOTAL drones (this includes yourself)
#define NUM_DRONES 3
//Size of the neighbor encoder for the network
#define NEIGHBORS 2
#define NBR_OBS_DIM 3

// Obstacle Avoidance Parameters
#define OBST_MAX 2.0f
#define SAFE_HEIGHT 0.5f

#ifdef ENABLE_4X4_CONTROLLER
	#define OBST_DIM 16	
#else
	#define OBST_DIM 32
#endif

#define REL_VEL false
#define REL_OMEGA false
#define REL_XYZ false
#define REL_ROT true


/**
 * @brief Defines the output thrusts from the neural network.
 * 
 */
typedef struct control_t_n {
	float thrust_0; 
	float thrust_1;
	float thrust_2;
	float thrust_3;	
} control_t_n;

typedef struct nn_output {
	float out_0;
	float out_1;
	float out_2;
}

/**
 * @brief Propogates the neural network to generate thrust values.
 * 
 * @param control_n 
 * @param state_array 
 * @param obstacle_embeds 
 */
void networkEvaluate(control_t_n *control_n, const float *state_array);

/**
 * @brief Encodes the input array of ToF readings.
 * 
 */
void obstacleEmbedder(float obstacle_inputs[OBST_DIM]);

/**
 * @brief Encodes the neighbor array.
 * 
 */
void neighborEmbedder(float neighbor_inputs[NEIGHBORS*NBR_OBS_DIM]);

#endif