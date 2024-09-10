#ifndef NETWORK_EVALUATE_TOF_H
#define NETWORK_EVALUATE_TOF_H

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "debug.h"
// When defined, enables the ToF module.
#define TOF_ENABLE 

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

// typedef struct nn_output {
// 	float out_0;
// 	float out_1;
// 	float out_2;
// }


/**
 * @brief Propogates the neural network to generate thrust values.
 * 
 * @param control_n 
 * @param state_array 
 * @param obstacle_embeds
 */
void networkEvaluateValue(control_t_n *control_n, const float *state_array);

#endif