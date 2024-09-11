#ifndef NETWORK_EVALUATE_TOF_VALUE_H
#define NETWORK_EVALUATE_TOF_VALUE_H

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "debug.h"
#include "network_evaluate_gil_policy.h"
// When defined, enables the ToF module.

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