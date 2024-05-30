#include "obst_daq.h"
#include "i2cdev.h"

static float SAFE_HEIGHT = 0.25f;

uint8_t tof_init(VL53L5CX_Configuration *tof_config) {
    // bool I2C_expander_status;
    uint8_t sensor_status;
    bool status = false;

    tof_config->platform = VL53L5CX_DEFAULT_I2C_ADDRESS;
    sensor_status = vl53l5cx_init(tof_config);
    if (sensor_status != 0) {
        DEBUG_PRINT("Failed to Initializae VL53L5CX\n");
    }

    #ifdef ENABLE_4X4_CONTROLLER
        // Sets the sensor to be 4x4.
        sensor_status = vl53l5cx_set_resolution(tof_config, VL53L5CX_RESOLUTION_4X4);
        if (sensor_status != 0) {
            DEBUG_PRINT("VL53L5CX [%i] Set Resolution: Fail\n", i); 
        }
        sensor_status = vl53l5cx_set_ranging_frequency_hz(tof_config, 30);

        if (sensor_status != 0) {
            DEBUG_PRINT("VL53L5CX [%i] Frequency Config: Fail\n", i); 
        } 
    #else
        // Sets the sensor to be 8x8.
        sensor_status = vl53l5cx_set_resolution(tof_config, VL53L5CX_RESOLUTION_8X8);
        if (sensor_status == 0) {
            DEBUG_PRINT("VL53L5CX Initialize: Pass\n"); 
        }
        sensor_status = vl53l5cx_set_ranging_frequency_hz(tof_config, 15);

        if (sensor_status == 0) {
            DEBUG_PRINT("VL53L5CX Frequency Config: Pass\n"); 
        } 
    #endif

    //Below function should be the last called in the init. 
    sensor_status = vl53l5cx_start_ranging(tof_config);
    if (sensor_status != 0) {
        DEBUG_PRINT("VL53L5CX Initialize: Fail\n"); 
    } 

    return sensor_status;
}

bool process_obst(float *obstacle_inputs, uint16_t *tof_input, uint8_t *tof_status) {
   	/**
   	 * NOTE: Use only the values of a specific column
     * The ToF lens flips the image plane vertically and horizontally
   	 */
    #ifdef ENABLE_4X4_CONTROLLER
		// ToF Measurements are noisy on takeoff.
		if (state->position.z > SAFE_HEIGHT) {
			int row_index = 4; //This denotes which row to start from. See note above.
            for (int j=0;j<NUM_SENSORS;j++) {
			    for (int i=0;i<4;i++) {				
                    int spad_index = row_index + i;
                    int sensor_index = (j*OBST_DIM);
                    // Check if the pixels are valid 
                    if ((tof_status[sensor_index + spad_index] == 9) || (tof_status[sensor_index + spad_index] == 5)) {
                        float obst_cap;

                        obst_cap = tof_input[sensor_index + spad_index] * 1.0f;
                        obst_cap = obst_cap / 1000.0f;

                        if ((obst_cap > OBST_MAX)) {
                            obst_cap = OBST_MAX;
                        }
                        obstacle_inputs[network_index] = obst_cap;
                        // obstacle_inputs[i] = OBST_MAX; //Ablate inputs to NN
                    } else {
                        // DEBUG_PRINT("Invalid Reading!");
                        obstacle_inputs[network_index] = OBST_MAX;
                    }
                    network_index++;
                }
            }
		} else {
			for (int i=0;i<OBST_DIM;i++) {
				obstacle_inputs[i] = OBST_MAX;
			}
		}
	#else
		int row_index = 32;
        for (int i=0;i<8;i++) {				
            int spad_index = row_index + i;
            // Check if the pixels are valid 
            if ((tof_status[spad_index] == 9) || (tof_status[spad_index] == 5)) {
                float obst_cap;

                obst_cap = tof_input[spad_index] * 1.0f;
                obst_cap = obst_cap / 1000.0f;

                if ((obst_cap > OBST_MAX)) {
                    obst_cap = OBST_MAX;
                }
                obstacle_inputs[i] = obst_cap;
                // obstacle_inputs[i] = OBST_MAX; //Ablate inputs to NN
            } else {
                // DEBUG_PRINT("Invalid Reading!");
                obstacle_inputs[i] = OBST_MAX;
            }
        }

	#endif

    return true;
}

bool tof_task(VL53L5CX_Configuration *tof_config, uint8_t* sensor_status, 
				uint16_t *tof_input, uint8_t *tof_status) {
    VL53L5CX_ResultsData ranging_data;
    tof_config->platform = VL53L5CX_DEFAULT_I2C_ADDRESS;
    vl53l5cx_check_data_ready(tof_config, sensor_status);

    if (*sensor_status){
        vl53l5cx_get_ranging_data(tof_config, &ranging_data);
        memcpy(tof_input, &ranging_data.distance_mm[0], OBST_DIM*OBST_DIM*sizeof(uint16_t));
        // memcpy(tof_target, (uint8_t *)(&ranging_data->nb_target_detected[0]), OBST_DIM*OBST_DIM);
        memcpy(tof_status, &ranging_data.target_status[0], OBST_DIM*OBST_DIM*sizeof(uint8_t));
    }
        
    // DEBUG_PRINT("ToF: (%u,%u,%u,%u)\n", tof_input[0], tof_input[1], tof_input[2], tof_input[3]);
    // Sets the "isToFStale" variable to false in order to indicate a fresh measurement is here. Yummy.
    return false;
}