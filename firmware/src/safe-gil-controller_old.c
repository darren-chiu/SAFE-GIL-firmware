#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "math3d.h"

#include "network_evaluate_gil.h"

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "safe-gil"

// Defining the below runs the imitation policy. 
//Otherwise it is for data colleciton
// #define RUN_EXPERIMENT
#define TOF_ENABLE

#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"

static int count = 0;

// static float state_array[23];
// static float thrust_mean[4] = {48054.22234259, 46984.33434234, 50876.45975656, 42471.35183533};
// // do the same thing for thrust std
// static float thrust_std[4] = {3781.97827489, 4069.80051149, 3844.66114359, 4089.51596848};

static control_t_n control_nn;

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


float clip(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}
bool nn_controller_enable = false;

void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick) {
  count++;

  // for (int i = 0; i < 23; i++) {
  //   state_array[i] = 0.0;
  // }
  // networkEvaluate(&control_nn, state_array);

  // for (int i = 0; i < 3000; i++) {
  //   DEBUG_PRINT("Actions: (%f,%f,%f,%f)\n", control_nn.thrust_0, control_nn.thrust_1, control_nn.thrust_2, control_nn.thrust_3);
    
  // }

  if (count == 250) {
    DEBUG_PRINT("ToF: (%f,%f,%f,%f,%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3], obstacle_inputs[4], obstacle_inputs[5], obstacle_inputs[6], obstacle_inputs[7]);
    
    count = 0;
    
  }

  

  #ifdef RUN_EXPERIMENT

    if (!nn_controller_enable){
      nn_controller_enable = state->position.z - 0.41 > 0.0;
    }

    if (nn_controller_enable) {
      control->controlMode = controlModeForce;
      // bool comm_status;

      // struct mat33 rot;
      // struct mat33 rot_desired;

      // Orientation
      // struct quat q = mkquat(state->attitudeQuaternion.x, 
      //             state->attitudeQuaternion.y, 
      //             state->attitudeQuaternion.z, 
      //             state->attitudeQuaternion.w);
      // rot = quat2rotmat(q);

      // struct quat q_desired = mkquat(setpoint->attitudeQuaternion.x, 
      //             setpoint->attitudeQuaternion.y, 
      //             setpoint->attitudeQuaternion.z, 
      //             setpoint->attitudeQuaternion.w);
      // rot_desired = quat2rotmat(q_desired);


      // angular velocity
      float omega_roll = sensors->gyro.x;
      float omega_pitch = sensors->gyro.y;
      float omega_yaw = sensors->gyro.z;

      // setpoint_array[0] = setpoint->position.x;
      // setpoint_array[1] = setpoint->position.y;
      // setpoint_array[2] = setpoint->position.z;

      // state_array[0] = state->position.x - setpoint->position.x;
      // state_array[1] = state->position.y - setpoint->position.y;
      // state_array[2] = state->position.z - setpoint->position.z;

      // TODO initialize state_array

      // initialize state_array
      
      state_array[0] = state->position.x ;
      state_array[1] = state->position.y;
      state_array[2] = state->position.z - 0.4;
      state_array[3] = sin(radians(state->attitude.roll));
      state_array[4] = cos(radians(state->attitude.roll));
      state_array[5] = sin(radians(state->attitude.pitch));
      state_array[6] = cos(radians(state->attitude.pitch));
      state_array[7] = sin(radians(state->attitude.yaw));
      state_array[8] = cos(radians(state->attitude.yaw));
      state_array[9] = state->velocity.x - 0.8364401720725966;
      state_array[10] = state->velocity.y;
      state_array[11] = state->velocity.z - 0.03843474001030632;
      state_array[12] = (omega_roll + 0.28475793) / 25.70296269;
      state_array[13] = (omega_pitch + 0.17994721) / 22.50442717;
      state_array[14] = (omega_yaw + 0.7365864) / 1.83997706;

      #ifdef DEBUG_LOCALIZATION
        if (count == 750) {
          DEBUG_PRINT("Estimation: (%f,%f,%f)\n", state->position.x,state->position.y,state->position.z);
          // DEBUG_PRINT("Desired: (%f,%f,%f)\n", setpoint->position.x,setpoint->position.y,setpoint->position.z);
          // DEBUG_PRINT("ERROR: (%f,%f,%f)\n", state_array[0], state_array[1], state_array[2]);
          DEBUG_PRINT("ToF: (%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3], obstacle_inputs[4], obstacle_inputs[5], obstacle_inputs[6], obstacle_inputs[7]);
          // DEBUG_PRINT("Thrusts: (%i,%i,%i,%i)\n", control->normalizedForces[0], control->normalizedForces[1], control->normalizedForces[2], control->normalizedForces[3]);
          count = 0;
        }
        count++;
      #endif

      // if (REL_VEL) {
      //   state_array[3] = state->velocity.x - setpoint->velocity.x;
      //   state_array[4] = state->velocity.y - setpoint->velocity.y;
      //   state_array[5] = state->velocity.z - setpoint->velocity.z;
      // } else {
      //   state_array[3] = state->velocity.x;
      //   state_array[4] = state->velocity.y;
      //   state_array[5] = state->velocity.z;
      // }
      // // TODO: ADD RELATIVE ROTATION MATRIX
      // if (REL_ROT) {
      //   state_array[6] = rot.m[0][0] - rot_desired.m[0][0];
      //   state_array[7] = rot.m[0][1] - rot_desired.m[0][1];
      //   state_array[8] = rot.m[0][2] - rot_desired.m[0][2];
      //   state_array[9] = rot.m[1][0] - rot_desired.m[1][0];
      //   state_array[10] = rot.m[1][1] - rot_desired.m[1][1];
      //   state_array[11] = rot.m[1][2] - rot_desired.m[1][2];
      //   state_array[12] = rot.m[2][0] - rot_desired.m[2][0];
      //   state_array[13] = rot.m[2][1] - rot_desired.m[2][1];
      //   state_array[14] = rot.m[2][2] - rot_desired.m[2][2];
      // } else {
      //   state_array[6] = rot.m[0][0];
      //   state_array[7] = rot.m[0][1];
      //   state_array[8] = rot.m[0][2];
      //   state_array[9] = rot.m[1][0];
      //   state_array[10] = rot.m[1][1];
      //   state_array[11] = rot.m[1][2];
      //   state_array[12] = rot.m[2][0];
      //   state_array[13] = rot.m[2][1];
      //   state_array[14] = rot.m[2][2];
      // }

      // if (REL_XYZ) {
      //   // rotate pos and vel
      //   struct vec rot_pos = mvmul(mtranspose(rot), mkvec(state_array[0], state_array[1], state_array[2]));
      //   struct vec rot_vel = mvmul(mtranspose(rot), mkvec(state_array[3], state_array[4], state_array[5]));

      //   state_array[0] = rot_pos.x;
      //   state_array[1] = rot_pos.y;
      //   state_array[2] = rot_pos.z;

      //   state_array[3] = rot_vel.x;
      //   state_array[4] = rot_vel.y;
      //   state_array[5] = rot_vel.z;
      // }

      // setpoint_array[3] = radians(setpoint->attitudeRate.roll);
      // setpoint_array[4] = radians(setpoint->attitudeRate.pitch);
      // setpoint_array[5] = radians(setpoint->attitudeRate.yaw);

      // if (REL_OMEGA) {
      //   state_array[15] = omega_roll - setpoint_array[4];
      //   state_array[16] = omega_pitch - setpoint_array[5];
      //   state_array[17] = omega_yaw - setpoint_array[6];
      // } else {
      //   state_array[15] = omega_roll;
      //   state_array[16] = omega_pitch;
      //   state_array[17] = omega_yaw;
      // }

      if (!isToFStale) {
        #ifdef TOF_ENABLE
          isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
          // obstacleEmbedder(obstacle_inputs);
        #endif
      }

      // #ifdef MULTI_DRONE_ENABLE
      //   updateNeighborInputs(state, neighbor_inputs);
      //   neighborEmbedder(neighbor_inputs);  
      // #endif

      state_array[15] = obstacle_inputs[0];
      state_array[16] = obstacle_inputs[1];
      state_array[17] = obstacle_inputs[2];
      state_array[18] = obstacle_inputs[3];
      state_array[19] = obstacle_inputs[4];
      state_array[20] = obstacle_inputs[5];
      state_array[21] = obstacle_inputs[6];
      state_array[22] = obstacle_inputs[7];

      networkEvaluate(&control_nn, state_array);
      // action_unnormalized = action * self.expert_actions_std + self.expert_actions_mean
      control_nn.thrust_0 = control_nn.thrust_0 * thrust_std[0] + thrust_mean[0];
      control_nn.thrust_1 = control_nn.thrust_1 * thrust_std[1] + thrust_mean[1];
      control_nn.thrust_2 = control_nn.thrust_2 * thrust_std[2] + thrust_mean[2];
      control_nn.thrust_3 = control_nn.thrust_3 * thrust_std[3] + thrust_mean[3];


      // convert thrusts to normalized Thrust
      uint16_t iThrust_0, iThrust_1, iThrust_2, iThrust_3; 
      iThrust_0 = (uint16_t) clip(control_nn.thrust_0, 0, 65535);
      iThrust_1 = (uint16_t) clip(control_nn.thrust_1, 0, 65535);
      iThrust_2 = (uint16_t) clip(control_nn.thrust_2, 0, 65535);
      iThrust_3 = (uint16_t) clip(control_nn.thrust_3, 0, 65535);

      // normalizeThrust(&control_nn, &iThrust_0, &iThrust_1, &iThrust_2, &iThrust_3);

      control->normalizedForces[0] = iThrust_0;
      control->normalizedForces[1] = iThrust_1;
      control->normalizedForces[2] = iThrust_2;
      control->normalizedForces[3] = iThrust_3;

    //   if (setpoint->mode.z == modeDisable) {
    //     control->normalizedForces[0] = 0;
    //     control->normalizedForces[1] = 0;
    //     control->normalizedForces[2] = 0;
    //     control->normalizedForces[3] = 0;
    //   } else {
    //     control->normalizedForces[0] = (uint16_t) thrust_coefficient * iThrust_0;
    //     control->normalizedForces[1] = (uint16_t) thrust_coefficient * iThrust_1;
    //     control->normalizedForces[2] = (uint16_t) thrust_coefficient * iThrust_2;
    //     control->normalizedForces[3] = (uint16_t) thrust_coefficient * iThrust_3;
    //   }
    // }


    // if ticks for position control
    // get nn
    // set setpoint
    // run pid
    }
    else {
      controllerPid(control, setpoint, sensors, state, tick);
    }
  #else
    // Call the PID controller 
    controllerPid(control, setpoint, sensors, state, tick);
    if (!isToFStale) {
      #ifdef TOF_ENABLE
        isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
        // obstacleEmbedder(obstacle_inputs);
      #endif
    }
  #endif
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










































#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "math3d.h"

#include "network_evaluate_gil.h"

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#define DEBUG_MODULE "safe-gil"

// Defining the below runs the imitation policy. 
//Otherwise it is for data colleciton
#define GUIDED_DATA_COLLECTION
#define TOF_ENABLE

#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"



#include "controller_pid.h"




#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;



static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}





static int count = 0;

// static float state_array[23];
// static float thrust_mean[4] = {48054.22234259, 46984.33434234, 50876.45975656, 42471.35183533};
// // do the same thing for thrust std
// static float thrust_std[4] = {3781.97827489, 4069.80051149, 3844.66114359, 4089.51596848};

static control_t_n control_nn;

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
  return controllerPidTest();
}




float clip(float v, float min, float max) {
    if (v < min) return min;
    if (v > max) return max;
    return v;
}



#ifdef GUIDED_DATA_COLLECTION
  // THIS IS FOR GUIDED DATA COLLECTION
  
  static attitude_t attitudeDesiredExpert;
#endif





void controllerOutOfTree(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, stabilizerStep_t stabilizerStep ) {
  

  #ifdef GUIDED_DATA_COLLECTION
    // THIS IS FOR GUIDED DATA COLLECTION
    // THIS IS A MODIFIED VERSION OF THE PID CONTROLLER
    control->controlMode = controlModeLegacy;

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
      // Rate-controled YAW is moving YAW angle setpoint
      if (setpoint->mode.yaw == modeVelocity) {
        attitudeDesired.yaw = capAngle(attitudeDesired.yaw + setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT);

        float yawMaxDelta = attitudeControllerGetYawMaxDelta();
        if (yawMaxDelta != 0.0f)
        {
        float delta = capAngle(attitudeDesired.yaw-state->attitude.yaw);
        // keep the yaw setpoint within +/- yawMaxDelta from the current yaw
          if (delta > yawMaxDelta)
          {
            attitudeDesired.yaw = state->attitude.yaw + yawMaxDelta;
          }
          else if (delta < -yawMaxDelta)
          {
            attitudeDesired.yaw = state->attitude.yaw - yawMaxDelta;
          }
        }
      } else if (setpoint->mode.yaw == modeAbs) {
        attitudeDesired.yaw = setpoint->attitude.yaw;
      } else if (setpoint->mode.quat == modeAbs) {
        struct quat setpoint_quat = mkquat(setpoint->attitudeQuaternion.x, setpoint->attitudeQuaternion.y, setpoint->attitudeQuaternion.z, setpoint->attitudeQuaternion.w);
        struct vec rpy = quat2rpy(setpoint_quat);
        attitudeDesired.yaw = degrees(rpy.z);
      }

      attitudeDesired.yaw = capAngle(attitudeDesired.yaw);
    }

    if (RATE_DO_EXECUTE(POSITION_RATE, stabilizerStep)) {
      positionController(&actuatorThrust, &attitudeDesired, setpoint, state);
    }

    attitudeDesiredExpert = attitudeDesired;

    if (RATE_DO_EXECUTE(ATTITUDE_RATE, stabilizerStep)) {
      // Switch between manual and automatic position control
      if (setpoint->mode.z == modeDisable) {
        actuatorThrust = setpoint->thrust;
      }
      if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {
        attitudeDesired.roll = setpoint->attitude.roll;
        attitudeDesired.pitch = setpoint->attitude.pitch;
      }

      attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                  attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                  &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);

      // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
      // value. Also reset the PID to avoid error buildup, which can lead to unstable
      // behavior if level mode is engaged later
      if (setpoint->mode.roll == modeVelocity) {
        rateDesired.roll = setpoint->attitudeRate.roll;
        attitudeControllerResetRollAttitudePID();
      }
      if (setpoint->mode.pitch == modeVelocity) {
        rateDesired.pitch = setpoint->attitudeRate.pitch;
        attitudeControllerResetPitchAttitudePID();
      }

      // TODO: Investigate possibility to subtract gyro drift.
      attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,
                              rateDesired.roll, rateDesired.pitch, rateDesired.yaw);

      attitudeControllerGetActuatorOutput(&control->roll,
                                          &control->pitch,
                                          &control->yaw);

      control->yaw = -control->yaw;

      cmd_thrust = control->thrust;
      cmd_roll = control->roll;
      cmd_pitch = control->pitch;
      cmd_yaw = control->yaw;
      r_roll = radians(sensors->gyro.x);
      r_pitch = -radians(sensors->gyro.y);
      r_yaw = radians(sensors->gyro.z);
      accelz = sensors->acc.z;
    }

    control->thrust = actuatorThrust;

    if (control->thrust == 0)
    {
      control->thrust = 0;
      control->roll = 0;
      control->pitch = 0;
      control->yaw = 0;

      cmd_thrust = control->thrust;
      cmd_roll = control->roll;
      cmd_pitch = control->pitch;
      cmd_yaw = control->yaw;

      attitudeControllerResetAllPID();
      positionControllerResetAllPID();

      // Reset the calculated YAW angle for rate control
      attitudeDesired.yaw = state->attitude.yaw;
    }


    if (!isToFStale) {
      #ifdef TOF_ENABLE
        isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
        // obstacleEmbedder(obstacle_inputs);
      #endif
    }


    #ifdef DEBUG_LOCALIZATION
        if (count == 750) {
          DEBUG_PRINT("Estimation: (%f,%f,%f)\n", state->position.x,state->position.y,state->position.z);
          // DEBUG_PRINT("Desired: (%f,%f,%f)\n", setpoint->position.x,setpoint->position.y,setpoint->position.z);
          // DEBUG_PRINT("ERROR: (%f,%f,%f)\n", state_array[0], state_array[1], state_array[2]);
          DEBUG_PRINT("ToF: (%f,%f,%f,%f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3], obstacle_inputs[4], obstacle_inputs[5], obstacle_inputs[6], obstacle_inputs[7]);
          // DEBUG_PRINT("Thrusts: (%i,%i,%i,%i)\n", control->normalizedForces[0], control->normalizedForces[1], control->normalizedForces[2], control->normalizedForces[3]);
          count = 0;
        }
        count++;
      #endif




  #else
    // THIS IS FOR UNGUIDED DATA COLLECTION
    // Call the PID controller 
    controllerPid(control, setpoint, sensors, state, stabilizerStep);
    if (!isToFStale) {
      #ifdef TOF_ENABLE
        isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
        // obstacleEmbedder(obstacle_inputs);
      #endif
    }
  #endif
}


float d_bound = 0.4f;
float d_bound_i;
static float roll_bound = 25.0f;
static float pitch_bound = 25.0f;
float roll_bound_i;
float pitch_bound_i;


struct control_t_n control_n;

float nn_input[7];

paramVarId_t controllerTypeId;

int controllerType;

// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {
  
  // tof_init(&tof_config);
  vTaskDelay(M2T(10));
  

  // set the stabilizer.controller to 5
  controllerTypeId = paramGetVarId("stabilizer", "controller");
  paramSetInt(controllerTypeId, 5);

  controllerType = paramGetInt(controllerTypeId);

  DEBUG_PRINT("Starting the controller\n");
  // print controllerType
  DEBUG_PRINT("Controller Type: %i\n", controllerType);




  while(1) {
    // vTaskDelay(M2T(100));


    // tof_init(&tof_config);
    vTaskDelay(M2T(100));

    // sample a random number between 0 and d_bound
    // d_bound_i = d_bound * (rand() / (float)RAND_MAX);

    // roll_bound_i = roll_bound * d_bound_i;
    // pitch_bound_i = pitch_bound * d_bound_i;

    controllerType = paramGetInt(controllerTypeId);
    DEBUG_PRINT("Controller Type: %i\n", controllerType);






    // vTaskDelay(M2T(67)); // 15Hz is roughly 67 ms intervals
    // #ifdef TOF_ENABLE
		// 	isToFStale = tof_task(&tof_config, &sensor_status, tof_input, tof_status);
		// #endif


    // networkEvaluate(&control_n, &nn_input);




  }
}


// ADD OBSEVATIONS LATER TODO
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



// LOG_GROUP_START(GUIDED_DEM)
// LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
// LOG_ADD(LOG_FLOAT, roll, &attitudeDesiredExpert.roll)
// LOG_ADD(LOG_FLOAT, pitch, &attitudeDesiredExpert.pitch)
// LOG_ADD(LOG_FLOAT, yaw, &attitudeDesiredExpert.yaw)
// LOG_GROUP_STOP(GUIDED_DEM)

// LOG_GROUP_START(GUIDANCE_ACTION)
// LOG_ADD(LOG_FLOAT, roll, &control_nn.thrust_1)
// LOG_ADD(LOG_FLOAT, pitch, &control_nn.thrust_2)
// LOG_GROUP_STOP(GUIDANCE_ACTION)