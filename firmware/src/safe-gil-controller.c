#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include "math3d.h"

#include "network_evaluate_gil.h"

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include <stdlib.h>

#define DEBUG_MODULE "safe-gil"


// #define GUIDED_DATA_COLLECTION

#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "obst_daq.h"

#include "timers.h"
#include "controller_pid.h"


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

// Observation Variables
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



logVarId_t logIdStateEstimateX;
logVarId_t logIdStateEstimateY;
logVarId_t logIdStateEstimateZ;
logVarId_t logIdStateEstimateVx;
logVarId_t logIdStateEstimateVy;
logVarId_t logIdStateEstimateVz;

void initLogIds(){
    logIdStateEstimateX = logGetVarId("stateEstimate", "x");
    logIdStateEstimateY = logGetVarId("stateEstimate", "y");
    logIdStateEstimateZ = logGetVarId("stateEstimate", "z");
    logIdStateEstimateVx = logGetVarId("kalman", "statePX");
    logIdStateEstimateVy = logGetVarId("kalman", "statePY");
    logIdStateEstimateVz = logGetVarId("kalman", "statePZ");
}


float getX() { return (float) logGetFloat(logIdStateEstimateX); }
float getY() { return (float) logGetFloat(logIdStateEstimateY); }
float getZ() { return (float) logGetFloat(logIdStateEstimateZ); }
float getVx() { return (float) logGetFloat(logIdStateEstimateVx); }
float getVy() { return (float) logGetFloat(logIdStateEstimateVy); }
float getVz() { return (float) logGetFloat(logIdStateEstimateVz); }



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
    // if (!isToFStale) {
    //   #ifdef TOF_ENABLE
    //     isToFStale = process_obst(state, obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
    //     // obstacleEmbedder(obstacle_inputs);
    //   #endif
    // }
  #endif
}


float d_bound = 0.49f;
float d_bound_i;
static float roll_bound = 25.0f;
static float pitch_bound = 25.0f;
float roll_bound_i;
float pitch_bound_i;

static float roll_dist = 0.0f;
static float pitch_dist = 0.0f;


struct control_t_n control_n;


// paramVarId_t controllerTypeId;

paramVarId_t recordingId;

// logVarId_t desAttitudeLogId;

int controllerType;

float desired_log_roll;


float roll_dist_i_min;
float roll_dist_i_max;
float pitch_dist_i_min;
float pitch_dist_i_max;


static float dt = 0.01f;
static float GZ = 9.81f;


// easy obstacle setting
// self.state_mean tensor([ 2.0000, -0.2500,  0.5000,  0.0000,  0.0000,  0.0000,  0.2500])
// self.state_var tensor([2.0000, 1.0000, 0.5000, 1.5000, 1.5000, 0.3000, 0.2500])
static float state_mean[7] = {2.0000f, -0.2500f, 0.5000f, 0.0000f, 0.0000f, 0.0000f, 0.2500f};
static float state_var[7] = {2.0000f, 1.0000f, 0.5000f, 1.5000f, 1.5000f, 0.3000f, 0.2500f};

// hard obstacle setting it is the same!

float getValue(const float *state_array, float d_bound_i, float roll, float pitch){
  float value = 0.0f;
  struct control_t_n deepreach_output;

  

  float next_state[6] = {state_array[0], state_array[1], state_array[2], state_array[3], state_array[4], state_array[5]};

  
  
  // def dsdt( state, control):
  //   dsdt = torch.zeros_like(state)
  //   dsdt[..., 0] = state[...,3]
  //   dsdt[..., 1] = state[...,4]
  //   # dsdt[..., 2] = state[...,5]
  //   dsdt[..., 3] = Gz*torch.tan( torch.deg2rad(control[...,2]) )
  //   dsdt[..., 4] = -Gz*torch.tan( torch.deg2rad(control[...,1]))
  //   # dsdt[..., 5] = (control[...,0]) - Gz
  //   return dsdt

  // traj1[i+1, :] = traj1[i, :] + dsdt(traj1[i, :], control[i, :]) * dt

  next_state[0] = next_state[0] + next_state[3] * dt;
  next_state[1] = next_state[1] + next_state[4] * dt;
  // next_state[2] = next_state[2] + next_state[5] * dt;
  next_state[3] = next_state[3] + GZ * tan( radians(-pitch));
  next_state[4] = next_state[4] - GZ * tan( radians(roll));


  // invert y and vy because of the coordinate system of reach
  // state_array[1] = -state_array[1];
  // state_array[4] = -state_array[4];
  next_state[1] = -next_state[1];
  next_state[4] = -next_state[4];

  
  float deepreach_input[8] = {1.4f, next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5], d_bound_i};
  // float deepreach_input[8] = {1.4f, 0.8479f, 0.4321, -0.3382, 0.3434, -0.3701, -0.2927, 0.45f};
  // state=torch.tensor([0,0,0.5,1.5,0,0])
  // float deepreach_input[8] = {1.4f, 0.0f, 0.0f, 0.5f, 1.5f, 0.0f, 0.0f, 0.45f};

  // convert the deepreach_input
  // input[..., 1:] = (coord[..., 1:] - self.state_mean) / self.state_var
  for (int i = 1; i < 8; i++) {
    deepreach_input[i] = (deepreach_input[i] - state_mean[i-1]) / state_var[i-1];
  }



  
  networkEvaluate(&deepreach_output, &deepreach_input);
  value = deepreach_output.thrust_0;
  return value;
}





static float values[4];

void appMain() {
  initLogIds();

   #ifdef TOF_ENABLE
    //Initialize sensor platform
		tof_init(&tof_config);
		vTaskDelay(M2T(10));
    //Start RTOS task for observations. The task will thus run at M2T(67) ~ 15Hz.
    ObservationTimer = xTimerCreate("ObservationTimer", M2T(67), pdTRUE, NULL, pullObs);
    xTimerStart(ObservationTimer, 20);
	#endif
  
  vTaskDelay(M2T(10));

  initLogIds();
  

  // set the stabilizer.controller to 5
  // controllerTypeId = paramGetVarId("stabilizer", "controller");
  // paramSetInt(controllerTypeId, 5);

  recordingId = paramGetVarId("usd", "logging");


  // controllerType = paramGetInt(controllerTypeId);

  DEBUG_PRINT("Starting the controller\n");
  // print controllerType
  // DEBUG_PRINT("Controller Type: %i\n", controllerType);


  // desAttitudeLogId = logGetVarId("GUIDED_DEM", "roll");

  float state_array[6];

  bool howering = false;

  while(1) {

    vTaskDelay(M2T(10));

    // sample a random number between 0 and d_bound
    d_bound_i = d_bound * (rand() / (float) RAND_MAX);

    // d_bound_i = d_bound;

    // DEBUG_PRINT("d_bound_i: %f\n", d_bound_i);

    roll_bound_i = roll_bound * d_bound_i;
    // roll_bound_i = 0.0f;
    pitch_bound_i = pitch_bound * d_bound_i;

    // get the state
    state_array[0] = getX();
    state_array[1] = getY();
    state_array[2] = getZ();
    state_array[3] = getVx();
    state_array[4] = getVy();
    state_array[5] = getVz();

    if (!howering){
      if (state_array[2] > 0.4f && state_array[0] < 0.3f && state_array[0] > -0.3f ) {
        howering = true;
        // set recording parameter to 1
        paramSetInt(recordingId, 1);
        DEBUG_PRINT("Recording\n");
      }
    }

    if (howering) {
      if (state_array[2] < 0.28f && state_array[0] > 3.6f) {
        howering = false;
        // set recording parameter to 0
        paramSetInt(recordingId, 0);
        DEBUG_PRINT("Recording saved\n");
      }
    }



    // print the state
    // DEBUG_PRINT("State: (%f,%f,%f,%f,%f,%f)\n", state_array[0], state_array[1], state_array[2], state_array[3], state_array[4], state_array[5]);

    roll_dist_i_max = roll_bound_i;
    roll_dist_i_min = -roll_bound_i;
    pitch_dist_i_max = pitch_bound_i;
    pitch_dist_i_min = -pitch_bound_i;


    // max pitch max roll
    values[0] = getValue(state_array, d_bound_i, roll_dist_i_max, pitch_dist_i_max);
    // max pitch min roll
    values[1] = getValue(state_array, d_bound_i, roll_dist_i_min, pitch_dist_i_max);
    // min pitch max roll
    values[2] = getValue(state_array, d_bound_i, roll_dist_i_max, pitch_dist_i_min);
    // min pitch min roll
    values[3] = getValue(state_array, d_bound_i, roll_dist_i_min, pitch_dist_i_min);

    // print values
    // DEBUG_PRINT("Values: %f, %f, %f, %f\n", values[0], values[1], values[2], values[3]);

    // get the index of the minimum value
    int min_index = 0;
    float min_value = values[0];
    for (int i = 0; i < 4; i++) {
      if (values[i] < min_value) {
        min_value = values[i];
        min_index = i;
      }
    }
    // print min_index
    // DEBUG_PRINT("Min Index: %i\n", min_index);

    if (!howering) {
      roll_dist = 0.0f;
      pitch_dist = 0.0f;
    }
    elseif (state_array[0] > 0.25f){
      // set roll_d and pitch_d according to the min_index
      if (min_index == 0) {
        roll_dist = roll_dist_i_max;
        pitch_dist = pitch_dist_i_max;
      } else if (min_index == 1) {
        roll_dist = roll_dist_i_min;
        pitch_dist = pitch_dist_i_max;
      } else if (min_index == 2) {
        roll_dist = roll_dist_i_max;
        pitch_dist = pitch_dist_i_min;
      } else if (min_index == 3) {
        roll_dist = roll_dist_i_min;
        pitch_dist = pitch_dist_i_min;
        // DEBUG_PRINT("roll_dist_i_min: %f\n", roll_dist_i_min);
      }
      else{DEBUG_PRINT("ERROR\n");}
    }

    // DEBUG_PRINT("Roll Dist: %f\n", roll_dist);
    // DEBUG_PRINT("Pitch Dist: %f\n", pitch_dist);

    if (!isToFStale) {
      isToFStale = process_obst(obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
    }
    count++;
    // DEBUG_PRINT("ToF: (%f,%f,%f,%f, %f, %f, %f, %f)\n", obstacle_inputs[0], obstacle_inputs[1], obstacle_inputs[2], obstacle_inputs[3], obstacle_inputs[4], obstacle_inputs[5], obstacle_inputs[6], obstacle_inputs[7]);








    // controllerType = paramGetInt(controllerTypeId);
    // DEBUG_PRINT("Controller Type: %i\n", controllerType);

    // desired_log_roll = logGetFloat(desAttitudeLogId);
    // DEBUG_PRINT("ROLL: %f\n", desired_log_roll);




  }
}


// PARAM_GROUP_START(guide_d)
// PARAM_ADD_CORE(PARAM_FLOAT, roll_d, &roll_dist)
// PARAM_ADD_CORE(PARAM_FLOAT, pitch_d, &pitch_dist)
// PARAM_GROUP_STOP(guide_d)

LOG_GROUP_START(guide_d)
LOG_ADD(LOG_FLOAT, roll_d, &roll_dist)
LOG_ADD(LOG_FLOAT, pitch_d, &pitch_dist)
LOG_GROUP_STOP(guide_d)


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



