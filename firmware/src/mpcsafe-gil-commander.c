#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"
#include "math3d.h"

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>


#define DEBUG_MODULE "mpcsafe-gil-commander"

#include "network_evaluate_gil_value.h"
#include "network_evaluate_gil_policy.h"
#include "obst_daq.h"

#include "estimator_kalman.h"
#include "motors.h"

#include "commander.h"

#include "timers.h"

uint8_t start = 0;


#define DELAY_TO_LAUNCH 500  //ms 
#define DELAY_AFTER_LAUNCH 300  //ms 



// TODO MAKE THE SENSOR READINGS SAME ORDER (GUESS: REVERT THEM)
// TODO DO THE INPUT AND OUTPUT SCALEINGS OF THE POLICY NETWORK done


// #define SAFEGIL_IM_TEST // if you want to test safegil im


// #define ENABLE_SAFETY_FILTER // if you want to filter the policy

#ifdef ENABLE_SAFETY_FILTER
  /** Ideal filter parameters: 
  *  For Two Obstacles (gets stuck):
  *  roll_bound = 10.0
  *  pitch_bound = 10.0f
  *  filter_threshold = 0.15f
  * 
  *  roll_bound = 10.0  // d bound is 0 for this
  *  pitch_bound = 10.0f
  *  filter_threshold = 0.1f
  * 
  *  For one Obstacle:
  *  roll_bound = 10.0f
  *  pitch_bound = 10.0f
  *  filter_threshold = 0.25f
  **/ 

  static float GZ = 9.81f;
  static float dt = 0.01f;

  static float state_mean[7] = {2.0000f, -0.2500f, 0.5000f, 0.0000f, 0.0000f, 0.0000f, 0.2500f};
  static float state_var[7] = {2.0000f, 0.75f, 0.5000f, 1.5000f, 1.5000f, 0.3000f, 0.2500f};

  static float roll_bound = 15.0f;
  static float pitch_bound = 15.0f;
  
  static float roll_opt_control = 0.0f;
  static float pitch_opt_control = 0.0f;

  float roll_opt_control_min;
  float roll_opt_control_max;
  float pitch_opt_control_min;
  float pitch_opt_control_max;

  float d_bound_i = 10.0/25.0; // Was 0.0

  static float values[4];

  static float current_value;

  static float filter_threshold = 0.0f;

#endif



// State variables for the stabilizer
static setpoint_t setpoint;



static state_t state;


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


void convertToSetpoint(setpoint_t *setpoint, float roll, float pitch){
  
  setpoint->attitude.roll = roll;
  setpoint->attitude.pitch = pitch;
  setpoint->attitudeRate.yaw = 0;
  // setpoint->thrust = thrust;
}

#ifdef ENABLE_SAFETY_FILTER
  float getValue(const float *state_array, float d_bound_i, float roll, float pitch){
    float value = 0.0f;
    struct control_t_n deepreach_output;

    float next_state[6] = {state_array[0], state_array[1], state_array[2], state_array[3], state_array[4], state_array[5]};

    next_state[0] = next_state[0] + next_state[3] * dt;
    next_state[1] = next_state[1] + next_state[4] * dt;
    // next_state[2] = next_state[2] + next_state[5] * dt;
    next_state[3] = next_state[3] + GZ * tan( radians(-pitch));
    next_state[4] = next_state[4] - GZ * tan( radians(roll));

    // invert y and vy because of the coordinate system of reach
    next_state[1] = -next_state[1];
    next_state[4] = -next_state[4];

    
    float deepreach_input[8] = {1.4f, next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5], d_bound_i};

    // convert the deepreach_input
    // input[..., 1:] = (coord[..., 1:] - self.state_mean) / self.state_var
    for (int i = 1; i < 8; i++) {
      deepreach_input[i] = (deepreach_input[i] - state_mean[i-1]) / state_var[i-1];
    }

    networkEvaluateValue(&deepreach_output, &deepreach_input);
    value = (deepreach_output.thrust_0 * 1.2 / 0.02) + 0.9;
    return value;
  }

  float getCurrentValue(const float *state_array, float d_bound_i){
    float value = 0.0f;
    struct control_t_n deepreach_output;

    float next_state[6] = {state_array[0], state_array[1], state_array[2], state_array[3], state_array[4], state_array[5]};

    // next_state[0] = next_state[0] + next_state[3] * dt;
    // next_state[1] = next_state[1] + next_state[4] * dt;
    // // next_state[2] = next_state[2] + next_state[5] * dt;
    // next_state[3] = next_state[3] + GZ * tan( radians(-pitch));
    // next_state[4] = next_state[4] - GZ * tan( radians(roll));

    // invert y and vy because of the coordinate system of reach
    next_state[1] = -next_state[1];
    next_state[4] = -next_state[4];

    
    float deepreach_input[8] = {1.4f, next_state[0], next_state[1], next_state[2], next_state[3], next_state[4], next_state[5], d_bound_i};

    // convert the deepreach_input
    // input[..., 1:] = (coord[..., 1:] - self.state_mean) / self.state_var
    for (int i = 1; i < 8; i++) {
      deepreach_input[i] = (deepreach_input[i] - state_mean[i-1]) / state_var[i-1];
    }

    networkEvaluateValue(&deepreach_output, &deepreach_input);
    value = (deepreach_output.thrust_0 * 1.2 / 0.02) + 0.9;
    return value;
  }
#endif



logVarId_t logIdStateEstimateX;
logVarId_t logIdStateEstimateY;
logVarId_t logIdStateEstimateZ;
logVarId_t logIdStateEstimateVx;
logVarId_t logIdStateEstimateVy;
logVarId_t logIdStateEstimateVz;

paramVarId_t recordingId;


// struct nn_output nn_output;
struct control_t_n control_n;

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


float clip(float n, float lower, float upper) {
  return fmax(lower, fmin(n, upper));
}

// 0.3 radians to degrees
float roll_upper = 3.0f * 180.0f / 3.14159265359f;
float roll_lower = -3.0f * 180.0f / 3.14159265359f;
float pitch_upper = 3.0f * 180.0f / 3.14159265359f;
float pitch_lower = -3.0f * 180.0f / 3.14159265359f;


static void positionSet(setpoint_t *setpoint, float x, float y, float z, float yaw)
{
  memset(setpoint, 0, sizeof(setpoint_t));
  setpoint->mode.x = modeAbs;
  setpoint->mode.y = modeAbs;
  setpoint->mode.z = modeAbs;
  setpoint->position.x = x;
  setpoint->position.y = y;
  setpoint->position.z = z;
  setpoint->mode.yaw = modeAbs;
  setpoint->attitude.yaw = yaw;
  setpoint->mode.roll = modeDisable;
  setpoint->mode.pitch = modeDisable;
  setpoint->mode.quat = modeDisable;
}


static void headToSetpoint(float x, float y, float z, float yaw)
{
  setpoint_t setpoint;
  positionSet(&setpoint, x, y, z, yaw);
  commanderSetSetpoint(&setpoint, 3);
}


void TakeOff(float height) {
  vTaskDelay(DELAY_TO_LAUNCH);
  point_t pos;
  memset(&pos, 0, sizeof(pos));
  estimatorKalmanGetEstimatedPos(&pos);

  uint32_t endheight = (uint32_t)(100 * (height - 0.3f));
  for(uint32_t i=0; i<endheight; i++) {
      headToSetpoint(pos.x, pos.y, 0.3f + (float)i / 100.0f, 0);
      vTaskDelay(30);
      DEBUG_PRINT("TakeOff: %f\n", 0.3f + (float)i / 100.0f);
  }

  vTaskDelay(DELAY_AFTER_LAUNCH); // wait for being stablize
}

static void Land(void) {
    point_t pos;
    memset(&pos, 0, sizeof(pos));
    estimatorKalmanGetEstimatedPos(&pos);

    float height = pos.z;
    float current_yaw = logGetFloat(logGetVarId("stateEstimate", "yaw"));

    for(int i=(int)100*height; i>100*0.07f; i--) {
        // setHoverAltitude((float)i / 100.0f);
        headToSetpoint(pos.x, pos.y, (float)i / 100.0f, current_yaw);
        vTaskDelay(40);
    }
    vTaskDelay(30);
    motorsSetRatio(MOTOR_M1, 0);
    motorsSetRatio(MOTOR_M2, 0);
    motorsSetRatio(MOTOR_M3, 0);
    motorsSetRatio(MOTOR_M4, 0);
    commanderRelaxPriority();
    vTaskDelay(200);
}








bool hover_yet = false;


// We still need an appMain() function, but we will not really use it. Just let it quietly sleep.
void appMain() {

  #ifdef TOF_ENABLE
    	//Initialize sensor platform
		tof_init(&tof_config);
		vTaskDelay(M2T(10));
    //Start RTOS task for observations. The task will thus run at M2T(67) ~ 15Hz.
    ObservationTimer = xTimerCreate("ObservationTimer", M2T(67), pdTRUE, NULL, pullObs);
    xTimerStart(ObservationTimer, 20);
	#endif

  initLogIds();

  recordingId = paramGetVarId("usd", "logging");

  #ifdef ENABLE_SAFETY_FILTER
    roll_opt_control_max = roll_bound;
    roll_opt_control_min = -roll_bound;
    pitch_opt_control_max = pitch_bound;
    pitch_opt_control_min = -pitch_bound;
  #endif

  int counter = 0;

  float height = 0.41f;

  float nn_input[10];


  float x;
  float y;
  float z;
  float vx;
  float vy;
  float vz;
  
  // tof_init(&tof_config);
  

  setpoint.mode.x = modeDisable;
  setpoint.mode.y = modeDisable;
  setpoint.mode.z = modeAbs;
  setpoint.mode.roll = modeAbs;
  setpoint.mode.pitch = modeAbs;
  setpoint.mode.yaw = modeAbs;
  setpoint.position.z = height;

  float state_array[6]; 
  float prev_state_array[6]; 


  DEBUG_PRINT("Waiting for start\n");
  vTaskDelay(M2T(100));

  while(1) {
    // write debug messages
    // vTaskDelay(M2T(2000));
    
    // DEBUG_PRINT("Safe-Gil Commander\n");
    

    if (start) {

      if (!hover_yet) {
        // DEBUG_PRINT("START HOVER\n");
        vTaskDelay(1000);
        // Hover to 41 cm and wait for the drone to stabilize
        TakeOff(height);

        for (int i = 0; i < 50; i++) {
          headToSetpoint(0.0f, 0.0f, height, 0.0f);
          vTaskDelay(30);
        }

        hover_yet = true;
        // start recording data
        // set recording parameter to 1
        paramSetInt(recordingId, 1);
        // DEBUG_PRINT("Recording\n");

      }
      else{
        counter++;

        // DEBUG_PRINT("HOVER FINISHED\n");

        x = getX();
        y = getY();
        nn_input[0] = getVx();
        nn_input[1] = getVy();
        z = getZ();


        DEBUG_PRINT("State: x:%f, y:%f, vx:%f, vy:%f\n", x, y, nn_input[0], nn_input[1]); // This debug print is necessary without safety filter
        // this debug print is necessary without safety filter


        if (counter > 1000 || x>4.10f || y>4.0f || z>0.7f || x<-0.7f || y<-4.0f || z < 0.2f){
          // stop the drone
          // DEBUG_PRINT("STOPPING\n");
          for (int i = 0; i < 10; i++) {
            headToSetpoint(x, y, z, 0);
            vTaskDelay(30);
          }

          // set recording parameter to 0
          paramSetInt(recordingId, 0);
          DEBUG_PRINT("Recording saved\n");
 
          // land the drone
          DEBUG_PRINT("LANDING\n");
          DEBUG_PRINT("Counter: %d\n", counter);


          Land();
          
          vTaskDelay(1000);
          start = 0;

        }
        else{


          // modify the inputs

          /**
           * @TODO: UMUT MODIFY HERE TO FIT YOUR NETWORK. 
           * "obstacle_inputs" are updated at the correct frequency in a separate process.
           */
          if (!isToFStale) {
            isToFStale = process_obst(obstacle_inputs, (uint16_t* )tof_input, (uint8_t* ) tof_status);
          }

          // putting the rounded observation inputs to the nn_input array
          // note that in the simulation the observation order is reversed. therefor you need to reverse the input order here. 
          for (int i=0;i<8;i++) {
            nn_input[2+i] = ((roundf(obstacle_inputs[7-i] * 100) / 100) - 2.0f) / 2.0f; // for centered obs // TODO: CHECK THIS REVERSAL
          }
          DEBUG_PRINT("Obstacle Inputs: %f, %f, %f, %f, %f, %f, %f, %f\n", nn_input[2], nn_input[3], nn_input[4], nn_input[5], nn_input[6], nn_input[7], nn_input[8], nn_input[9]); // This debug print is necessary without safety filter
          // this debug print is necessary without safety filter

          #ifndef ENABLE_SAFETY_FILTER
            networkEvaluatePolicy(&control_n, &nn_input);
            // self.expert_actions_mean = torch.tensor([ 4.8858308e+04, -5.7000000e-02, -1.7360000e+00 ])
            // self.expert_actions_std = torch.tensor([1.293753e+03, 4.344000e+00, 3.436000e+00 ])
            // action_unnormalized = action * self.expert_actions_std + self.expert_actions_mean 

            control_n.thrust_0 = control_n.thrust_0 * roll_upper;
            control_n.thrust_1 = control_n.thrust_1 * pitch_upper;
            
            control_n.thrust_0 = clip(control_n.thrust_0, roll_lower, roll_upper);
            control_n.thrust_1 = clip(control_n.thrust_1, pitch_lower, pitch_upper);
          #endif

          #ifdef ENABLE_SAFETY_FILTER
            // get the state
            state_array[0] = getX();
            state_array[1] = getY();
            state_array[2] = getZ();
            state_array[3] = getVx();
            state_array[4] = getVy();
            state_array[5] = getVz();

            // query the value at previous state and previous control.
            // current_value = getValue(prev_state_array, d_bound_i, control_n.thrust_0, control_n.thrust_1);

            // query the value at previous state and previous control.
            current_value = getCurrentValue(state_array, d_bound_i);
            
            // for (int i = 0; i < 6; i++) {
            //   prev_state_array[i] = state_array[i];
            // }
            // if value is below treshold
            // get the optimum control
            if ((current_value < filter_threshold) && (state_array[0] > 0.2)) {
              DEBUG_PRINT("(V,x,y): (%f,%f,%f)\n", current_value, state_array[0], state_array[1]);
              
              // DEBUG_PRINT("Activate Safety Filter: %f\n", current_value);

              // max pitch max roll
              values[0] = getValue(state_array, d_bound_i, roll_opt_control_max, pitch_opt_control_max);
              // max pitch min roll
              values[1] = getValue(state_array, d_bound_i, roll_opt_control_min, pitch_opt_control_max);
              // min pitch max roll
              values[2] = getValue(state_array, d_bound_i, roll_opt_control_max, pitch_opt_control_min);
              // min pitch min roll
              values[3] = getValue(state_array, d_bound_i, roll_opt_control_min, pitch_opt_control_min);

              int max_index = 0;
              float max_value = -1.0f * INFINITY;
              for (int i = 0; i < 4; i++) {
                if (values[i] > max_value) {
                  max_value = values[i];
                  max_index = i;
                }
              }

              if (max_index == 0) {
                roll_opt_control = roll_opt_control_max;
                pitch_opt_control = pitch_opt_control_max;
              } else if (max_index == 1) {
                roll_opt_control = roll_opt_control_min;
                pitch_opt_control = pitch_opt_control_max;
              } else if (max_index == 2) {
                roll_opt_control = roll_opt_control_max;
                pitch_opt_control = pitch_opt_control_min;
              } else if (max_index == 3) {
                roll_opt_control = roll_opt_control_min;
                pitch_opt_control = pitch_opt_control_min;
                // DEBUG_PRINT("roll_opt_control_min: %f\n", roll_opt_control_min);
              }

              // set the setpoint values to the optimum control
              control_n.thrust_0 = roll_opt_control;
              control_n.thrust_1 = pitch_opt_control;
              DEBUG_PRINT("ROLL: %f, PITCH: %f\n", roll_opt_control, pitch_opt_control);

            } else {

              networkEvaluatePolicy(&control_n, &nn_input);
              // self.expert_actions_mean = torch.tensor([ 4.8858308e+04, -5.7000000e-02, -1.7360000e+00 ])
              // self.expert_actions_std = torch.tensor([1.293753e+03, 4.344000e+00, 3.436000e+00 ])
              // action_unnormalized = action * self.expert_actions_std + self.expert_actions_mean 

              
              #ifdef SAFEGIL_IM_TEST
                // SAFEGIL IM TEST
                control_n.thrust_0 = control_n.thrust_0 * 7.216f;
                control_n.thrust_1 = control_n.thrust_1 * 4.801f;
              #else
                // BC IM TEST [6.295 4.773]
                control_n.thrust_0 = control_n.thrust_0 * 6.295f;
                control_n.thrust_1 = control_n.thrust_1 * 4.773f;
              #endif

              control_n.thrust_0 = clip(control_n.thrust_0, roll_lower, roll_upper);
              control_n.thrust_1 = clip(control_n.thrust_1, pitch_lower, pitch_upper);

            }

          #endif

          // debug print control_n values
          // DEBUG_PRINT(" roll: %f, pitch: %f \n", control_n.thrust_0, control_n.thrust_1); // This debug print is necessary without safety filter
          // this debug print is necessary without safety filter
          
          // convert to setpoint
          convertToSetpoint( &setpoint, control_n.thrust_0, control_n.thrust_1);


          commanderSetSetpoint(&setpoint, 3);
          vTaskDelay(30);

          // headToSetpoint(0.0f, 0.0f, height, 0.0f);
          // vTaskDelay(30);
        }
      }
    }
    else {
      // dont do anything
      vTaskDelay(1000);
      // DEBUG_PRINT("Waiting for start\n");
      // x = getX();
      // y = getY();
      // z = getZ();
      // vx = getVx();
      // vy = getVy();
      // vz = getVz();
      // DEBUG_PRINT("State: %f, %f, %f, %f, %f, %f\n", x, y, z, vx, vy, vz);

    }
  }

}

// Value, Desired Control
LOG_GROUP_START(gil)
// LOG_ADD(LOG_FLOAT, value, &current_value)
LOG_ADD(LOG_FLOAT, control_r, &control_n.thrust_0)
LOG_ADD(LOG_FLOAT, control_p, &control_n.thrust_1)
LOG_ADD(LOG_FLOAT, obs1, &obstacle_inputs[0])
LOG_ADD(LOG_FLOAT, obs2, &obstacle_inputs[1])
LOG_ADD(LOG_FLOAT, obs3, &obstacle_inputs[2])
LOG_ADD(LOG_FLOAT, obs4, &obstacle_inputs[3])
LOG_ADD(LOG_FLOAT, obs5, &obstacle_inputs[4])
LOG_ADD(LOG_FLOAT, obs6, &obstacle_inputs[5])
LOG_ADD(LOG_FLOAT, obs7, &obstacle_inputs[6])
LOG_ADD(LOG_FLOAT, obs8, &obstacle_inputs[7])
LOG_GROUP_STOP(gil)



/**
 * [Documentation for the ring group ...]
 */
PARAM_GROUP_START(im_test)
/**
 * @brief to start the flight
 */
PARAM_ADD_CORE(PARAM_UINT8, start, &start)
PARAM_GROUP_STOP(im_test)
