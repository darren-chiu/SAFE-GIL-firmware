#include "debug.h"
#include "log.h"
#include "param.h"
#include "controller.h"
#include "controller_pid.h"

#include "app.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>


#define DEBUG_MODULE "safe-gil-commander"

#include "network_evaluate_gil.h"
#include "obst_daq.h"

#include "estimator_kalman.h"
#include "motors.h"

#include "commander.h"

#include "timers.h"

uint8_t start = 0;


#define DELAY_TO_LAUNCH 500  //ms 
#define DELAY_AFTER_LAUNCH 300  //ms 



#define SAFEGIL_IM_TEST // if you want to test safegil im







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



// control_bounds = [torch.tensor([40000, -25, -25]), torch.tensor([60000, 25, 25]) ]

// float thrust_upper = 60000;
// float thrust_lower = 40000;
float roll_upper = 25;
float roll_lower = -25;
float pitch_upper = 25;
float pitch_lower = -25;



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

  // you can set start to 1 here if you want automatic start


  int counter = 0;

  float height = 0.41f;

  float nn_input[12];


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


  DEBUG_PRINT("Waiting for start\n");
  vTaskDelay(M2T(100));

  while(1) {
    // write debug messages
    // vTaskDelay(M2T(2000));
    
    // DEBUG_PRINT("Safe-Gil Commander\n");
    

    if (start) {



      if (!hover_yet) {
        DEBUG_PRINT("START HOVER\n");
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
        DEBUG_PRINT("Recording\n");

        // for (int i = 0; i < 50; i++) {
        //   headToSetpoint( (float)i / 1000.0f , 0.0f, height, 0.0f); // @TODO: check if this is correct
        //   vTaskDelay(20);
        // }
      }
      else{
        counter++;
        


        DEBUG_PRINT("HOVER FINISHED\n");

        
        nn_input[0] = getX();
        nn_input[1] = getY();
        nn_input[2] = getVx();
        nn_input[3] = getVy();
        z = getZ();


        DEBUG_PRINT("State: %f, %f, %f, %f\n", nn_input[0], nn_input[1], nn_input[2], nn_input[3]);


        if (counter > 390 || nn_input[0]>3.75f || nn_input[1]>1.4f || z>0.7f || nn_input[0]<-0.7f || nn_input[1]<-1.0f || z < 0.2f){
          // stop the drone
          DEBUG_PRINT("STOPPING\n");
          for (int i = 0; i < 10; i++) {
            headToSetpoint(nn_input[0], nn_input[1], z, 0);
            vTaskDelay(30);
          }

          // set recording parameter to 0
          paramSetInt(recordingId, 0);
          DEBUG_PRINT("Recording saved\n");

          // land the drone
          DEBUG_PRINT("LANDING\n");
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
          for (int i=0;i<8;i++) {
            nn_input[4+i] = ((roundf(obstacle_inputs[i] * 100) / 100) - 2.0f) / 16.0f; // for centered obs
          }
          DEBUG_PRINT("Obstacle Inputs: %f, %f, %f, %f, %f, %f, %f, %f\n", nn_input[4], nn_input[5], nn_input[6], nn_input[7], nn_input[8], nn_input[9], nn_input[10], nn_input[11]);

          networkEvaluate(&control_n, &nn_input);
          // self.expert_actions_mean = torch.tensor([ 4.8858308e+04, -5.7000000e-02, -1.7360000e+00 ])
          // self.expert_actions_std = torch.tensor([1.293753e+03, 4.344000e+00, 3.436000e+00 ])
          // action_unnormalized = action * self.expert_actions_std + self.expert_actions_mean 

          
          #ifdef SAFEGIL_IM_TEST
            // SAFEGIL IM TEST
            control_n.thrust_0 = control_n.thrust_0 * 7.216f;
            control_n.thrust_1 = control_n.thrust_1 * 4.801f;
          #else
            // BC IM TEST
            control_n.thrust_0 = control_n.thrust_0 * 6.462f + (-0.197f);
            control_n.thrust_1 = control_n.thrust_1 * 5.174f + (-1.296f);
          #endif

          control_n.thrust_0 = clip(control_n.thrust_0, roll_lower, roll_upper);
          control_n.thrust_1 = clip(control_n.thrust_1, pitch_lower, pitch_upper);

          // debug print control_n values
          DEBUG_PRINT(" roll: %f, pitch: %f \n", control_n.thrust_0, control_n.thrust_1);

          
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
      DEBUG_PRINT("Waiting for start\n");
      x = getX();
      y = getY();
      z = getZ();
      vx = getVx();
      vy = getVy();
      vz = getVz();
      DEBUG_PRINT("State: %f, %f, %f, %f, %f, %f\n", x, y, z, vx, vy, vz);

    }
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



/**
 * [Documentation for the ring group ...]
 */
PARAM_GROUP_START(im_test)
/**
 * @brief to start the flight
 */
PARAM_ADD_CORE(PARAM_UINT8, start, &start)
PARAM_GROUP_STOP(im_test)




