
#include "math3d.h"
#include "stabilizer_types.h"
#include <math.h>
#include "controller_nn.h"
#include "log.h"
#include "param.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usec_time.h"
#include "peer_localization.h"
#include "debug.h"
#include <stdbool.h>

#define MAX_THRUST 0.15f
// PWM to thrust coefficients
#define A 2.130295e-11f
#define B 1.032633e-6f
#define C 5.484560e-4f

static bool enableBigQuad = false;

static float maxThrustFactor = 0.70f;
static bool relVel = true;
static bool relOmega = true;
static bool relXYZ = true;
static bool neighborObs = true;
static uint16_t freq = 500;

static control_t_n control_n;
static struct mat33 rot;
//static float state_array[18]; // for the corl network
static float state_array[24]; // for the obstacle avoiding policy
// static float state_array[22];

// vars related to neighbor drone(s)
// multiple neighbors
//static point_t pos_neighbors_prev[NUM_IDS];
//static point_t pos_neighbors_curr[NUM_IDS]; // idx into this array is the cfid
static float pos_neighbors_curr[NUM_IDS][4];
static float pos_neighbors_prev[NUM_IDS][4];
static int cf_ids[NUM_IDS] = {0};
static float deltaPoses[NUM_IDS][3];
static float pos_neighbors_rel[NUM_IDS][3] = {0};
static float vel_neighbors_rel[NUM_IDS][3] = {0};
static float vel_neighbors_prev[NUM_IDS][3] = {0};
static float neighbors_state_array[NEIGHBORS][6] = {0};
static float pos_raw[3];
static int cfid; // cfid of some neighbor to use as a timer
static float dt = 1.0;
static float maxPeerLocAgeMillis = 2000;
static const float weight = 0.95; // weight param for exponential filter
static uint16_t isStale = false;
static int motor1;
static int pwm0;

static float log_posx, log_posy, log_posz;

// hard code the obstacle positions for now
static const float obstacle_obs[K_OBSTACLES][OBST_OBS_DIM] = {
        {20.3,   20.3, 5e-7}, // x y z radius
        {20.3,   20.3, 5e-7},
};
static float obstacle_state_array[K_OBSTACLES][OBST_OBS_DIM];
static const float box_radius = 5.0; // half-width of the room box (from simulator)
static const float min_wall_dist = 0.0;
static const float max_wall_dist = 2.0;


static uint32_t usec_eval;



void controllerNNInit(void) {
	control_n.thrust_0 = 0.0f;
	control_n.thrust_1 = 0.0f;
	control_n.thrust_2 = 0.0f;
	control_n.thrust_3 = 0.0f;
}



bool controllerNNTest(void) {
	return true;
}


void controllerNNEnableBigQuad(void)
{
	enableBigQuad = true;
}



// range of action -1 ... 1, need to scale to range 0 .. 1
float scale(float v) {
	return 0.5f * (v + 1);
}



float clip(float v, float min, float max) {
	if (v < min) return min;
	if (v > max) return max;
	return v;
}

float exponentialFilter(float x, float y_prev, float w) {
  // x = current data point, y_prev = smoothed data point from previous timestep
  // w = weight parameter, 0 <= w <= 1
  return w * x + (1-w) * y_prev;
}


void controllerNN(control_t *control,
				  setpoint_t *setpoint,
				  const sensorData_t *sensors,
				  const state_t *state,
				  const uint32_t tick)
{
	control->enableDirectThrust = true;
    //	if (!RATE_DO_EXECUTE(/*RATE_100_HZ*/freq, tick)) {
    //	  return;
    //	}

    // DEBUG_PRINT("tick: %f\n", tick);
	// Orientation
	struct quat q = mkquat(state->attitudeQuaternion.x,
						   state->attitudeQuaternion.y,
						   state->attitudeQuaternion.z,
						   state->attitudeQuaternion.w);
	rot = quat2rotmat(q);

	// angular velocity
	float omega_roll = radians(sensors->gyro.x);
	float omega_pitch = radians(sensors->gyro.y);
	float omega_yaw = radians(sensors->gyro.z);

	// the state vector
	// TODO: clip error?
	// TODO: clip velocity?
	state_array[0] = state->position.x - setpoint->position.x;
	state_array[1] = state->position.y - setpoint->position.y;
	state_array[2] = state->position.z - setpoint->position.z;
	pos_raw[0] = state->position.x;
	pos_raw[1] = state->position.y;
	pos_raw[2] = state->position.z;
	if (relVel) {
		state_array[3] = state->velocity.x - setpoint->velocity.x;
		state_array[4] = state->velocity.y - setpoint->velocity.y;
		state_array[5] = state->velocity.z - setpoint->velocity.z;
	} else {
		state_array[3] = state->velocity.x;
		state_array[4] = state->velocity.y;
		state_array[5] = state->velocity.z;
	}
	state_array[6] = rot.m[0][0];
	state_array[7] = rot.m[0][1];
	state_array[8] = rot.m[0][2];
	state_array[9] = rot.m[1][0];
	state_array[10] = rot.m[1][1];
	state_array[11] = rot.m[1][2];
	state_array[12] = rot.m[2][0];
	state_array[13] = rot.m[2][1];
	state_array[14] = rot.m[2][2];

	if (relXYZ) {
		// rotate pos and vel
		struct vec rot_pos = mvmul(mtranspose(rot), mkvec(state_array[0], state_array[1], state_array[2]));
		struct vec rot_vel = mvmul(mtranspose(rot), mkvec(state_array[3], state_array[4], state_array[5]));

		state_array[0] = rot_pos.x;
		state_array[1] = rot_pos.y;
		state_array[2] = rot_pos.z;

		state_array[3] = rot_vel.x;
		state_array[4] = rot_vel.y;
		state_array[5] = rot_vel.z;
	}

	if (relOmega) {
		state_array[15] = omega_roll - radians(setpoint->attitudeRate.roll);
		state_array[16] = omega_pitch - radians(setpoint->attitudeRate.pitch);
		state_array[17] = omega_yaw - radians(setpoint->attitudeRate.yaw);
	} else {
		state_array[15] = omega_roll;
		state_array[16] = omega_pitch;
		state_array[17] = omega_yaw;
	}
    // distance to floor
    state_array[18] = clip(state->position.x + box_radius, min_wall_dist, max_wall_dist);
    state_array[19] = clip(state->position.y + box_radius, min_wall_dist, max_wall_dist);
    state_array[20] = clip(state->position.z, min_wall_dist, max_wall_dist); // distance to floor
    state_array[21] = clip(box_radius - state->position.x, min_wall_dist, max_wall_dist);
    state_array[22] = clip(box_radius - state->position.y, min_wall_dist, max_wall_dist);
    state_array[23] = clip((box_radius * 2.0) - state->position.z, min_wall_dist, max_wall_dist); // dist to ceiling


	//get neighbor pos/vel data and update at 10hz
  TickType_t const time = xTaskGetTickCount();

  bool doAgeFilter = maxPeerLocAgeMillis >= 0;

  for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {

    peerLocalizationOtherPosition_t const *otherPos = peerLocalizationGetPositionByIdx(i);
    if (otherPos == NULL || otherPos->id == 0) {
      continue;
    }

    if (doAgeFilter && (time - otherPos->pos.timestamp > maxPeerLocAgeMillis)) {
      isStale = 1;
      continue;
    }else {
      isStale = 0;
    }

//    DEBUG_PRINT("x: %f\n", otherPos->pos.x);
      pos_neighbors_curr[(int)otherPos->id][0] = otherPos->pos.x;
      pos_neighbors_curr[(int)otherPos->id][1] = otherPos->pos.y;
      pos_neighbors_curr[(int)otherPos->id][2] = otherPos->pos.z;
//    pos_neighbors_curr[(int)otherPos->id][0] = exponentialFilter(otherPos->pos.x, pos_neighbors_prev[otherPos->id][0], weight);
//    pos_neighbors_curr[(int)otherPos->id][1] = exponentialFilter(otherPos->pos.y, pos_neighbors_prev[otherPos->id][1], weight);
//    pos_neighbors_curr[(int)otherPos->id][2] = exponentialFilter(otherPos->pos.z, pos_neighbors_prev[otherPos->id][2], weight);
    pos_neighbors_curr[(int)otherPos->id][3] = otherPos->pos.timestamp;
    cf_ids[(int)otherPos->id] = (int)otherPos->id;
  }


	if (true) { // update pos/vel every 2ms

    int n_row = 0;
    for (int j = 0; j < NUM_IDS; j++) {
//      DEBUG_PRINT("cf_ids[%d] = %d\n",j,cf_ids[j]);
      if (cf_ids[j] == 0) {
        continue;
      }
      dt = (pos_neighbors_curr[j][3] - pos_neighbors_prev[j][3]) / 1000.0; // dt in seconds
//      DEBUG_PRINT("dt: %f\n", dt);
//      DEBUG_PRINT("New timestamp: %f, old timestamp: %f\n", pos_neighbors_curr[j].timestamp, pos_neighbors_prev[j].timestamp);
      deltaPoses[j][0] = pos_neighbors_curr[j][0] - pos_neighbors_prev[j][0];
      deltaPoses[j][1] = pos_neighbors_curr[j][1] - pos_neighbors_prev[j][1];
      deltaPoses[j][2] = pos_neighbors_curr[j][2] - pos_neighbors_prev[j][2];

      // get the relative positions
      pos_neighbors_rel[j][0] = pos_neighbors_curr[j][0] - state->position.x;
      pos_neighbors_rel[j][1] = pos_neighbors_curr[j][1] - state->position.y;
      pos_neighbors_rel[j][2] = pos_neighbors_curr[j][2] - state->position.z;

      if (dt != 0) {
        // update the velocity estimate
        float vx = (deltaPoses[j][0] / dt) - state->velocity.x;
        float vy = (deltaPoses[j][1] / dt) - state->velocity.y;
        float vz = (deltaPoses[j][2] / dt) - state->velocity.z;

        vel_neighbors_rel[j][0] = exponentialFilter(vx, vel_neighbors_prev[j][0], weight);
        vel_neighbors_rel[j][1] = exponentialFilter(vy, vel_neighbors_prev[j][1], weight);
        vel_neighbors_rel[j][2] = exponentialFilter(vz, vel_neighbors_prev[j][2], weight);
      }

      if (relXYZ) {
        // rotate neighbor pos and vel. Untested
        struct vec rot_neighbor_pos = mvmul(mtranspose(rot),
                                            mkvec(pos_neighbors_rel[j][0], pos_neighbors_rel[j][1], pos_neighbors_rel[j][2]));
        struct vec rot_neighbor_vel = mvmul(mtranspose(rot),
                                            mkvec(vel_neighbors_rel[j][0], vel_neighbors_rel[j][1], vel_neighbors_rel[j][2]));
        pos_neighbors_rel[j][0] = rot_neighbor_pos.x;
        pos_neighbors_rel[j][1] = rot_neighbor_pos.y;
        pos_neighbors_rel[j][2] = rot_neighbor_pos.z;

        vel_neighbors_rel[j][0] = rot_neighbor_vel.x;
        vel_neighbors_rel[j][1] = rot_neighbor_vel.y;
        vel_neighbors_rel[j][2] = rot_neighbor_vel.z;
      }

      pos_neighbors_prev[j][0] = pos_neighbors_curr[j][0];
      pos_neighbors_prev[j][1] = pos_neighbors_curr[j][1];
      pos_neighbors_prev[j][2] = pos_neighbors_curr[j][2];
      pos_neighbors_prev[j][3] = pos_neighbors_curr[j][3];

      vel_neighbors_prev[j][0] = vel_neighbors_rel[j][0];
      vel_neighbors_prev[j][1] = vel_neighbors_rel[j][1];
      vel_neighbors_prev[j][2] = vel_neighbors_rel[j][2];

      // update the neighbor obs
      neighbors_state_array[n_row][0] = pos_neighbors_rel[j][0];
      neighbors_state_array[n_row][1] = pos_neighbors_rel[j][1];
      neighbors_state_array[n_row][2] = pos_neighbors_rel[j][2];
      neighbors_state_array[n_row][3] = vel_neighbors_rel[j][0];
      neighbors_state_array[n_row][4] = vel_neighbors_rel[j][1];
      neighbors_state_array[n_row][5] = vel_neighbors_rel[j][2];
      n_row++;

    }
	}

    if(NEIGHBOR_AVOIDANCE){
        neighborEmbeddings(neighbors_state_array);
    }

  if(OBSTACLE_AVOIDANCE){
      // get relative obstacle observations and feed it to the obstacle encoder
      for(int i = 0; i < K_OBSTACLES; i++) {
        obstacle_state_array[i][0] = obstacle_obs[i][0] - state->position.x;
        obstacle_state_array[i][1] = obstacle_obs[i][1] - state->position.y;
        obstacle_state_array[i][2] = obstacle_obs[i][2] - state->position.z;
        obstacle_state_array[i][3] = obstacle_obs[i][3];
      }
      obstacleEmbeddings(obstacle_state_array);
  }




  // run the neural neural network
	networkEvaluate(&control_n, state_array);

	// convert thrusts to directly to PWM
	// need to hack the firmware (stablizer.c and power_distribution_stock.c)
	int PWM_0, PWM_1, PWM_2, PWM_3;
	thrusts2PWM(&control_n, &PWM_0, &PWM_1, &PWM_2, &PWM_3);

	if (setpoint->mode.z == modeDisable) {
		control->motorRatios[0] = 0;
		control->motorRatios[1] = 0;
		control->motorRatios[2] = 0;
		control->motorRatios[3] = 0;
	} else {
		control->motorRatios[0] = PWM_0;
		control->motorRatios[1] = PWM_1;
		control->motorRatios[2] = PWM_2;
		control->motorRatios[3] = PWM_3;
	}
}


void thrusts2PWM(control_t_n *control_n,
	int *PWM_0, int *PWM_1, int *PWM_2, int *PWM_3){

	#if 0
	// scaling and cliping
	control_n->thrust_0 = MAX_THRUST * clip(scale(control_n->thrust_0), 0.0, 1.0);
	control_n->thrust_1 = MAX_THRUST * clip(scale(control_n->thrust_1), 0.0, 1.0);
	control_n->thrust_2 = MAX_THRUST * clip(scale(control_n->thrust_2), 0.0, 1.0);
	control_n->thrust_3 = MAX_THRUST * clip(scale(control_n->thrust_3), 0.0, 1.0);

	// motor 0
	*PWM_0 = (int)((-B + sqrtf(B * B - 4 * A * (C - control_n->thrust_0))) / (2 * A));
	// motor 1
	*PWM_1 = (int)((-B + sqrtf(B * B - 4 * A * (C - control_n->thrust_1))) / (2 * A));
	// motor 2
	*PWM_2 = (int)((-B + sqrtf(B * B - 4 * A * (C - control_n->thrust_2))) / (2 * A));
	// motor 3
	*PWM_3 = (int)((-B + sqrtf(B * B - 4 * A * (C - control_n->thrust_3))) / (2 * A));
	#else

	// scaling and cliping
	if (enableBigQuad) {
		// Big quad => output angular velocity of rotors

		// // motor 0
		// *PWM_0 = maxThrustFactor * UINT16_MAX * sqrtf(clip(scale(control_n->thrust_0), 0.0, 1.0));
		// // motor 1
		// *PWM_1 = maxThrustFactor * UINT16_MAX * sqrtf(clip(scale(control_n->thrust_1), 0.0, 1.0));
		// // motor
		// *PWM_2 = maxThrustFactor * UINT16_MAX * sqrtf(clip(scale(control_n->thrust_2), 0.0, 1.0));
		// // motor 3
		// *PWM_3 = maxThrustFactor * UINT16_MAX * sqrtf(clip(scale(control_n->thrust_3), 0.0, 1.0));

		// motor 0
		*PWM_0 = maxThrustFactor * UINT16_MAX * clip(scale(control_n->thrust_0), 0.0, 1.0);
		// motor 1
		*PWM_1 = maxThrustFactor * UINT16_MAX * clip(scale(control_n->thrust_1), 0.0, 1.0);
		// motor
		*PWM_2 = maxThrustFactor * UINT16_MAX * clip(scale(control_n->thrust_2), 0.0, 1.0);
		// motor 3
		*PWM_3 = maxThrustFactor * UINT16_MAX * clip(scale(control_n->thrust_3), 0.0, 1.0);

	} else {
		// Regular Crazyflie => output thrust directly
		// motor 0
		*PWM_0 = 1.0f * UINT16_MAX * clip(scale(control_n->thrust_0), 0.0, 1.0);
		// motor 1
		*PWM_1 = 1.0f * UINT16_MAX * clip(scale(control_n->thrust_1), 0.0, 1.0);
		// motor
		*PWM_2 = 1.0f * UINT16_MAX * clip(scale(control_n->thrust_2), 0.0, 1.0);
		// motor 3
		*PWM_3 = 1.0f * UINT16_MAX * clip(scale(control_n->thrust_3), 0.0, 1.0);
	}

	#endif
}

PARAM_GROUP_START(ctrlNN)
PARAM_ADD(PARAM_FLOAT, max_thrust, &maxThrustFactor)
PARAM_ADD(PARAM_UINT8, rel_vel, &relVel)
PARAM_ADD(PARAM_UINT8, rel_omega, &relOmega)
PARAM_ADD(PARAM_UINT8, rel_xyz, &relXYZ)
PARAM_ADD(PARAM_UINT8, use_neighbor_obs, &neighborObs)
PARAM_ADD(PARAM_UINT8, num_neighbors, NEIGHBORS)
PARAM_ADD(PARAM_UINT16, freq, &freq)
PARAM_GROUP_STOP(ctrlNN)

LOG_GROUP_START(ctrlNN)
//LOG_ADD(LOG_FLOAT, out0, &control_n.thrust_0)
//LOG_ADD(LOG_FLOAT, out1, &control_n.thrust_1)
//LOG_ADD(LOG_FLOAT, out2, &control_n.thrust_2)
//LOG_ADD(LOG_FLOAT, out3, &control_n.thrust_3)
//
LOG_ADD(LOG_FLOAT, posX, &state_array[0])
LOG_ADD(LOG_FLOAT, posY, &state_array[1])
LOG_ADD(LOG_FLOAT, posZ, &state_array[2])
//LOG_ADD(LOG_FLOAT, obst_rawx, &obstacle_obs[0][0])
//LOG_ADD(LOG_FLOAT, obst_rawy, &obstacle_obs[0][1])

//LOG_ADD(LOG_FLOAT, obstX, &obstacle_state_array[0][0])
//LOG_ADD(LOG_FLOAT, obstY, &obstacle_state_array[0][1])
//LOG_ADD(LOG_FLOAT, obstZ, &obstacle_state_array[0][2])
//LOG_ADD(LOG_FLOAT, floor_dist, &state_array[18])
//
//LOG_ADD(LOG_FLOAT, in3, &state_array[3])
//LOG_ADD(LOG_FLOAT, in4, &state_array[4])
//LOG_ADD(LOG_FLOAT, in5, &state_array[5])
//
//LOG_ADD(LOG_FLOAT, in15, &state_array[15])
//LOG_ADD(LOG_FLOAT, in16, &state_array[16])
//LOG_ADD(LOG_FLOAT, in17, &state_array[17])
// rel pos of whatever the first neighbor drone is
//LOG_ADD(LOG_FLOAT, nPos0, &neighbors_state_array[0][0])
//LOG_ADD(LOG_FLOAT, nPos1, &neighbors_state_array[0][1])
//LOG_ADD(LOG_FLOAT, nPos2, &neighbors_state_array[0][2])
//LOG_ADD(LOG_FLOAT, nVel0, &neighbors_state_array[0][3])
//LOG_ADD(LOG_FLOAT, nVel1, &neighbors_state_array[0][4])
//LOG_ADD(LOG_FLOAT, nVel2, &neighbors_state_array[0][5])
//LOG_ADD(LOG_UINT16, motor1, &motor1)
//LOG_ADD(LOG_UINT16, pwm0, &pwm0)
//
//LOG_ADD(LOG_UINT32, usec_eval, &usec_eval)
//LOG_ADD(LOG_UINT8, isStale, &isStale)

LOG_GROUP_STOP(ctrlNN)