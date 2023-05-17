
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

static const float maxThrustFactor = 0.70f;
static const bool relVel = false;
static const bool relOmega = false;
static const bool relXYZ = false;

//static uint16_t freq = 500;
control_t_n control_n;

float pos_neighbors_curr[NUM_IDS][4];
float pos_neighbors_prev[NUM_IDS][4];
float deltaPoses[NUM_IDS][3];

float pos_neighbors_rel[NUM_IDS][3] = {0};
float vel_neighbors_rel[NUM_IDS][3] = {0};
float vel_neighbors_prev[NUM_IDS][3] = {0};

static const float maxPeerLocAgeMillis = 2000;
static const float weight = 0.95; // weight param for exponential filter
static uint16_t isStale = false;

static const float min_wall_dist = 0.0;
static const float max_wall_dist = 3.0;

// Neighbor
static const int NEIGHBOR_NUM = 6;

// Obstacle (inf height)
static const float obst_pos[OBST_NUM][2] = {
        {0.0, 0.0}, // x y z
        {-2.0, -2.0},
};

static const float obst_sdfs[9][2] = {
        {-0.1, -0.1},
        {-0.1, 0.0},
        {-0.1, 0.1},
        {0.0, -0.1},
        {0.0, 0.0},
        {0.0, 0.1},
        {0.1, -0.1},
        {0.1, 0.0},
        {0.1, 0.1},
};


// Control
int pre_control_command[4] = {0};


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
    // This file is specifically for 8 drones, 6 neighbors
	control->enableDirectThrust = true;
	float state_array[19];
    struct mat33 rot;

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
    state_array[18] = clip(state->position.z, min_wall_dist, max_wall_dist); // distance to floor


	// Get neighbor pos/vel data and update at 10hz
    TickType_t const time = xTaskGetTickCount();
    bool doAgeFilter = maxPeerLocAgeMillis >= 0;

    // For neighbors
    int cur_agent_num = 0;
    float rel_dist_neighbors[6][2] = {0};

    for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {
        peerLocalizationOtherPosition_t const *otherPos = peerLocalizationGetPositionByIdx(i);
        if (otherPos == NULL || otherPos->id == 0) {
          continue;
        }
        if (doAgeFilter && (time - otherPos->pos.timestamp > maxPeerLocAgeMillis)) {
          isStale = 1;
          continue;
        } else {
          isStale = 0;
        }

        if ((otherPos->pos.timestamp - pos_neighbors_prev[j][3]) / 1000.0 == 0){
            // dt in seconds; this is used for future calculate rel velocity
            continue;
        }

        pos_neighbors_curr[(int)otherPos->id][0] = otherPos->pos.x;
        pos_neighbors_curr[(int)otherPos->id][1] = otherPos->pos.y;
        pos_neighbors_curr[(int)otherPos->id][2] = otherPos->pos.z;
        pos_neighbors_curr[(int)otherPos->id][3] = otherPos->pos.timestamp;

        float tmp_dist_x = otherPos->pos.x - state->position.x;
        float tmp_dist_y = otherPos->pos.y - state->position.y;
        float tmp_dist_z = otherPos->pos.z - state->position.z;

        rel_dist_neighbors[cur_agent_num][0] = otherPos->id;
        rel_dist_neighbors[cur_agent_num][1] = sqrtf(powf(tmp_dist_x, 2) + powf(tmp_dist_y, 2) + powf(tmp_dist_z, 2));
        cur_agent_num++;
    }

    // TODO: If cur_agent_num <= 5, then, skip following operation, return command same as the previous control command
    // That means I need to save previous control command
    if (cur_agent_num <= 5) {
        control->motorRatios[0] = pre_control_command[0];
        control->motorRatios[1] = pre_control_command[1];
        control->motorRatios[2] = pre_control_command[2];
        control->motorRatios[3] = pre_control_command[3];
        return;
    }

    float swap_rel_dist[2] = {0};
    for (int di = 0; di < cur_agent_num - 1; di++){
        for (int dj = 0; dj < cur_agent_num - di - 1; dj++) {
            if (rel_dist_neighbors[dj][1] > rel_dist_neighbors[dj + 1][1]){
                swap_rel_dist[0] = rel_dist_neighbors[dj][0];
                swap_rel_dist[1] = rel_dist_neighbors[dj][1];

                rel_dist_neighbors[dj][0] = rel_dist_neighbors[dj + 1][0];
                rel_dist_neighbors[dj][1] = rel_dist_neighbors[dj + 1][1];

                rel_dist_neighbors[dj + 1][0] = swap_rel_dist[0];
                rel_dist_neighbors[dj + 1][1] = swap_rel_dist[1];
            }
        }
    }

	float neighbors_state_array[NEIGHBORS * NBR_OBS_DIM] = {0};
	if (true) {
        for (int k = 0; k < NEIGHBOR_NUM; k++) {
            int j = (int)rel_dist_neighbors[k][0];

            float dt = (pos_neighbors_curr[j][3] - pos_neighbors_prev[j][3]) / 1000.0; // dt in seconds

            deltaPoses[j][0] = pos_neighbors_curr[j][0] - pos_neighbors_prev[j][0];
            deltaPoses[j][1] = pos_neighbors_curr[j][1] - pos_neighbors_prev[j][1];
            deltaPoses[j][2] = pos_neighbors_curr[j][2] - pos_neighbors_prev[j][2];

            // get the relative positions
            pos_neighbors_rel[j][0] = pos_neighbors_curr[j][0] - state->position.x;
            pos_neighbors_rel[j][1] = pos_neighbors_curr[j][1] - state->position.y;
            pos_neighbors_rel[j][2] = pos_neighbors_curr[j][2] - state->position.z;

            // update the velocity estimate
            float vx = (deltaPoses[j][0] / dt) - state->velocity.x;
            float vy = (deltaPoses[j][1] / dt) - state->velocity.y;
            float vz = (deltaPoses[j][2] / dt) - state->velocity.z;

            vel_neighbors_rel[j][0] = exponentialFilter(vx, vel_neighbors_prev[j][0], weight);
            vel_neighbors_rel[j][1] = exponentialFilter(vy, vel_neighbors_prev[j][1], weight);
            vel_neighbors_rel[j][2] = exponentialFilter(vz, vel_neighbors_prev[j][2], weight);

            if (relXYZ) {
                // rotate neighbor pos and vel. Untested
                struct vec rot_neighbor_pos = mvmul(mtranspose(rot), mkvec(pos_neighbors_rel[j][0], pos_neighbors_rel[j][1], pos_neighbors_rel[j][2]));
                struct vec rot_neighbor_vel = mvmul(mtranspose(rot), mkvec(vel_neighbors_rel[j][0], vel_neighbors_rel[j][1], vel_neighbors_rel[j][2]));
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
            neighbors_state_array[k * NBR_OBS_DIM + 0] = pos_neighbors_rel[j][0];
            neighbors_state_array[k * NBR_OBS_DIM + 1] = pos_neighbors_rel[j][1];
            neighbors_state_array[k * NBR_OBS_DIM + 2] = pos_neighbors_rel[j][2];
            neighbors_state_array[k * NBR_OBS_DIM + 3] = vel_neighbors_rel[j][0];
            neighbors_state_array[k * NBR_OBS_DIM + 4] = vel_neighbors_rel[j][1];
            neighbors_state_array[k * NBR_OBS_DIM + 5] = vel_neighbors_rel[j][2];
        }
    }

    if(NEIGHBOR_AVOIDANCE){
        neighborEmbedder(neighbors_state_array);
    }

    if(OBSTACLE_AVOIDANCE){
    	float base_pos_x = (float) state->position.x;
    	float base_pos_y = (float) state->position.y;
        float obstacle_state_array[OBST_OBS_DIM];

        // get relative obstacle observations and feed it to the obstacle encoder
        for(int sdf_id = 0; sdf_id < 9; sdf_id++){
            float cur_pos_x = base_pos_x + obst_sdfs[sdf_id][0];
            float cur_pos_y = base_pos_y + obst_sdfs[sdf_id][1];
            float min_dist = 100.0;
            float tmp_dist = 0.0;
            for(int obst_id = 0; obst_id < OBST_NUM; obst_id++) {
                tmp_dist = sqrtf(powf(cur_pos_x - obst_pos[obst_id][0], 2) + powf(cur_pos_y - obst_pos[obst_id][1], 2));
                if(tmp_dist < min_dist){
                    min_dist = tmp_dist;
                }
            }
            obstacle_state_array[sdf_id] = min_dist;
        }
    }
    obstacleEmbedder(obstacle_state_array);

    singleHeadAttention();

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
PARAM_GROUP_STOP(ctrlNN)

LOG_GROUP_START(ctrlNN)
LOG_ADD(LOG_FLOAT, posX, &state_array[0])
LOG_ADD(LOG_FLOAT, posY, &state_array[1])
LOG_ADD(LOG_FLOAT, posZ, &state_array[2])
LOG_GROUP_STOP(ctrlNN)