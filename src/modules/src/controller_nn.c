
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
static float state_array[18];
// static float state_array[22];

// vars related to neighbor drone(s)
static float pos_neighbor_prev[3] = {10};
static float pos_neighbor_curr[3] = {10};
static float timestamp_prev = 0.0f;
static float timestamp_curr = 0.0f;
static float dt = 0.0f;
static float deltaPos[3];
static float vel_neighbor_rel[3] = {0};
static float neighbor_state_array[6];
static float maxPeerLocAgeMillis = 2000;

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


void controllerNN(control_t *control, 
				  setpoint_t *setpoint, 
				  const sensorData_t *sensors, 
				  const state_t *state, 
				  const uint32_t tick)
{
	control->enableDirectThrust = true;
	if (!RATE_DO_EXECUTE(/*RATE_100_HZ*/freq, tick)) {
		return;
	}

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
	// state_array[18] = control_n.thrust_0;
	// state_array[19] = control_n.thrust_1;
	// state_array[20] = control_n.thrust_2;
	// state_array[21] = control_n.thrust_3;

	//get neighbor pos/vel data and update every 0.01s
	//TODO: This only supports one other neighbor. Vectorize to support N neighbors
  TickType_t const time = xTaskGetTickCount();
  bool doAgeFilter = maxPeerLocAgeMillis >= 0;

  for (int i = 0; i < PEER_LOCALIZATION_MAX_NEIGHBORS; ++i) {

    peerLocalizationOtherPosition_t const *otherPos = peerLocalizationGetPositionByIdx(i);
    if (otherPos == NULL || otherPos->id == 0) {
        continue;
    }

    if (doAgeFilter && (time - otherPos->pos.timestamp > maxPeerLocAgeMillis)) {
        continue;
    }

    pos_neighbor_curr[0] = otherPos->pos.x;
    pos_neighbor_curr[1] = otherPos->pos.y;
    pos_neighbor_curr[2] = otherPos->pos.z;
    timestamp_curr = otherPos->pos.timestamp;

  }

  dt = timestamp_curr - timestamp_prev;
  if (dt > 10) { // update pos/vel every 100ms
    dt = dt / 1000.0; // convert to seconds
    deltaPos[0] = pos_neighbor_curr[0] - pos_neighbor_prev[0];
    deltaPos[1] = pos_neighbor_curr[1] - pos_neighbor_prev[1];
    deltaPos[2] = pos_neighbor_curr[2] - pos_neighbor_prev[2];

    // update the velocity estimate
    vel_neighbor_rel[0] = (deltaPos[0] / dt) - state_array[3];
    vel_neighbor_rel[1] = (deltaPos[1] / dt) - state_array[4];
    vel_neighbor_rel[2] = (deltaPos[2] / dt) - state_array[5];


    if (relXYZ) {
      // rotate neighbor pos and vel
      struct vec rot_neighbor_pos = mvmul(mtranspose(rot),
                                          mkvec(pos_neighbor_curr[0], pos_neighbor_curr[1], pos_neighbor_curr[2]));
      struct vec rot_neighbor_vel = mvmul(mtranspose(rot),
                                          mkvec(vel_neighbor_rel[0], vel_neighbor_rel[1], vel_neighbor_rel[2]));

      pos_neighbor_curr[0] = rot_neighbor_pos.x;
      pos_neighbor_curr[1] = rot_neighbor_pos.y;
      pos_neighbor_curr[2] = rot_neighbor_pos.z;

      vel_neighbor_rel[0] = rot_neighbor_vel.x;
      vel_neighbor_rel[1] = rot_neighbor_vel.y;
      vel_neighbor_rel[2] = rot_neighbor_vel.z;
    }

    pos_neighbor_prev[0] = pos_neighbor_curr[0];
    pos_neighbor_prev[1] = pos_neighbor_curr[1];
    pos_neighbor_prev[2] = pos_neighbor_curr[2];
    timestamp_prev = timestamp_curr;
  }

  neighbor_state_array[0] = pos_neighbor_curr[0];
  neighbor_state_array[1] = pos_neighbor_curr[1];
  neighbor_state_array[2] = pos_neighbor_curr[2];
  neighbor_state_array[3] = vel_neighbor_rel[0];
  neighbor_state_array[4] = vel_neighbor_rel[1];
  neighbor_state_array[5] = vel_neighbor_rel[2];



//  neighbor_state_array[0] = 1;
//  neighbor_state_array[1] = 0;
//  neighbor_state_array[2] = 0;
//  neighbor_state_array[3] = 0;
//  neighbor_state_array[4] = 0;
//  neighbor_state_array[5] = 0.5;

	// run the neural neural network
	uint64_t start = usecTimestamp();
//    networkEvaluate(&control_n, state_array);
  networkEvaluate(&control_n, state_array, neighbor_state_array); // with neighbor obs
	usec_eval = (uint32_t) (usecTimestamp() - start);

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
PARAM_ADD(PARAM_UINT16, freq, &freq)
PARAM_GROUP_STOP(ctrlNN)

LOG_GROUP_START(ctrlNN)
LOG_ADD(LOG_FLOAT, out0, &control_n.thrust_0)
LOG_ADD(LOG_FLOAT, out1, &control_n.thrust_1)
LOG_ADD(LOG_FLOAT, out2, &control_n.thrust_2)
LOG_ADD(LOG_FLOAT, out3, &control_n.thrust_3)

//LOG_ADD(LOG_FLOAT, in0, &state_array[0])
//LOG_ADD(LOG_FLOAT, in1, &state_array[1])
//LOG_ADD(LOG_FLOAT, in2, &state_array[2])
//
//LOG_ADD(LOG_FLOAT, in3, &state_array[3])
//LOG_ADD(LOG_FLOAT, in4, &state_array[4])
//LOG_ADD(LOG_FLOAT, in5, &state_array[5])
//
//LOG_ADD(LOG_FLOAT, in15, &state_array[15])
//LOG_ADD(LOG_FLOAT, in16, &state_array[16])
//LOG_ADD(LOG_FLOAT, in17, &state_array[17])
LOG_ADD(LOG_FLOAT, nPos0, &neighbor_state_array[0])
LOG_ADD(LOG_FLOAT, nPos1, &neighbor_state_array[1])
LOG_ADD(LOG_FLOAT, nPos2, &neighbor_state_array[2])
LOG_ADD(LOG_FLOAT, nVel0, &neighbor_state_array[3])
LOG_ADD(LOG_FLOAT, nVel1, &neighbor_state_array[4])
LOG_ADD(LOG_FLOAT, nVel2, &neighbor_state_array[5])
LOG_ADD(LOG_FLOAT, time, &timestamp_curr)

LOG_ADD(LOG_UINT32, usec_eval, &usec_eval)

LOG_GROUP_STOP(ctrlNN)