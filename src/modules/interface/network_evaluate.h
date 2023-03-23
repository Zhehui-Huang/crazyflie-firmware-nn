#ifndef __NETWORK_EVALUATE_H__
#define __NETWORK_EVALUATE_H__

#include <math.h>
#include <stdbool.h>
#include "debug.h"

#define NEIGHBORS 0
#define NBR_OBS_DIM 6
#define OBST_OBS_DIM 4  // ignore this if you are using a multi-agent policy without obstacle encoder or just a single agent policy
#define K_OBSTACLES 2  // ignore this if you are using a multi-agent policy without obstacle encoder or just a single agent policy
#define NUM_IDS 100 // Number of unique cfids

/*
 * since the network outputs thrust on each motor,
 * we need to define a struct which stores the values
*/
typedef struct control_t_n {
	float thrust_0; 
	float thrust_1;
	float thrust_2;
	float thrust_3;	
} control_t_n;

void networkEvaluate(control_t_n *control_n, const float *state_array);
void neighborEmbeddings(const float neighbor_array[NEIGHBORS][NBR_OBS_DIM]);
void obstacleEmbeddings(const float obstacles_array[K_OBSTACLES][OBST_OBS_DIM]);

#endif
