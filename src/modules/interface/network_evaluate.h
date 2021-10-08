#ifndef __NETWORK_EVALUATE_H__
#define __NETWORK_EVALUATE_H__

#include <math.h>
#include <stdbool.h>
#include "debug.h"

#define NEIGHBORS 1
#define NUM_OBS 6
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
void neighborEmbeddings(const float neighbor_array[NEIGHBORS][NUM_OBS]);

#endif