/*
 * ukf.h
 *
 *  Created on: May 1, 2025
 *      Author: qiaomein
 */

#ifndef INC_UKF_H_
#define INC_UKF_H_


#include "arm_math.h"


// DEFINE MOTION MODEL AND SENSOR MODEL
#define N_STATES 5U

// DEFINE UKF TUNING PARAMETERS
#define ALPHA .001f
#define BETA 2.0f
#define KAPPA (3-N_STATES)


#endif /* INC_UKF_H_ */
