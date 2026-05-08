#include "includes.h"
#include "gyro.h"
#include "kalman.h"
#include "filter.h"
#include <math.h>

variance_t varStruct;
kalman_t   kalmanFilterStateRate[3];

void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.0001f;      //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
    filter->e = 1.0f;
    
    // Initialize PT1 filter for Kalman gain smoothing
    // 50 Hz cutoff with 32kHz sample rate (REFRESH_RATE = 1/32000 = 0.00003125s)
    pt1FilterInit(&filter->kFilter, pt1FilterGain(50, REFRESH_RATE), 0.0f);
}

void kalman_init(void)
{
    setPointNew = 0;
    memset(&varStruct, 0, sizeof(varStruct));
    init_kalman(&kalmanFilterStateRate[ROLL], filterConfig.roll_q);
    init_kalman(&kalmanFilterStateRate[PITCH], filterConfig.pitch_q);
    init_kalman(&kalmanFilterStateRate[YAW], filterConfig.yaw_q);
    varStruct.inverseN = 1.0f/filterConfig.w;
}

#pragma GCC push_options
#pragma GCC optimize("O3")
void update_kalman_covariance(volatile axisData_t *gyroRateData)
{
    varStruct.xWindow[ varStruct.windex] = gyroRateData->x;
    varStruct.yWindow[ varStruct.windex] = gyroRateData->y;
    varStruct.zWindow[ varStruct.windex] = gyroRateData->z;

    varStruct.xSumMean +=  varStruct.xWindow[ varStruct.windex];
    varStruct.ySumMean +=  varStruct.yWindow[ varStruct.windex];
    varStruct.zSumMean +=  varStruct.zWindow[ varStruct.windex];

    // calc varianceElement
    float xvarianceElement = varStruct.xWindow[ varStruct.windex] - varStruct.xMean;
    float yvarianceElement = varStruct.yWindow[ varStruct.windex] - varStruct.yMean;
    float zvarianceElement = varStruct.zWindow[ varStruct.windex] - varStruct.zMean;

    xvarianceElement = xvarianceElement * xvarianceElement;
    yvarianceElement = yvarianceElement * yvarianceElement;
    zvarianceElement = zvarianceElement * zvarianceElement;

    varStruct.xSumVar += xvarianceElement;
    varStruct.ySumVar += yvarianceElement;
    varStruct.zSumVar += zvarianceElement;

    varStruct.xvarianceWindow[varStruct.windex] = xvarianceElement;
    varStruct.yvarianceWindow[varStruct.windex] = yvarianceElement;
    varStruct.zvarianceWindow[varStruct.windex] = zvarianceElement;

    varStruct.windex++;
    if ( varStruct.windex > filterConfig.w)
    {
         varStruct.windex = 0;
    }

    varStruct.xSumMean -=  varStruct.xWindow[ varStruct.windex];
    varStruct.ySumMean -=  varStruct.yWindow[ varStruct.windex];
    varStruct.zSumMean -=  varStruct.zWindow[ varStruct.windex];
    varStruct.xSumVar -=   varStruct.xvarianceWindow[varStruct.windex];
    varStruct.ySumVar -=   varStruct.yvarianceWindow[varStruct.windex];
    varStruct.zSumVar -=   varStruct.zvarianceWindow[varStruct.windex];

    //New mean
    varStruct.xMean =  varStruct.xSumMean *  varStruct.inverseN;
    varStruct.yMean =  varStruct.ySumMean *  varStruct.inverseN;
    varStruct.zMean =  varStruct.zSumMean *  varStruct.inverseN;
    varStruct.xVar =   varStruct.xSumVar *  varStruct.inverseN;
    varStruct.yVar =   varStruct.ySumVar *  varStruct.inverseN;
    varStruct.zVar =   varStruct.zSumVar *  varStruct.inverseN;

    float squirt;
    arm_sqrt_f32(varStruct.xVar, &squirt);
    kalmanFilterStateRate[ROLL].r = squirt * VARIANCE_SCALE;
    arm_sqrt_f32(varStruct.yVar, &squirt);
    kalmanFilterStateRate[PITCH].r = squirt * VARIANCE_SCALE;
    arm_sqrt_f32(varStruct.zVar, &squirt);
    kalmanFilterStateRate[YAW].r = squirt * VARIANCE_SCALE;
}

inline float kalman_process(kalman_t* kalmanState, volatile float input) {

  //project the state ahead using acceleration
  kalmanState->x += (kalmanState->x - kalmanState->lastX) * kalmanState->k;

  //figure out how much to boost or reduce our error in the estimate based on setpoint target.
  //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
  //update last state
  kalmanState->lastX = kalmanState->x;

  float e = CONSTRAIN(kalmanState->r / 45.0f + 0.005f, 0.005f, 0.9f);
  //make the 1 a configurable value for testing purposes
  e = -SQUARE(e - 1.0f) * 0.7f + (e - 1.0f) * (1.0f - 0.7f) + 1.0f;
  kalmanState->e = e;

  kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

  //measurement update
  kalmanState->k = kalmanState->p / (kalmanState->p + 10.0f);
  kalmanState->k = pt1FilterApply(&kalmanState->kFilter, kalmanState->k);
  kalmanState->x += kalmanState->k * (input - kalmanState->x);
  kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
  return kalmanState->x;
}

void kalman_update(volatile axisData_t* input, filteredData_t* output)
{
    output->rateData.x = kalman_process(&kalmanFilterStateRate[ROLL], input->x);
    output->rateData.y = kalman_process(&kalmanFilterStateRate[PITCH], input->y);
    output->rateData.z = kalman_process(&kalmanFilterStateRate[YAW], input->z);
}
#pragma GCC pop_options
