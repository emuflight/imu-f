#include "includes.h"
#include "gyro.h"
#include "filter.h"
#include "kalman.h"
#include "ptnFilter.h"

volatile filter_config_t filterConfig =
{
	DEFAULT_ROLL_Q,
	DEFAULT_PITCH_Q,
	DEFAULT_YAW_Q,
	MIN_WINDOW_SIZE,

	(float)DEFAULT_ROLL_Q,
	(float)DEFAULT_PITCH_Q,
	(float)DEFAULT_YAW_Q,

	(float)BASE_LPF_HZ,
	(float)BASE_LPF_HZ,
	(float)BASE_LPF_HZ,

	40.0f,

	BASE_LPF_HZ,
	BASE_LPF_HZ,
	BASE_LPF_HZ,

	2500,
	2,
	1,
	10,
	500,
};

// PT1 Low Pass filter
bool acc_filter_initialized = false;
typedef struct pt1Filter_s {
    float state;
    float k;
} pt1Filter_t;

pt1Filter_t ax_filter;
pt1Filter_t ay_filter;
pt1Filter_t az_filter;

pt1Filter_t frequencyChangeFilterX;
pt1Filter_t frequencyChangeFilterY;
pt1Filter_t frequencyChangeFilterZ;

float pt1FilterGain(uint16_t f_cut, float dT);
void  pt1FilterInit(pt1Filter_t *filter, float k, float val);
float pt1FilterApply(pt1Filter_t *filter, float input);

ptnFilter_t lpfFilterStateRate;
volatile uint32_t setPointNew;
volatile axisDataInt_t setPointInt;
volatile axisData_t oldSetPoint;
volatile axisData_t setPoint;
volatile int allowFilterInit = 1;

float sharpness;
float ptnScale;

void allow_filter_init(void)
{
	allowFilterInit = 1;
}

void ptnFilter_init(float freq, ptnFilter_axis_t *filterState)
{
	ptnFilterInit(freq, filterState, filterConfig.ptX);
}

void filter_init(void)
{
//#define ACC_CUTOFF    (60.0f)
#define ACC_READ_RATE (1.0f / 1000.0f)

	memset((uint32_t *)&setPoint, 0, sizeof(axisData_t));
	memset((uint32_t *)&oldSetPoint, 0, sizeof(axisData_t));
	memset((uint32_t *)&setPointInt, 0, sizeof(axisDataInt_t));
	kalman_init();
	ptnFilter_init(filterConfig.base_roll_lpf_hz, &(lpfFilterStateRate.x));
	ptnFilter_init(filterConfig.base_pitch_lpf_hz, &(lpfFilterStateRate.y));
	ptnFilter_init(filterConfig.base_yaw_lpf_hz, &(lpfFilterStateRate.z));

	// set imuf acc cutoff frequency
	const float k = pt1FilterGain((float)filterConfig.acc_lpf_hz, ACC_READ_RATE);
	pt1FilterInit(&ax_filter, k, 0.0f);
	pt1FilterInit(&ay_filter, k, 0.0f);
	pt1FilterInit(&az_filter, k, 0.0f);

	pt1FilterInit(&frequencyChangeFilterX, pt1FilterGain(20, REFRESH_RATE), 0.0f);
	pt1FilterInit(&frequencyChangeFilterY, pt1FilterGain(20, REFRESH_RATE), 0.0f);
	pt1FilterInit(&frequencyChangeFilterZ, pt1FilterGain(20, REFRESH_RATE), 0.0f);

	sharpness = (float)filterConfig.sharpness / 15000.0f;
	switch (filterConfig.ptX) {
		case 1:
			ptnScale = 1.0f;
		case 2:
			ptnScale = 1.553773974f;
		case 3:
			ptnScale = 1.961459177f;
		case 4:
			ptnScale = 2.298959223f;
	}
}

float errorMultiplierX, errorMultiplierY, errorMultiplierZ;
float errorX, errorY, errorZ;
float setpointGyroRatioX, setpointGyroRatioY, setpointGyroRatioZ;
uint16_t oldRollHz, oldPitchHz, oldYawHz;

void filter_data(volatile axisData_t *gyroRateData, volatile axisData_t *gyroAccData, float gyroTempData, filteredData_t *filteredData)
{
	if (allowFilterInit)
	{
		allowFilterInit = 0;
		//convert the ints to floats
		filterConfig.roll_q = (float)filterConfig.i_roll_q;
		filterConfig.pitch_q = (float)filterConfig.i_pitch_q;
		filterConfig.yaw_q = (float)filterConfig.i_yaw_q;
		filter_init();
	}

	if (setPointNew)
	{
		memcpy((uint32_t *)&setPoint, (uint32_t *)&setPointInt, sizeof(axisData_t));
	}

	kalman_update(gyroRateData, filteredData);

	filteredData->rateData.x = ptnFilterApply(filteredData->rateData.x, &(lpfFilterStateRate.x));
	filteredData->rateData.y = ptnFilterApply(filteredData->rateData.y, &(lpfFilterStateRate.y));
	filteredData->rateData.z = ptnFilterApply(filteredData->rateData.z, &(lpfFilterStateRate.z));

	errorX = fabsf(setPoint.x - filteredData->rateData.x) * sharpness;
	errorY = fabsf(setPoint.y - filteredData->rateData.y) * sharpness;
	errorZ = fabsf(setPoint.z - filteredData->rateData.z) * sharpness;

	errorMultiplierX = CONSTRAIN(1 + fabsf(setPoint.x - filteredData->rateData.x) * sharpness, 1.0f, 10.0f);
	errorMultiplierY = CONSTRAIN(1 + fabsf(setPoint.y - filteredData->rateData.y) * sharpness, 1.0f, 10.0f);
	errorMultiplierZ = CONSTRAIN(1 + fabsf(setPoint.z - filteredData->rateData.z) * sharpness, 1.0f, 10.0f);

	setpointGyroRatioX = (100.0f + fabsf(setPoint.x)) / (100.0f + fabsf(filteredData->rateData.x));
	if (fabsf(setPoint.x) < fabsf(filteredData->rateData.x))
	{
		setpointGyroRatioX = 1.0f / setpointGyroRatioX;
	}

	setpointGyroRatioY = (100.0f + fabsf(setPoint.y)) / (100.0f + fabsf(filteredData->rateData.y));
	if (fabsf(setPoint.y) < fabsf(filteredData->rateData.y))
	{
		setpointGyroRatioY = 1.0f / setpointGyroRatioY;
	}

	setpointGyroRatioZ = (100.0f + fabsf(setPoint.z)) / (100.0f + fabsf(filteredData->rateData.z));
	if (fabsf(setPoint.z) < fabsf(filteredData->rateData.z))
	{
		setpointGyroRatioZ = 1.0f / setpointGyroRatioZ;
	}

	switch (filterConfig.dynamicType) {
		case 0:
		filterConfig.roll_lpf_hz = filterConfig.base_roll_lpf_hz;
		filterConfig.pitch_lpf_hz = filterConfig.base_pitch_lpf_hz;
		filterConfig.yaw_lpf_hz = filterConfig.base_yaw_lpf_hz;
		break;
		case 1:

		// give a boost to the setpoint, used to caluclate the filter cutoff, based on the error and setpoint/gyrodata

			errorMultiplierX = CONSTRAIN(errorX * fabsf(1.0f - (setPoint.x / filteredData->rateData.x)) + 1.0f, 1.0f, 10.0f);
			errorMultiplierY = CONSTRAIN(errorY * fabsf(1.0f - (setPoint.y / filteredData->rateData.y)) + 1.0f, 1.0f, 10.0f);
			errorMultiplierZ = CONSTRAIN(errorZ * fabsf(1.0f - (setPoint.z / filteredData->rateData.z)) + 1.0f, 1.0f, 10.0f);


			if (setPointNew)
			{
				setPointNew = 0;
				if (setPoint.x != 0.0f && oldSetPoint.x != setPoint.x)
				{
					filterConfig.roll_lpf_hz = CONSTRAIN(filterConfig.base_roll_lpf_hz * fabsf(1.0f - ((setPoint.x * errorMultiplierX) / filteredData->rateData.x)), filterConfig.dynamicMin, filterConfig.dynamicMax);
				}
				if (setPoint.y != 0.0f && oldSetPoint.y != setPoint.y)
				{
					filterConfig.pitch_lpf_hz = CONSTRAIN(filterConfig.base_pitch_lpf_hz * fabsf(1.0f - ((setPoint.y * errorMultiplierY) / filteredData->rateData.y)), filterConfig.dynamicMin, filterConfig.dynamicMax);
				}
				if (setPoint.z != 0.0f && oldSetPoint.z != setPoint.z)
				{
					filterConfig.yaw_lpf_hz = CONSTRAIN(filterConfig.base_yaw_lpf_hz * fabsf(1.0f - ((setPoint.z * errorMultiplierZ) / filteredData->rateData.z)), filterConfig.dynamicMin, filterConfig.dynamicMax);
				}
				memcpy((uint32_t *)&oldSetPoint, (uint32_t *)&setPoint, sizeof(axisData_t));
			}
		break;
		case 2:

			filterConfig.roll_lpf_hz = CONSTRAIN(filterConfig.base_roll_lpf_hz * fabsf((1.0f - setpointGyroRatioX) * errorMultiplierX), filterConfig.dynamicMin, filterConfig.dynamicMax);

			filterConfig.pitch_lpf_hz = CONSTRAIN(filterConfig.base_pitch_lpf_hz * fabsf((1.0f - setpointGyroRatioY) * errorMultiplierY), filterConfig.dynamicMin, filterConfig.dynamicMax);

			filterConfig.yaw_lpf_hz = CONSTRAIN(filterConfig.base_yaw_lpf_hz * fabsf((1.0f - setpointGyroRatioY) * errorMultiplierY), filterConfig.dynamicMin, filterConfig.dynamicMax);
		break;
		case 3:

			filterConfig.roll_lpf_hz = CONSTRAIN(errorX + filterConfig.base_roll_lpf_hz + fabsf(filteredData->rateData.x / 5), filterConfig.dynamicMin, filterConfig.dynamicMax);

			filterConfig.pitch_lpf_hz = CONSTRAIN(errorY + filterConfig.base_pitch_lpf_hz + fabsf(filteredData->rateData.y / 5), filterConfig.dynamicMin, filterConfig.dynamicMax);

			filterConfig.yaw_lpf_hz = CONSTRAIN(errorZ + filterConfig.base_yaw_lpf_hz + fabsf(filteredData->rateData.z / 5), filterConfig.dynamicMin, filterConfig.dynamicMax);
		break;
		case 4:

			// X AXIS
			if (setPoint.x != 0.0f && oldSetPoint.x != setPoint.x)
			{
				filterConfig.roll_lpf_hz = CONSTRAIN(filterConfig.base_roll_lpf_hz * fabsf((1.0f - setpointGyroRatioX) * errorMultiplierX), filterConfig.dynamicMin, filterConfig.dynamicMax);
			}

			// Y AXIS
			if (setPoint.y != 0.0f && oldSetPoint.y != setPoint.y)
			{
				filterConfig.pitch_lpf_hz = CONSTRAIN(filterConfig.base_pitch_lpf_hz * fabsf((1.0f - setpointGyroRatioY) * errorMultiplierY), filterConfig.dynamicMin, filterConfig.dynamicMax);
			}

			// Z AXIS
			if (setPoint.z != 0.0f && oldSetPoint.z != setPoint.z)
			{
			filterConfig.yaw_lpf_hz = CONSTRAIN(filterConfig.base_yaw_lpf_hz * fabsf((1.0f - setpointGyroRatioY) * errorMultiplierY), filterConfig.dynamicMin, filterConfig.dynamicMax);
			}
		break;
		case 5:

			filterConfig.roll_lpf_hz = pt1FilterApply(&frequencyChangeFilterX, CONSTRAIN(filterConfig.base_roll_lpf_hz * fabsf((1.0f - setpointGyroRatioX) * errorMultiplierX), filterConfig.dynamicMin, filterConfig.dynamicMax));

			filterConfig.pitch_lpf_hz = pt1FilterApply(&frequencyChangeFilterX, CONSTRAIN(filterConfig.base_pitch_lpf_hz * fabsf((1.0f - setpointGyroRatioY) * errorMultiplierY), filterConfig.dynamicMin, filterConfig.dynamicMax));

			filterConfig.yaw_lpf_hz = pt1FilterApply(&frequencyChangeFilterX, CONSTRAIN(filterConfig.base_yaw_lpf_hz * fabsf((1.0f - setpointGyroRatioY) * errorMultiplierY), filterConfig.dynamicMin, filterConfig.dynamicMax));

		break;
	}

if (oldRollHz > (filterConfig.roll_lpf_hz + 2.5f) || oldRollHz < (filterConfig.roll_lpf_hz - 2.5f))
{
	ptnFilterUpdate(filterConfig.roll_lpf_hz, &(lpfFilterStateRate.x), ptnScale);
	oldRollHz = filterConfig.roll_lpf_hz;
}

if (oldPitchHz > (filterConfig.pitch_lpf_hz + 2.5f) || oldPitchHz < (filterConfig.pitch_lpf_hz - 2.5f))
{
	ptnFilterUpdate(filterConfig.pitch_lpf_hz, &(lpfFilterStateRate.y), ptnScale);
	oldPitchHz = filterConfig.pitch_lpf_hz;
}

if (oldYawHz > (filterConfig.yaw_lpf_hz + 2.5f) || oldYawHz < (filterConfig.yaw_lpf_hz - 2.5f))
{
	ptnFilterUpdate(filterConfig.yaw_lpf_hz, &(lpfFilterStateRate.y), ptnScale);
	oldYawHz = filterConfig.yaw_lpf_hz;
}

	//no need to filter ACC is used in quaternions
	filteredData->accData.x = gyroAccData->x;
	filteredData->accData.y = gyroAccData->y;
	filteredData->accData.z = gyroAccData->z;

	//should filter this
	filteredData->tempC = gyroTempData;
}

float pt1FilterGain(uint16_t f_cut, float dT)
{
    float RC = 1 / ( 2 * M_PI_FLOAT * f_cut);
    return dT / (RC + dT);
}

void pt1FilterInit(pt1Filter_t *filter, float k, float val)
{
    filter->state = val;
    filter->k = k;
}

float pt1FilterApply(pt1Filter_t *filter, float input)
{
    filter->state = filter->state + filter->k * (input - filter->state);
    return filter->state;
}

void filter_acc(volatile axisData_t *gyroAccData)
{
	if (!acc_filter_initialized)
	{
		acc_filter_initialized = true;
		ax_filter.state = gyroAccData->x;
		ay_filter.state = gyroAccData->y;
		az_filter.state = gyroAccData->z;
	}
	else
	{
		 gyroAccData->x = pt1FilterApply(&ax_filter,  gyroAccData->x);
		 gyroAccData->y = pt1FilterApply(&ay_filter,  gyroAccData->y);
		 gyroAccData->z = pt1FilterApply(&az_filter,  gyroAccData->z);
	}
}
