#include "includes.h"
#include "ptnFilter.h"

// PTn Low Pass filter
void ptnFilterInit(ptnFilter_axis_t *filter, uint8_t order, float f_cut) {
	// AdjCutHz = CutHz /(sqrtf(powf(2, 1/Order) -1))
	const float ScaleF[] = { 1.0f, 1.553773974f, 1.961459177f, 2.298959223f };
	int n;
	float Adj_f_cut;
	filter->order = order;
	for (n = 1; n <= filter->order; n++)
		filter->state[n] = 0.0f;
	Adj_f_cut = (float)f_cut * ScaleF[filter->order - 1];
	filter->k = DT / ((1.0f / (2.0f * M_PI_FLOAT * Adj_f_cut)) + DT);
} // ptnFilterInit

void ptnFilterUpdate(float f_cut, ptnFilter_axis_t *filter, float ScaleF) {
  float Adj_f_cut;
  Adj_f_cut = (float)f_cut * ScaleF;
  filter->k = DT / ((1.0f / (2.0f * M_PI_FLOAT * Adj_f_cut)) + DT);
}

float ptnFilterApply(float input, ptnFilter_axis_t *filter) {
int n;
	filter->state[0] = input;
	for (n = 1; n <= filter->order; n++)
		filter->state[n] += (filter->state[n - 1] - filter->state[n])
				* filter->k;
	return filter->state[filter->order];
} // ptnFilterApply
