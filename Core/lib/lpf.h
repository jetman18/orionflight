#ifndef LPF_H
#define LPF_H

#ifdef __cplusplus
extern "C" {
#endif


#include"maths.h"
#include"math.h"
// 1oder

typedef struct{
    float a;
    float b;
}lpf_var;

static inline float pt1FilterGain(float f_cut, float dT){
    float RC = 1 / (2 * M_PIf * f_cut);
    return dT / (RC + dT);
}

static inline float pt1FilterApply( float input,float f_cut,float dT)
{
	static float kk;
	float RC = 1.0f / (2 *M_PIf * f_cut);
    float gain_k =(float)dT / (RC + dT);
    kk = kk + gain_k*(input - kk);
    return kk;
}

static inline float pt2FilterGain(float f_cut, float dT)
{
    const float order = 2.0f;
    const float orderCutoffCorrection = 1 / sqrtf(powf(2, 1.0f / order) - 1);
    float RC = 1 / (2 * orderCutoffCorrection * M_PIf * f_cut);
    // float RC = 1 / (2 * 1.553773974f * M_PIf * f_cut);
    // where 1.553773974 = 1 / sqrt( (2^(1 / order) - 1) ) and order is 2
    return dT / (RC + dT);
}

static inline float pt2FilterApply(float input,float f_cut,float dT)
{
    float k_gain = pt2FilterGain(f_cut,dT);
    static float vl1,vl2;
    vl1 = vl1 + k_gain * (input - vl1);
    vl2 = vl2 + k_gain * (vl1 - vl2);
    return vl2;
}


#ifdef __cplusplus
}
#endif
#endif
