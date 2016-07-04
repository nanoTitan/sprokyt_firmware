#ifndef _MATH_EXT_H_
#define _MATH_EXT_H_

#include "math.h"
#include <stdint.h>

#define M_1_255  0.003921568627451
#define M_180_PI 57.29577951308233

typedef struct 
{
	float x;
	float y;
	float z;
} EulerAngle_t;

typedef struct 
{
	float w;
	float x;
	float y;
	float z;
} Quaternion_t;

static float clampf(float x, float min, float max)
{
	if (x < min)	return min;
	if (x > max)	return max;
	return x;
}

static int clamp(int x, int min, int max)
{
	if (x < min)	return min;
	if (x > max)	return max;
	return x;
}

static float RadiansToDeg(float radians)
{
	return radians * M_180_PI;
}

static float map(uint8_t x, uint8_t in_min, uint8_t in_max, float out_min, uint8_t out_max)
{
	if (x <= in_min)
		return out_min;
	
	if (x >= in_max)
		return out_max;
	
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

#endif // _MATH_EXT_H_
