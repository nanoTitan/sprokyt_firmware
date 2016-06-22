#ifndef _MATH_EXT_H_
#define _MATH_EXT_H_

#include "math.h"

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

#endif // _MATH_EXT_H_
