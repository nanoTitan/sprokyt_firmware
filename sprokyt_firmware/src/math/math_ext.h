#ifndef _MATH_EXT_H_
#define _MATH_EXT_H_

#define OneOver255 0.0039215f

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

#endif // _MATH_EXT_H_
