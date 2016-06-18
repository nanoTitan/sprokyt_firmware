#include "math.h"

float minf(float x, float y)
{
	if (y < x)
		return y;
	return x;
}

int min(int x, int y)
{
	if (y < x)
		return y;
	return x;
}

float maxf(float x, float y)
{
	if (y > x)
		return y;
	return x;
}

int max(int x, int y)
{
	if (y > x)
		return y;
	return x;
}

float clampf(float x, float min, float max)
{
	if (x < min)	return min;
	if (x > max)	return max;
	return x;
}

int clamp(int x, int min, int max)
{
	if (x < min)	return min;
	if (x > max)	return max;
	return x;
}
