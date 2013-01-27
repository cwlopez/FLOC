#include "AP_Math.h"

// a varient of asin() that checks the input ranges and ensures a
// valid angle as output. If nan is given as input then zero is
// returned.
float safe_asin(float v)
{
    if (isnan(v)) {
        return 0.0;
    }
    if (v >= 1.0) {
        return PI/2;
    }
    if (v <= -1.0) {
        return -PI/2;
    }
    return asin(v);
}

// a varient of sqrt() that checks the input ranges and ensures a
// valid value as output. If a negative number is given then 0 is
// returned. The reasoning is that a negative number for sqrt() in our
// code is usually caused by small numerical rounding errors, so the
// real input should have been zero
float safe_sqrt(float v)
{
    float ret = sqrt(v);
    if (isnan(ret)) {
        return 0;
    }
    return ret;
}
//ADDED------------------------------------------------------------------------------------------
//an approximation of the sqrt function reported to be ~75% faster with 99% of the sqrt precision
//see www.codeproject.com//Articles/69941/Best-Square-Root-Method-Algorithm-Function_Precisi
/*#define SQRT_MAGIC_F 0x5f3759df
float safe_fsqrt(float v)
{
	const float xhalf = 0.5f*x;
	union
	{
		float x;
		int i;
	} u;
	u.x = x;
	u.i = SQRT_MAGIC_F - (u.i >> 1);
	
	return x*u.x*(1.5f-xhalf*u.x*u.x);
}
//----------------------------------------------------------------------------------------------
*/
// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
// Note that this relies the set of rotations being complete. The
// optional 'found' parameter is for the test suite to ensure that it is.
enum Rotation rotation_combination(enum Rotation r1, enum Rotation r2, bool *found)
{
    Vector3f tv1, tv2;
    enum Rotation r;
    tv1(1,2,3);
    tv1.rotate(r1);
    tv1.rotate(r2);

    for (r=ROTATION_NONE; r<ROTATION_MAX;
         r = (enum Rotation)((uint8_t)r+1)) {
        Vector3f diff;
        tv2(1,2,3);
        tv2.rotate(r);
        diff = tv1 - tv2;
        if (diff.length() < 1.0e-6) {
            // we found a match
            if (found) {
                *found = true;
            }
            return r;
        }
    }

    // we found no matching rotation. Someone has edited the
    // rotations list and broken its completeness property ...
    if (found) {
        *found = false;
    }
    return ROTATION_NONE;
}
