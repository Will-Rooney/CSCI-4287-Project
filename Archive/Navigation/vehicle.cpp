/*
* vehicle.cpp
*
* Author(s): Will Rooney
*
* Date Created: 04/18/2017
* Last Modified: 04/18/2017
* Last Modified By: Will Rooney
*
* Description: Implementation of vehicle.h - INCOMPLETE
*
*/

#include "vehicle.h"

void Vehicle::applyForce(Vector2f force) {
	// apply force current acceleration to get desired acceleration
	Vector2f desired_accel = a + force;

	// Use desired acceleration to control motors; a = current acceleration
	// TODO - MOTOR CONTROL
	/* e.g., 
	*	calculate appropriate speed for each motor to match the desired acceleration: speedRight & speedLeft
	*		analogWrite(mtrRightPin, speedRight)
	*		analogWrite(mtrLeftPin, speedLeft)
	*/
}

bool Vehicle::seek() {
	Vector2f desired = target - pos;
	float d = desired.magnitude();

	// First check fwd_d_ultra & fwd_d_laser and determine if the path is blocked
		// if (blocked) return false

	// Next check left_d & right_d to see if we are getting too close to a wall
		// TODO - Addjust 'desired' to center itself inbetween walls
	float threshold = 0.5; // How close we can get (in cm) to the wall before steering away
	if (left_d < threshold) {
		// adjust to the right based on: right_d - left_d
	}
	else if (right_d < threshold) {
		// adjust to the left based on: left_d - right_d
	}
	else if (fwd_d_laser < d - threshold)

	// Gradually slow down if approaching target
	if (d < 20) { // May need to modify so that we don't have to come to a complete stop each time we reach the next node
		float m = 1.0 * (maxSpeed / 20) * d;
		float len = desired.length();
		desired *= m / len;
	}
	else { // Full speed ahead
		// Normalize desired and scale to max speed
		desired.normalize();
		desired.multiply(maxSpeed);
	}

	// Steering = desired - velocity
	Vector2f steer = desired - v;
	steer.limit(maxForce);

	applyForce(steer); // Set motor speeds to adjust to new acceleration vector
	return true; // Path is not blocked so far, return true
}

void Vehicle::updateTarget(float x, float y) {
	target.set(x, y);
}

void Vehicle::updatePos(float x, float y) {
	pos.set(x, y);
}

void Vehicle::updateV(float vx, float vy) {
	v.set(vx, vy);
}

void Vehicle::updateA(float ax, float ay) {
	a.set(ax, ay);
}

void Vehicle::updateDist(float left, float right, float fwd_ultra, float fwd_laser) {
	left_d = left;
	right_d = right;
	fwd_d_ultra = fwd_ultra;
	fwd_d_laser = fwd_laser;
}