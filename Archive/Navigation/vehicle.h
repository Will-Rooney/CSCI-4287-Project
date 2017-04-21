/*
* vehicle.h
*
* Author(s): Will Rooney
*
* Date Created: 04/18/2017
* Last Modified: 04/18/2017
* Last Modified By: Will Rooney
*
* Description: Class definition for vehicle.
*	Vehicle in this sense is the autonomous car and provides functionality
*	to get/set position coordinates and to control the car motors based on desired position,
*	velocity, acceleration, orientation, and distance sensing.
*
*
*	Behavior Based on Craig Reynolds's "Steering Behaviors For Autonomous Characters": http://www.red3d.com/cwr/steer/gdc99/
*		Explanation and examples: The Coding Train https://www.youtube.com/watch?v=JIz2L4tn5kM
*	1) Action/Selection: Move from 'current' Node to 'next' Node
*	2) Steering: Force application to alter desired steering behavior
*		Steering = desired - velocity
*	3) Locomotion: Motor control
*
*  
* Magician Chassis:
*	Vehicle Specs:
*		• Max Motor Voltage: 6VDC
*		• No Load Speed: 90A +- 10rpm
*		• No Load Current: 190mA (max 250mA)
*		• Torque: 800 gf.cm
*		• Stall Current: ~1A
*		• 65mm Diameter wheels (30mm Wide)
*	Vehicle Docs:
*		• Datasheet: http://cdn.sparkfun.com/datasheets/Robotics/DG01D.pdf
*		• Assembly Instructions: http://cdn.sparkfun.com/datasheets/Robotics/MagicianChassisInst.pdf
*		• Product Video: https://www.sparkfun.com/videos#all/zbFTi1kl-e4/7
*
*/

#ifndef VEHICLE_H
#define VEHICLE_H

#include "../MathLibrary/Vector2.h"

/* Arduion.h to provide Arduino Library functions */
#if defined(ARDUINO) && ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

/* Motor IO Pins - TODO */
#define mtrRightPin 5
#define mtrLeftPin	6

/* Motor speeds - TODO */
#define mtrMin		0
#define mtrMax		0

#define	mtrSlow		0
#define	mtrMedium	0
#define	mtrFast		0
#define	mtrStopped	0

/*	Motor direction */
#define	dirMtrLeftFwd	1
#define	dirMtrLeftBwd	0

#define	dirMtrRightFwd	0
#define	dirMtrRightBwd	1

class Vehicle {
public:
	Vehicle() : pos(0.0, 0.0), v(0.0, 0.0), a(0.0, 0.0), maxSpeed(100), maxForce(0.1) {}
	Vehicle(float x, float y) : pos(x, y), v(0.0, 0.0), a(0.0, 0.0), maxSpeed(100), maxForce(0.1) {}

	void updatePos(float x, float y);
	void updateV(float vx, float vy);
	void updateA(float ax, float ay);
	void updateDist(float left, float right, float fwd_ultra, float fwd_laser);
	void updateTarget(float x, float y);

	void applyForce(Vector2f force); // Motor Control Here
	bool seek(); // returns false if path is blocked, else returns true


	/* Might not use:
	 * All motor control could be implemented in the applyForce() func
	 */
	/*
	void forward();					// Move straight forward; need a way to keep track of distance moved
	void backward();				// Move straight backward; need a way to keep track of distance moved
	
	void pivotRight();				// Stop then Pivot right 90 degrees
	void pivotRight(float deg);		// Stop then Pivot right 0-360 degrees
	void pivotLeft();				// Stop then Pivot left 90 degrees
	void pivotLeft(float deg);		// Stop then Pivot left 0-360 degrees
	
	void arcRight();				// Stop then Arc right 90 degrees; update (x, y)
	void arcRight(float deg);		// Stop then Arc right 0-360 degrees; update (x, y)
	void arcLeft();					// Stop then Arc left 90 degrees; update (x, y)
	void arcLeft(float deg);		// Stop then Arc left 0-360 degrees; update (x, y)

	void turnAround();				// Pivot 180 degrees

	void stop();					// Stop immediately
	*/

private:
	Vector2f pos, v, a, target;
	float maxSpeed, maxForce; // Testing will be needed to hone these values
	float r; // max distance to wall when centered; may not need this, we only care if we get too close to a wall
	float left_d, right_d, fwd_d_ultra, fwd_d_laser;
};

#endif