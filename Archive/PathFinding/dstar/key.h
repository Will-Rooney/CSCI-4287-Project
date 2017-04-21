/*
* key.h
*
* Author: Will Rooney
* Date Created: 04/05/2017
* Date Modified: 04/08/2017
*
* Description:
*  Defines class Key.
*	From "Sven Koenig & Maxim Likhachev D* Lite" Paper:
*		"The priority of a vertex in the priority queue is always
*		the same as its key, which is a vector with two components:
*
*		k(s) = [k1(s); k2(s)]
*		k1(s) = min( g(s), rhs(s) + h(s, s_goal) )
*		k2(s) = min( g(s), rhs(s) )
*
*		The first component of te keys k1(s) corresponds directly
*		to the f-values f(s) := g*(s) + h(s, s_goal) used by A* ...
*		The second component of the keys k2(s) corresponds to the g-values of A*"
*
*		...
*		"If the robot moves and ten detects cost changes again, then the
*		constants need to get added up. We do this in the variable km..."
*/

#ifndef KEY_H_DSTAR
#define KEY_H_DSTAR

#include <algorithm>
#include <limits>

class Key {
public:
	Key() : k1(std::numeric_limits<float>::infinity()), k2(std::numeric_limits<float>::infinity()) {}
	Key(float k1, float k2) : k1(k1), k2(k2) {}
	Key(float g, float rhs, float h, float km) {
		this->k1 = std::min(g, rhs + h + km);
		this->k2 = std::min(g, rhs);
	}

	void update(float k1, float k2) {
		this->k1 = k1;
		this->k2 = k2;
	}

	void swap(Key& k) {
		std::swap(this->k1, k.k1);
		std::swap(this->k2, k.k2);
	}
	Key& operator=(Key k) {
		k.swap(*this);
		return *this;
	}
	bool operator < (const Key &kp) {
		return (k1 < kp.k1 || (k1 == kp.k1 && k2 < kp.k2));
	}

	float k1, k2;
};

#endif