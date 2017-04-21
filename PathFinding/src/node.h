/*
* node.h
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/21/2017
*
* Description:
*  Defines class Node with  (x,y) coordinates,
*  came from node (parent), neighboring nodes,
*  and traversal costs
*/

#ifndef NODE_H_DSTAR
#define NODE_H_DSTAR

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <math.h>
#define BYTE_INF 255

class Node {
public:
	Node();
	Node(byte row, byte col, byte up, byte right, byte down, byte left);

	void setActions(byte up, byte right, byte down, byte left);
	void setPos(byte row, byte col);

	Node& operator=(Node n);

	bool operator == (const Node& n) const;
	bool operator != (const Node& n) const;

	byte actions[4]; // available actions == { UP, RIGHT, DOWN, LEFT }; edge cost == number of squares to next node | if action is unavailable edge cost == 200
	byte row, col;
	byte g, rhs; 
};

#endif
