/*
* node.cpp
*
* Author: Will Rooney
* Date Created: 03/24/2017
* Date Modified: 04/21/2017
*
* Description:
*  Implementation of node.h
*/

#include "node.h"

Node::Node() {
	this->col = 0;
	this->row = 0;
	this->rhs = BYTE_INF;
	this->g = BYTE_INF;

	/* Set Neighbor Costs */
	this->actions[0] = BYTE_INF;
	this->actions[1] = BYTE_INF;
	this->actions[2] = BYTE_INF;
	this->actions[3] = BYTE_INF;
	
}

Node::Node(byte row, byte col, byte up, byte right, byte down, byte left) {
	this->row = row;
	this->col = col;
	this->rhs = BYTE_INF;
	this->g = BYTE_INF;

	/* Set Neighbor Costs */
	this->actions[0] = up;
	this->actions[1] = right;
	this->actions[2] = down;
	this->actions[3] = left;
}

void Node::setActions(byte up, byte right, byte down, byte left) {
	this->actions[0] = up;
	this->actions[1] = right;
	this->actions[2] = down;
	this->actions[3] = left;
}

void Node::setPos(byte row, byte col) {
	this->row = row;
	this->col = col;
}

Node& Node::operator=(Node n) {
	this->row = n.row;
	this->col = n.col;
	this->g = n.g;
	this->rhs = n.rhs;
	this->actions[0] = n.actions[0];
	this->actions[1] = n.actions[1];
	this->actions[2] = n.actions[2];
	this->actions[3] = n.actions[3];
	return *this;
}

bool Node::operator == (const Node& n) const {
	return (this->row == n.row && this->col == n.col);
}
bool Node::operator != (const Node& n) const { return !(*this == n); }



